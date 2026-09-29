/* 
  DisasterShield - Main Environmental Node M001
  ESP32

  Maintains the original M001 behaviour:
  - Startup beep
  - SOS button / long-press clear
  - Original SOS buzzer pattern: 200 ON / 200 OFF / 200 ON / 600 OFF
  - Evacuation buzzer pattern
  - WiFiManager
  - MQTT / FreeMQTT
  - 16x2 I2C LCD
  - Green / Yellow / Red LEDs
  - MQ-2 / MQ-7 relative percentage display
  - Water percentage
  - Worker W001 MQTT alerts
  - No raw ADC values published to the dashboard

  Updated:
  - IR/flame sensor moved to GPIO15 (D15)
  - Flame detection is debounced/confirmed to reduce false FIRE alarms
  - MPU6050 earthquake-like shaking uses acceleration + gyro sensor fusion
  - Weather context from live Open-Meteo data
  - Weather data is published to MQTT
  - Flood / landslide context uses local sensors + weather context
  - Weather is NOT used to claim earthquake or volcanic eruption prediction

  NEW:
  - MPU6050 SHAKE ALERT (demo-friendly, more sensitive than earthquake)
    with 0-100 intensity, buzzer pattern, LCD alert screen
  - APP ALERT STREAM on MQTT topic .../M001/alert (edge-triggered JSON)
  - Demo command: publish TEST to .../M001/command/shake
  - SOS button:
      short press          -> SOS ON
      2 s long press       -> SOS OFF (on release)
      5 s hold             -> WiFi reset (works at runtime)
      hold at boot         -> WiFi reset (original behaviour kept)

  IMPORTANT:
  - MQ percentages are relative to startup clean-air baseline, NOT ppm.
  - Water percentage is normalized ADC level, not calibrated water depth.
  - MPU6050 detects strong shaking/earthquake-like motion; it is not a
    calibrated seismic instrument.
  - Open-Meteo weather data is contextual weather/model information, not a
    guaranteed disaster forecast.
*/

#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <math.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define LCD_SDA_PIN 32
#define LCD_SCL_PIN 33

#define MPU_SDA_PIN 21
#define MPU_SCL_PIN 22

#define MQ2_PIN     34
#define MQ7_PIN     35
#define WATER_PIN    4

// User is using the IR/flame sensor on D15 = GPIO15.
#define IR_PIN      15

// Analog microphone/preamp module.
#define MIC_PIN     36

#define SOS_PIN     27

#define GREEN_LED   13
#define YELLOW_LED  12
#define RED_LED     26
#define BUZZER_PIN  14

// ============================================================
// IR OBJECT SENSOR SETTINGS
// ============================================================
// GPIO15 is connected to an IR obstacle/object detection module.
// It detects reflected IR from nearby objects; it is NOT a flame
// sensor. Therefore it must NEVER be used to declare FIRE.
const bool IR_OBJECT_ACTIVE_LOW = true;
const int IR_SAMPLE_COUNT = 7;
const int IR_REQUIRED_ACTIVE = 5;
const int IR_CONFIRM_CYCLES = 2;
const unsigned long IR_STARTUP_IGNORE_MS = 5000;

// ============================================================
// WEATHER LOCATION
// ============================================================
//
// Set these to the latitude/longitude of the deployment site.
//
// Example:
//   WEATHER_LAT = 10.79;
//   WEATHER_LON = 78.70;
//
// Open-Meteo does not require an API key.
//
float WEATHER_LAT = 10.80206338201709;
float WEATHER_LON = 78.73188225988986;

const unsigned long WEATHER_UPDATE_INTERVAL =
  10UL * 60UL * 1000UL;

// ============================================================
// OBJECTS
// ============================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MPU6050 mpu;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiManager wifiManager;

WiFiClientSecure weatherClient;

// ============================================================
// MQTT
// ============================================================

const char* MQTT_SERVER   = "broker.freemqtt.com";
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "freemqtt";
const char* MQTT_PASSWORD = "public";

const char* DATA_TOPIC =
  "smartmanhole/demo01/manhole/M001/data";

const char* EVENT_TOPIC =
  "smartmanhole/demo01/manhole/M001/event";

const char* COMMAND_TOPIC =
  "smartmanhole/demo01/manhole/M001/command/#";

const char* WORKER_DATA_TOPIC =
  "smartmanhole/demo01/worker/W001/data";

const char* ALIVE_TOPIC =
  "smartmanhole/demo01/manhole/M001/alive";

const char* ACK_TOPIC =
  "smartmanhole/demo01/manhole/M001/command/ack";

// NEW: alert stream for the custom app
const char* ALERT_TOPIC =
  "smartmanhole/demo01/manhole/M001/alert";

const char* NODE_ID = "M001";
const char* ZONE    = "ZONE_A";

// ============================================================
// USER-READABLE LOCAL SENSOR VALUES
// ============================================================

float gasLevelPercent = 0.0;
float coLevelPercent = 0.0;
float waterPercent = 0.0;
float acousticPercent = 0.0;

int mq2Raw = 0;
int mq7Raw = 0;
int waterRaw = 0;

// ============================================================
// MQ BASELINE
// ============================================================

float mq2Baseline = 0;
float mq7Baseline = 0;
bool mqCalibrated = false;

const int MQ_CAL_SAMPLES = 60;
const int MQ_CAL_DELAY_MS = 100;

// Relative rise only; NOT ppm.
const float GAS_ALERT_PERCENT = 45.0;

// ============================================================
// IR OBJECT SENSOR STATE
// ============================================================

bool irObjectDetected = false;
bool irRawActive = false;
int irActiveSamples = 0;
int irConfirmCycles = 0;
unsigned long irLastRead = 0;
const unsigned long IR_READ_INTERVAL = 250;

// Kept for dashboard compatibility. This is deliberately NEVER set true
// by the IR object sensor. Use a real flame sensor before enabling fire detection.
bool flameDetected = false;

bool gasDetected = false;

// ============================================================
// MICROPHONE
// ============================================================

float micRms = 0;
float micBaseline = 0;
const int MIC_SAMPLES = 80;

// ============================================================
// MPU6050 / EARTHQUAKE SENSOR FUSION
// ============================================================

bool mpuAvailable = false;

float accX = 0;
float accY = 0;
float accZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float accMagnitude = 0;
float dynamicAcceleration = 0;
float accelerationChange = 0;
float gyroMagnitude = 0;

float previousAccMagnitude = 9.81;

bool groundMovement = false;
bool earthquakeDetected = false;

int shakeHits = 0;
unsigned long shakeWindowStart = 0;
unsigned long lastEarthquakeTime = 0;

const unsigned long SHAKE_WINDOW = 1500;
const unsigned long EARTHQUAKE_COOLDOWN = 10000;

// Fusion thresholds.
// A strong acceleration spike OR combined accel + gyro movement
// creates a shaking sample. Several samples are required.
const float SHAKE_ACCEL_CHANGE = 1.20;  // m/s2
const float STRONG_ACCEL_CHANGE = 2.50; // m/s2
const float SHAKE_GYRO = 1.00;          // rad/s
const float GROUND_MOVEMENT_THRESHOLD = 1.50;

// ============================================================
// SHAKE ALERT (demo-friendly, more sensitive than earthquake)
// ============================================================

bool shakeAlertActive = false;
bool shakeAlertSample = false;
bool shakeTestRequested = false;

float shakeRawIntensity = 0;
float shakeIntensity = 0;      // 0-100 (relative, not magnitude)
int shakeAlertCount = 0;
int shakeAlertHits = 0;

unsigned long shakeAlertWindowStart = 0;
unsigned long shakeAlertStart = 0;

const unsigned long SHAKE_ALERT_WINDOW = 1200;
const unsigned long SHAKE_ALERT_HOLD   = 6000;   // alert stays visible 6 s
const int   SHAKE_ALERT_HITS           = 3;
const float SHAKE_ALERT_ACCEL_CHANGE   = 0.80;   // m/s2
const float SHAKE_ALERT_GYRO           = 1.50;   // rad/s

// ============================================================
// WEATHER STATE
// ============================================================

bool weatherAvailable = false;
String weatherStatus = "NO DATA";

float weatherTempC = 0;
float weatherHumidity = 0;
float weatherRainMm = 0;
float weatherPrecipitationMm = 0;
float weatherPressureHpa = 0;
float weatherWindKmh = 0;
float weatherWindGustKmh = 0;
float weatherSoilMoisture = 0;
int weatherCode = -1;

float forecastRain6h = 0;
float forecastPrecipitation6h = 0;

unsigned long lastWeatherUpdate = 0;

// Weather-derived contextual indicators.
bool heavyRainContext = false;
bool landslideWeatherContext = false;
bool fireWeatherContext = false;

// ============================================================
// HAZARD STATES
// ============================================================

bool floodRisk = false;
bool landslideRisk = false;

// IMPORTANT:
// fireRisk means confirmed local fire/flame.
// fireWeatherContext is separate and does NOT mean fire detected.
bool fireRisk = false;

String hazardStatus = "SAFE";

// Compatibility/status index for existing dashboards.
// This is a configurable fusion index, NOT probability.
int fusionIndex = 0;

// ============================================================
// WORKER STATUS
// ============================================================

bool workerSOS = false;
bool workerFall = false;
bool workerRescue = false;
String workerStatus = "NORMAL";

// ============================================================
// LOCAL SOS
// ============================================================

bool sosActive = false;

bool lastButtonState = HIGH;
bool buttonState = HIGH;

unsigned long buttonPressTime = 0;
unsigned long lastDebounceTime = 0;

const unsigned long DEBOUNCE_TIME = 50;
const unsigned long LONG_PRESS_TIME = 2000;        // 2 s  -> SOS OFF
const unsigned long WIFI_RESET_PRESS_TIME = 5000;  // 5 s  -> WiFi reset
bool buttonLongHandled = false;

// ============================================================
// BUZZER PATTERNS
// ============================================================

enum BuzzerMode
{
  BUZZER_OFF,
  BUZZER_SOS,
  BUZZER_FIRE,
  BUZZER_EARTHQUAKE,
  BUZZER_EVACUATE,
  BUZZER_SHAKE
};

BuzzerMode buzzerMode = BUZZER_OFF;
int buzzerStep = 0;
unsigned long buzzerTimer = 0;

// ============================================================
// TIMERS
// ============================================================

unsigned long lastSensorRead = 0;
unsigned long lastPublish = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastAlivePublish = 0;

const unsigned long SENSOR_INTERVAL = 250;
const unsigned long PUBLISH_INTERVAL = 2000;
const unsigned long LCD_INTERVAL = 1800;

// LCD pages: every page is shown for LCD_INTERVAL milliseconds.
// This restores the continuous scrolling/page-by-page information display.
int lcdPage = 0;
const int LCD_PAGE_COUNT = 15;

const unsigned long ALIVE_INTERVAL = 10000;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);

void readSensors();
void readWater();
void readGas();
void readIRObject();
void readMic();
void readMPU();

void calibrateMQ();

void updateWeather();
bool extractCurrentNumber(const String& json, const char* key, float& value);
bool extractCurrentInt(const String& json, const char* key, int& value);
float extractFirstHourlyValue(const String& json, const char* key);
float extractHourlySum(const String& json, const char* key, int count);

void calculateHazards();
void updateLEDs();

void updateShakeAlert();
void checkAlerts();
void publishAlert(const char* type, const char* severity,
                  const char* title, const char* message);

void checkSOS();
void updateBuzzer();
void setBuzzerMode(BuzzerMode mode);

void updateLCD();
void showWelcomeSequence();

void publishData();
void publishEvent(const char* eventName);

void processWorkerData(const String& message);

float clamp100(float value);
float relativeGasPercent(int raw, float baseline);

// ============================================================
// HELPERS
// ============================================================

float clamp100(float value)
{
  if (value < 0) return 0;
  if (value > 100) return 100;
  return value;
}

float relativeGasPercent(int raw, float baseline)
{
  if (!mqCalibrated || baseline <= 1)
    return 0;

  float percent =
    ((float)raw - baseline) / baseline * 100.0;

  return clamp100(percent);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=================================");
  Serial.println("      DISASTERSHIELD M001");
  Serial.println("   MAIN ENVIRONMENTAL NODE");
  Serial.println("=================================");

  // ---------- GPIO ----------
  pinMode(WATER_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ7_PIN, INPUT);

  // D15 / GPIO15.
  // INPUT is used to avoid forcing the ESP32 boot strap pin.
  // If your module output floats, use an external 10k pull-up to 3.3V.
  pinMode(IR_PIN, INPUT);

  pinMode(MIC_PIN, INPUT);
  pinMode(SOS_PIN, INPUT_PULLUP);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // ---------- LCD ----------
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  showWelcomeSequence();

  // ---------- STARTUP BEEP ----------
  // Maintained from the original M001 logic:
  // 150 ms ON, then OFF.
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);

  // ---------- MPU6050 ----------
  Wire1.begin(MPU_SDA_PIN, MPU_SCL_PIN);

  if (mpu.begin(0x68, &Wire1))
  {
    mpuAvailable = true;

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050: OK");
  }
  else
  {
    mpuAvailable = false;
    Serial.println("MPU6050: NOT FOUND");
  }

  // ---------- WiFi ----------
  // Hold SOS during startup to erase WiFi settings.
  if (digitalRead(SOS_PIN) == LOW)
  {
    Serial.println("WiFi reset requested");

    wifiManager.resetSettings();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Reset");
    lcd.setCursor(0, 1);
    lcd.print("Restarting...");
    delay(1000);

    ESP.restart();
  }

  wifiManager.setConfigPortalTimeout(180);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  if (!wifiManager.autoConnect("DisasterShield-M001"))
  {
    lcd.clear();
    lcd.print("WiFi Failed");
    delay(1500);
    ESP.restart();
  }

  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString().c_str());
  delay(1800);

  // ---------- MQTT ----------
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);

  connectMQTT();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQTT ");
  lcd.print(mqttClient.connected() ? "Connected" : "Offline");
  lcd.setCursor(0, 1);
  lcd.print("M001 Data Ready");
  delay(1500);

  // ---------- MQ CALIBRATION ----------
  calibrateMQ();

  // ---------- WEATHER ----------
  updateWeather();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("M001 READY");

  lcd.setCursor(0, 1);
  lcd.print("System Online");

  updateLEDs();

  delay(1000);

  Serial.println("DisasterShield M001 READY");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  if (!mqttClient.connected())
    connectMQTT();

  mqttClient.loop();

  checkSOS();
  updateBuzzer();

  // ---------- SENSOR UPDATE ----------
  if (millis() - lastSensorRead >= SENSOR_INTERVAL)
  {
    lastSensorRead = millis();

    readSensors();
    calculateHazards();
    updateLEDs();
    checkAlerts();   // NEW: app alerts (edge-triggered)
  }

  // ---------- WEATHER UPDATE ----------
  if (millis() - lastWeatherUpdate >= WEATHER_UPDATE_INTERVAL)
  {
    updateWeather();
  }

  // ---------- MQTT DATA ----------
  if (millis() - lastPublish >= PUBLISH_INTERVAL)
  {
    lastPublish = millis();
    publishData();
  }

  // ---------- LCD ----------
  if (millis() - lastLCDUpdate >= LCD_INTERVAL)
  {
    lastLCDUpdate = millis();
    updateLCD();
  }

  // ---------- ALIVE ----------
  if (millis() - lastAlivePublish >= ALIVE_INTERVAL)
  {
    lastAlivePublish = millis();

    if (mqttClient.connected())
    {
      mqttClient.publish(
        ALIVE_TOPIC,
        "ONLINE",
        true
      );
    }
  }
}

// ============================================================
// MQTT CONNECTION
// ============================================================

void connectMQTT()
{
  if (mqttClient.connected())
    return;

  if (WiFi.status() != WL_CONNECTED)
    return;

  String clientID =
    "DisasterShield_M001_" +
    String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.println("Connecting MQTT...");

  if (mqttClient.connect(
        clientID.c_str(),
        MQTT_USER,
        MQTT_PASSWORD))
  {
    Serial.println("MQTT connected");

    mqttClient.subscribe(COMMAND_TOPIC);
    mqttClient.subscribe(WORKER_DATA_TOPIC);

    mqttClient.publish(
      ALIVE_TOPIC,
      "ONLINE",
      true
    );
  }
  else
  {
    Serial.print("MQTT failed: ");
    Serial.println(mqttClient.state());
  }
}

// ============================================================
// MQTT CALLBACK
// ============================================================

void mqttCallback(
  char* topic,
  byte* payload,
  unsigned int length)
{
  String message;

  for (unsigned int i = 0; i < length; i++)
    message += (char)payload[i];

  message.trim();

  String topicString = String(topic);

  // ---------- MAIN NODE SOS ----------
  if (topicString ==
      "smartmanhole/demo01/manhole/M001/command/sos")
  {
    if (message == "ON" || message == "SOS_ON")
    {
      sosActive = true;
      setBuzzerMode(BUZZER_SOS);

      publishEvent("SOS");
      Serial.println("REMOTE SOS ON");
    }
    else if (message == "OFF" || message == "SOS_OFF")
    {
      sosActive = false;
      setBuzzerMode(BUZZER_OFF);

      publishEvent("SOS_CLEARED");
      Serial.println("REMOTE SOS OFF");
    }

    mqttClient.publish(
      ACK_TOPIC,
      "SOS COMMAND RECEIVED"
    );
  }

  // ---------- DEADMAN ----------
  else if (topicString ==
           "smartmanhole/demo01/manhole/M001/command/deadman")
  {
    if (message == "ALERT")
    {
      publishEvent("DEADMAN_ALERT");
      setBuzzerMode(BUZZER_SOS);
    }

    mqttClient.publish(
      ACK_TOPIC,
      "DEADMAN COMMAND RECEIVED"
    );
  }

  // ---------- SHAKE ALERT DEMO ----------
  else if (topicString ==
           "smartmanhole/demo01/manhole/M001/command/shake")
  {
    if (message == "TEST" || message == "ON")
      shakeTestRequested = true;

    mqttClient.publish(
      ACK_TOPIC,
      "SHAKE TEST RECEIVED"
    );
  }

  // ---------- WORKER W001 ----------
  else if (topicString == WORKER_DATA_TOPIC)
  {
    processWorkerData(message);
  }
}

// ============================================================
// WORKER DATA
// ============================================================

void processWorkerData(const String& message)
{
  if (message.indexOf("\"sos\":true") >= 0)
    workerSOS = true;
  else if (message.indexOf("\"sos\":false") >= 0)
    workerSOS = false;

  if (message.indexOf("\"fall\":true") >= 0)
    workerFall = true;
  else if (message.indexOf("\"fall\":false") >= 0)
    workerFall = false;

  if (message.indexOf("\"rescue\":true") >= 0)
    workerRescue = true;
  else if (message.indexOf("\"rescue\":false") >= 0)
    workerRescue = false;

  if (message.indexOf("\"status\":\"SAFE\"") >= 0)
    workerStatus = "SAFE";

  if (message.indexOf("\"status\":\"NORMAL\"") >= 0)
    workerStatus = "NORMAL";

  if (workerSOS)
    publishEvent("WORKER_SOS");

  if (workerFall)
    publishEvent("WORKER_FALL");

  if (workerRescue)
    publishEvent("WORKER_RESCUE");
}

// ============================================================
// SENSOR READ
// ============================================================

void readSensors()
{
  readWater();
  readGas();
  readIRObject();
  readMic();
  readMPU();
  updateShakeAlert();   // NEW

  if (mqCalibrated)
  {
    gasLevelPercent =
      relativeGasPercent(
        mq2Raw,
        mq2Baseline
      );

    coLevelPercent =
      relativeGasPercent(
        mq7Raw,
        mq7Baseline
      );
  }

  gasDetected =
    (gasLevelPercent >= GAS_ALERT_PERCENT ||
     coLevelPercent >= GAS_ALERT_PERCENT);

  acousticPercent =
    clamp100(
      ((micRms /
        (micBaseline > 5 ? micBaseline : 5))
       - 1.0) * 100.0
    );
}

// ============================================================
// WATER
// ============================================================

void readWater()
{
  waterRaw = analogRead(WATER_PIN);

  waterPercent =
    clamp100(
      ((float)waterRaw / 4095.0) * 100.0
    );
}

// ============================================================
// GAS
// ============================================================

void readGas()
{
  mq2Raw = analogRead(MQ2_PIN);
  mq7Raw = analogRead(MQ7_PIN);
}

// ============================================================
// IR OBJECT SENSOR
// ============================================================

void readIRObject()
{
  if (millis() < IR_STARTUP_IGNORE_MS)
  {
    irObjectDetected = false;
    irConfirmCycles = 0;
    return;
  }

  if (millis() - irLastRead < IR_READ_INTERVAL)
    return;

  irLastRead = millis();

  int activeCount = 0;

  for (int i = 0; i < IR_SAMPLE_COUNT; i++)
  {
    int state = digitalRead(IR_PIN);
    bool active = IR_OBJECT_ACTIVE_LOW ? (state == LOW) : (state == HIGH);

    if (active)
      activeCount++;

    delayMicroseconds(500);
  }

  irActiveSamples = activeCount;
  irRawActive = (activeCount >= IR_REQUIRED_ACTIVE);

  if (irRawActive)
  {
    if (irConfirmCycles < IR_CONFIRM_CYCLES)
      irConfirmCycles++;
  }
  else
  {
    if (irConfirmCycles > 0)
      irConfirmCycles--;
  }

  irObjectDetected = (irConfirmCycles >= IR_CONFIRM_CYCLES);

  // IMPORTANT: an ordinary IR object sensor cannot identify fire.
  // Never convert its object detection into flameDetected/fireRisk.
  flameDetected = false;
}

// ============================================================
// MICROPHONE
// ============================================================

void readMic()
{
  long sum = 0;

  for (int i = 0; i < MIC_SAMPLES; i++)
  {
    sum += analogRead(MIC_PIN);
    delayMicroseconds(120);
  }

  float mean =
    (float)sum / MIC_SAMPLES;

  double squareSum = 0;

  for (int i = 0; i < MIC_SAMPLES; i++)
  {
    int value = analogRead(MIC_PIN);

    float difference =
      value - mean;

    squareSum +=
      difference * difference;

    delayMicroseconds(120);
  }

  micRms =
    sqrt(
      squareSum / MIC_SAMPLES
    );

  if (micBaseline <= 1)
  {
    micBaseline = micRms;
  }
  else if (micRms < micBaseline * 1.5)
  {
    micBaseline =
      (micBaseline * 0.95) +
      (micRms * 0.05);
  }
}

// ============================================================
// MQ CALIBRATION
// ============================================================

void calibrateMQ()
{
  Serial.println();
  Serial.println("MQ calibration started.");
  Serial.println("Keep sensors in normal clean air.");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQ CALIBRATION");
  lcd.setCursor(0, 1);
  lcd.print("Keep air clean");

  double mq2Sum = 0;
  double mq7Sum = 0;

  for (int i = 0; i < MQ_CAL_SAMPLES; i++)
  {
    mq2Sum += analogRead(MQ2_PIN);
    mq7Sum += analogRead(MQ7_PIN);

    delay(MQ_CAL_DELAY_MS);
  }

  mq2Baseline =
    mq2Sum / MQ_CAL_SAMPLES;

  mq7Baseline =
    mq7Sum / MQ_CAL_SAMPLES;

  mqCalibrated = true;

  Serial.println("MQ calibration complete.");
}

// ============================================================
// MPU6050 SENSOR FUSION
// ============================================================

void readMPU()
{
  shakeAlertSample = false;   // NEW: reset every cycle

  if (!mpuAvailable)
    return;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(
    &accel,
    &gyro,
    &temperature
  );

  accX = accel.acceleration.x;
  accY = accel.acceleration.y;
  accZ = accel.acceleration.z;

  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;

  accMagnitude =
    sqrt(
      accX * accX +
      accY * accY +
      accZ * accZ
    );

  dynamicAcceleration =
    fabs(accMagnitude - 9.81);

  accelerationChange =
    fabs(
      accMagnitude -
      previousAccMagnitude
    );

  previousAccMagnitude =
    accMagnitude;

  gyroMagnitude =
    sqrt(
      gyroX * gyroX +
      gyroY * gyroY +
      gyroZ * gyroZ
    );

  // NEW: shake alert intensity (0-100) and sample detection
  shakeRawIntensity = clamp100(
    fmaxf(accelerationChange / STRONG_ACCEL_CHANGE,
          gyroMagnitude / 4.0f) * 100.0f);

  shakeAlertSample =
    (accelerationChange >= SHAKE_ALERT_ACCEL_CHANGE ||
     gyroMagnitude >= SHAKE_ALERT_GYRO);

  groundMovement =
    (dynamicAcceleration >=
     GROUND_MOVEMENT_THRESHOLD);

  // Acceleration + gyro fusion:
  // both must show movement, OR a very strong acceleration spike
  // must occur. Four samples are required inside 1.5 seconds.
  bool accelGyroFusion =
    (
      accelerationChange >=
        SHAKE_ACCEL_CHANGE
      &&
      gyroMagnitude >=
        SHAKE_GYRO
    );

  bool strongAcceleration =
    (
      accelerationChange >=
        STRONG_ACCEL_CHANGE
    );

  bool shakingSample =
    (
      accelGyroFusion ||
      strongAcceleration
    );

  unsigned long now = millis();

  if (shakingSample)
  {
    if (
      shakeWindowStart == 0 ||
      now - shakeWindowStart >
        SHAKE_WINDOW
    )
    {
      shakeWindowStart = now;
      shakeHits = 0;
    }

    shakeHits++;

    if (
      shakeHits >= 4 &&
      now - lastEarthquakeTime >=
        EARTHQUAKE_COOLDOWN
    )
    {
      earthquakeDetected = true;

      lastEarthquakeTime = now;

      publishEvent(
        "EARTHQUAKE_SHAKING"
      );

      Serial.println(
        "EARTHQUAKE-LIKE SHAKING DETECTED"
      );

      shakeHits = 0;
      shakeWindowStart = now;
    }
  }

  if (
    earthquakeDetected &&
    now - lastEarthquakeTime > 4000
  )
  {
    earthquakeDetected = false;
  }
}

// ============================================================
// SHAKE ALERT
// ============================================================

void updateShakeAlert()
{
  unsigned long now = millis();

  // Peak-hold with decay so the app can display it
  if (shakeRawIntensity > shakeIntensity)
    shakeIntensity = shakeRawIntensity;
  else
    shakeIntensity *= 0.90;

  // Demo trigger from MQTT command
  if (shakeTestRequested)
  {
    shakeTestRequested = false;
    shakeIntensity = 80;

    if (!shakeAlertActive)
      shakeAlertCount++;

    shakeAlertActive = true;
    shakeAlertStart = now;
    return;
  }

  if (shakeAlertSample)
  {
    if (shakeAlertWindowStart == 0 ||
        now - shakeAlertWindowStart > SHAKE_ALERT_WINDOW)
    {
      shakeAlertWindowStart = now;
      shakeAlertHits = 0;
    }

    shakeAlertHits++;

    if (shakeAlertHits >= SHAKE_ALERT_HITS)
    {
      if (!shakeAlertActive)
        shakeAlertCount++;

      shakeAlertActive = true;
      shakeAlertStart = now;   // refreshed while shaking continues
    }
  }

  if (shakeAlertActive &&
      now - shakeAlertStart > SHAKE_ALERT_HOLD)
  {
    shakeAlertActive = false;
    shakeAlertHits = 0;
    shakeAlertWindowStart = 0;
  }
}

// ============================================================
// APP ALERTS (edge-triggered, one message per new alert)
// ============================================================

void publishAlert(const char* type, const char* severity,
                  const char* title, const char* message)
{
  if (!mqttClient.connected())
    return;

  char payload[512];

  snprintf(
    payload,
    sizeof(payload),
    "{"
      "\"nodeId\":\"%s\","
      "\"zone\":\"%s\","
      "\"type\":\"%s\","
      "\"severity\":\"%s\","
      "\"title\":\"%s\","
      "\"message\":\"%s\","
      "\"intensity\":%.0f,"
      "\"hazardStatus\":\"%s\","
      "\"timestamp\":%lu"
    "}",
    NODE_ID, ZONE, type, severity, title, message,
    shakeIntensity, hazardStatus.c_str(), millis()
  );

  mqttClient.publish(ALERT_TOPIC, payload);

  Serial.print("ALERT: ");
  Serial.println(title);
}

void checkAlerts()
{
  static bool pShake = false, pEq = false, pGas = false,
              pFlood = false, pLand = false,
              pWSOS = false, pWFall = false, pSOS = false;

  if (shakeAlertActive && !pShake)
  {
    publishEvent("SHAKE_ALERT");
    publishAlert("SHAKE", "WARNING", "Shaking Detected",
                 "Vibration detected on M001 (MPU6050)");
  }
  if (!shakeAlertActive && pShake)
    publishAlert("SHAKE", "CLEARED", "Shaking Stopped",
                 "M001 vibration back to normal");

  if (earthquakeDetected && !pEq)
    publishAlert("EARTHQUAKE", "EVACUATE", "Earthquake-like Shaking",
                 "Strong sustained shaking detected. Evacuate.");

  if (gasDetected && !pGas)
    publishAlert("GAS", "WARNING", "Gas Level Rising",
                 "MQ-2 / MQ-7 rose above baseline threshold");

  if (floodRisk && !pFlood)
    publishAlert("FLOOD", "EVACUATE", "Flood Risk",
                 "High water level with rain context");

  if (landslideRisk && !pLand)
    publishAlert("LANDSLIDE", "EVACUATE", "Landslide Risk",
                 "Ground movement with rain/soil context");

  if (workerSOS && !pWSOS)
    publishAlert("WORKER_SOS", "EMERGENCY", "Worker SOS",
                 "Worker W001 pressed SOS");

  if (workerFall && !pWFall)
    publishAlert("WORKER_FALL", "EMERGENCY", "Worker Fall",
                 "Worker W001 fall detected");

  if (sosActive && !pSOS)
    publishAlert("SOS", "EMERGENCY", "Node SOS",
                 "SOS activated on M001");

  pShake = shakeAlertActive;
  pEq = earthquakeDetected;
  pGas = gasDetected;
  pFlood = floodRisk;
  pLand = landslideRisk;
  pWSOS = workerSOS;
  pWFall = workerFall;
  pSOS = sosActive;
}

// ============================================================
// WEATHER JSON HELPERS
// ============================================================

bool extractCurrentNumber(
  const String& json,
  const char* key,
  float& value)
{
  String pattern = "\"";
  pattern += key;
  pattern += "\":";

  int currentPos =
    json.indexOf("\"current\"");

  if (currentPos < 0)
    return false;

  int pos =
    json.indexOf(
      pattern,
      currentPos
    );

  if (pos < 0)
    return false;

  pos += pattern.length();

  int commaPos =
    json.indexOf(',', pos);

  int bracePos =
    json.indexOf('}', pos);

  int endPos = commaPos;

  if (
    endPos < 0 ||
    (bracePos >= 0 &&
     bracePos < endPos)
  )
  {
    endPos = bracePos;
  }

  if (endPos < 0)
    return false;

  String number =
    json.substring(
      pos,
      endPos
    );

  number.trim();

  if (
    number == "null" ||
    number.length() == 0
  )
    return false;

  value = number.toFloat();

  return true;
}

bool extractCurrentInt(
  const String& json,
  const char* key,
  int& value)
{
  float temp;

  if (!extractCurrentNumber(
        json,
        key,
        temp))
  {
    return false;
  }

  value = (int)temp;
  return true;
}

float extractFirstHourlyValue(
  const String& json,
  const char* key)
{
  String pattern = "\"";
  pattern += key;
  pattern += "\":[";

  int hourlyPos =
    json.indexOf("\"hourly\"");

  if (hourlyPos < 0)
    return -1;

  int pos =
    json.indexOf(
      pattern,
      hourlyPos
    );

  if (pos < 0)
    return -1;

  pos += pattern.length();

  int commaPos =
    json.indexOf(',', pos);

  int closePos =
    json.indexOf(']', pos);

  int endPos = commaPos;

  if (
    endPos < 0 ||
    (closePos >= 0 &&
     closePos < endPos)
  )
  {
    endPos = closePos;
  }

  if (endPos < 0)
    return -1;

  String number =
    json.substring(
      pos,
      endPos
    );

  number.trim();

  if (
    number == "null" ||
    number.length() == 0
  )
    return -1;

  return number.toFloat();
}

float extractHourlySum(
  const String& json,
  const char* key,
  int count)
{
  String pattern = "\"";
  pattern += key;
  pattern += "\":[";

  int hourlyPos =
    json.indexOf("\"hourly\"");

  if (hourlyPos < 0)
    return -1;

  int pos =
    json.indexOf(
      pattern,
      hourlyPos
    );

  if (pos < 0)
    return -1;

  pos += pattern.length();

  float sum = 0;
  int found = 0;

  while (
    found < count &&
    pos < (int)json.length()
  )
  {
    int commaPos =
      json.indexOf(',', pos);

    int closePos =
      json.indexOf(']', pos);

    int endPos = commaPos;

    if (
      endPos < 0 ||
      (closePos >= 0 &&
       closePos < endPos)
    )
    {
      endPos = closePos;
    }

    if (endPos < 0)
      break;

    String number =
      json.substring(
        pos,
        endPos
      );

    number.trim();

    if (
      number != "null" &&
      number.length() > 0
    )
    {
      sum += number.toFloat();
      found++;
    }

    pos = endPos + 1;
  }

  if (found == 0)
    return -1;

  return sum;
}

// ============================================================
// REAL WEATHER UPDATE
// ============================================================

void updateWeather()
{
  lastWeatherUpdate = millis();

  if (WiFi.status() != WL_CONNECTED)
  {
    weatherAvailable = false;
    weatherStatus = "WIFI OFF";
    return;
  }

  if (
    WEATHER_LAT == 0.0 &&
    WEATHER_LON == 0.0
  )
  {
    weatherAvailable = false;
    weatherStatus = "SET LAT/LON";

    Serial.println(
      "Weather disabled: set WEATHER_LAT and WEATHER_LON."
    );

    return;
  }

  Serial.println();
  Serial.println("Updating live weather...");

  String url =
    "https://api.open-meteo.com/v1/forecast";

  url += "?latitude=";
  url += String(WEATHER_LAT, 6);

  url += "&longitude=";
  url += String(WEATHER_LON, 6);

  url +=
    "&current=temperature_2m,"
    "relative_humidity_2m,"
    "precipitation,"
    "rain,"
    "pressure_msl,"
    "wind_speed_10m,"
    "wind_gusts_10m,"
    "weather_code";

  url +=
    "&hourly=precipitation,"
    "rain,"
    "soil_moisture_0_to_10cm";

  url +=
    "&forecast_hours=6"
    "&timezone=auto";

  // Open-Meteo uses HTTPS.
  weatherClient.setInsecure();

  HTTPClient http;

  if (!http.begin(
        weatherClient,
        url))
  {
    weatherAvailable = false;
    weatherStatus = "HTTP BEGIN ERR";

    Serial.println(
      "Weather HTTP begin failed."
    );

    return;
  }

  http.setTimeout(8000);

  int httpCode =
    http.GET();

  if (httpCode != HTTP_CODE_OK)
  {
    weatherAvailable = false;

    weatherStatus =
      "HTTP " +
      String(httpCode);

    Serial.print(
      "Weather HTTP error: "
    );
    Serial.println(httpCode);

    http.end();
    return;
  }

  String response =
    http.getString();

  http.end();

  if (response.length() < 50)
  {
    weatherAvailable = false;
    weatherStatus = "BAD DATA";
    return;
  }

  bool ok = true;

  ok &= extractCurrentNumber(
    response,
    "temperature_2m",
    weatherTempC
  );

  ok &= extractCurrentNumber(
    response,
    "relative_humidity_2m",
    weatherHumidity
  );

  extractCurrentNumber(
    response,
    "precipitation",
    weatherPrecipitationMm
  );

  extractCurrentNumber(
    response,
    "rain",
    weatherRainMm
  );

  extractCurrentNumber(
    response,
    "pressure_msl",
    weatherPressureHpa
  );

  extractCurrentNumber(
    response,
    "wind_speed_10m",
    weatherWindKmh
  );

  extractCurrentNumber(
    response,
    "wind_gusts_10m",
    weatherWindGustKmh
  );

  extractCurrentInt(
    response,
    "weather_code",
    weatherCode
  );

  float soil =
    extractFirstHourlyValue(
      response,
      "soil_moisture_0_to_10cm"
    );

  if (soil >= 0)
    weatherSoilMoisture = soil;

  float sixHourRain =
    extractHourlySum(
      response,
      "rain",
      6
    );

  if (sixHourRain >= 0)
    forecastRain6h = sixHourRain;

  float sixHourPrecip =
    extractHourlySum(
      response,
      "precipitation",
      6
    );

  if (sixHourPrecip >= 0)
    forecastPrecipitation6h =
      sixHourPrecip;

  weatherAvailable = ok;
  weatherStatus =
    weatherAvailable
    ? "LIVE"
    : "PARTIAL";

  // ---------- Weather contexts ----------
  heavyRainContext =
    (
      weatherRainMm >= 10.0 ||
      forecastRain6h >= 25.0 ||
      forecastPrecipitation6h >= 30.0
    );

  landslideWeatherContext =
    (
      forecastRain6h >= 20.0 &&
      weatherSoilMoisture >= 0.30
    );

  fireWeatherContext =
    (
      weatherTempC >= 35.0 &&
      weatherHumidity <= 35.0 &&
      weatherWindKmh >= 20.0 &&
      weatherRainMm < 1.0
    );

  Serial.println("Weather updated.");

  Serial.print("Temperature: ");
  Serial.print(weatherTempC, 1);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(weatherHumidity, 1);
  Serial.println(" %");

  Serial.print("Rain now: ");
  Serial.print(weatherRainMm, 1);
  Serial.println(" mm");

  Serial.print("6h rain forecast: ");
  Serial.print(forecastRain6h, 1);
  Serial.println(" mm");

  Serial.print("Pressure: ");
  Serial.print(weatherPressureHpa, 1);
  Serial.println(" hPa");

  Serial.print("Wind: ");
  Serial.print(weatherWindKmh, 1);
  Serial.println(" km/h");
}

// ============================================================
// HAZARD FUSION
// ============================================================

void calculateHazards()
{
  // FLOOD:
  // local water + significant current/forecast rain.
  floodRisk =
    (
      waterPercent >= 70.0 &&
      (
        heavyRainContext ||
        waterPercent >= 85.0
      )
    );

  // LANDSLIDE:
  // local ground movement + rainfall/soil moisture context.
  landslideRisk =
    (
      groundMovement &&
      (
        landslideWeatherContext ||
        forecastRain6h >= 15.0
      )
    );

  // FIRE:
  // ONLY the local fire sensor can set fireRisk.
  // Weather is kept separately as fireWeatherContext.
  fireRisk = false;

  int score = 0;

  if (floodRisk)
    score += 35;

  if (landslideRisk)
    score += 35;

  if (fireRisk)
    score += 35;

  if (earthquakeDetected)
    score += 50;

  // NEW: shake alert adds a smaller score
  if (shakeAlertActive && !earthquakeDetected)
    score += 15;

  if (gasDetected)
    score += 25;

  if (workerSOS || workerFall || workerRescue)
    score += 50;

  if (sosActive)
    score += 50;

  if (score > 100)
    score = 100;

  fusionIndex = score;

  // Hazard status.
  if (
    sosActive ||
    workerSOS ||
    workerRescue
  )
  {
    hazardStatus = "EMERGENCY";
  }
  else if (
    earthquakeDetected ||
    fireRisk ||
    floodRisk ||
    landslideRisk
  )
  {
    hazardStatus = "EVACUATE";
  }
  else if (
    gasDetected ||
    groundMovement ||
    shakeAlertActive ||     // NEW
    heavyRainContext ||
    waterPercent >= 60.0
  )
  {
    hazardStatus = "WARNING";
  }
  else if (
    waterPercent >= 40.0 ||
    acousticPercent >= 30.0 ||
    fireWeatherContext
  )
  {
    hazardStatus = "WATCH";
  }
  else
  {
    hazardStatus = "SAFE";
  }

  // Buzzer priority.
  if (
    sosActive ||
    workerSOS ||
    workerRescue
  )
  {
    setBuzzerMode(BUZZER_SOS);
  }
  else if (earthquakeDetected)
  {
    setBuzzerMode(BUZZER_EARTHQUAKE);
  }
  else if (
    floodRisk ||
    landslideRisk
  )
  {
    setBuzzerMode(BUZZER_EVACUATE);
  }
  else if (shakeAlertActive)     // NEW (lowest priority)
  {
    setBuzzerMode(BUZZER_SHAKE);
  }
  else
  {
    setBuzzerMode(BUZZER_OFF);
  }
}

// ============================================================
// LEDS
// ============================================================

void updateLEDs()
{
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  if (
    sosActive ||
    workerSOS ||
    workerRescue ||
    flameDetected ||
    earthquakeDetected ||
    floodRisk ||
    landslideRisk
  )
  {
    digitalWrite(RED_LED, HIGH);
  }
  else if (
    hazardStatus == "WARNING" ||
    hazardStatus == "WATCH"
  )
  {
    digitalWrite(YELLOW_LED, HIGH);
  }
  else
  {
    digitalWrite(GREEN_LED, HIGH);
  }
}

// ============================================================
// SOS BUTTON
// Short press          -> SOS ON
// 2 s+ press + release -> SOS OFF
// 5 s hold             -> WiFi reset
// ============================================================

void checkSOS()
{
  bool reading = digitalRead(SOS_PIN);

  if (reading != lastButtonState)
    lastDebounceTime = millis();

  if (millis() - lastDebounceTime > DEBOUNCE_TIME)
  {
    if (reading != buttonState)
    {
      buttonState = reading;

      if (buttonState == LOW)
      {
        buttonPressTime = millis();
        buttonLongHandled = false;
      }
      else
      {
        unsigned long duration = millis() - buttonPressTime;

        // 2-5 s long press released -> SOS OFF
        if (duration >= LONG_PRESS_TIME)
        {
          if (sosActive)
          {
            sosActive = false;
            setBuzzerMode(BUZZER_OFF);
            publishEvent("SOS_CLEARED");
            Serial.println("SOS CLEARED");
          }
        }
        // Short press -> SOS ON
        else if (!sosActive)
        {
          sosActive = true;
          setBuzzerMode(BUZZER_SOS);
          publishEvent("SOS");
          Serial.println("SHORT PRESS -> SOS");
        }
      }
    }
  }

  // 5 s hold (while still pressed) -> WiFi reset
  if (buttonState == LOW &&
      !buttonLongHandled &&
      millis() - buttonPressTime >= WIFI_RESET_PRESS_TIME)
  {
    buttonLongHandled = true;

    Serial.println("WiFi reset requested (5 s hold)");

    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Reset");
    lcd.setCursor(0, 1);
    lcd.print("Restarting...");

    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
  }

  lastButtonState = reading;
}

// ============================================================
// BUZZER MODE
// ============================================================

void setBuzzerMode(BuzzerMode mode)
{
  if (buzzerMode != mode)
  {
    buzzerMode = mode;
    buzzerStep = 0;
    buzzerTimer = millis();
  }

  if (mode == BUZZER_OFF)
  {
    digitalWrite(
      BUZZER_PIN,
      LOW
    );
  }
}

// ============================================================
// BUZZER PATTERNS
// ============================================================

void updateBuzzer()
{
  unsigned long now = millis();

  // OFF
  if (buzzerMode == BUZZER_OFF)
  {
    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    return;
  }

  // ----------------------------------------------------------
  // SOS
  // Original pattern retained:
  // ON 200 / OFF 200 / ON 200 / OFF 600
  // ----------------------------------------------------------

  if (buzzerMode == BUZZER_SOS)
  {
    switch (buzzerStep)
    {
      case 0:
        digitalWrite(BUZZER_PIN, HIGH);

        if (now - buzzerTimer >= 200)
        {
          buzzerTimer = now;
          buzzerStep = 1;
        }
        break;

      case 1:
        digitalWrite(BUZZER_PIN, LOW);

        if (now - buzzerTimer >= 200)
        {
          buzzerTimer = now;
          buzzerStep = 2;
        }
        break;

      case 2:
        digitalWrite(BUZZER_PIN, HIGH);

        if (now - buzzerTimer >= 200)
        {
          buzzerTimer = now;
          buzzerStep = 3;
        }
        break;

      case 3:
        digitalWrite(BUZZER_PIN, LOW);

        if (now - buzzerTimer >= 600)
        {
          buzzerTimer = now;
          buzzerStep = 0;
        }
        break;
    }

    return;
  }

  // ----------------------------------------------------------
  // FIRE
  // ON 700 / OFF 300
  // ----------------------------------------------------------

  if (buzzerMode == BUZZER_FIRE)
  {
    if (buzzerStep == 0)
    {
      digitalWrite(BUZZER_PIN, HIGH);

      if (now - buzzerTimer >= 700)
      {
        buzzerTimer = now;
        buzzerStep = 1;
      }
    }
    else
    {
      digitalWrite(BUZZER_PIN, LOW);

      if (now - buzzerTimer >= 300)
      {
        buzzerTimer = now;
        buzzerStep = 0;
      }
    }

    return;
  }

  // ----------------------------------------------------------
  // EARTHQUAKE-LIKE SHAKING
  // 3 quick pulses + pause
  // ----------------------------------------------------------

  if (buzzerMode == BUZZER_EARTHQUAKE)
  {
    switch (buzzerStep)
    {
      case 0:
        digitalWrite(BUZZER_PIN, HIGH);
        if (now - buzzerTimer >= 150)
        {
          buzzerTimer = now;
          buzzerStep = 1;
        }
        break;

      case 1:
        digitalWrite(BUZZER_PIN, LOW);
        if (now - buzzerTimer >= 150)
        {
          buzzerTimer = now;
          buzzerStep = 2;
        }
        break;

      case 2:
        digitalWrite(BUZZER_PIN, HIGH);
        if (now - buzzerTimer >= 150)
        {
          buzzerTimer = now;
          buzzerStep = 3;
        }
        break;

      case 3:
        digitalWrite(BUZZER_PIN, LOW);
        if (now - buzzerTimer >= 150)
        {
          buzzerTimer = now;
          buzzerStep = 4;
        }
        break;

      case 4:
        digitalWrite(BUZZER_PIN, HIGH);
        if (now - buzzerTimer >= 150)
        {
          buzzerTimer = now;
          buzzerStep = 5;
        }
        break;

      default:
        digitalWrite(BUZZER_PIN, LOW);
        if (now - buzzerTimer >= 800)
        {
          buzzerTimer = now;
          buzzerStep = 0;
        }
        break;
    }

    return;
  }

  // ----------------------------------------------------------
  // EVACUATE
  // 300 ms ON / 300 ms OFF
  // ----------------------------------------------------------

  if (buzzerMode == BUZZER_EVACUATE)
  {
    if (now - buzzerTimer >= 300)
    {
      buzzerTimer = now;

      if (buzzerStep == 0)
      {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerStep = 1;
      }
      else
      {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerStep = 0;
      }
    }

    return;
  }

  // ----------------------------------------------------------
  // SHAKE ALERT: 2 short beeps + pause
  // ----------------------------------------------------------

  if (buzzerMode == BUZZER_SHAKE)
  {
    unsigned long dur =
      (buzzerStep == 0 || buzzerStep == 2) ? 100 :
      (buzzerStep == 1) ? 100 : 1200;

    digitalWrite(BUZZER_PIN,
                 (buzzerStep == 0 || buzzerStep == 2) ? HIGH : LOW);

    if (now - buzzerTimer >= dur)
    {
      buzzerTimer = now;
      buzzerStep = (buzzerStep + 1) % 4;
    }

    return;
  }
}

// ============================================================
// MQTT DATA
// ============================================================

void publishData()
{
  if (!mqttClient.connected())
    return;

  char payload[2048];

  snprintf(
    payload,
    sizeof(payload),

    "{"
      "\"nodeId\":\"%s\","
      "\"zone\":\"%s\","

      "\"gasLevel\":%.1f,"
      "\"coLevel\":%.1f,"
      "\"waterLevel\":%.1f,"
      "\"acousticActivity\":%.1f,"

      "\"irObjectDetected\":%s,"
      "\"flameDetected\":%s,"
      "\"groundMovement\":%s,"
      "\"earthquakeDetected\":%s,"

      "\"shakeAlert\":%s,"
      "\"shakeIntensity\":%.0f,"
      "\"shakeCount\":%d,"

      "\"weatherAvailable\":%s,"
      "\"weatherStatus\":\"%s\","
      "\"temperatureC\":%.1f,"
      "\"humidity\":%.1f,"
      "\"rainNowMm\":%.2f,"
      "\"precipitationNowMm\":%.2f,"
      "\"pressureHpa\":%.1f,"
      "\"windKmh\":%.1f,"
      "\"windGustKmh\":%.1f,"
      "\"forecastRain6h\":%.2f,"
      "\"forecastPrecipitation6h\":%.2f,"
      "\"soilMoisture\":%.3f,"
      "\"weatherCode\":%d,"

      "\"floodRisk\":%s,"
      "\"landslideRisk\":%s,"
      "\"fireRisk\":%s,"
      "\"fireWeatherContext\":%s,"

      "\"workerSOS\":%s,"
      "\"workerFall\":%s,"
      "\"workerRescue\":%s,"
      "\"workerStatus\":\"%s\","

      "\"fusionIndex\":%d,"
      "\"hazardStatus\":\"%s\","
      "\"sos\":%s"

    "}",

    NODE_ID,
    ZONE,

    gasLevelPercent,
    coLevelPercent,
    waterPercent,
    acousticPercent,

    irObjectDetected ? "true" : "false",
    flameDetected ? "true" : "false",
    groundMovement ? "true" : "false",
    earthquakeDetected ? "true" : "false",

    shakeAlertActive ? "true" : "false",
    shakeIntensity,
    shakeAlertCount,

    weatherAvailable ? "true" : "false",
    weatherStatus.c_str(),

    weatherTempC,
    weatherHumidity,
    weatherRainMm,
    weatherPrecipitationMm,
    weatherPressureHpa,
    weatherWindKmh,
    weatherWindGustKmh,
    forecastRain6h,
    forecastPrecipitation6h,
    weatherSoilMoisture,
    weatherCode,

    floodRisk ? "true" : "false",
    landslideRisk ? "true" : "false",
    fireRisk ? "true" : "false",
    fireWeatherContext ? "true" : "false",

    workerSOS ? "true" : "false",
    workerFall ? "true" : "false",
    workerRescue ? "true" : "false",
    workerStatus.c_str(),

    fusionIndex,
    hazardStatus.c_str(),
    sosActive ? "true" : "false"
  );

  mqttClient.publish(
    DATA_TOPIC,
    payload
  );

  Serial.println();
  Serial.println("========== M001 STATUS ==========");

  Serial.print("Gas Level: ");
  Serial.print(gasLevelPercent, 1);
  Serial.println("%");

  Serial.print("CO Level: ");
  Serial.print(coLevelPercent, 1);
  Serial.println("%");

  Serial.print("Water Level: ");
  Serial.print(waterPercent, 1);
  Serial.println("%");

  Serial.print("IR Object: ");
  Serial.println(
    irObjectDetected ? "DETECTED" : "NONE"
  );

  Serial.println("Fire: NOT AVAILABLE (IR object sensor is not a flame sensor)");

  Serial.print("Earthquake/Shaking: ");
  Serial.println(
    earthquakeDetected ? "DETECTED" : "NORMAL"
  );

  Serial.print("Shake Alert: ");
  Serial.print(
    shakeAlertActive ? "ACTIVE" : "NORMAL"
  );
  Serial.print(" (intensity ");
  Serial.print(shakeIntensity, 0);
  Serial.println("%)");

  Serial.print("Weather: ");
  Serial.println(
    weatherAvailable ? "LIVE" : weatherStatus
  );

  if (weatherAvailable)
  {
    Serial.print("Temperature: ");
    Serial.print(weatherTempC, 1);
    Serial.println(" C");

    Serial.print("Rain now: ");
    Serial.print(weatherRainMm, 1);
    Serial.println(" mm");

    Serial.print("6h rain: ");
    Serial.print(forecastRain6h, 1);
    Serial.println(" mm");

    Serial.print("Wind: ");
    Serial.print(weatherWindKmh, 1);
    Serial.println(" km/h");
  }

  Serial.print("Flood: ");
  Serial.println(
    floodRisk ? "RISK" : "NORMAL"
  );

  Serial.print("Landslide: ");
  Serial.println(
    landslideRisk ? "RISK" : "NORMAL"
  );

  Serial.print("Fire weather context: ");
  Serial.println(
    fireWeatherContext ? "ELEVATED" : "NORMAL"
  );

  Serial.print("Status: ");
  Serial.println(hazardStatus);

  Serial.println("=================================");
}

// ============================================================
// MQTT EVENT
// ============================================================

void publishEvent(
  const char* eventName)
{
  if (!mqttClient.connected())
    return;

  char payload[512];

  snprintf(
    payload,
    sizeof(payload),

    "{"
      "\"nodeId\":\"%s\","
      "\"zone\":\"%s\","
      "\"event\":\"%s\","
      "\"hazardStatus\":\"%s\","
      "\"timestamp\":%lu"
    "}",

    NODE_ID,
    ZONE,
    eventName,
    hazardStatus.c_str(),
    millis()
  );

  mqttClient.publish(
    EVENT_TOPIC,
    payload
  );

  Serial.print("EVENT: ");
  Serial.println(eventName);
}

// ============================================================
// LCD
// ============================================================

void printLCDLine(uint8_t row, const String& text)
{
  lcd.setCursor(0, row);
  String out = text;
  if (out.length() > 16)
    out = out.substring(0, 16);
  lcd.print(out);

  for (int i = out.length(); i < 16; i++)
    lcd.print(' ');
}

void showWelcomeSequence()
{
  lcd.clear();
  printLCDLine(0, "DisasterShield");
  printLCDLine(1, "M001 Main Node");
  delay(900);

  // Marquee-style welcome message.
  lcd.clear();
  lcd.setCursor(16, 0);
  lcd.print("Welcome to DisasterShield");
  for (int i = 0; i < 18; i++)
  {
    lcd.scrollDisplayLeft();
    delay(120);
  }
  lcd.clear();
  printLCDLine(0, "Initializing...");
  printLCDLine(1, "Sensors + MQTT");
  delay(700);
}

void updateLCD()
{
  lcd.clear();

  // Emergency page always overrides normal pages.
  if (sosActive)
  {
    printLCDLine(0, "!!! SOS !!!");
    printLCDLine(1, "HELP REQUIRED");
    return;
  }

  // NEW: shake alert screen
  if (shakeAlertActive)
  {
    printLCDLine(0, "!! SHAKING !!");
    printLCDLine(1, "Intensity: " + String((int)shakeIntensity) + "%");
    return;
  }

  switch (lcdPage)
  {
    case 0:
      printLCDLine(0, WiFi.status() == WL_CONNECTED ? "WiFi: CONNECTED" : "WiFi: OFFLINE");
      printLCDLine(1, mqttClient.connected() ? "MQTT: CONNECTED" : "MQTT: OFFLINE");
      break;

    case 1:
      printLCDLine(0, "MQ2 Gas: " + String((int)gasLevelPercent) + "%");
      printLCDLine(1, "MQ7 CO:  " + String((int)coLevelPercent) + "%");
      break;

    case 2:
      printLCDLine(0, "Water: " + String((int)waterPercent) + "%");
      printLCDLine(1, "Acoustic: " + String((int)acousticPercent) + "%");
      break;

    case 3:
      printLCDLine(0, "Ground: " + String(groundMovement ? "MOVING" : "STABLE"));
      printLCDLine(1, "Shake: " + String(earthquakeDetected ? "DETECTED" : "NORMAL"));
      break;

    case 4:
      printLCDLine(0, "IR Object: " + String(irObjectDetected ? "YES" : "NO"));
      printLCDLine(1, "Fire sensor: N/A");
      break;

    case 5:
      if (weatherAvailable)
      {
        printLCDLine(0, "Temp: " + String(weatherTempC, 1) + "C");
        printLCDLine(1, "Humidity: " + String(weatherHumidity, 0) + "%");
      }
      else
      {
        printLCDLine(0, "Weather: NO DATA");
        printLCDLine(1, weatherStatus);
      }
      break;

    case 6:
      printLCDLine(0, "Rain: " + String(weatherRainMm, 1) + "mm");
      printLCDLine(1, "Precip6h: " + String(forecastPrecipitation6h, 1) + "mm");
      break;

    case 7:
      printLCDLine(0, "Wind: " + String(weatherWindKmh, 1) + "km/h");
      printLCDLine(1, "Gust: " + String(weatherWindGustKmh, 1) + "km/h");
      break;

    case 8:
      printLCDLine(0, "Pressure: " + String(weatherPressureHpa, 0));
      printLCDLine(1, "Soil: " + String(weatherSoilMoisture, 2));
      break;

    case 9:
      printLCDLine(0, "Weather: " + weatherStatus);
      printLCDLine(1, "Code: " + String(weatherCode));
      break;

    case 10:
      printLCDLine(0, "Worker: " + workerStatus);
      if (workerSOS || workerFall || workerRescue)
        printLCDLine(1, "WORKER ALERT");
      else
        printLCDLine(1, "Worker: NORMAL");
      break;

    case 11:
      printLCDLine(0, "Status: " + hazardStatus);
      printLCDLine(1, "Fusion: " + String(fusionIndex));
      break;

    case 12:
      printLCDLine(0, "IP Address:");
      if (WiFi.status() == WL_CONNECTED)
        printLCDLine(1, WiFi.localIP().toString());
      else
        printLCDLine(1, "Not connected");
      break;

    case 13:
      printLCDLine(0, "Node: " + String(NODE_ID));
      printLCDLine(1, "Zone: " + String(ZONE));
      break;

    // NEW: shake alert status page
    case 14:
      printLCDLine(0, "Shake: " + String(shakeAlertActive ? "ALERT" : "NORMAL"));
      printLCDLine(1, "Count: " + String(shakeAlertCount));
      break;
  }

  lcdPage++;
  if (lcdPage >= LCD_PAGE_COUNT)
    lcdPage = 0;
}
