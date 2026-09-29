/*
  ============================================================
                 MINESHIELD WORKER NODE - W001
  ============================================================

  ESP32 Worker Wearable

  ------------------------------------------------------------
  HARDWARE
  ------------------------------------------------------------

  MAX30102
      SDA -> GPIO 32
      SCL -> GPIO 33

  MPU6050
      SDA -> GPIO 16
      SCL -> GPIO 17

  SOS / WIFI RESET / DEADMAN RESPONSE BUTTON
      GPIO 27 -> BUTTON -> GND

  BUZZER
      + -> GPIO 14
      - -> GND


  ------------------------------------------------------------
  BUTTON LOGIC
  ------------------------------------------------------------

  NORMAL:

      Short press
          -> SOS ON

      Hold 5 seconds
          -> WIFI RESET
          -> ESP32 RESTARTS
          -> SOS DOES NOT ACTIVATE


  SOS ACTIVE:

      Hold 2 seconds
          -> SOS OFF


  DEADMAN ACTIVE:

      Short press
          -> WORKER RESPONDED
          -> DEADMAN OFF

      Hold 5 seconds
          -> WIFI RESET


  ------------------------------------------------------------
  MQTT
  ------------------------------------------------------------

  Broker:
      broker.freemqtt.com

  Port:
      1883

  Username:
      freemqtt

  Password:
      public


  W001 TOPICS:

      smartmanhole/demo01/worker/W001/data

      smartmanhole/demo01/worker/W001/event

      smartmanhole/demo01/worker/W001/alive

      smartmanhole/demo01/worker/W001/command/sos

      smartmanhole/demo01/worker/W001/command/deadman

      smartmanhole/demo01/worker/W001/command/ack


  ------------------------------------------------------------
  IMPORTANT
  ------------------------------------------------------------

  MAX30102:
      Heart rate is calculated from beat detection.

  SpO2:
      This code does NOT fabricate SpO2.
      It remains 0 until a validated SpO2 algorithm is added.

  MPU6050:
      Direct acceleration and gyro measurements.

  Prototype only. Not a certified mining safety device.
*/


// ============================================================
// LIBRARIES
// ============================================================

#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#include <Wire.h>

#include "MAX30105.h"
#include "heartRate.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// ============================================================
// PIN DEFINITIONS
// ============================================================

// MAX30102
#define MAX_SDA_PIN 32
#define MAX_SCL_PIN 33

// MPU6050
#define MPU_SDA_PIN 16
#define MPU_SCL_PIN 17

// Button
#define SOS_BUTTON_PIN 25

// Buzzer
#define BUZZER_PIN 27


// ============================================================
// DEVICE INFORMATION
// ============================================================

const char* WORKER_ID = "W001";

const char* ZONE_ID = "ZONE_01";


// ============================================================
// MQTT SETTINGS
// ============================================================

const char* MQTT_BROKER =
  "broker.freemqtt.com";

const int MQTT_PORT = 1883;

const char* MQTT_USERNAME =
  "freemqtt";

const char* MQTT_PASSWORD =
  "public";


// ============================================================
// MQTT TOPICS
// ============================================================

const char* DATA_TOPIC =
  "smartmanhole/demo01/worker/W001/data";

const char* EVENT_TOPIC =
  "smartmanhole/demo01/worker/W001/event";

const char* ALIVE_TOPIC =
  "smartmanhole/demo01/worker/W001/alive";

const char* SOS_COMMAND_TOPIC =
  "smartmanhole/demo01/worker/W001/command/sos";

const char* DEADMAN_COMMAND_TOPIC =
  "smartmanhole/demo01/worker/W001/command/deadman";

const char* ACK_TOPIC =
  "smartmanhole/demo01/worker/W001/command/ack";


// ============================================================
// NETWORK
// ============================================================

WiFiClient espClient;

PubSubClient mqttClient(espClient);

WiFiManager wifiManager;


// ============================================================
// I2C
// ============================================================

TwoWire I2C_MAX =
  TwoWire(0);

TwoWire I2C_MPU =
  TwoWire(1);


// ============================================================
// SENSOR OBJECTS
// ============================================================

MAX30105 max30102;

Adafruit_MPU6050 mpu;


// ============================================================
// SENSOR STATUS
// ============================================================

bool max30102Available = false;

bool mpuAvailable = false;


// ============================================================
// MAX30102
// ============================================================

long irValue = 0;

float heartRate = 0.0;

bool heartRateValid = false;

// No fake SpO2
int spo2 = 0;


// ============================================================
// MPU6050
// ============================================================

float accelX = 0.0;

float accelY = 0.0;

float accelZ = 0.0;

float gyroX = 0.0;

float gyroY = 0.0;

float gyroZ = 0.0;

float totalAcceleration = 0.0;


// ============================================================
// SOS
// ============================================================

bool sosActive = false;


// ============================================================
// DEADMAN
// ============================================================

bool deadmanActive = false;

bool rescueEscalated = false;

unsigned long deadmanStartTime = 0;

// 5 second worker response period
const unsigned long DEADMAN_TIMEOUT = 5000;


// ============================================================
// BUTTON
// ============================================================

bool buttonDown = false;

bool actionAlreadyDone = false;

unsigned long buttonPressStart = 0;


// ============================================================
// BUZZER
// ============================================================

int buzzerStep = 0;

unsigned long buzzerTimer = 0;


// ============================================================
// TIMERS
// ============================================================

unsigned long lastSensorRead = 0;

unsigned long lastMQTTPublish = 0;

unsigned long lastAlivePublish = 0;

unsigned long lastMQTTAttempt = 0;


// Sensor
const unsigned long SENSOR_INTERVAL = 100;


// MQTT
const unsigned long MQTT_INTERVAL = 2000;


// Alive
const unsigned long ALIVE_INTERVAL = 10000;


// MQTT reconnect
const unsigned long MQTT_RECONNECT_INTERVAL = 3000;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void connectMQTT();

void mqttCallback(
  char* topic,
  byte* payload,
  unsigned int length
);

void activateSOS();

void clearSOS();

void checkButton();

void updateBuzzer();

void readSensors();

void readMAX30102();

void readMPU6050();

void updateDeadman();

void startDeadman();

void clearDeadman();

void publishData();

void publishEvent(
  const char* eventName
);

void publishACK(
  const char* message
);

void publishAlive();

void resetWiFi();

String cleanMessage(
  String message
);


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "       MINESHIELD WORKER W001"
  );

  Serial.println(
    "======================================"
  );


  // ==========================================================
  // GPIO
  // ==========================================================

  pinMode(
    SOS_BUTTON_PIN,
    INPUT_PULLUP
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  // ==========================================================
  // STARTUP BEEP
  // ==========================================================

  digitalWrite(
    BUZZER_PIN,
    HIGH
  );

  delay(150);

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  // ==========================================================
  // MAX30102
  // ==========================================================

  Serial.println();
  Serial.println(
    "Starting MAX30102..."
  );


  I2C_MAX.begin(
    MAX_SDA_PIN,
    MAX_SCL_PIN,
    100000
  );


  delay(100);


  if (
    max30102.begin(
      I2C_MAX,
      I2C_SPEED_STANDARD
    )
  )
  {
    max30102Available = true;

    Serial.println(
      "MAX30102: CONNECTED"
    );


    max30102.setup();


    max30102.setPulseAmplitudeRed(
      0x0A
    );

    max30102.setPulseAmplitudeIR(
      0x0A
    );

    max30102.setPulseAmplitudeGreen(
      0
    );
  }
  else
  {
    max30102Available = false;

    Serial.println(
      "MAX30102: NOT FOUND"
    );
  }


  // ==========================================================
  // MPU6050
  // ==========================================================

  Serial.println();
  Serial.println(
    "Starting MPU6050..."
  );


  I2C_MPU.begin(
    MPU_SDA_PIN,
    MPU_SCL_PIN,
    100000
  );


  delay(100);


  if (
    mpu.begin(
      0x68,
      &I2C_MPU
    )
  )
  {
    mpuAvailable = true;

    Serial.println(
      "MPU6050: CONNECTED"
    );


    mpu.setAccelerometerRange(
      MPU6050_RANGE_8_G
    );

    mpu.setGyroRange(
      MPU6050_RANGE_500_DEG
    );

    mpu.setFilterBandwidth(
      MPU6050_BAND_21_HZ
    );
  }
  else
  {
    // Try 0x69 too

    Serial.println(
      "MPU6050 not found at 0x68"
    );


    if (
      mpu.begin(
        0x69,
        &I2C_MPU
      )
    )
    {
      mpuAvailable = true;

      Serial.println(
        "MPU6050: CONNECTED at 0x69"
      );


      mpu.setAccelerometerRange(
        MPU6050_RANGE_8_G
      );

      mpu.setGyroRange(
        MPU6050_RANGE_500_DEG
      );

      mpu.setFilterBandwidth(
        MPU6050_BAND_21_HZ
      );
    }
    else
    {
      mpuAvailable = false;

      Serial.println(
        "MPU6050: NOT FOUND"
      );
    }
  }


  // ==========================================================
  // WIFI MANAGER
  // ==========================================================

  Serial.println();
  Serial.println(
    "Starting WiFiManager..."
  );


  wifiManager.setConfigPortalTimeout(
    180
  );


  bool wifiConnected =
    wifiManager.autoConnect(
      "MineShield-W001"
    );


  if (!wifiConnected)
  {
    Serial.println(
      "WiFi connection failed."
    );

    delay(2000);

    ESP.restart();
  }


  Serial.println();
  Serial.println(
    "WiFi connected!"
  );

  Serial.print(
    "IP address: "
  );

  Serial.println(
    WiFi.localIP()
  );


  // ==========================================================
  // MQTT
  // ==========================================================

  mqttClient.setServer(
    MQTT_BROKER,
    MQTT_PORT
  );

  mqttClient.setCallback(
    mqttCallback
  );

  mqttClient.setBufferSize(
    1024
  );


  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "        WORKER W001 READY"
  );

  Serial.println(
    "======================================"
  );

  Serial.println();

  Serial.println(
    "BUTTON:"
  );

  Serial.println(
    "Short press = SOS"
  );

  Serial.println(
    "Hold 2 sec during SOS = SOS OFF"
  );

  Serial.println(
    "Short press during DEADMAN = RESPONSE"
  );

  Serial.println(
    "Hold 5 sec = WIFI RESET"
  );

  Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // BUTTON
  // ----------------------------------------------------------

  checkButton();


  // ----------------------------------------------------------
  // MQTT CONNECTION
  // ----------------------------------------------------------

  if (
    !mqttClient.connected()
  )
  {
    if (
      millis() - lastMQTTAttempt >=
      MQTT_RECONNECT_INTERVAL
    )
    {
      lastMQTTAttempt =
        millis();

      connectMQTT();
    }
  }
  else
  {
    mqttClient.loop();
  }


  // ----------------------------------------------------------
  // BUTTON AGAIN
  // ----------------------------------------------------------

  checkButton();


  // ----------------------------------------------------------
  // DEADMAN
  // ----------------------------------------------------------

  updateDeadman();


  // ----------------------------------------------------------
  // BUZZER
  // ----------------------------------------------------------

  updateBuzzer();


  // ----------------------------------------------------------
  // SENSOR READING
  // ----------------------------------------------------------

  if (
    millis() - lastSensorRead >=
    SENSOR_INTERVAL
  )
  {
    lastSensorRead =
      millis();

    readSensors();
  }


  // ----------------------------------------------------------
  // MQTT DATA
  // ----------------------------------------------------------

  if (
    millis() - lastMQTTPublish >=
    MQTT_INTERVAL
  )
  {
    lastMQTTPublish =
      millis();

    publishData();
  }


  // ----------------------------------------------------------
  // ALIVE
  // ----------------------------------------------------------

  if (
    millis() - lastAlivePublish >=
    ALIVE_INTERVAL
  )
  {
    lastAlivePublish =
      millis();

    publishAlive();
  }


  delay(5);
}


// ============================================================
// BUTTON LOGIC
// ============================================================

void checkButton()
{
  bool pressed =
    (
      digitalRead(
        SOS_BUTTON_PIN
      ) == LOW
    );


  // ==========================================================
  // BUTTON JUST PRESSED
  // ==========================================================

  if (
    pressed &&
    !buttonDown
  )
  {
    buttonDown = true;

    actionAlreadyDone = false;

    buttonPressStart =
      millis();


    Serial.println();
    Serial.println(
      "BUTTON PRESSED"
    );
  }


  // ==========================================================
  // BUTTON BEING HELD
  // ==========================================================

  if (
    pressed &&
    buttonDown
  )
  {
    unsigned long heldTime =
      millis() -
      buttonPressStart;


    // ========================================================
    // SOS ACTIVE
    //
    // 2 SECOND HOLD = SOS OFF
    // ========================================================

    if (
      sosActive &&
      !actionAlreadyDone &&
      heldTime >= 2000
    )
    {
      actionAlreadyDone = true;

      clearSOS();


      Serial.println(
        "2 SECOND HOLD -> SOS OFF"
      );
    }


    // ========================================================
    // NORMAL / DEADMAN
    //
    // 5 SECOND HOLD = WIFI RESET
    //
    // This is checked regardless of deadman state.
    // ========================================================

    if (
      !sosActive &&
      !actionAlreadyDone &&
      heldTime >= 5000
    )
    {
      actionAlreadyDone = true;


      Serial.println();
      Serial.println(
        "======================================"
      );

      Serial.println(
        "5 SECOND HOLD"
      );

      Serial.println(
        "WIFI RESET"
      );

      Serial.println(
        "SOS WILL NOT ACTIVATE"
      );

      Serial.println(
        "======================================"
      );


      resetWiFi();
    }
  }


  // ==========================================================
  // BUTTON RELEASED
  // ==========================================================

  if (
    !pressed &&
    buttonDown
  )
  {
    unsigned long pressDuration =
      millis() -
      buttonPressStart;


    buttonDown = false;


    // ========================================================
    // IF LONG-PRESS ACTION ALREADY OCCURRED
    //
    // Do NOT generate SOS.
    // ========================================================

    if (
      actionAlreadyDone
    )
    {
      actionAlreadyDone = false;

      return;
    }


    // ========================================================
    // DEADMAN ACTIVE
    //
    // SHORT PRESS = WORKER RESPONSE
    //
    // This MUST happen before normal SOS logic.
    // ========================================================

    if (
      deadmanActive &&
      pressDuration < 2000
    )
    {
      clearDeadman();

      Serial.println(
        "WORKER BUTTON RESPONSE -> DEADMAN CLEARED"
      );

      return;
    }


    // ========================================================
    // NORMAL
    //
    // SHORT PRESS = SOS
    // ========================================================

    if (
      !sosActive &&
      !deadmanActive &&
      pressDuration < 5000
    )
    {
      activateSOS();

      Serial.println(
        "SHORT PRESS -> SOS ACTIVATED"
      );
    }
  }
}


// ============================================================
// ACTIVATE SOS
// ============================================================

void activateSOS()
{
  if (sosActive)
  {
    return;
  }


  sosActive = true;


  buzzerStep = 0;

  buzzerTimer =
    millis();


  Serial.println();
  Serial.println(
    "**************************************"
  );

  Serial.println(
    "          WORKER SOS ACTIVE"
  );

  Serial.println(
    "**************************************"
  );


  publishEvent(
    "SOS_PRESSED"
  );


  publishACK(
    "SOS ACTIVE"
  );
}


// ============================================================
// CLEAR SOS
// ============================================================

void clearSOS()
{
  sosActive = false;


  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  buzzerStep = 0;


  Serial.println();
  Serial.println(
    "WORKER SOS CLEARED"
  );


  publishEvent(
    "SOS_CLEARED"
  );


  publishACK(
    "SOS CLEARED"
  );
}


// ============================================================
// START DEADMAN
// ============================================================

void startDeadman()
{
  deadmanActive = true;

  rescueEscalated = false;

  deadmanStartTime =
    millis();


  buzzerStep = 0;

  buzzerTimer =
    millis();


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "       DEAD-MAN CHECK STARTED"
  );

  Serial.println(
    "       RESPOND WITH BUTTON"
  );

  Serial.println(
    "       TIME: 5 SECONDS"
  );

  Serial.println(
    "======================================"
  );


  publishACK(
    "DEADMAN ACTIVE"
  );


  publishEvent(
    "DEADMAN_ALERT"
  );
}


// ============================================================
// CLEAR DEADMAN
// ============================================================

void clearDeadman()
{
  deadmanActive = false;

  rescueEscalated = false;


  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  buzzerStep = 0;


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "       WORKER RESPONDED"
  );

  Serial.println(
    "       DEAD-MAN CLEARED"
  );

  Serial.println(
    "======================================"
  );


  publishACK(
    "DEADMAN CLEARED"
  );


  publishEvent(
    "WORKER_RESPONDED"
  );
}


// ============================================================
// DEADMAN
// ============================================================

void updateDeadman()
{
  if (!deadmanActive)
  {
    return;
  }


  // ----------------------------------------------------------
  // 5 SECOND TIMEOUT
  // ----------------------------------------------------------

  if (
    millis() -
    deadmanStartTime >=
    DEADMAN_TIMEOUT
  )
  {
    if (!rescueEscalated)
    {
      rescueEscalated = true;


      Serial.println();
      Serial.println(
        "**************************************"
      );

      Serial.println(
        "       WORKER UNRESPONSIVE"
      );

      Serial.println(
        "          RESCUE REQUIRED"
      );

      Serial.println(
        "**************************************"
      );


      publishACK(
        "WORKER_UNRESPONSIVE"
      );


      publishEvent(
        "RESCUE_REQUIRED"
      );


      /*
        IMPORTANT:

        Keep deadmanActive TRUE.

        Therefore the buzzer continues.

        The control app can clear the alarm
        using DEADMAN OFF.
      */
    }
  }
}


// ============================================================
// BUZZER
// ============================================================

void updateBuzzer()
{
  bool alarmActive =
    (
      sosActive ||
      deadmanActive
    );


  if (!alarmActive)
  {
    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    buzzerStep = 0;

    return;
  }


  unsigned long now =
    millis();


  /*
      Pattern:

      ON  200 ms
      OFF 200 ms
      ON  200 ms
      OFF 600 ms
  */


  switch (buzzerStep)
  {
    // --------------------------------------------------------
    // ON
    // --------------------------------------------------------

    case 0:

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );


      if (
        now - buzzerTimer >=
        200
      )
      {
        buzzerTimer =
          now;

        buzzerStep = 1;
      }

      break;


    // --------------------------------------------------------
    // OFF
    // --------------------------------------------------------

    case 1:

      digitalWrite(
        BUZZER_PIN,
        LOW
      );


      if (
        now - buzzerTimer >=
        200
      )
      {
        buzzerTimer =
          now;

        buzzerStep = 2;
      }

      break;


    // --------------------------------------------------------
    // ON
    // --------------------------------------------------------

    case 2:

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );


      if (
        now - buzzerTimer >=
        200
      )
      {
        buzzerTimer =
          now;

        buzzerStep = 3;
      }

      break;


    // --------------------------------------------------------
    // LONG OFF
    // --------------------------------------------------------

    case 3:

      digitalWrite(
        BUZZER_PIN,
        LOW
      );


      if (
        now - buzzerTimer >=
        600
      )
      {
        buzzerTimer =
          now;

        buzzerStep = 0;
      }

      break;
  }
}


// ============================================================
// SENSOR READING
// ============================================================

void readSensors()
{
  readMAX30102();

  readMPU6050();
}


// ============================================================
// MAX30102
// ============================================================

void readMAX30102()
{
  if (!max30102Available)
  {
    irValue = 0;

    heartRate = 0;

    heartRateValid = false;

    spo2 = 0;

    return;
  }


  irValue =
    max30102.getIR();


  // ----------------------------------------------------------
  // NO FINGER
  // ----------------------------------------------------------

  if (
    irValue < 50000
  )
  {
    heartRate = 0;

    heartRateValid = false;

    spo2 = 0;

    return;
  }


  // ----------------------------------------------------------
  // HEART RATE
  // ----------------------------------------------------------

  if (
    checkForBeat(irValue)
  )
  {
    static unsigned long lastBeat =
      0;


    unsigned long now =
      millis();


    unsigned long delta =
      now -
      lastBeat;


    lastBeat =
      now;


    if (
      delta > 250 &&
      delta < 2000
    )
    {
      float bpm =
        60.0 /
        (
          delta /
          1000.0
        );


      if (
        bpm >= 40 &&
        bpm <= 200
      )
      {
        heartRate =
          bpm;

        heartRateValid =
          true;
      }
    }
  }


  // ----------------------------------------------------------
  // NO FABRICATED SpO2
  // ----------------------------------------------------------

  spo2 = 0;
}


// ============================================================
// MPU6050
// ============================================================

void readMPU6050()
{
  if (!mpuAvailable)
  {
    accelX = 0;

    accelY = 0;

    accelZ = 0;

    gyroX = 0;

    gyroY = 0;

    gyroZ = 0;

    totalAcceleration = 0;

    return;
  }


  sensors_event_t accel;

  sensors_event_t gyro;

  sensors_event_t temp;


  mpu.getEvent(
    &accel,
    &gyro,
    &temp
  );


  accelX =
    accel.acceleration.x;

  accelY =
    accel.acceleration.y;

  accelZ =
    accel.acceleration.z;


  gyroX =
    gyro.gyro.x;

  gyroY =
    gyro.gyro.y;

  gyroZ =
    gyro.gyro.z;


  totalAcceleration =
    sqrt(
      accelX * accelX +
      accelY * accelY +
      accelZ * accelZ
    );
}


// ============================================================
// MQTT CONNECTION
// ============================================================

void connectMQTT()
{
  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    return;
  }


  if (
    mqttClient.connected()
  )
  {
    return;
  }


  Serial.print(
    "Connecting to FreeMQTT..."
  );


  /*
      Unique client ID.

      M001 and W001 can therefore
      connect to the same broker.
  */

  String clientID =
    "MineShield_W001_" +
    String(
      (uint32_t)ESP.getEfuseMac(),
      HEX
    );


  if (
    mqttClient.connect(
      clientID.c_str(),
      MQTT_USERNAME,
      MQTT_PASSWORD
    )
  )
  {
    Serial.println(
      " CONNECTED"
    );


    // --------------------------------------------------------
    // SOS
    // --------------------------------------------------------

    bool sosSubscribed =
      mqttClient.subscribe(
        SOS_COMMAND_TOPIC
      );


    // --------------------------------------------------------
    // DEADMAN
    // --------------------------------------------------------

    bool deadmanSubscribed =
      mqttClient.subscribe(
        DEADMAN_COMMAND_TOPIC
      );


    Serial.print(
      "SOS subscription: "
    );

    Serial.println(
      sosSubscribed
        ? "OK"
        : "FAILED"
    );


    Serial.print(
      "Deadman subscription: "
    );

    Serial.println(
      deadmanSubscribed
        ? "OK"
        : "FAILED"
    );


    publishACK(
      "W001 ONLINE"
    );


    publishEvent(
      "WORKER_ONLINE"
    );


    publishAlive();
  }
  else
  {
    Serial.print(
      " FAILED, MQTT state = "
    );

    Serial.println(
      mqttClient.state()
    );
  }
}


// ============================================================
// MQTT CALLBACK
// ============================================================

void mqttCallback(
  char* topic,
  byte* payload,
  unsigned int length
)
{
  String message = "";


  for (
    unsigned int i = 0;
    i < length;
    i++
  )
  {
    message +=
      (char)payload[i];
  }


  message =
    cleanMessage(
      message
    );


  String topicString =
    String(topic);


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "         MQTT COMMAND RECEIVED"
  );

  Serial.print(
    "TOPIC: "
  );

  Serial.println(
    topicString
  );

  Serial.print(
    "MESSAGE: "
  );

  Serial.println(
    message
  );

  Serial.println(
    "======================================"
  );


  // ==========================================================
  // SOS COMMAND
  // ==========================================================

  if (
    topicString ==
    SOS_COMMAND_TOPIC
  )
  {
    // --------------------------------------------------------
    // SOS ON
    // --------------------------------------------------------

    if (
      message == "ON" ||
      message == "SOS" ||
      message == "SOS_ON" ||
      message == "TRIGGER" ||
      message == "TRIGGER_SOS" ||
      message == "1" ||
      message == "TRUE"
    )
    {
      activateSOS();

      publishACK(
        "SOS COMMAND RECEIVED"
      );

      return;
    }


    // --------------------------------------------------------
    // SOS OFF
    // --------------------------------------------------------

    if (
      message == "OFF" ||
      message == "SOS_OFF" ||
      message == "CLEAR" ||
      message == "SOS_CLEAR" ||
      message == "0" ||
      message == "FALSE"
    )
    {
      clearSOS();

      return;
    }
  }


  // ==========================================================
  // DEADMAN COMMAND
  // ==========================================================

  if (
    topicString ==
    DEADMAN_COMMAND_TOPIC
  )
  {
    // --------------------------------------------------------
    // START DEADMAN
    // --------------------------------------------------------

    if (
      message == "ON" ||
      message == "START" ||
      message == "START_CHECK" ||
      message == "START_SAFETY_CHECK" ||
      message == "DEADMAN" ||
      message == "DEADMAN_ON" ||
      message == "ALERT" ||
      message == "1" ||
      message == "TRUE"
    )
    {
      startDeadman();

      return;
    }


    // --------------------------------------------------------
    // CLEAR DEADMAN
    // --------------------------------------------------------

    if (
      message == "OFF" ||
      message == "DEADMAN_OFF" ||
      message == "CLEAR" ||
      message == "RESPONDED" ||
      message == "RESPONSE" ||
      message == "WORKER_RESPONDED" ||
      message == "0" ||
      message == "FALSE"
    )
    {
      clearDeadman();

      return;
    }
  }
}


// ============================================================
// CLEAN MQTT MESSAGE
// ============================================================

String cleanMessage(
  String message
)
{
  message.trim();

  message.toUpperCase();

  // Remove quotation marks if app sends "ON"
  message.replace(
    "\"",
    ""
  );

  message.trim();

  return message;
}


// ============================================================
// PUBLISH DATA
// ============================================================

void publishData()
{
  if (
    !mqttClient.connected()
  )
  {
    return;
  }


  String json = "{";


  // ----------------------------------------------------------
  // WORKER ID
  // ----------------------------------------------------------

  json +=
    "\"workerId\":\"";

  json +=
    WORKER_ID;

  json += "\",";


  // ----------------------------------------------------------
  // ZONE
  // ----------------------------------------------------------

  json +=
    "\"zone\":\"";

  json +=
    ZONE_ID;

  json += "\",";


  // ----------------------------------------------------------
  // HEART RATE
  // ----------------------------------------------------------

  json +=
    "\"heartRate\":";

  json +=
    String(
      heartRate,
      1
    );

  json += ",";


  json +=
    "\"heartRateValid\":";

  json +=
    (
      heartRateValid
      ? "true"
      : "false"
    );

  json += ",";


  // ----------------------------------------------------------
  // SpO2
  // ----------------------------------------------------------

  json +=
    "\"spo2\":";

  json +=
    String(
      spo2
    );

  json += ",";


  // ----------------------------------------------------------
  // MAX30102 IR
  // ----------------------------------------------------------

  json +=
    "\"max30102IR\":";

  json +=
    String(
      irValue
    );

  json += ",";


  // ----------------------------------------------------------
  // MPU
  // ----------------------------------------------------------

  json +=
    "\"accelX\":";

  json +=
    String(
      accelX,
      3
    );

  json += ",";


  json +=
    "\"accelY\":";

  json +=
    String(
      accelY,
      3
    );

  json += ",";


  json +=
    "\"accelZ\":";

  json +=
    String(
      accelZ,
      3
    );

  json += ",";


  json +=
    "\"gyroX\":";

  json +=
    String(
      gyroX,
      3
    );

  json += ",";


  json +=
    "\"gyroY\":";

  json +=
    String(
      gyroY,
      3
    );

  json += ",";


  json +=
    "\"gyroZ\":";

  json +=
    String(
      gyroZ,
      3
    );

  json += ",";


  json +=
    "\"totalAcceleration\":";

  json +=
    String(
      totalAcceleration,
      3
    );

  json += ",";


  // ----------------------------------------------------------
  // SOS
  // ----------------------------------------------------------

  json +=
    "\"sos\":";

  json +=
    (
      sosActive
      ? "true"
      : "false"
    );

  json += ",";


  // ----------------------------------------------------------
  // DEADMAN
  // ----------------------------------------------------------

  json +=
    "\"deadman\":";

  json +=
    (
      deadmanActive
      ? "true"
      : "false"
    );

  json += ",";


  // ----------------------------------------------------------
  // RESCUE
  // ----------------------------------------------------------

  json +=
    "\"rescueRequired\":";

  json +=
    (
      rescueEscalated
      ? "true"
      : "false"
    );

  json += ",";


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  json +=
    "\"wifiRSSI\":";

  json +=
    String(
      WiFi.RSSI()
    );

  json += ",";


  // ----------------------------------------------------------
  // UPTIME
  // ----------------------------------------------------------

  json +=
    "\"uptime\":";

  json +=
    String(
      millis()
    );


  json += "}";


  mqttClient.publish(
    DATA_TOPIC,
    json.c_str()
  );


  Serial.println();
  Serial.println(
    "WORKER DATA:"
  );

  Serial.println(
    json
  );
}


// ============================================================
// PUBLISH EVENT
// ============================================================

void publishEvent(
  const char* eventName
)
{
  if (
    !mqttClient.connected()
  )
  {
    Serial.print(
      "MQTT OFFLINE - EVENT: "
    );

    Serial.println(
      eventName
    );

    return;
  }


  String json = "{";


  json +=
    "\"workerId\":\"";

  json +=
    WORKER_ID;

  json += "\",";


  json +=
    "\"zone\":\"";

  json +=
    ZONE_ID;

  json += "\",";


  json +=
    "\"event\":\"";

  json +=
    eventName;

  json += "\",";


  json +=
    "\"timestamp\":";

  json +=
    String(
      millis()
    );


  json += "}";


  mqttClient.publish(
    EVENT_TOPIC,
    json.c_str()
  );


  Serial.print(
    "EVENT: "
  );

  Serial.println(
    eventName
  );
}


// ============================================================
// PUBLISH ACK
// ============================================================

void publishACK(
  const char* message
)
{
  if (
    !mqttClient.connected()
  )
  {
    return;
  }


  String json = "{";


  json +=
    "\"workerId\":\"";

  json +=
    WORKER_ID;

  json += "\",";


  json +=
    "\"message\":\"";

  json +=
    message;

  json += "\",";


  json +=
    "\"timestamp\":";

  json +=
    String(
      millis()
    );


  json += "}";


  mqttClient.publish(
    ACK_TOPIC,
    json.c_str()
  );


  Serial.print(
    "ACK: "
  );

  Serial.println(
    message
  );
}


// ============================================================
// PUBLISH ALIVE
// ============================================================

void publishAlive()
{
  if (
    !mqttClient.connected()
  )
  {
    return;
  }


  String json = "{";


  json +=
    "\"workerId\":\"";

  json +=
    WORKER_ID;

  json += "\",";


  json +=
    "\"status\":\"ONLINE\",";


  json +=
    "\"uptime\":";

  json +=
    String(
      millis()
    );


  json += "}";


  mqttClient.publish(
    ALIVE_TOPIC,
    json.c_str()
  );
}


// ============================================================
// WIFI RESET
// ============================================================

void resetWiFi()
{
  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "             WIFI RESET"
  );

  Serial.println(
    "======================================"
  );


  // ----------------------------------------------------------
  // STOP ALL ALARMS
  // ----------------------------------------------------------

  sosActive = false;

  deadmanActive = false;

  rescueEscalated = false;


  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  buzzerStep = 0;


  // ----------------------------------------------------------
  // SEND FINAL STATUS IF MQTT AVAILABLE
  // ----------------------------------------------------------

  if (
    mqttClient.connected()
  )
  {
    publishEvent(
      "WIFI_RESET"
    );

    publishACK(
      "WIFI RESET"
    );

    mqttClient.loop();

    delay(100);
  }


  // ----------------------------------------------------------
  // DISCONNECT MQTT
  // ----------------------------------------------------------

  mqttClient.disconnect();


  // ----------------------------------------------------------
  // RESET WIFI CREDENTIALS
  // ----------------------------------------------------------

  WiFiManager wm;

  wm.resetSettings();


  Serial.println(
    "WiFi credentials erased."
  );

  Serial.println(
    "Restarting ESP32..."
  );


  delay(1000);


  ESP.restart();
}
