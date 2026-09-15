// ============================================================
// myBlind - ESP8266 Blind Controller
// Encoder, Motor, Contact
//
// myBlind1 Alpha   = 1,1,1
// myBlind2 Bravo   = 1,0,1
// myBlind3 Charlie = 1,0,0
// myBlind4 Echo    = 1,1,0
// myBlind5 Foxtrot = 0,1,0
// myBlind6 Golf    = 1,0,0
// myBlind7 Hotel   = 1,1,0
// myBlind8 Delta   = 1,0,0
// ============================================================

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>  // 1.61
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>

// ============================================================
// CONFIGURATION
// ============================================================

#define CONFIG_FILE "/config.json"

// ============================================================
// PINS
// ============================================================

#define STATUSLIGHT D3
#define UPBUTTONPIN D8
#define DOWNBUTTONPIN D7
#define CONTACTSENSOR A0
#define LEDPIN D4

// ============================================================
// TIMING / MOTOR
// ============================================================

#define MQTTUPDATE 60000
#define ERRORCHECK 300
#define UPDATEPOSTIONDELAY 250
#define LEDBRIGHTNESS 64
#define MINSPEED 500
#define MAXSPEED 1023

// ============================================================
// EEPROM
// ============================================================

#define EEPROM_SIZE 32
#define ADDR_MAGIC 0
#define ADDR_VERSION 1
#define ADDR_DATA 2
#define EEPROM_MAGIC 0xA5
#define EEPROM_VERSION 1

// ============================================================
// LED COLOURS
// ============================================================

enum LedColor {
  LED_YELLOW,  // DOWN
  LED_RED,     // WIFI DOWN / 3 x Obstacle Detected Blocked / 5 x System Failure
  LED_GREEN,   // EEPROM Saved
  LED_BLUE,    // UP
  LED_PINK,    // LittleFS failed
  LED_PURPLE,  // EEPROM Error
  LED_CYAN,    // WiFi Connected
  LED_ORANGE,  // MQTT Failure
  LED_MINT,    // CONTACT SENSOR OPEN
  LED_OFF
};

// ============================================================
// MOTION STATE
// ============================================================

enum MotionState {
  MOTION_IDLE,
  MOTION_UP,
  MOTION_DOWN
};

#define MQTT_POSITION_INTERVAL 2000

unsigned long lastMQTTPosition = 0;

MotionState motionState = MOTION_IDLE;

// ============================================================
// LED STATE
// ============================================================

bool ledActive = false;
bool ledOn = false;

unsigned long ledLastToggle = 0;
const unsigned long ledInterval = 200;  // ms on/off

// ============================================================
// RESET STATE
// ============================================================

bool resetWiFi = false;
bool resetEnc = false;
bool resetAll = false;

const unsigned long BOOT_HOLD_TIME = 2000;  // ms
unsigned long holdStart = 0;

int ledFlashesRemaining = 0;
LedColor currentLedColor = LED_OFF;

// ============================================================
// OTA
// ============================================================

const char* update_path = "/firmware";
const char* update_username = "myblind";
const char* update_password = "local99";

// ============================================================
// MQTT TOPICS
// ============================================================

String blind_target_topic;
String blind_position_topic;
String blind_state_topic;
String blind_gettarget_topic;
String blind_error_topic;
String blind_contactsensor_topic;

// bool ignoreNextTargetPayload = true;

// ============================================================
// WEB SERVER / OTA UPDATE
// ============================================================

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// ============================================================
// MQTT BUFFERS
// ============================================================

char mqttServer[40] = "192.168.0.45";
char mqttPort[6] = "1883";
char mqttUser[32] = "mosquitto";
char mqttPass[32] = "@S0mmer99";
char mqttClientId[32] = "myblind";

// ============================================================
// BOOLEAN FLAGS
// ============================================================

bool contactSensor = false;
bool motorWire = false;
bool encoderWire = false;

// ============================================================
// DEBUG OUTPUT
// ============================================================

bool debugOutput = false;

// ============================================================
// WIFI MANAGER PARAMETERS
// ============================================================

WiFiManagerParameter p_mqtt_server(
  "mqtt_server",
  "MQTT Server",
  mqttServer,
  40);

WiFiManagerParameter p_mqtt_port(
  "mqtt_port",
  "MQTT Port",
  mqttPort,
  6);

WiFiManagerParameter p_mqtt_user(
  "mqtt_user",
  "MQTT User",
  mqttUser,
  32);

WiFiManagerParameter p_mqtt_pass(
  "mqtt_pass",
  "MQTT Password",
  mqttPass,
  32);

WiFiManagerParameter p_mqtt_clientid(
  "mqtt_clientid",
  "MQTT Client ID",
  mqttClientId,
  32);

// Boolean params as strings
char contactSensorStr[2] = "0";
char motorWireStr[2] = "0";
char encoderWireStr[2] = "0";

WiFiManagerParameter p_contact(
  "contactSensor",
  "Contact Sensor",
  contactSensorStr,
  2);

WiFiManagerParameter p_motor(
  "motorWire",
  "Motor Wire",
  motorWireStr,
  2);

WiFiManagerParameter p_encoder(
  "encoderWire",
  "Encoder Wire",
  encoderWireStr,
  2);

// ============================================================
// TIMERS
// ============================================================

#define WIFI_CHECK_INTERVAL 30000UL
#define MQTT_RETRY_INTERVAL 15000UL
#define HARD_RESET_TIMEOUT 300000UL
#define BOOT_GRACE_PERIOD 30000UL
#define LONGPRESSTIME 1000UL

// ============================================================
// OBJECTS
// ============================================================

WiFiClient espClient;
PubSubClient mqtt(espClient);
WiFiManager wm;

Adafruit_NeoPixel pixels =
  Adafruit_NeoPixel(1, STATUSLIGHT, NEO_GRB + NEO_KHZ800);

// ============================================================
// STATE
// ============================================================

unsigned long lastMQTTAttempt = 0;
unsigned long lastHealthyTick = 0;
unsigned long bootTime = 0;
unsigned long lastMsg = 0 - MQTTUPDATE + 10000;

unsigned long buttonTimer;

boolean DownButtonActive = false;
boolean DownLongPressActive = false;
boolean UpButtonActive = false;
boolean UpLongPressActive = false;

bool forceWiFiPortal = false;

// ============================================================
// CONTACT SENSOR
// ============================================================

int contactSensorValue;
boolean contactSensorState;
boolean contactSensorState_1;

unsigned long lastCSCheck = 0;
const unsigned long CS_INTERVAL = 100;  // ms

// ============================================================
// ENCODER
// ============================================================

Encoder* myEnc;

// Encoder pins
int ENC_PIN_A;
int ENC_PIN_B;

// ============================================================
// MOTOR
// ============================================================

// Motor pins
int E1;
int M1;

// ============================================================
// POSITION / STATUS
// ============================================================

int closedPosition;
int openPosition;
int currentPosition;
int targetPosition;

int travelLength;
int targetDeadband = 100;

byte statusEEPROM;

int travelPercent;
int targetPercent;

int status;  // 4=open, 3=opening, 2=stopped, 1=closing, 0=closed

int switchPosition;
int acceleration;
int motorspeed = 0;

static unsigned long stopPressTime = 0;
const unsigned long STOP_DEBOUNCE_MS = 60;  // 30–50ms is ideal

// ============================================================
// SAFETY CUT-OFF VARIABLES
// ============================================================

unsigned long movementStartTimer;
long movementStartPosition;

boolean errorFlag = false;

// ============================================================
// LITTLEFS
// ============================================================

bool loadConfig() {

  if (!LittleFS.exists(CONFIG_FILE))
    return false;

  File f = LittleFS.open(CONFIG_FILE, "r");

  if (!f)
    return false;

  StaticJsonDocument<512> doc;

  if (deserializeJson(doc, f) != DeserializationError::Ok) {
    f.close();
    return false;
  }

  f.close();

  strlcpy(
    mqttServer,
    doc["server"] | mqttServer,
    sizeof(mqttServer));

  strlcpy(
    mqttPort,
    doc["port"] | mqttPort,
    sizeof(mqttPort));

  strlcpy(
    mqttUser,
    doc["user"] | mqttUser,
    sizeof(mqttUser));

  strlcpy(
    mqttPass,
    doc["pass"] | mqttPass,
    sizeof(mqttPass));

  strlcpy(
    mqttClientId,
    doc["clientId"] | mqttClientId,
    sizeof(mqttClientId));

  contactSensor = doc["contactSensor"] | false;
  motorWire = doc["motorWire"] | false;
  encoderWire = doc["encoderWire"] | false;

  Serial.println("📂 Config loaded from LittleFS");

  return true;
}

void saveConfig() {

  StaticJsonDocument<512> doc;

  doc["server"] = mqttServer;
  doc["port"] = mqttPort;
  doc["user"] = mqttUser;
  doc["pass"] = mqttPass;
  doc["clientId"] = mqttClientId;

  doc["contactSensor"] = contactSensor;
  doc["motorWire"] = motorWire;
  doc["encoderWire"] = encoderWire;

  File f = LittleFS.open(CONFIG_FILE, "w");

  if (!f)
    return;

  serializeJson(doc, f);
  f.close();

  Serial.println("💾 Config saved to LittleFS");
}

// ============================================================
// WIFI MANAGER CALLBACK
// ============================================================

void saveParamsCallback() {

  Serial.println("🧾 Params changed, saving...");

  strlcpy(
    mqttServer,
    p_mqtt_server.getValue(),
    sizeof(mqttServer));

  strlcpy(
    mqttPort,
    p_mqtt_port.getValue(),
    sizeof(mqttPort));

  strlcpy(
    mqttUser,
    p_mqtt_user.getValue(),
    sizeof(mqttUser));

  strlcpy(
    mqttPass,
    p_mqtt_pass.getValue(),
    sizeof(mqttPass));

  strlcpy(
    mqttClientId,
    p_mqtt_clientid.getValue(),
    sizeof(mqttClientId));

  // Boolean values
  contactSensor = (p_contact.getValue()[0] == '1');
  motorWire = (p_motor.getValue()[0] == '1');
  encoderWire = (p_encoder.getValue()[0] == '1');

  saveConfig();
}

// ============================================================
// ENCODER + MOTOR SETUP
// ============================================================

void setupEncoderAndMotor() {

  // Delete old encoder if it exists
  if (myEnc) {
    delete myEnc;
    myEnc = nullptr;
  }

  // Assign encoder pins based on encoderWire parameter
  if (encoderWire) {
    ENC_PIN_A = D5;
    ENC_PIN_B = D6;
  } else {
    ENC_PIN_A = D6;
    ENC_PIN_B = D5;
  }

  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);

  myEnc = new Encoder(ENC_PIN_A, ENC_PIN_B);

  if (myEnc)
    Serial.println("✅ Encoder initialized!");
  else
    Serial.println("❌ Encoder failed to initialize!");

  // Motor pins based on motorWire parameter
  if (motorWire) {
    E1 = 4;
    M1 = 5;
  } else {
    E1 = 5;
    M1 = 4;
  }

  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);

  Serial.printf("Motor pins: E1=%d M1=%d\n", E1, M1);
  Serial.printf(
    "Encoder pins: A=%d B=%d\n",
    ENC_PIN_A,
    ENC_PIN_B);
}

// ============================================================
// MQTT TOPIC HELPER
// ============================================================

String getMqttTopic(String type) {
  return String(mqttClientId) + "/" + type;
}

// ============================================================
// MQTT CALLBACK
// ============================================================

void callback(char* topic, byte* payload, unsigned int length) {

  String strTopic = String(topic);
  String received = "";

  for (unsigned int i = 0; i < length; i++) {
    received += (char)payload[i];
  }

  Serial.print("📨 MQTT received on topic: ");
  Serial.print(strTopic);
  Serial.print(" | payload: ");
  Serial.println(received);

  if (strTopic == blind_target_topic) {

    // ---- IGNORE FIRST SLIDER PAYLOAD ----
    // if (ignoreNextTargetPayload) {
    //   ignoreNextTargetPayload = false;
    //   Serial.println("⚠️ Ignoring initial slider value");
    //   return;
    // }

    switchPosition = received.toInt();

    Serial.print("🎯 Target position set to: ");
    Serial.println(switchPosition);

    target();
  }
}

// ============================================================
// DEBUG
// ============================================================

void debug() {

  if (!debugOutput)
    return;

  Serial.print("Status: ");
  Serial.print(status);

  Serial.print(" | ");

  if (status == 4)
    Serial.print("Open   ");
  else if (status == 3)
    Serial.print("Opening");
  else if (status == 2)
    Serial.print("Stopped");
  else if (status == 1)
    Serial.print("Closing");
  else if (status == 0)
    Serial.print("Closed ");

  Serial.print(" | ");

  Serial.print("Pos: ");
  Serial.print(currentPosition);

  Serial.print(" | ");

  Serial.print("Target Pos: ");
  Serial.print(targetPosition);

  Serial.print(" | ");

  Serial.print("Closed Pos: ");
  Serial.print(closedPosition);

  Serial.print(" | ");

  Serial.print("Open Pos: ");
  Serial.print(openPosition);

  Serial.print(" | ");

  Serial.print(travelPercent);
  Serial.print(" %");

  Serial.print(" | ");

  Serial.print("Err: ");
  Serial.print(errorFlag);

  Serial.print(" | ");

  Serial.print("CS: ");
  Serial.print(contactSensor);
  Serial.print(" - ");
  Serial.print(contactSensorState);

  Serial.print(" | ");

  Serial.print("MW: ");
  Serial.print(motorWire);

  Serial.print(" | ");

  Serial.print("EW: ");
  Serial.print(encoderWire);

  Serial.print(" | ");

  Serial.print("MS: ");
  Serial.println(motorspeed);
}

// ============================================================
// LED
// ============================================================

void flash_LED(int noTimes, LedColor color) {

  ledFlashesRemaining = (noTimes * 2) - 1;
  currentLedColor = color;
  ledActive = true;

  // Turn ON immediately
  ledOn = true;

  setLedColor(color);

  ledLastToggle = millis();
}

void setLedColor(LedColor color) {

  switch (color) {

    case LED_YELLOW:
      pixels.setPixelColor(0, 255, 165, 0);
      break;

    case LED_RED:
      pixels.setPixelColor(0, 255, 0, 0);
      break;

    case LED_GREEN:
      pixels.setPixelColor(0, 0, 255, 0);
      break;

    case LED_BLUE:
      pixels.setPixelColor(0, 0, 100, 255);
      break;

    case LED_PINK:
      pixels.setPixelColor(0, 255, 0, 165);
      break;

    case LED_PURPLE:
      pixels.setPixelColor(0, 165, 0, 255);
      break;

    case LED_CYAN:
      pixels.setPixelColor(0, 0, 255, 255);
      break;

    case LED_ORANGE:
      pixels.setPixelColor(0, 255, 40, 0);
      break;

    case LED_MINT:
      pixels.setPixelColor(0, 0, 255, 180);
      break;

    default:
      pixels.setPixelColor(0, 0, 0, 0);
      break;
  }

  pixels.show();
}

void updateLED() {

  if (!ledActive)
    return;

  unsigned long now = millis();

  if (now - ledLastToggle < ledInterval)
    return;

  ledLastToggle = now;

  if (ledOn) {
    setLedColor(LED_OFF);
    ledOn = false;
  } else {
    setLedColor(currentLedColor);
    ledOn = true;
  }

  ledFlashesRemaining--;

  if (ledFlashesRemaining <= 0) {
    ledActive = false;
    setLedColor(LED_OFF);
  }
}

// ============================================================
// EEPROM SAVE
// ============================================================

void savePosition() {

  int addr = ADDR_DATA;

  EEPROM.put(addr, closedPosition);
  addr += sizeof(closedPosition);

  EEPROM.put(addr, openPosition);
  addr += sizeof(openPosition);

  EEPROM.put(addr, currentPosition);
  addr += sizeof(currentPosition);

  EEPROM.put(addr, statusEEPROM);

  if (EEPROM.commit()) {
    Serial.println("EEPROM saved");
    flash_LED(2, LED_GREEN);
  } else {
    Serial.println("EEPROM save FAILED");
    flash_LED(5, LED_PURPLE);
  }
}

// ============================================================
// MOTION SERVICE
// ============================================================

void serviceMotion() {

  if (mqtt.connected())
    mqtt.loop();

  yield();

  if (myEnc) {
    currentPosition = myEnc->read();
  }

  travelLength = abs(openPosition - closedPosition);

  if (travelLength == 0)
    travelLength = 1;

  targetDeadband = max(100, travelLength / 200);

  travelPercent =
    100 - (abs(openPosition - currentPosition) * 100 / travelLength);

  travelPercent = constrain(
    travelPercent,
    0,
    100);

  // Publish position continuously while moving
  if (
    motionState != MOTION_IDLE && millis() - lastMQTTPosition >= MQTT_POSITION_INTERVAL) {

    lastMQTTPosition = millis();

    sendMQTTPosition();
  }

  int distanceToTarget =
    abs(targetPosition - currentPosition);

  int softStopDistance =
    travelLength / 5;

  int finalCreepDistance =
    travelLength / 80;

  if (distanceToTarget < softStopDistance) {

    // -------------------------------------------------
    // FINAL CREEP
    // -------------------------------------------------

    if (distanceToTarget < finalCreepDistance) {

      float ratio =
        float(distanceToTarget) / finalCreepDistance;

      ratio = pow(ratio, 3.0);

      motorspeed =
        300 + (MINSPEED - 300) * ratio;

    }

    // -------------------------------------------------
    // NORMAL DECELERATION
    // -------------------------------------------------

    else {

      float ratio =
        float(distanceToTarget) / softStopDistance;

      ratio = pow(ratio, 2.0);

      motorspeed =
        MINSPEED + (MAXSPEED - MINSPEED) * ratio;
    }

  } else {

    // -------------------------------------------------
    // ACCELERATION
    // -------------------------------------------------

    acceleration += 10;

    if (acceleration > MAXSPEED)
      acceleration = MAXSPEED;

    motorspeed = acceleration;
  }

  debug();
}

// ============================================================
// RESET FUNCTIONS
// ============================================================

void resetWM() {
  wm.resetSettings();
}

void resetEEPROMPositions() {

  Serial.println("🔁 Resetting EEPROM positions");

  closedPosition = 0;
  openPosition = 100;
  currentPosition = 0;
  statusEEPROM = 0;

  EEPROM.put(ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.put(ADDR_VERSION, EEPROM_VERSION);

  int addr = ADDR_DATA;

  EEPROM.put(addr, closedPosition);
  addr += sizeof(closedPosition);

  EEPROM.put(addr, openPosition);
  addr += sizeof(openPosition);

  EEPROM.put(addr, currentPosition);
  addr += sizeof(currentPosition);

  EEPROM.put(addr, statusEEPROM);

  if (EEPROM.commit()) {
    Serial.println("✅ EEPROM reset OK");
    flash_LED(2, LED_GREEN);
  } else {
    Serial.println("❌ EEPROM commit failed");
    flash_LED(5, LED_PURPLE);
  }
}

// ============================================================
// MQTT
// ============================================================

void sendMQTTMessageLite() {

  char cs[2];

  itoa(
    contactSensorState,
    cs,
    2);

  mqtt.publish(
    blind_contactsensor_topic.c_str(),
    cs);

  Serial.print(blind_contactsensor_topic);
  Serial.print(" ");
  Serial.println(cs);

  lastMsg = millis();
}

void sendMQTTPosition() {

  if (!mqtt.connected())
    return;

  char blindposition[10];

  itoa(
    travelPercent,
    blindposition,
    10);

  mqtt.publish(
    blind_position_topic.c_str(),
    blindposition);
}

void sendMQTTMessage() {

  char blindstate[16];
  char blindposition[10];
  char getTarget[10];
  char error[2];
  char cs[2];

  itoa(
    errorFlag,
    error,
    2);

  itoa(
    contactSensorState,
    cs,
    2);

  itoa(
    switchPosition,
    getTarget,
    10);

  if (status == 4)
    snprintf(blindstate, 8, "STOPPED"),
      itoa(100, blindposition, 10);

  else if (status == 3)
    snprintf(blindstate, 11, "INCREASING"),
      itoa(travelPercent, blindposition, 10);

  else if (status == 2)
    snprintf(blindstate, 8, "STOPPED"),
      itoa(travelPercent, blindposition, 10);

  else if (status == 1)
    snprintf(blindstate, 11, "DECREASING"),
      itoa(travelPercent, blindposition, 10);

  else if (status == 0)
    snprintf(blindstate, 8, "STOPPED"),
      itoa(0, blindposition, 10);

  mqtt.publish(
    blind_error_topic.c_str(),
    error);

  mqtt.publish(
    blind_gettarget_topic.c_str(),
    getTarget);

  mqtt.publish(
    blind_state_topic.c_str(),
    blindstate);

  mqtt.publish(
    blind_position_topic.c_str(),
    blindposition);

  mqtt.publish(
    blind_contactsensor_topic.c_str(),
    cs);

  Serial.print(blind_error_topic);
  Serial.print(" ");
  Serial.print(error);

  Serial.print(" | ");

  Serial.print(blind_position_topic);
  Serial.print(" ");
  Serial.print(blindposition);

  Serial.print(" | ");

  Serial.print(blind_state_topic);
  Serial.print(" ");
  Serial.print(blindstate);

  Serial.print(" | Set: ");

  Serial.print(blind_target_topic);
  Serial.print(" ");
  Serial.print(switchPosition);

  Serial.print(" | ");

  Serial.print(blind_contactsensor_topic);
  Serial.print(" ");
  Serial.println(cs);

  lastMsg = millis();
}

// ============================================================
// START MOTION
// ============================================================

void targetDown() {

  status = 1;  // closing

  lastMQTTPosition = 0;

  sendMQTTMessage();

  movementStartTimer = millis();
  movementStartPosition = currentPosition;

  errorFlag = false;

  stopPressTime = 0;

  motionState = MOTION_DOWN;
}

void targetUp() {

  status = 3;  // opening

  lastMQTTPosition = 0;

  sendMQTTMessage();

  movementStartTimer = millis();
  movementStartPosition = currentPosition;

  errorFlag = false;

  stopPressTime = 0;

  motionState = MOTION_UP;
}

// ============================================================
// TARGET MOTION
// ============================================================

void handleTargetMotion() {

  if (motionState == MOTION_IDLE)
    return;

  // -------------------------------------------------
  // STOP BUTTON DEBOUNCE
  // -------------------------------------------------

  bool stopPressed =
    (digitalRead(DOWNBUTTONPIN) == HIGH || digitalRead(UPBUTTONPIN) == HIGH);

  if (stopPressed) {

    if (stopPressTime == 0) {

      stopPressTime = millis();

    } else if (
      millis() - stopPressTime >= STOP_DEBOUNCE_MS) {

      Serial.println("⏹ Motion stopped by button");

      finishMotion();

      return;
    }

  } else {

    stopPressTime = 0;
  }

  serviceMotion();

  // -------------------------------------------------
  // DIRECTION HANDLING
  // -------------------------------------------------

  if (motionState == MOTION_UP) {

    if (currentPosition >= targetPosition) {
      finishMotion();
      return;
    }

    motorUp();
  }

  if (motionState == MOTION_DOWN) {

    if (currentPosition <= targetPosition) {
      finishMotion();
      return;
    }

    motorDown();
  }

  // -------------------------------------------------
  // ERROR DETECTION
  // -------------------------------------------------

  if (millis() - movementStartTimer > ERRORCHECK) {

    if (currentPosition == movementStartPosition) {

      Serial.println("🚨 Encoder stall detected");

      errorFlag = true;

      finishMotion();

      return;
    }

    movementStartPosition = currentPosition;
    movementStartTimer = millis();
  }
}

// ============================================================
// FINISH MOTION
// ============================================================

void finishMotion() {

  motorStop();

  motionState = MOTION_IDLE;

  travelPercent = map(
    currentPosition,
    closedPosition,
    openPosition,
    0,
    100);

  travelPercent = constrain(
    travelPercent,
    0,
    100);

  if (!errorFlag) {

    if (travelPercent >= 100)
      status = 4;

    else if (travelPercent <= 0)
      status = 0;

    else
      status = 2;

    switchPosition = travelPercent;

    savePosition();

  } else {

    flash_LED(3, LED_RED);

    targetPosition = currentPosition;
    switchPosition = travelPercent;
  }

  sendMQTTMessage();
}

// ============================================================
// TARGET
// ============================================================

void target() {

  travelLength =
    abs(openPosition - closedPosition);

  if (travelLength == 0)
    travelLength = 1;

  targetDeadband =
    max(100, travelLength / 200);

  targetPercent =
    100 - switchPosition;

  targetPosition =
    openPosition - (targetPercent * travelLength / 100);

  if (
    abs(currentPosition - targetPosition) > targetDeadband) {

    if (
      contactSensor && contactSensorState == 1) {

      targetPosition = currentPosition;
      switchPosition = travelPercent;

      flash_LED(3, LED_MINT);

      sendMQTTMessage();

      return;
    }

    if (currentPosition > targetPosition)
      targetDown();
    else
      targetUp();
  }
}

// ============================================================
// DOWN BUTTON
// ============================================================

void downButtonCheck() {

  if (motionState != MOTION_IDLE)
    return;

  if (
    digitalRead(DOWNBUTTONPIN) == HIGH && digitalRead(UPBUTTONPIN) == LOW) {

    if (DownButtonActive == false) {

      DownButtonActive = true;
      buttonTimer = millis();
    }

    if (
      millis() - buttonTimer > LONGPRESSTIME && DownLongPressActive == false) {

      DownLongPressActive = true;

      status = 1;
      motionState = MOTION_DOWN;
      acceleration = MINSPEED;

      while (digitalRead(DOWNBUTTONPIN) == HIGH) {

        serviceMotion();
        motorDown();
      }
    }

  } else {

    if (DownButtonActive == true) {

      if (DownLongPressActive == true) {

        status = 2;

        debug();

        DownLongPressActive = false;

        Serial.println("Released Button!");

        motorStop();

        delay(UPDATEPOSTIONDELAY);

        if (myEnc) {
          currentPosition = myEnc->read();
        }

        savePosition();

        if (digitalRead(DOWNBUTTONPIN) == HIGH) {

          closedPosition = currentPosition;

          status = 0;

          savePosition();

          sendMQTTMessage();

        } else {

          pixels.setPixelColor(
            0,
            0,
            0,
            0);

          pixels.show();

          switchPosition = travelPercent;

          sendMQTTMessage();
        }

      } else {

        switchPosition = 0;

        target();
      }

      DownButtonActive = false;
    }
  }
}

// ============================================================
// UP BUTTON
// ============================================================

void upButtonCheck() {

  if (motionState != MOTION_IDLE)
    return;

  if (
    digitalRead(UPBUTTONPIN) == HIGH && digitalRead(DOWNBUTTONPIN) == LOW) {

    if (UpButtonActive == false) {

      UpButtonActive = true;
      buttonTimer = millis();
    }

    if (
      millis() - buttonTimer > LONGPRESSTIME && UpLongPressActive == false) {

      UpLongPressActive = true;

      status = 3;
      motionState = MOTION_UP;
      acceleration = MINSPEED;

      while (digitalRead(UPBUTTONPIN) == HIGH) {

        serviceMotion();
        motorUp();
      }
    }

  } else {

    if (UpButtonActive == true) {

      if (UpLongPressActive == true) {

        status = 2;

        debug();

        UpLongPressActive = false;

        Serial.println("Released Button!");

        motorStop();

        delay(UPDATEPOSTIONDELAY);

        if (myEnc) {
          currentPosition = myEnc->read();
        }

        savePosition();

        if (digitalRead(UPBUTTONPIN) == HIGH) {

          openPosition = currentPosition;

          status = 4;

          savePosition();

          sendMQTTMessage();

        } else {

          pixels.setPixelColor(
            0,
            0,
            0,
            0);

          pixels.show();

          switchPosition = travelPercent;

          sendMQTTMessage();
        }

      } else {

        switchPosition = 100;

        target();
      }

      UpButtonActive = false;
    }
  }
}

// ============================================================
// MOTOR
// ============================================================

void motorUp() {

  setLedColor(LED_BLUE);

  analogWrite(
    E1,
    motorspeed);

  digitalWrite(
    M1,
    LOW);
}

void motorDown() {

  setLedColor(LED_YELLOW);

  analogWrite(
    M1,
    motorspeed);

  digitalWrite(
    E1,
    LOW);
}

void motorStop() {

  motorspeed = 0;

  digitalWrite(
    E1,
    LOW);

  digitalWrite(
    M1,
    LOW);

  acceleration = MINSPEED;
}

// ============================================================
// CONTACT SENSOR
// ============================================================

void checkContactSensor() {

  unsigned long now = millis();

  if (now - lastCSCheck > CS_INTERVAL) {

    lastCSCheck = now;

    contactSensorValue = analogRead(A0);

    contactSensorState =
      (contactSensorValue < 512);

    digitalWrite(
      LEDPIN,
      contactSensorState ? HIGH : LOW);

    if (
      contactSensorState != contactSensorState_1) {

      contactSensorState_1 =
        contactSensorState;

      sendMQTTMessageLite();
    }
  }
}

// ============================================================
// WIFI
// ============================================================

void handleWiFi() {

  static unsigned long lastReconnectAttempt = 0;
  static unsigned long lostSince = 0;
  static bool wifiRecovering = false;
  static int failCount = 0;

  // -------------------------------------------------
  // WIFI CONNECTED
  // -------------------------------------------------

  if (WiFi.status() == WL_CONNECTED) {

    if (wifiRecovering) {

      Serial.println("✅ WiFi restored");

      wifiRecovering = false;
      failCount = 0;

      mqtt.disconnect();

      lastMQTTAttempt = 0;

      flash_LED(1, LED_CYAN);

      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
    }

    lastHealthyTick = millis();

    lostSince = 0;

    return;
  }

  // -------------------------------------------------
  // WIFI LOST DETECTED
  // -------------------------------------------------

  if (!wifiRecovering) {

    Serial.println(
      "❌ WiFi lost — recovery started");

    wifiRecovering = true;
    lostSince = millis();
  }

  flash_LED(1, LED_RED);

  // -------------------------------------------------
  // FAST RETRY LOOP
  // Every 10s, not 15s
  // -------------------------------------------------

  if (
    millis() - lastReconnectAttempt < 10000)
    return;

  lastReconnectAttempt = millis();

  Serial.println(
    "🔄 Attempting WiFi reconnect...");

  // FAST PATH: try reconnect first
  // No stack reset

  WiFi.reconnect();

  delay(50);  // tiny yield only

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println(
      "✅ Reconnected instantly");

    return;
  }

  failCount++;

  // -------------------------------------------------
  // ONLY RESET STACK AFTER MULTIPLE FAILS
  // 15min
  // -------------------------------------------------

  if (failCount >= 90) {

    Serial.println(
      "⚠️ Multiple failures → resetting WiFi stack");

    WiFi.disconnect(false);

    delay(100);

    WiFi.mode(WIFI_OFF);

    delay(300);

    WiFi.mode(WIFI_STA);

    WiFi.setSleepMode(WIFI_NONE_SLEEP);

    WiFi.setAutoReconnect(true);

    failCount = 0;
  }

  Serial.println("📡 Retry queued");
}

// ============================================================
// MQTT CONNECTION
// ============================================================

void handleMQTT() {

  if (WiFi.status() != WL_CONNECTED) {

    if (mqtt.connected())
      mqtt.disconnect();

    return;
  }

  if (mqtt.connected()) {

    mqtt.loop();

    return;
  }

  if (
    millis() - lastMQTTAttempt < MQTT_RETRY_INTERVAL)
    return;

  lastMQTTAttempt = millis();

  Serial.println(
    "🔄 MQTT reconnecting...");

  bool ok =
    (strlen(mqttUser) > 0)
      ? mqtt.connect(
        mqttClientId,
        mqttUser,
        mqttPass)
      : mqtt.connect(
        mqttClientId);

  if (ok) {

    Serial.println("✅ MQTT connected");

    mqtt.subscribe(
      blind_target_topic.c_str());

    // ignoreNextTargetPayload = true;

    sendMQTTMessage();

    lastHealthyTick = millis();

  } else {

    Serial.print("❌ MQTT failed rc=");
    Serial.println(mqtt.state());

    flash_LED(1, LED_ORANGE);
  }
}

// ============================================================
// HEALTH MONITOR
// ============================================================

void healthMonitor() {

  if (
    millis() - bootTime < BOOT_GRACE_PERIOD)
    return;

  // If WiFi is down, do NOT trigger system reset
  // Let handleWiFi recover it

  if (WiFi.status() != WL_CONNECTED) {

    lastHealthyTick = millis();

    return;
  }

  if (
    millis() - lastHealthyTick > HARD_RESET_TIMEOUT) {

    Serial.println(
      "🚨 SYSTEM STALLED — REBOOTING");

    flash_LED(5, LED_RED);

    delay(100);

    ESP.restart();
  }
}

// ============================================================
// PRINT CONFIG
// ============================================================

void printConfig() {

  Serial.println(
    "\n===== myBlind V4.0 =====");

  Serial.println(
    "UP BUTTON         = WM RESET");

  Serial.println(
    "DOWN BUTTON       = EEPROM Reset");

  Serial.println(
    "UP + DOWN BUTTON  = Factory Reset");

  Serial.println("\n--- CONFIG ---");

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("MQTT Server: ");
  Serial.println(mqttServer);

  Serial.print("MQTT Port: ");
  Serial.println(mqttPort);

  Serial.print("MQTT User: ");
  Serial.println(
    strlen(mqttUser)
      ? mqttUser
      : "(none)");

  Serial.print("MQTT Client ID: ");
  Serial.println(mqttClientId);

  Serial.print("Contact Sensor: ");
  Serial.println(contactSensor);

  Serial.print("Motor Wire: ");
  Serial.println(motorWire);

  Serial.print("Encoder Wire: ");
  Serial.println(encoderWire);

  Serial.println("----------------");
}

// ============================================================
// RESET SETUP
// ============================================================

void resetSetup() {

  // Wait for long-press window
  while (
    millis() - holdStart < BOOT_HOLD_TIME) {

    bool upPressed =
      (digitalRead(UPBUTTONPIN) == HIGH);

    bool downPressed =
      (digitalRead(DOWNBUTTONPIN) == HIGH);

    // Visual feedback while waiting

    int brightness =
      map(
        millis() - holdStart,
        0,
        BOOT_HOLD_TIME,
        10,
        255);

    pixels.setPixelColor(
      0,
      brightness,
      brightness,
      0);

    pixels.show();

    if (!upPressed && !downPressed) {

      // Released early → abort reset

      pixels.setPixelColor(
        0,
        0,
        0,
        0);

      pixels.show();

      break;
    }

    yield();
  }

  // Evaluate result

  bool upPressed =
    (digitalRead(UPBUTTONPIN) == HIGH);

  bool downPressed =
    (digitalRead(DOWNBUTTONPIN) == HIGH);

  if (upPressed && downPressed) {

    resetAll = true;

  } else if (upPressed) {

    resetWiFi = true;

  } else if (downPressed) {

    resetEnc = true;
  }

  // Execute reset

  if (
    resetWiFi || resetEnc || resetAll) {

    Serial.println(
      "⚠️ Boot long-press reset");

    if (resetAll) {

      Serial.println(
        "Factory reset");

      resetWM();
      resetEEPROMPositions();

    } else if (resetWiFi) {

      Serial.println(
        "WiFi reset");

      resetWM();

    } else if (resetEnc) {

      Serial.println(
        "Encoder reset");

      resetEEPROMPositions();
    }

    delay(500);

    ESP.restart();
  }
}

// ============================================================
// WEB CONTROL
// ============================================================

#define WEB_VERSION "4.1.0"

const char WEB_PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html lang="en">

<head>

<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>myBlind</title>

<style>

body{
  font-family:-apple-system,BlinkMacSystemFont,
  "Segoe UI",Arial,sans-serif;
  background:#f2f4f7;
  color:#222;
  margin:0;
  padding:20px;
}

.container{
  max-width:800px;
  margin:auto;
}

h1{
  margin:0;
  font-size:32px;
}

.subtitle{
  color:#666;
  margin-top:4px;
  margin-bottom:20px;
}

.card{
  background:white;
  border-radius:12px;
  padding:18px;
  margin-bottom:18px;
  box-shadow:0 2px 8px rgba(0,0,0,.08);
}

h2{
  font-size:19px;
  margin:0 0 12px 0;
}

table{
  width:100%;
  border-collapse:collapse;
}

th,td{
  padding:10px 6px;
  text-align:left;
  border-bottom:1px solid #eee;
}

th{
  font-weight:600;
  color:#555;
}

tr:last-child td{
  border-bottom:none;
}

.value{
  text-align:right;
  font-weight:600;
}

.good{
  color:#16803c;
  font-weight:600;
}

.bad{
  color:#c62828;
  font-weight:600;
}

.button{
  display:inline-block;
  padding:12px 18px;
  background:#1976d2;
  color:white;
  text-decoration:none;
  border-radius:8px;
  font-weight:600;
  border:none;
  cursor:pointer;
  font-size:15px;
}

.stop-button{
  width:100%;
  grid-column:1 / -1;
  padding:13px 8px;
  background:#c62828;
  color:white;
  border:0;
  border-radius:8px;
  font-size:15px;
  font-weight:600;
  cursor:pointer;
  box-sizing:border-box;
}

.stop-button:active{
  background:#a51f1f;
  transform:scale(.97);
}

.button:active{
  transform:scale(.97);
}

.buttonRow{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:10px;
  margin-top:15px;
}

.buttonUpper{
  background:#1976d2;
}

.buttonLower{
  background:#f2c94c;
}

.buttonStop{
  background:#c62828;
}

.position{
  text-align:center;
  padding:10px 0 15px;
}

.positionValue{
  font-size:56px;
  font-weight:700;
  line-height:1;
}

.positionLabel{
  color:#777;
  font-size:13px;
  margin-top:7px;
}

.slider{
  width:100%;
  accent-color:#1976d2;
}

.sliderValue{
  text-align:center;
  margin-top:8px;
  color:#666;
}

.quick{
  display:grid;
  grid-template-columns:repeat(5,1fr);
  gap:7px;
  margin-top:14px;
}

.quick button{
  padding:11px 4px;
  background:#1976d2;
  color:white;
  border:0;
  border-radius:8px;
  font-weight:600;
  cursor:pointer;
}

.limitButton{
  width:100%;
  padding:13px 8px;
  border:0;
  border-radius:8px;
  color:white;
  font-size:15px;
  font-weight:600;
  cursor:pointer;
}

.hardwareSelect{
  padding:8px 12px;
  border:1px solid #ccc;
  border-radius:6px;
  background:white;
  font-size:15px;
  font-weight:600;
}

.limitButton:active{
  transform:scale(.97);
}

.upper{
  background:#1976d2;
}

.lower{
  background:#f2c94c;
  color:#000;
}

.limitValue{
  font-size:16px;
  font-weight:600;
  text-align:right;
}

.footer{
  text-align:center;
  color:#777;
  font-size:13px;
  margin-top:25px;
}

@media(max-width:520px){

  body{
    padding:12px;
  }

  .positionValue{
    font-size:50px;
  }

  .buttonRow{
    grid-template-columns:1fr;
  }

  .quick{
    grid-template-columns:repeat(5,1fr);
  }

}

</style>

</head>

<body>

<div class="container">

<h1 id="hostname">myBlind</h1>

<div class="subtitle">
Blind control unit
</div>


<!-- ================================================= -->
<!-- POSITION -->
<!-- ================================================= -->

<div class="card">

<h2>Blind Position</h2>

<div class="position">

<div id="position" class="positionValue">
--%
</div>

<div class="positionLabel">
CURRENT POSITION
</div>

</div>


<div class="buttonRow">

<button
class="limitButton upper"
onclick="move(100)">
▲ OPEN
</button>

<button
class="limitButton lower"
onclick="move(0)">
▼ CLOSE
</button>

</div>


<input
id="slider"
class="slider"
type="range"
min="0"
max="100"
value="0"
oninput="sliderChanged(this.value)"
onchange="setPosition(this.value)"
>


<div class="sliderValue">
Target: <strong id="target">--%</strong>
</div>


<div class="quick">

<button onclick="move(0)">0%</button>
<button onclick="move(25)">25%</button>
<button onclick="move(50)">50%</button>
<button onclick="move(75)">75%</button>
<button onclick="move(100)">100%</button>

</div>


<div class="buttonRow">

<button
class="stop-button"
onclick="stopBlind()">
■ STOP
</button>

</div>

</div>


<!-- ================================================= -->
<!-- LIMITS -->
<!-- ================================================= -->

<div class="card">

<h2>Blind Limits</h2>

<table>

<tr>

<td>
Upper Limit
</td>

<td
class="limitValue"
id="openPosition">
--
</td>

</tr>

<tr>

<td>
Lower Limit
</td>

<td
class="limitValue"
id="closedPosition">
--
</td>

</tr>

</table>


<div class="buttonRow">

<button
class="limitButton upper"
onclick="setUpperLimit()">
SET UPPER LIMIT
</button>

<button
class="limitButton lower"
onclick="setLowerLimit()">
SET LOWER LIMIT
</button>

</div>

</div>


<!-- ================================================= -->
<!-- STATUS -->
<!-- ================================================= -->

<div class="card">

<h2>Blind Status</h2>

<table>

<tr>
<td>Encoder Position</td>
<td class="value" id="encoder">--</td>
</tr>

<tr>
<td>Target Position</td>
<td class="value" id="targetPosition">--</td>
</tr>

<tr>
<td>Travel Length</td>
<td class="value" id="travel">--</td>
</tr>

<tr>
<td>Motion</td>
<td class="value" id="motion">--</td>
</tr>

<tr>
<td>Contact Sensor</td>
<td class="value" id="contact">--</td>
</tr>

<tr>
<td>Error</td>
<td class="value" id="error">--</td>
</tr>

</table>

</div>


<!-- ================================================= -->
<!-- HARDWARE -->
<!-- ================================================= -->

<div class="card">

<h2>Hardware Configuration</h2>

<table>

<tr>
<td>Encoder Wire</td>
<td class="value">

<select id="encoderSetting" class="hardwareSelect">
<option value="0">0</option>
<option value="1">1</option>
</select>

</td>
</tr>

<tr>
<td>Motor Wire</td>
<td class="value">

<select id="motorSetting" class="hardwareSelect">
<option value="0">0</option>
<option value="1">1</option>
</select>

</td>
</tr>

<tr>
<td>Contact Wire</td>
<td class="value">

<select id="contactSetting" class="hardwareSelect">
<option value="0">0</option>
<option value="1">1</option>
</select>

</td>
</tr>

</table>

<div style="margin-top:15px;">

<button
class="button"
style="width:100%;box-sizing:border-box;"
onclick="saveHardwareSettings()">
SAVE HARDWARE SETTINGS
</button>

</div>

</div>


<!-- ================================================= -->
<!-- SYSTEM -->
<!-- ================================================= -->

<div class="card">

<h2>System Status</h2>

<table>

<tr>
<td>Wi-Fi</td>
<td class="value" id="wifi">--</td>
</tr>

<tr>
<td>IP Address</td>
<td class="value" id="ip">--</td>
</tr>

<tr>
<td>MQTT</td>
<td class="value" id="mqtt">--</td>
</tr>

<tr>
<td>Uptime</td>
<td class="value" id="uptime">--</td>
</tr>

<tr>
<td>Version</td>
<td class="value" id="version">--</td>
</tr>

<tr>
<td>Debug Output</td>
<td class="value">

<select
id="debugSetting"
class="hardwareSelect"
onchange="setDebugOutput(this.value)">

<option value="0">OFF</option>
<option value="1">ON</option>

</select>

</td>
</tr>

</table>


<br>

<a
class="button"
href="/firmware">
Firmware Update
</a>

</div>


<div class="footer">

myBlind • ESP8266

</div>

</div>


<script>

// =================================================
// API
// =================================================

function api(url){

  fetch(
    url,
    {cache:"no-store"}
  )

  .then(response => {

    if(!response.ok)
      throw new Error("HTTP "+response.status);

    return response.text();

  })

  .then(result => {

    updateStatus();

  })

  .catch(error => {

    console.log(
      "API error:",
      error
    );

  });

}

// =================================================
// MOVE
// =================================================

function move(position){

  document.getElementById(
    "slider"
  ).value=position;

  document.getElementById(
    "target"
  ).innerText=position+"%";

  api(
    "/api/position?value="+position
  );

}

// =================================================
// SET POSITION
// =================================================

function setPosition(position){

  move(position);

}

function sliderChanged(position){

  document.getElementById(
    "target"
  ).innerText=position+"%";

}

// =================================================
// STOP
// =================================================

function stopBlind(){

  api(
    "/api/stop"
  );

}

// =================================================
// UPPER LIMIT
// =================================================

function setUpperLimit(){

  if(
    !confirm(
      "Set the current encoder position as the UPPER limit?"
    )
  )
    return;

  api(
    "/api/setUpperLimit"
  );

}

// =================================================
// LOWER LIMIT
// =================================================

function setLowerLimit(){

  if(
    !confirm(
      "Set the current encoder position as the LOWER limit?"
    )
  )
    return;

  api(
    "/api/setLowerLimit"
  );

}

// =================================================
// HARDWARE CONFIGURATION
// =================================================

function saveHardwareSettings(){

  const encoder =
    document.getElementById(
      "encoderSetting"
    ).value;

  const motor =
    document.getElementById(
      "motorSetting"
    ).value;

  const contact =
    document.getElementById(
      "contactSetting"
    ).value;

  if(!confirm(
    "Save these hardware settings and restart myBlind?"
  )){
    return;
  }

  fetch(
    "/api/setHardware" +
    "?encoder=" + encoder +
    "&motor=" + motor +
    "&contact=" + contact,
    {
      cache:"no-store"
    }
  )

  .then(
    response => response.text()
  )

  .then(
    message => {

      alert(message);

    }
  )

  .catch(
    error => {

      alert(
        "Failed to save hardware settings"
      );

    }
  );

}

// =================================================
// DEBUG OUTPUT
// =================================================

function setDebugOutput(value){

  fetch(
    "/api/setDebug?value=" + value,
    {cache:"no-store"}
  )

  .then(
    response => response.text()
  )

  .then(
    message => {
      console.log(message);
      updateStatus();
    }
  )

  .catch(
    error => {
      console.log(
        "Failed to change debug output:",
        error
      );
    }
  );
}

// =================================================
// TEXT
// =================================================

function setText(id,value){

  const el =
    document.getElementById(id);

  if(el)
    el.innerText=value;

}

// =================================================
// STATUS
// =================================================

function updateStatus(){

  fetch(
    "/api/status",
    {cache:"no-store"}
  )

  .then(
    response => response.json()
  )

  .then(
    data => {

      setText(
        "hostname",
        data.hostname
      );

      setText(
        "position",
        data.position+"%"
      );

      setText(
        "target",
        data.target+"%"
      );

      setText(
        "targetPosition",
        data.targetPosition
      );

      setText(
        "encoder",
        data.encoder
      );

      setText(
        "travel",
        data.travel
      );

      setText(
        "motion",
        data.motion
      );

      setText(
        "contact",
        data.contactSensor
          ? (data.contact ? "OPEN" : "CLOSED")
          : "DISABLED"
      );

      setText(
        "error",
        data.error
          ? "YES"
          : "NO"
      );

      setText(
        "wifi",
        data.wifi
          ? "CONNECTED"
          : "OFFLINE"
      );

      setText(
        "ip",
        data.ip
      );

      setText(
        "mqtt",
        data.mqtt
          ? "CONNECTED"
          : "OFFLINE"
      );

      setText(
        "version",
        data.version
      );

      setText(
        "uptime",
        data.uptime
      );

      setText(
        "openPosition",
        data.openPosition
      );

      setText(
        "closedPosition",
        data.closedPosition
      );

      const debugSetting =
        document.getElementById(
          "debugSetting"
        );

      if (
        document.activeElement !== debugSetting
      ) {

        debugSetting.value =
          data.debugOutput ? "1" : "0";
      }

      const encoderSetting =
        document.getElementById(
          "encoderSetting"
        );

      const motorSetting =
        document.getElementById(
          "motorSetting"
        );

      const contactSetting =
        document.getElementById(
          "contactSetting"
        );

      if(
        document.activeElement !==
        encoderSetting
      ){

        encoderSetting.value =
          data.encoderWire ? "1" : "0";
      }

      if(
        document.activeElement !==
        motorSetting
      ){

        motorSetting.value =
          data.motorWire ? "1" : "0";
      }

      if(
        document.activeElement !==
        contactSetting
      ){

        contactSetting.value =
          data.contactSensor ? "1" : "0";
      }

      const slider =
        document.getElementById(
          "slider"
        );

      if(
        document.activeElement !== slider
      ){

        slider.value =
          data.position;
      }

    }
  )

  .catch(() => {

    setText(
      "wifi",
      "OFFLINE"
    );

  });

}

// =================================================
// INITIAL STATUS
// =================================================

updateStatus();

// =================================================
// REFRESH
// =================================================

setInterval(
  updateStatus,
  1000
);

</script>

</body>

</html>

)rawliteral";

// ============================================================
// WEB ROOT
// ============================================================

void handleWebRoot() {

  httpServer.send_P(
    200,
    "text/html; charset=utf-8",
    WEB_PAGE);
}

// ============================================================
// WEB STATUS
// ============================================================

String motionStateName() {

  if (motionState == MOTION_UP)
    return "OPENING";

  if (motionState == MOTION_DOWN)
    return "CLOSING";

  return "STOPPED";
}

String positionStateName() {

  if (status == 4)
    return "OPEN";

  if (status == 3)
    return "OPENING";

  if (status == 2)
    return "STOPPED";

  if (status == 1)
    return "CLOSING";

  return "CLOSED";
}

void handleWebStatus() {

  String json = "{";

  // -------------------------------------------------
  // WEB / DEVICE
  // -------------------------------------------------

  json +=
    "\"version\":\"" + String(WEB_VERSION) + "\",";

  json +=
    "\"hostname\":\"" + String(mqttClientId) + "\",";

  // -------------------------------------------------
  // BLIND POSITION
  // -------------------------------------------------

  json +=
    "\"position\":" + String(travelPercent) + ",";

  json +=
    "\"target\":" + String(switchPosition) + ",";

  json +=
    "\"targetPosition\":" + String(targetPosition) + ",";

  json +=
    "\"encoder\":" + String(currentPosition) + ",";

  json +=
    "\"travel\":" + String(travelLength) + ",";

  // -------------------------------------------------
  // LIMITS
  // -------------------------------------------------

  json +=
    "\"openPosition\":" + String(openPosition) + ",";

  json +=
    "\"closedPosition\":" + String(closedPosition) + ",";

  // -------------------------------------------------
  // STATE
  // -------------------------------------------------

  json +=
    "\"state\":\"" + positionStateName() + "\",";

  json +=
    "\"motion\":\"" + motionStateName() + "\",";

  // -------------------------------------------------
  // CONTACT SENSOR
  // -------------------------------------------------

  json +=
    "\"contact\":" + String(contactSensorState ? "true" : "false") + ",";

  // -------------------------------------------------
  // ERROR
  // -------------------------------------------------

  json +=
    "\"error\":" + String(errorFlag ? "true" : "false") + ",";

  // -------------------------------------------------
  // HARDWARE CONFIGURATION
  // -------------------------------------------------

  json +=
    "\"encoderWire\":" + String(encoderWire ? "true" : "false") + ",";

  json +=
    "\"motorWire\":" + String(motorWire ? "true" : "false") + ",";

  json +=
    "\"contactSensor\":" + String(contactSensor ? "true" : "false") + ",";

  // -------------------------------------------------
  // NETWORK
  // -------------------------------------------------

  json +=
    "\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";

  json +=
    "\"mqtt\":" + String(mqtt.connected() ? "true" : "false") + ",";

  json +=
    "\"ip\":\"" + WiFi.localIP().toString() + "\",";

  // -------------------------------------------------
  // DEBUG OUTPUT
  // -------------------------------------------------

  json +=
    "\"debugOutput\":" + String(debugOutput ? "true" : "false") + ",";

  // -------------------------------------------------
  // UPTIME
  // -------------------------------------------------

  unsigned long uptimeSeconds =
    millis() / 1000UL;

  unsigned long days =
    uptimeSeconds / 86400UL;

  uptimeSeconds %= 86400UL;

  unsigned long hours =
    uptimeSeconds / 3600UL;

  uptimeSeconds %= 3600UL;

  unsigned long minutes =
    uptimeSeconds / 60UL;

  unsigned long seconds =
    uptimeSeconds % 60UL;

  String uptime = "";

  if (days > 0) {
    uptime +=
      String(days) + "d ";
  }

  if (
    hours > 0 || days > 0) {

    uptime +=
      String(hours) + "h ";
  }

  if (
    minutes > 0 || hours > 0 || days > 0) {

    uptime +=
      String(minutes) + "m ";
  }

  uptime +=
    String(seconds) + "s";

  json +=
    "\"uptime\":\"" + uptime + "\"";

  // -------------------------------------------------
  // CLOSE JSON
  // -------------------------------------------------

  json += "}";

  httpServer.send(
    200,
    "application/json; charset=utf-8",
    json);
}

// ============================================================
// WEB POSITION
// ============================================================

void handleWebPosition() {

  if (!httpServer.hasArg("value")) {

    httpServer.send(
      400,
      "text/plain; charset=utf-8",
      "Missing position");

    return;
  }

  int value =
    httpServer.arg("value").toInt();

  value =
    constrain(
      value,
      0,
      100);

  Serial.print(
    "🌐 Web target position: ");

  Serial.println(value);

  switchPosition = value;

  target();

  httpServer.send(
    200,
    "text/plain; charset=utf-8",
    "OK");
}

// ============================================================
// WEB STOP
// ============================================================

void handleWebStop() {

  Serial.println("🌐 Web STOP");

  if (motionState != MOTION_IDLE) {

    motorStop();

    motionState = MOTION_IDLE;

    if (myEnc) {
      currentPosition = myEnc->read();
    }

    travelLength =
      abs(
        openPosition - closedPosition);

    if (travelLength == 0)
      travelLength = 1;

    travelPercent =
      100 - (abs(openPosition - currentPosition) * 100 / travelLength);

    travelPercent =
      constrain(
        travelPercent,
        0,
        100);

    switchPosition =
      travelPercent;

    if (travelPercent >= 100)
      status = 4;

    else if (travelPercent <= 0)
      status = 0;

    else
      status = 2;

    targetPosition =
      currentPosition;

    savePosition();

    sendMQTTMessage();
  }

  httpServer.send(
    200,
    "text/plain; charset=utf-8",
    "OK");
}

// ============================================================
// WEB SET UPPER LIMIT
// ============================================================

void handleWebSetUpperLimit() {

  if (myEnc) {
    currentPosition =
      myEnc->read();
  }

  openPosition =
    currentPosition;

  travelLength =
    abs(
      openPosition - closedPosition);

  if (travelLength == 0)
    travelLength = 1;

  travelPercent =
    100 - (abs(openPosition - currentPosition) * 100 / travelLength);

  travelPercent =
    constrain(
      travelPercent,
      0,
      100);

  targetPosition =
    currentPosition;

  switchPosition =
    travelPercent;

  savePosition();

  Serial.print(
    "🌐 Web: Upper limit set to encoder position: ");

  Serial.println(openPosition);

  httpServer.send(
    200,
    "text/plain; charset=utf-8",
    "Upper limit saved");
}

// ============================================================
// WEB SET LOWER LIMIT
// ============================================================

void handleWebSetLowerLimit() {

  if (myEnc) {
    currentPosition =
      myEnc->read();
  }

  closedPosition =
    currentPosition;

  travelLength =
    abs(
      openPosition - closedPosition);

  if (travelLength == 0)
    travelLength = 1;

  travelPercent =
    100 - (abs(openPosition - currentPosition) * 100 / travelLength);

  travelPercent =
    constrain(
      travelPercent,
      0,
      100);

  targetPosition =
    currentPosition;

  switchPosition =
    travelPercent;

  savePosition();

  Serial.print(
    "🌐 Web: Lower limit set to encoder position: ");

  Serial.println(closedPosition);

  httpServer.send(
    200,
    "text/plain; charset=utf-8",
    "Lower limit saved");
}

// ============================================================
// WEB SET HARDWARE CONFIGURATION
// ============================================================

void handleWebSetHardware() {

  if (
    !httpServer.hasArg("encoder") || !httpServer.hasArg("motor") || !httpServer.hasArg("contact")) {

    httpServer.send(
      400,
      "text/plain; charset=utf-8",
      "Missing hardware settings");

    return;
  }

  bool newEncoder =
    httpServer.arg("encoder") == "1";

  bool newMotor =
    httpServer.arg("motor") == "1";

  bool newContact =
    httpServer.arg("contact") == "1";

  Serial.println(
    "🌐 Web: Hardware configuration change");

  Serial.print("   Encoder: ");
  Serial.println(newEncoder);

  Serial.print("   Motor: ");
  Serial.println(newMotor);

  Serial.print("   Contact: ");
  Serial.println(newContact);

  // Update the SAME variables used by WiFiManager

  encoderWire = newEncoder;
  motorWire = newMotor;
  contactSensor = newContact;

  // Save using the SAME LittleFS config

  saveConfig();

  httpServer.send(
    200,
    "text/plain; charset=utf-8",
    "Hardware settings saved. Restarting...");

  delay(500);

  ESP.restart();
}

void handleWebSetDebug() {

  if (!httpServer.hasArg("value")) {

    httpServer.send(
      400,
      "text/plain; charset=utf-8",
      "Missing debug value");

    return;
  }

  debugOutput =
    httpServer.arg("value") == "1";

  Serial.print("🌐 Debug Output: ");
  Serial.println(
    debugOutput
      ? "ON"
      : "OFF");

  httpServer.send(
    200,
    "text/plain; charset=utf-8",
    debugOutput
      ? "Debug output ON"
      : "Debug output OFF");
}

// ============================================================
// WEB 404
// ============================================================

void handleWebNotFound() {

  httpServer.send(
    404,
    "text/plain; charset=utf-8",
    "Not Found");
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  // Serial.setDebugOutput(true);

  pinMode(
    LEDPIN,
    OUTPUT);

  pinMode(
    UPBUTTONPIN,
    INPUT);

  pinMode(
    DOWNBUTTONPIN,
    INPUT);

  delay(1000);  // let GPIO settle

  pixels.begin();

  // -------------------------------------------------
  // RESETTING SETTINGS
  // -------------------------------------------------

  EEPROM.begin(EEPROM_SIZE);

  holdStart = millis();

  resetSetup();

  // -------------------------------------------------
  // HARD RESET WIFI STACK
  // -------------------------------------------------

  WiFi.persistent(false);

  WiFi.disconnect(true);

  WiFi.mode(WIFI_OFF);

  delay(300);

  WiFi.mode(WIFI_STA);

  WiFi.setSleep(false);

  // -------------------------------------------------
  // LITTLEFS
  // -------------------------------------------------

  if (!LittleFS.begin()) {

    Serial.println(
      "❌ LittleFS mount failed");

    flash_LED(
      3,
      LED_PINK);

  } else {

    loadConfig();
  }

  // -------------------------------------------------
  // WIFI MANAGER PARAMETERS
  // -------------------------------------------------

  sprintf(
    contactSensorStr,
    "%d",
    contactSensor ? 1 : 0);

  sprintf(
    motorWireStr,
    "%d",
    motorWire ? 1 : 0);

  sprintf(
    encoderWireStr,
    "%d",
    encoderWire ? 1 : 0);

  wm.setSaveParamsCallback(
    saveParamsCallback);

  wm.addParameter(
    &p_mqtt_server);

  wm.addParameter(
    &p_mqtt_port);

  wm.addParameter(
    &p_mqtt_user);

  wm.addParameter(
    &p_mqtt_pass);

  wm.addParameter(
    &p_mqtt_clientid);

  wm.addParameter(
    &p_contact);

  wm.addParameter(
    &p_motor);

  wm.addParameter(
    &p_encoder);

  wm.setConnectTimeout(30);

  wm.setClass("invert");

  // -------------------------------------------------
  // CHECK IF WE HAVE SAVED CREDENTIALS
  // -------------------------------------------------

  bool hasCredentials =
    (WiFi.SSID().length() > 0);

  if (hasCredentials) {

    Serial.println(
      "📶 Saved WiFi credentials found — trying connect...");

    WiFi.hostname(
      mqttClientId);

    WiFi.setPhyMode(
      WIFI_PHY_MODE_11N);

    WiFi.setOutputPower(
      17.5);

    WiFi.begin();

    unsigned long startAttempt =
      millis();

    while (
      WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {

      delay(250);

      Serial.print(".");
    }

  } else {

    Serial.println(
      "⚠️ No WiFi credentials stored");
  }

  // -------------------------------------------------
  // IF STILL NOT CONNECTED → START PORTAL
  // -------------------------------------------------

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println(
      "\n⚠️ Starting WiFiManager (no valid WiFi connection)");

    String apName =
      "myBlind-" + String(ESP.getChipId(), HEX);

    Serial.println(
      "AP: " + apName);

    wm.autoConnect(
      apName.c_str(),
      "password123");

  } else {

    Serial.println(
      "\n✅ WiFi connected");

    flash_LED(
      1,
      LED_CYAN);
  }

  // -------------------------------------------------
  // HARDEN WIFI AFTER CONNECTION
  // -------------------------------------------------

  WiFi.setAutoReconnect(true);

  WiFi.setSleepMode(
    WIFI_NONE_SLEEP);

  Serial.println(
    "WiFi connected: " + WiFi.localIP().toString());

  // -------------------------------------------------
  // MQTT
  // -------------------------------------------------

  mqtt.setServer(
    mqttServer,
    atoi(mqttPort));

  mqtt.setKeepAlive(60);

  mqtt.setSocketTimeout(10);

  mqtt.setCallback(callback);

  blind_target_topic =
    getMqttTopic(
      "setTargetPosition");

  blind_position_topic =
    getMqttTopic(
      "getCurrentPosition");

  blind_state_topic =
    getMqttTopic(
      "getPositionState");

  blind_gettarget_topic =
    getMqttTopic(
      "getTargetPosition");

  blind_error_topic =
    getMqttTopic(
      "getObstructionDetected");

  blind_contactsensor_topic =
    getMqttTopic(
      "getContactSensorState");

  printConfig();

  bootTime = millis();

  lastHealthyTick = millis();

  // -------------------------------------------------
  // WEB CONTROL UNIT
  // -------------------------------------------------

  httpServer.on(
    "/",
    HTTP_GET,
    handleWebRoot);

  httpServer.on(
    "/api/status",
    HTTP_GET,
    handleWebStatus);

  httpServer.on(
    "/api/setDebug",
    HTTP_GET,
    handleWebSetDebug);

  httpServer.on(
    "/api/position",
    HTTP_GET,
    handleWebPosition);

  httpServer.on(
    "/api/setUpperLimit",
    HTTP_GET,
    handleWebSetUpperLimit);

  httpServer.on(
    "/api/setLowerLimit",
    HTTP_GET,
    handleWebSetLowerLimit);

  httpServer.on(
    "/api/setHardware",
    HTTP_GET,
    handleWebSetHardware);

  httpServer.on(
    "/api/stop",
    HTTP_GET,
    handleWebStop);

  httpServer.onNotFound(
    handleWebNotFound);

  // -------------------------------------------------
  // MDNS + OTA
  // -------------------------------------------------

  if (!MDNS.begin(mqttClientId)) {

    Serial.println(
      "Error setting up MDNS responder!");

    while (1)
      delay(1000);
  }

  httpUpdater.setup(
    &httpServer,
    update_path,
    update_username,
    update_password);

  httpServer.begin();

  MDNS.addService(
    "http",
    "tcp",
    80);

  char buf[16];

  sprintf(
    buf,
    "%d.%d.%d.%d",
    WiFi.localIP()[0],
    WiFi.localIP()[1],
    WiFi.localIP()[2],
    WiFi.localIP()[3]);

  Serial.printf(
    "HTTPUpdate ready → http://%s%s\n",
    buf,
    update_path);

  digitalWrite(
    LEDPIN,
    HIGH);

  // -------------------------------------------------
  // EEPROM
  // -------------------------------------------------

  byte magic;
  byte version;

  EEPROM.get(
    ADDR_MAGIC,
    magic);

  EEPROM.get(
    ADDR_VERSION,
    version);

  if (
    magic != EEPROM_MAGIC || version != EEPROM_VERSION) {

    Serial.println(
      "EEPROM invalid or version mismatch → init defaults");

    // Defaults

    closedPosition = 0;
    openPosition = 100;
    currentPosition = 0;
    statusEEPROM = 0;

    // Write magic + version

    EEPROM.put(
      ADDR_MAGIC,
      EEPROM_MAGIC);

    EEPROM.put(
      ADDR_VERSION,
      EEPROM_VERSION);

    // Write data

    int addr = ADDR_DATA;

    EEPROM.put(
      addr,
      closedPosition);

    addr += sizeof(closedPosition);

    EEPROM.put(
      addr,
      openPosition);

    addr += sizeof(openPosition);

    EEPROM.put(
      addr,
      currentPosition);

    addr += sizeof(currentPosition);

    EEPROM.put(
      addr,
      statusEEPROM);

    EEPROM.commit();

    flash_LED(
      5,
      LED_PURPLE);

  } else {

    Serial.println(
      "EEPROM valid → loading data");

    int addr = ADDR_DATA;

    EEPROM.get(
      addr,
      closedPosition);

    addr += sizeof(closedPosition);

    EEPROM.get(
      addr,
      openPosition);

    addr += sizeof(openPosition);

    EEPROM.get(
      addr,
      currentPosition);

    addr += sizeof(currentPosition);

    EEPROM.get(
      addr,
      statusEEPROM);

    flash_LED(
      1,
      LED_GREEN);
  }

  // -------------------------------------------------
  // ENCODER + TRAVEL MATH
  // -------------------------------------------------

  // EEPROM load finished

  setupEncoderAndMotor();

  if (myEnc) {

    myEnc->write(
      currentPosition);

  } else {

    Serial.println(
      "❌ Encoder not initialised!");
  }

  travelLength =
    abs(
      openPosition - closedPosition);

  if (travelLength == 0)
    travelLength = 1;

  targetDeadband =
    max(
      100,
      travelLength / 200);

  travelPercent =
    100 - (abs(openPosition - currentPosition) * 100 / travelLength);

  if (travelPercent > 100)
    travelPercent = 100;

  if (travelPercent < 0)
    travelPercent = 0;

  acceleration = MINSPEED;

  switchPosition =
    travelPercent;

  pixels.setBrightness(
    LEDBRIGHTNESS);

  setLedColor(
    LED_OFF);

  debug();

  Serial.println(
    "===== READY =====");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  updateLED();

  yield();

  handleTargetMotion();

  handleWiFi();

  handleMQTT();

  healthMonitor();

  if (contactSensor)
    checkContactSensor();

  downButtonCheck();

  upButtonCheck();

  httpServer.handleClient();

  MDNS.update();

  if (mqtt.connected()) {

    if (
      millis() - lastMsg > MQTTUPDATE)
      sendMQTTMessage();

    mqtt.loop();

    lastHealthyTick = millis();
  }
}