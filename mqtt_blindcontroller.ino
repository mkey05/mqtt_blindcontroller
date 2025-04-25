#include <ESP8266WiFi.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP_EEPROM.h>
#include <ESP8266mDNS.h>

#define DEBUG 0
#define CONTACT 0

#define STATUSLED D3
#define LONGPRESSTIME 1000
#define UPBUTTONPIN D8
#define DOWNBUTTONPIN D7
#define CONTACTSENSOR A0
#define LEDPIN D4
#define FINETOLERANCE 1200
#define MQTTUPDATE 30000
#define ERRORCHECK 1000
#define RECONNECTTIME 5000
#define UPDATEPOSITION 1250
#define UPDATEPOSTIONDELAY 250
#define LEDBRIGHTNESS 64
#define MINSPEED 800
#define MAXSPEED 1023
#define RESTARTTIMER 172800000  //48h

//USE ESP8266 Boards library V2.42
//UP + DOWN BUTTON = Reset All
//UP BUTTON Ignore EEPROM values
//DOWN BUTTON Reset EEPROM

//myBlind1 = 1,1
//myBlind2 = 1,0
//myBlind3 = 1,0
//myBlind4 = 1,1
//myBlind5 = 0,0
//myBlind6 = 1,0
//myBlind7 = 1,1
//myBlind8 = 1,0

#define ENCODERWIRE 1
#define MOTORWIRE 1

#define mqtt_server "xxx.xxx.xxx.xxx"
const char* mqtt_user = "xxx";
const char* mqtt_pass = "xxx";
const char* host = "myblind1";
const char* update_path = "/firmware";
const char* update_username = host;
const char* update_password = "local";

String switch1;
String strTopic;
String strPayload;
char cstr[16];
char blindstate[16];
char blindposition[10];
char getTarget[10];
char error[2];
char cs[2];

String blind_target_topic, blind_position_topic, blind_state_topic, blind_gettarget_topic, blind_error_topic, blind_contactsensor_topic;

//Setup the web server for http OTA updates.
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, STATUSLED, NEO_GRB + NEO_KHZ800);

#if ENCODERWIRE
Encoder myEnc(14, 12);
#else
Encoder myEnc(12, 14);
#endif

#if MOTORWIRE
int E1 = 4;
int M1 = 5;
#else
int E1 = 5;
int M1 = 4;
#endif

long lastMsg = 0 - MQTTUPDATE + 10000;
long reconnectwait = 0 - RECONNECTTIME;
long lastReconnectAttempt = 0;
long now = 0;
long lastMQTTLoop = 0;
unsigned long previousMillis = 0;
long buttonTimer = 0;
boolean DownButtonActive = false;
boolean DownLongPressActive = false;
boolean UpButtonActive = false;
boolean UpLongPressActive = false;
boolean setTarget = false;

int contactSensorValue;
boolean contactSensorState, contactSensorState_1;

int closedPosition;
int openPosition;
int currentPosition;
int targetPosition;
int travelLength;
int travelPercent, targetPercent;
int status;  //4 is open, 3, opening, 2 stopped, 1 is closing, 0 is closed
int switchPosition;
int acceleration;
int motorspeed = 0;

//Safety cut off variables
long movementStartTimer;
long movementTimer;
long movementStartPosition;
boolean errorFlag = false;
boolean stopFlag = false;

//EEPROM Address
int addrstatus = 0;
int addrcurrentPosition = 4;
int addrclosedPosition = 8;
int addropenPosition = 12;

String getMqttTopic(String type) {
  return String(host) + "/" + type;
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  strTopic = String((char*)topic);
  if (strTopic == blind_target_topic) {
    switch1 = String((char*)payload);
    switchPosition = switch1.toInt();
    target();
  }
}

#if DEBUG
void debug() {
  Serial.print("Status: ");
  Serial.print(status);
  Serial.print(" | ");
  if (status == 4) Serial.print("Open   ");
  else if (status == 3) Serial.print("Opening");
  else if (status == 2) Serial.print("Stopped");
  else if (status == 1) Serial.print("Closing");
  else if (status == 0) Serial.print("Closed ");
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
  Serial.print("MS: ");
  Serial.print(motorspeed);
  Serial.print(" | ");
  Serial.print("Err: ");
  Serial.print(errorFlag);
  Serial.print(" | ");
  Serial.print("CS: ");
  Serial.print(contactSensorValue);
  Serial.print(" - ");
  Serial.println(contactSensorState);
}
#endif

void flash_LED(int noTimes, String LEDcolor) {
  int x = 0;
  while (x <= noTimes - 1) {
    if (LEDcolor == "Yellow") {
      pixels.setPixelColor(0, 255, 165, 0);  //yellow
    } else if (LEDcolor == "Red") {
      pixels.setPixelColor(0, 255, 0, 0);  //red
    } else if (LEDcolor == "Green") {
      pixels.setPixelColor(0, 0, 255, 0);  //green
    } else if (LEDcolor == "Blue") {
      pixels.setPixelColor(0, 0, 100, 255);  //blue
    } else if (LEDcolor == "Pink") {
      pixels.setPixelColor(0, 255, 0, 165);  //pink
    } else if (LEDcolor == "Purple") {
      pixels.setPixelColor(0, 165, 0, 255);  //purple
    } else if (LEDcolor == "Cyan") {
      pixels.setPixelColor(0, 0, 255, 255);  //cyan
    } else if (LEDcolor == "Orange") {
      pixels.setPixelColor(0, 255, 40, 0);  // orange
    } else {
      pixels.setPixelColor(0, 204, 0, 204);
    }
    ++x;
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
    delay(250);
  }
}

void whileloop() {
  now = millis();
  if (now - lastMQTTLoop > UPDATEPOSITION) {
    client.loop();
    lastMQTTLoop = now;
#if DEBUG
    Serial.println("client.loop active");
#endif
  }
  acceleration++;
  if (acceleration < MAXSPEED) motorspeed = acceleration;
  else motorspeed = MAXSPEED, acceleration = MAXSPEED;
  currentPosition = myEnc.read();
  //currentPosition = currentPosition;
  travelPercent = 100 - (abs(openPosition - currentPosition) * 100 / travelLength);
  if (travelPercent > 100) travelPercent = 100;
  if (travelPercent < 0) travelPercent = 0;
#if DEBUG
  debug();
#endif
  delay(10);
}

void resetAll() {
  for (unsigned int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
  wifiManager.resetSettings();
  EEPROM.put(addrclosedPosition, 0);
  EEPROM.put(addropenPosition, 2000);
  EEPROM.put(addrcurrentPosition, 1000);
  EEPROM.put(addrstatus, 2);
  boolean ok1 = EEPROM.commit();
#if DEBUG
  Serial.println((ok1) ? "Commit OK" : "Commit failed");
#endif
  pixels.setPixelColor(0, 0, 0, 0);
  pixels.show();
}

void sendMQTTMessageLite() {
  itoa(contactSensorState, cs, 2);
  itoa(errorFlag, error, 2);
  client.publish(blind_error_topic.c_str(), error);
  client.publish(blind_contactsensor_topic.c_str(), cs);
#if DEBUG
  Serial.print(blind_contactsensor_topic);
  Serial.print(" ");
  Serial.print(cs);
  Serial.print(" | ");
  Serial.print(blind_error_topic);
  Serial.print(" ");
  Serial.println(error);
#endif
  lastMsg = millis();
}

void sendMQTTMessage() {
  itoa(errorFlag, error, 2);
  itoa(contactSensorState, cs, 2);
  itoa(switchPosition, getTarget, 10);

  if (status == 4) snprintf(blindstate, 8, "STOPPED"), itoa(100, blindposition, 10);
  else if (status == 3) snprintf(blindstate, 11, "INCREASING"), itoa(travelPercent, blindposition, 10);
  else if (status == 2) snprintf(blindstate, 8, "STOPPED"), itoa(travelPercent, blindposition, 10);
  else if (status == 1) snprintf(blindstate, 11, "DECREASING"), itoa(travelPercent, blindposition, 10);
  else if (status == 0) snprintf(blindstate, 8, "STOPPED"), itoa(0, blindposition, 10);

  client.publish(blind_error_topic.c_str(), error);
  client.publish(blind_gettarget_topic.c_str(), getTarget);
  client.publish(blind_state_topic.c_str(), blindstate);
  client.publish(blind_position_topic.c_str(), blindposition);
  client.publish(blind_contactsensor_topic.c_str(), cs);

#if DEBUG
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
  Serial.print(" | ");
  Serial.print("Set: ");
  Serial.print(blind_target_topic);
  Serial.print(" ");
  Serial.print(switchPosition);
  Serial.print(" | ");
  Serial.print(blind_contactsensor_topic);
  Serial.print(" ");
  Serial.println(cs);
#endif
  lastMsg = millis();
}

void target() {
  acceleration = MINSPEED;
  travelLength = abs(openPosition - closedPosition);
  targetPercent = 100 - switchPosition;
  targetPosition = openPosition - (targetPercent * travelLength / 100);
  if (travelPercent > 100) travelPercent = 100;
  if (travelPercent < 0) travelPercent = 0;
  movementStartTimer = millis();
  movementStartPosition = currentPosition;
  errorFlag = false;
  stopFlag = false;

///DOWN
#if CONTACT
  if ((currentPosition > targetPosition) && abs(currentPosition - targetPosition) > FINETOLERANCE && contactSensorState != 1) {
#else
  if ((currentPosition > targetPosition) && abs(currentPosition - targetPosition) > FINETOLERANCE) {
#endif
    pixels.setPixelColor(0, 255, 165, 0);  //yellow
    pixels.show();
    status = 1;
    sendMQTTMessage();
#if DEBUG
    debug();
#endif
    while (currentPosition > targetPosition) {
      whileloop();
      analogWrite(M1, motorspeed);
      digitalWrite(E1, LOW);
      if (digitalRead(DOWNBUTTONPIN) == HIGH or digitalRead(UPBUTTONPIN) == HIGH) {
        stopFlag = true;
        errorFlag = false;
        break;
      }
      if ((millis() - movementStartTimer) > ERRORCHECK) {
        if (currentPosition == movementStartPosition) {
          errorFlag = true;
          break;
        } else {
          movementStartPosition = currentPosition;
          movementStartTimer = millis();
          errorFlag = false;
        }
      }
    }
  }

/// UP
#if CONTACT
  if ((currentPosition < targetPosition) && abs(currentPosition - targetPosition) > FINETOLERANCE && contactSensorState != 1) {
#else
  if ((currentPosition < targetPosition) && abs(currentPosition - targetPosition) > FINETOLERANCE) {
#endif
    pixels.setPixelColor(0, 0, 100, 255);  //blue
    pixels.show();
    status = 3;
    sendMQTTMessage();
#if DEBUG
    debug();
#endif
    while (currentPosition < targetPosition) {
      whileloop();
      analogWrite(E1, motorspeed);
      digitalWrite(M1, LOW);
      if (digitalRead(DOWNBUTTONPIN) == HIGH or digitalRead(UPBUTTONPIN) == HIGH) {
        stopFlag = true;
        errorFlag = false;
        break;
      }
      if ((millis() - movementStartTimer) > ERRORCHECK) {
        if (currentPosition == movementStartPosition) {
          errorFlag = true;
          break;
        } else {
          movementStartPosition = currentPosition;
          movementStartTimer = millis();
          errorFlag = false;
        }
      }
    }
  }

  acceleration = MINSPEED;
  motorspeed = 0;
  digitalWrite(E1, LOW);
  digitalWrite(M1, LOW);
  delay(UPDATEPOSTIONDELAY);
  currentPosition = myEnc.read();

  if (errorFlag == false) {
    if (travelPercent == 100) status = 4;
    else if (travelPercent == 0) status = 0;
    else status = 2;
    switchPosition = travelPercent;
#if DEBUG
    debug();
#endif
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
    EEPROM.put(addrcurrentPosition, currentPosition);
    EEPROM.put(addrstatus, status);
    boolean ok1 = EEPROM.commit();
#if DEBUG
    Serial.println((ok1) ? "Commit OK" : "Commit failed");
#endif
    flash_LED(1, String("Green"));
    sendMQTTMessage();
  } else {
    if (travelPercent == 100) status = 4;
    else if (travelPercent == 0) status = 0;
    else status = 2;
    switchPosition = travelPercent;
#if DEBUG
    debug();
#endif
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
    EEPROM.put(addrcurrentPosition, currentPosition);
    EEPROM.put(addrstatus, status);
    boolean ok1 = EEPROM.commit();
#if DEBUG
    Serial.println((ok1) ? "Commit OK" : "Commit failed");
#endif
    flash_LED(1, String("Red"));
    sendMQTTMessage();
  }
}

void downButtonCheck() {
  if (digitalRead(DOWNBUTTONPIN) == HIGH) {
    if (DownButtonActive == false) {
      DownButtonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > LONGPRESSTIME) && (DownLongPressActive == false)) {  // Button Held
      DownLongPressActive = true;
      status = 1;
      sendMQTTMessage();
      acceleration = MINSPEED;
      while (digitalRead(DOWNBUTTONPIN) == HIGH) {
        whileloop();
        analogWrite(M1, motorspeed);
        digitalWrite(E1, LOW);
        pixels.setPixelColor(0, 255, 165, 0);  //yellow
        pixels.show();
        currentPosition = myEnc.read();
      }
    }
  } else {
    if (DownButtonActive == true) {
      if (DownLongPressActive == true) {
        status = 2;
        sendMQTTMessage();
#if DEBUG
        debug();
#endif
        DownLongPressActive = false;
        acceleration = MINSPEED;
#if DEBUG
        Serial.println("Released Button!");
#endif
        motorspeed = 0;
        digitalWrite(E1, LOW);
        digitalWrite(M1, LOW);
        delay(UPDATEPOSTIONDELAY);
        currentPosition = myEnc.read();

        if (digitalRead(DOWNBUTTONPIN) == HIGH) {
          closedPosition = currentPosition;
          status = 0;
          EEPROM.put(addrclosedPosition, closedPosition);
          EEPROM.put(addrcurrentPosition, currentPosition);
          EEPROM.put(addrstatus, status);
          boolean ok1 = EEPROM.commit();
#if DEBUG
          Serial.println((ok1) ? "Commit OK" : "Commit failed");
          Serial.println("Closed Position Updated!");
#endif
          flash_LED(3, String("Yellow"));
          flash_LED(1, String("Purple"));
          sendMQTTMessage();
        } else {
          pixels.setPixelColor(0, 0, 0, 0);
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

void upButtonCheck() {
  if (digitalRead(UPBUTTONPIN) == HIGH) {
    if (UpButtonActive == false) {
      UpButtonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > LONGPRESSTIME) && (UpLongPressActive == false)) {
      UpLongPressActive = true;
      status = 3;
      sendMQTTMessage();
      acceleration = MINSPEED;
      while (digitalRead(UPBUTTONPIN) == HIGH) {
        whileloop();
        analogWrite(E1, motorspeed);
        digitalWrite(M1, LOW);
        pixels.setPixelColor(0, 0, 100, 255);  //blue
        pixels.show();
        currentPosition = myEnc.read();
      }
    }
  } else {
    if (UpButtonActive == true) {
      if (UpLongPressActive == true) {
        status = 2;
        sendMQTTMessage();
#if DEBUG
        debug();
#endif
        UpLongPressActive = false;
        acceleration = MINSPEED;
#if DEBUG
        Serial.println("Released Button!");
#endif
        motorspeed = 0;
        digitalWrite(E1, LOW);
        digitalWrite(M1, LOW);
        delay(UPDATEPOSTIONDELAY);
        currentPosition = myEnc.read();

        if (digitalRead(UPBUTTONPIN) == HIGH) {
          openPosition = currentPosition;
          status = 4;
          EEPROM.put(addropenPosition, openPosition);
          EEPROM.put(addrcurrentPosition, currentPosition);
          EEPROM.put(addrstatus, status);
          boolean ok1 = EEPROM.commit();
#if DEBUG
          Serial.println((ok1) ? "Commit OK" : "Commit failed");
          Serial.println("Open Position Updated!");
#endif
          flash_LED(3, String("Blue"));
          flash_LED(1, String("Purple"));
          sendMQTTMessage();
        } else {
          pixels.setPixelColor(0, 0, 0, 0);
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

void checkContactSensor() {
  contactSensorValue = analogRead(CONTACTSENSOR);
  delay(3);
  if (contactSensorValue == 1024) contactSensorState = 0, digitalWrite(LEDPIN, LOW);  // Wemos BUILTIN_LED is active Low, so high is off;
  else contactSensorState = 1, digitalWrite(LEDPIN, HIGH);
  if (contactSensorState != contactSensorState_1) {
    contactSensorState_1 = contactSensorState;
    sendMQTTMessageLite();
  }
}

boolean reconnect() {
#if DEBUG
  Serial.print("Attempting MQTT connection...");
#endif
  if (client.connect(host, mqtt_user, mqtt_pass)) {
#if DEBUG
    Serial.println("connected");
#endif
    client.subscribe(blind_target_topic.c_str());
    flash_LED(1, String("Orange"));
  }
  return client.connected();
}

void setup() {
#if DEBUG
  Serial.begin(115200);
#endif
  pinMode(UPBUTTONPIN, INPUT);
  pinMode(DOWNBUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);  // initialize digital LEDPIN as an output.
  pixels.begin();           // This initializes the NeoPixel library.
  pixels.setBrightness(LEDBRIGHTNESS);
  pixels.setPixelColor(0, 255, 0, 0);  //red
  pixels.show();
  delay(4000);

  if (digitalRead(DOWNBUTTONPIN) == HIGH and digitalRead(UPBUTTONPIN) == HIGH) {
    flash_LED(3, String("Green"));
    flash_LED(4, String("Yellow"));
    flash_LED(5, String("Orange"));
    flash_LED(6, String("Red"));
#if DEBUG
    Serial.println("Resetting All Settings...");
#endif
    resetAll();
    delay(5000);
  }

#if DEBUG
  Serial.println("Starting...");
  Serial.println("Blind Controller v3.6");
  Serial.println("UP + DOWN BUTTON  = Reset All");
  Serial.println("UP BUTTON         = EEPROM Ignore");
  Serial.println("DOWN BUTTON       = EEPROM Reset");
#endif

  pixels.setPixelColor(0, 255, 40, 0);  // orange
  pixels.show();
  lastReconnectAttempt = 0;

  checkContactSensor();

  blind_target_topic = getMqttTopic("setTargetPosition");
  blind_position_topic = getMqttTopic("getCurrentPosition");
  blind_state_topic = getMqttTopic("getPositionState");
  blind_gettarget_topic = getMqttTopic("getTargetPosition");
  blind_error_topic = getMqttTopic("getObstructionDetected");
  blind_contactsensor_topic = getMqttTopic("getContactSensorState");

  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect(host);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);  //callback is the function that gets called for a topic sub
  delay(100);

  if (!MDNS.begin(host)) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  httpUpdater.setup(&httpServer, update_path, update_username, update_password);

  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
#if DEBUG
  char buf[16];
  sprintf(buf, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

  Serial.printf("HTTPUpdateServer ready! Open http://%s%s in your browser and login with username '%s' and your password\n", buf, update_path, update_username);
#endif
  delay(10);
  digitalWrite(LEDPIN, HIGH);  // Wemos BUILTIN_LED is active Low, so high is off
#if DEBUG
  Serial.print("Read EEPROM...");
#endif
  EEPROM.begin(16);

  if (digitalRead(DOWNBUTTONPIN) == LOW and digitalRead(UPBUTTONPIN) == HIGH) {
    delay(100);
    flash_LED(3, String("Pink"));
#if DEBUG
    Serial.println("Ignored!");
#endif
  } else if (digitalRead(DOWNBUTTONPIN) == HIGH and digitalRead(UPBUTTONPIN) == LOW) {
    closedPosition = 0;
    openPosition = 2000;
    currentPosition = 1000;
    EEPROM.put(addrclosedPosition, closedPosition);
    EEPROM.put(addropenPosition, openPosition);
    EEPROM.put(addrcurrentPosition, currentPosition);
    EEPROM.put(addrstatus, status);
    boolean ok2 = EEPROM.commitReset();
#if DEBUG
    Serial.println((ok2) ? "Commit OK" : "Commit failed");
#endif
    delay(100);
    flash_LED(3, String("Red"));
#if DEBUG
    Serial.println("Reset!");
#endif
  } else {
    EEPROM.get(addrclosedPosition, closedPosition);
    EEPROM.get(addropenPosition, openPosition);
    EEPROM.get(addrcurrentPosition, currentPosition);
    EEPROM.get(addrstatus, status);
    delay(100);
    flash_LED(2, String("Purple"));
#if DEBUG
    Serial.println("Success!");
#endif
  }

  if (ENCODERWIRE == 0) Serial.println("UP = CCW || DOWN = CW"), flash_LED(1, String("Yellow"));
  else {
#if DEBUG
    Serial.println("UP = CW || DOWN = CCW");
#endif
    flash_LED(1, String("Blue"));
  }

  myEnc.write(currentPosition);
  travelLength = abs(openPosition - closedPosition);
  travelPercent = 100 - (abs(openPosition - currentPosition) * 100 / travelLength);
  if (travelPercent > 100) travelPercent = 100;
  if (travelPercent < 0) travelPercent = 0;
  acceleration = MINSPEED;
  switchPosition = travelPercent;
  delay(100);
  pixels.setPixelColor(0, 0, 0, 0);
  pixels.show();
#if DEBUG
  debug();
#endif
}

void loop() {
  if (!client.connected()) {
    pixels.setPixelColor(0, 255, 40, 0);  // orange
    downButtonCheck();
    upButtonCheck();
    now = millis();
    if (now - lastReconnectAttempt > RECONNECTTIME) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    checkContactSensor();
    downButtonCheck();
    upButtonCheck();
    if (millis() - lastMsg > MQTTUPDATE) sendMQTTMessageLite();
    client.loop();
    httpServer.handleClient();  //handles requests for the firmware update page
  }
  if (millis() > RESTARTTIMER & motorspeed == 0) ESP.restart();
}
