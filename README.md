# mqtt_blindcontroller
Powerful MQTT Blind Controller

You will find a full guide here:
https://www.hackster.io/mkey05/powerful-mqtt-blind-controller-d5df5d

Before uploading the code make sure to enter the MQTT credentials (MQTT server, user, password) in Arduino IDE. I had to use the ESP8266 library version 2.42 as the latest version gave me issues. All the other libraries needed are the following and the most recent should work:

Encoder, Adafruit NeoPixel, PubSubClient, WiFiManager, ESP_EEPROM

The host is called 'myblind' by default but can be changed.

You can also change the direction of the Motor (MOTORWIRE) and wiring of the Encoder (ENCODERWIRE) - use 1 or 0 to do so but leave it for now.
Once the code is uploaded successfully you shall be able to see and connect to 'myBlind' wifi network via WiFiManager. Connect and enter your wifi details and if done correctly it will connect to your local wifi network and will be ready to run!

You will  be able to upload future updates via web server which its useful if you use more than one.
Attention: After saving the WiFi credentials successfully,keep the DOWN Button pressed to Reset the EEPROM to Default settings.
Otherwise it will crash as the controller tries to read saved EEPROM values initially which are not there yet! If it didn't work the first time unplug it and power it up again while keeping the DOWN Button pressed.

For now keep the computer connected and attach the 12V Power adapter to power up the motor!
Use the Serial Monitor at 115200 baud. A long press of the UP Button should rotate the Motor clockwise and the Encoder value of the current position (Pos) should go up too. If that's not the case change the ENCODERWIRE value to the opposite number (either 1 or 0). The reason behind this is some of the motor encoders are wired one way and some the other.

It's the same obviously using the DOWN Button but the other way around.

How to use it!
A continued long press UP will move the blind up.
A continued long press DOWN will move the blind down.

Set up Position of Blind (100%)
A continued long press UP will move the blind up - keep pressing UP until it reaches the up position followed quick short press UP. LED flashes 3 x Blue, 1 x Purple to confirm new up position

Set downPosition of Blind (0%)
A continued long press DOWN will move the blind down - keep pressing DOWN until it reaches the down position followed quick short press DOWN. LED flashes 3 x Yellow, 1 x Purple to confirm new up position

Ignore EEPROM values Press UP BUTTON for 5 seconds while powering up. It's followed by the LED flashing 3 x Pink

Reset EEPROM values Press DOWN BUTTON for 5 seconds while powering up. It's followed by the LED flashing 3 x Red

Reset All Settings incl. WiFi credentials Press UP + DOWN BUTTON for 5seconds while powering upIt's followed by the LED flashing 3 x Green, 4 x Yellow, 5 x Orange, 6 x Red

LED sequence during normal Start Up
Red = initially when powered on
Orange = connecting to MQTT server
2 x Purple = Reading EEPROM values successful
1 x Blue = UP Button rotates motor clockwise OR 1 x Yellow = UP Button rotates motor counterclockwise
1 x Orange = MQTT connected
LED off

LED Statuses 
LED Off = blind stopped or unpowered ;)
Orange = No MQTT connection, Buttons still work
Blue = Blind going up
Yellow = Blind going down
1 x Green = arrived at Target Position
1 x Red = Error, Motor didn't move for 1 second

MQTT and Homekit
Implementation for config.json in Homebridge as an accessory via the amazing plugin mqttthing by arachnetch. I personally use a Raspberry P Zero W for Homebridge and local MQTT server powered by Mosquito. Setting up Homebridge and MQTT is a different topic and you will find plenty of information on their websites!

{
"type": "windowCovering",
"name": "myblind",
"url": "mqtt://xxxxxxx",
"username": "xxxx",
"password": "xxxx",
"logMqtt": true,
"topics": {
"getCurrentPosition": "myblind/getCurrentPosition",
"getPositionState": "myblind/getPositionState",
"getTargetPosition": "myblind/getTargetPosition",
"setTargetPosition": "myblind/setTargetPosition",
"getObstructionDetected": "myblind/getObstructionDetected"
},
"accessory": "mqttthing"
}
