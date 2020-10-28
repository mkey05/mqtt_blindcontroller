# mqtt_blindcontroller
Powerful MQTT Blind Controller

Before uploading the code make sure to enter the MQTT credentials (MQTT server, user, password) in Arduino IDE. I had to use the ESP8266 library version 2.42 as the latest version gave me issues. All the other libraries needed are the following and the most recent should work:

Encoder
Adafruit NeoPixel
PubSubClient
WiFiManager
ESP_EEPROM

The host is called 'myblind' by default but can be changed.

You can also change the direction of the Motor (MOTORWIRE) and wiring of the Encoder (ENCODERWIRE) - use 1 or 0 to do so but leave it for now.
Once the code is uploaded successfully you shall be able to see and connect to 'myBlind' wifi network via WiFiManager. Connect and enter your wifi details and if done correctly it will connect to your local wifi network and will be ready to run!

You will  be able to upload future updates via web server which its useful if you use more than one.
Attention: After saving the WiFi credentials successfully,keep the DOWN Button pressed to Reset the EEPROM to Default settings.
Otherwise it will crash as the controller tries to read saved EEPROM values initially which are not there yet! If it didn't work the first time unplug it and power it up again while keeping the DOWN Button pressed.

For now keep the computer connected and attach the 12V Power adapter to power up the motor!
Use the Serial Monitor at 115200 baud. A long press of the UP Button should rotate the Motor clockwise and the Encoder value of the current position (Pos) should go up too. If that's not the case change the ENCODERWIRE value to the opposite number (either 1 or 0). The reason behind this is some of the motor encoders are wired one way and some the other.

It's the same obviously using the DOWN Button but the other way around.
