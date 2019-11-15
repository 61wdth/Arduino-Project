#include <ESP32Servo.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <LiquidCrystal_I2C.h>

#define WLAN_SSID       "AndroidHotspot1768"
#define WLAN_PASS       "qkrtpwls"

#define AIO_SERVER      "io.adafruit.com"

#define AIO_SERVERPORT  1883                  

#define AIO_USERNAME    "jsph666"

#define AIO_KEY         "ff23556d70ce43e097f32932b4254da3"

int output=2;
Servo smotor;
int servoPin = 23;
int redNum = 19;
int blueNum = 18;

WiFiClient client;     // Create an ESP8266 WiFiClient class to connect to the MQTT server.
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);        // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.

Adafruit_MQTT_Subscribe OpenDoor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/OpenDoor");


void MQTT_connect();

void setup() {
  
  Serial.begin(115200);
  delay(10);
  lcd.begin();
  lcd.backlight();
  smotor.attach(servoPin);
  smotor.write(0);
  pinMode(redNum, OUTPUT);
  pinMode(blueNum, OUTPUT);
 // Connect to WiFi access point.

  Serial.println(); Serial.println();

  Serial.print("Connecting to ");

  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.print(".");

  }

  Serial.println();

 Serial.println("WiFi connected");

  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  mqtt.subscribe(&OpenDoor);
  

}

uint32_t x=0;



void loop() {
   MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  

  while ((subscription = mqtt.readSubscription(5000))) {

    if (subscription == &OpenDoor) {

      Serial.print(F("Got: "));

      Serial.println((char *)OpenDoor.lastread);
      
       if (!strcmp((char*) OpenDoor.lastread, "ON")){

        digitalWrite(blueNum,HIGH);
        for(int angle = 0; angle <= 150; angle++){
          smotor.write(angle);
          Serial.println(angle);
          delay(5);
        }
        
        delay(5000);
        digitalWrite(blueNum, LOW);
        digitalWrite(redNum, HIGH);

        for(int angle = 150; angle >= 0; angle--){
          smotor.write(angle);
          Serial.println(angle);
          delay(5);
        }
        digitalWrite(redNum, LOW);
        
      }

      if (!strcmp((char*) OpenDoor.lastread, "OFF")){

        
        for(int angle = 150; angle >= 0; angle--){
          smotor.write(angle);
          Serial.println(angle);
          delay(5);
        }
        digitalWrite(redNum, HIGH);
        delay(5000);
        digitalWrite(redNum, LOW);

      }

      Serial.print(F("Got: "));

      Serial.println((char *)OpenDoor.lastread);
      
       if (!strcmp((char*) OpenDoor.lastread, "IN")){

        lcd.clear();
        lcd.print("IN THE ROOM");
        
      }

      if (!strcmp((char*) OpenDoor.lastread, "OUT")){

        lcd.clear();
        lcd.print("NOT IN THE ROOM");

      }

      if (!strcmp((char*) OpenDoor.lastread, "ARRIVE SOON")){

        lcd.clear();
        lcd.print("ARRIVE SOON");

      }

}

}
}

void MQTT_connect(){

  int8_t ret;

  // Stop if already connected.

  if (mqtt.connected()) {

    return;

  }

 Serial.print("Connecting to MQTT... ");

uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected

       Serial.println(mqtt.connectErrorString(ret));

       Serial.println("Retrying MQTT connection in 5 seconds...");

       mqtt.disconnect();

       delay(5000);  // wait 5 seconds

       retries--;

       if (retries == 0) {

         // basically die and wait for WDT to reset me

         while (1);

       }

  }

  Serial.println("MQTT Connected!");

}
