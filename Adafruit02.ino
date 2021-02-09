
/************************* Bibliotecas *********************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* Variaveis *********************************/

const int RELE = 0;                 //Define pino da rele como 0-D3
const int LED = 2;                  //Define pino do led como 2-D4

/************************* WiFi *********************************/

#define WLAN_SSID       "SUA INTERNET"
#define WLAN_PASS       "SENHA DA SUA INTERNET"

/************************* Adafruit.io *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "SEU ID ADAFRUIT"
#define AIO_KEY         "SUA KEY ADAFRUIT"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/


Adafruit_MQTT_Publish luz = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/led");
Adafruit_MQTT_Subscribe botao = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/rele");

/*************************** Código ************************************/
void MQTT_connect();

void setup() {  
  pinMode(RELE, OUTPUT); //Declara pino do rele como saída
  pinMode(LED, OUTPUT);  //Declara pino do led como saída
  
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

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

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&botao); //realiza subscrição
}

uint32_t x=0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {

    //SALA
    if (subscription == &botao) {      //se subscrito em sala
      
      if (strcmp((char *)botao.lastread, "ON") == 0) { //se a última leitura de ligar for equivalente a 0
         digitalWrite(RELE, LOW);
         digitalWrite(LED, HIGH);
         luz.publish(1);
                                                       }
      if (strcmp((char *)botao.lastread, "OFF") == 0) { //se a última leitura de desligar for equivalente a 0
        digitalWrite(RELE, HIGH);
        digitalWrite(LED, LOW);
        luz.publish(0);
                                                          }
                               }


}}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
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
