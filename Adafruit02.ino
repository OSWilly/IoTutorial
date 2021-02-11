
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

#define AIO_SERVER      "io.adafruit.com"      // padrão
#define AIO_SERVERPORT  1883                   // padrão
#define AIO_USERNAME    "SEU ID ADAFRUIT"
#define AIO_KEY         "SUA KEY ADAFRUIT"

/************ Configuração padrão (não alterar!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Configurar a classe cliente MQTT informando o cliente WIFI, MQTT server e detalhes de login.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

Adafruit_MQTT_Publish luz = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/led");          // altere o "led" para o nome do feed que você criou no site do adafruit
Adafruit_MQTT_Subscribe botao = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/rele");   // altere o "rele" para o nome do feed que você criou no site do adafruit

/*************************** Código ************************************/
void MQTT_connect();

void setup() {  
  pinMode(RELE, OUTPUT); //Declara pino do rele como saída
  pinMode(LED, OUTPUT);  //Declara pino do led como saída
  
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

  // Conexão com o WiFi.
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

  // Configurar as inscrições no MQTT (subscription).
  mqtt.subscribe(&botao); 
}

uint32_t x=0;

void loop() {
  // Verifica, conecta e reconecta, se necessário, com o MQTT 
  MQTT_connect();

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

// Função para conectar e reconectar com o MQTT se necessário.
void MQTT_connect() {
  int8_t ret;

  // Para se já estiver conectado.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { 
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
