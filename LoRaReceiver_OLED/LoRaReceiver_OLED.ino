/**
 * @file LoRaReceiver_OLED.ino
 * @author Pablo Encinas García (p02engap@uco.es)
 * @brief
 * @version 0.1
 * @date 2024-07-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include "encod.h"
#include "NTP.h"
#include <string.h>
#include "PubSubClient.h"
#include <WiFi.h>

// MACROS
#define RF_FREQUENCY 915000000  // Hz
#define TX_OUTPUT_POWER 14      // dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 112 // Define the payload size here

#define SSID "iPhone de Pablo"                   // CAMBIAR PARA OTRO WiFi
#define PASSWORD "ensipensi"                    // CAMBIAR PARA OTRO WiFi KGPnoCg3bDEX6FBharSY->MOVISTAR_0959 || 6GLHBT4gre2j2QJ2->IOT
const char *mqtt_server = "broker.hivemq.com"; // CAMBIAR PARA OTRA IP

// Variables que usaremos
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
int16_t txNumber;
int16_t rssi, rxSize;
bool lora_idle = true;
char decode_data[112];
char buff[128];
char buff_equipo[8];
char buff_variable[8];
char buff_dato[128];
char date_char[16];
char hour_char[16];

String message;
String topic;
String sade_check;
long lastMsg = 0;
char msg[50];
int value = 0;
char char_message[116];
char char_topic[64];

// Instanciamos las clases
SSD1306Wire oled(0x3c, 500000, 4, 15, GEOMETRY_128_64, 16); // addr , freq , i2c group , resolution , rst
Base64Class encode_msg;
WiFiClient espClient;
WiFiUDP wifiUdp;
NTP ntp(wifiUdp);
PubSubClient client(espClient);

/**
 * @brief Función para incializar MCU, Oled y la comunicación
 *
 */
void init()
{
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  oled.init();
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void Setup_WiFi()
{
  delay(10);
  Serial.printf("Connecting to %s\r\n", SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\r\nWifi connected\r\n");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.println();
  oled.clear();
  oled.drawString(0,0,"Conectado a WiFi");
  oled.display();
}

void Callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32"))
    {
      Serial.println("connected");
      oled.clear();
      oled.drawString(0,0,"Conexion MQTT ok!");
      oled.display();
      
      // Subscribe
      client.subscribe("SADE/time/output");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  init();
  oled.drawString(0, 0, "LoRa Receiver");
  oled.display();
  txNumber = 0;
  rssi = 0;
  RadioEvents.RxDone = OnRxDone;
  Setup_WiFi();
  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60);   // last sunday in october 3:00, timezone +60min (+1 GMT)

  ntp.begin();

  client.setServer(mqtt_server, 1883);
  client.setCallback(Callback);
}

void loop()
{

  if (lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  oled.clear();
  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);

  encode_msg.decode(decode_data, rxpacket, sizeof(rxpacket));
  Serial.printf("Mensaje Rx decodificado: \"%s\"\r\n", decode_data);

  String str_decode_data = String(decode_data);

  int i = str_decode_data.indexOf("/");
  int j = str_decode_data.indexOf("*");

  str_decode_data.substring(5, i).toCharArray(buff_equipo, sizeof(buff_equipo));
  str_decode_data.substring(i + 1, j).toCharArray(buff_variable, sizeof(buff_variable));
  str_decode_data.substring(j + 1).toCharArray(buff_dato, sizeof(buff_dato));

  Serial.println(buff_equipo);
  oled.drawString(0,50,buff_equipo);
  oled.display();
  Serial.println(buff_variable);
  Serial.println(buff_dato);

  sade_check = str_decode_data.substring(0, 4); // SADE#Equipo1/t*%0.2f, %0.2f, %0.2f es lo que se recibe
  Serial.println(sade_check);

  if (sade_check == "SADE")
  {

    if (!client.connected())
    {
      reconnect();
    }
    client.loop();
    long now = millis();
    if (now - lastMsg > 5000)
    {
      ntp.update();
      lastMsg = now;
      String date = ntp.formattedTime("%d-%B-%Y");
      String hour = ntp.formattedTime("%T");

      message = date + " " + hour + ", " + String(buff_dato);
      Serial.println(message);

      topic = "Fabiano/Sadeco/Equipos/" + String(buff_equipo) + "/" + String(buff_variable) + "/" + "RecSi";
      Serial.println(topic);

      message.toCharArray(char_message, sizeof(char_message));
      topic.toCharArray(char_topic, sizeof(char_message));

      client.publish(char_topic, char_message);
      
    }
  }

  sprintf(buff, "rssi: %d", rssi);
  oled.drawString(0, 10, buff);
  oled.display();

  lora_idle = true;
}