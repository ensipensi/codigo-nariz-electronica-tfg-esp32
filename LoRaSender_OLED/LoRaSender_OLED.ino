#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include "encod.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_AHTX0.h"
#include <GPSPlus.h>
#include <HardwareSerial.h>
#include "ADS1X15.h"

#define RF_FREQUENCY 915000000  // Hz
#define TX_OUTPUT_POWER 5       // dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz, \
                                 //  1: 250 kHz, \
                                 //  2: 500 kHz, \
                                 //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5, \
                                 //  2: 4/6, \
                                 //  3: 4/7, \
                                 //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 30 // Define the payload size here
#define SDA 42
#define SCL 41

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
double txNumber = 0.0;
char buff_temperature[30];
char buff_pressure[30];
char buff_humidity[30];
bool lora_idle = true;
char encoded_data[64];
static RadioEvents_t RadioEvents;
uint32_t x = 0;

float humidity = 0.0;
float temperature = 0.0;
float pressure = 0.0;

float lectura1 = 0.0;
float lectura2 = 0.0;
float lectura3 = 0.0;
float lectura4 = 0.0;

double latitud = 37.900065;
double longitud = -4.760949;

char lat_long_oled[64];
char s_lecture[64];

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
Base64Class encode_msg;
ADS1115 ADS(0x48, &Wire1);
Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire1); // I2C
Adafruit_AHTX0 aht;
GPSPlus gps;
HardwareSerial ss(1);

void OnTxDone(void);
void OnTxTimeout(void);

void init(void)
{
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin(41, 42);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  oled.init();

  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Lora Sender");
  oled.display();
  delay(1000);
  oled.clear();
  oled.display();

  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    oled.drawString(0, 0, "BMP Init Failed, try again!");
    oled.display();
    oled.clear();
    while (1)
      delay(10);
  }

  oled.clear();
  if (!aht.begin(&Wire1))
  {
    Serial.println(F("Could not find a valid AHT20 sensor, check wiring or"
                     "try a different address"));
    oled.drawString(0, 0, "AHT Init Failed, try again!");
    oled.display();
    oled.clear();

    while (1)
      ;
  }
  oled.clear();
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void setup()
{
  init();
  ss.begin(9600, SERIAL_8N1, 20, 19);

  while (!ADS.begin())
  {
    Serial.println("ERROR en la inicializacion del convertidor");
    oled.drawString(0, 0, "ADC Init Failed, try again!");
    oled.display();
    oled.clear();
  }

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
}

void loop()
{
  sensors_event_t hum, temp;

  oled.clear();
  aht.getEvent(&hum, &temp);
  humidity = hum.relative_humidity;
  if (bmp.takeForcedMeasurement())
  {
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
  }

  /*************************************GPS*************************************/
  // while (ss.available() > 0)
  // {
  //   if (gps.encode(ss.read()))
  //     displayInfo();
  // }

  int16_t val_0 = ADS.readADC(0);
  int16_t val_1 = ADS.readADC(1);
  int16_t val_2 = ADS.readADC(2);
  int16_t val_3 = ADS.readADC(3);

  float f = ADS.toVoltage(1);

  lectura1 = val_0 * f;
  lectura2 = val_1 * f;
  lectura3 = val_2 * f;
  lectura4 = val_3 * f;

  sprintf(buff_temperature, "temperature= %f C", temperature);
  sprintf(buff_pressure, "pressure= %f Pa", pressure);
  sprintf(buff_humidity, "humidity= %f %", humidity);
  sprintf(lat_long_oled, "%f, %f", latitud, longitud);
  sprintf(s_lecture, "s0: %0.2f V, s1: %0.2f V,\ns2: %0.2f V, s3= %0.2f V", lectura1, lectura2, lectura3, lectura4);
  oled.clear();
  oled.drawString(0, 0, buff_temperature);
  oled.drawString(0, 10, buff_pressure);
  oled.drawString(0, 20, buff_humidity);
  oled.drawString(0, 30, lat_long_oled);
  oled.drawString(0, 40, s_lecture);

  oled.display();

  if (lora_idle == true)
  {
    uint8_t x = rand();
    sprintf(txpacket, "SADE#Equipo1/t*37.899176, -4.757763, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f ", humidity, temperature, pressure, lectura1, lectura2, lectura3, lectura4); // start a package

    Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));

    encode_msg.encode(encoded_data, txpacket, strlen(txpacket));
    Serial.printf("Mensaje codificado: \"%s\"\r\n", encoded_data);

    Radio.Send((uint8_t *)encoded_data, strlen(encoded_data)); // send the package out
    lora_idle = false;
  }
  Radio.IrqProcess();
  x = random(15, 1500);
  Serial.println(x);
  delay(x);
}

void OnTxDone(void)
{
  Serial.println("TX done......");
  lora_idle = true;
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.println("TX Timeout......");
  lora_idle = true;
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    latitud = gps.location.lat();
    longitud = gps.location.lng();
  }
}
