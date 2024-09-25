/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 * 2. The network access status of LoRaWAN is displayed on the screen.
 * 
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

#include "LoRaWan_APP.h"
#include <stdio.h>
#include "Adafruit_BMP280.h"
#include "Adafruit_AHTX0.h"
#include "ADS1X15.h"

Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire1);;
Adafruit_AHTX0 aht;
ADS1115 ADS(0x48, &Wire1);


/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0x70 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = { 0x10, 0x2A, 0x47, 0x2D, 0x5D, 0xE0, 0xF0, 0x17, 0x26, 0x66, 0x2A, 0x95, 0xDE, 0x54, 0x0C, 0xC7 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x76, 0x44, 0xA2, 0xC6, 0xFB, 0xE0, 0x55, 0xA5, 0xEF, 0x40, 0xE1, 0xFE, 0x8C, 0x0B, 0xF8, 0x74 };
uint8_t appSKey[] = { 0x94, 0x33, 0x94, 0xAD, 0x1A, 0xAC, 0xB1, 0xF5, 0xAC, 0x6F, 0x6B, 0x3D, 0xBE, 0x82, 0x98, 0x29 };
uint32_t devAddr =  ( uint32_t )0x260B66FE;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;


/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;
/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
  /********************************** En esta parte del código enviamos los datos **********************************/
  /********************* En esta parte lo que vemos es el ejemplo que viene con la librería *********************/
  // appDataSize = 4;
  // appData[0] = 0x00;
  // appData[1] = 0x01;
  // appData[2] = 0x02;
  // appData[3] = 0x03;

  /********************* Ahora vemos como se envían los datos a ttn *********************/
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  float temperature = 0;  //example 22.55 *C
  float pressure = 0;    //example 72.5 %
  float hum = 0;

  /* Tendría que poner aquí el GPS */

  if (bmp.takeForcedMeasurement()) {
    // can now print out the new measurements
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure()/100;
    hum = humidity.relative_humidity;
    Serial.println(hum);
  }

  int16_t val_0 = ADS.readADC(0);  
  int16_t val_1 = ADS.readADC(1);  
  int16_t val_2 = ADS.readADC(2);  
  int16_t val_3 = ADS.readADC(3);  

  float f = ADS.toVoltage(1);  //  voltage factor
  
  int int_temp = temperature * 100; //remove comma
  int int_pressure = pressure * 10; //remove comma
  int int_hum = hum * 100;
  int int_val_0 = val_0 * f * 100;
  int int_val_1 = val_1 * f * 100;
  int int_val_2 = val_2 * f * 100;
  int int_val_3 = val_3 * f * 100;



  appDataSize = 14; // El tamaño del dato que envío es de 4 bytes. Hay que tener en cuenta que 2 bytes son 2^16 bits = 65.536 bits 
  appData[0] = int_temp >> 8; // byte alto 
  appData[1] = int_temp; // byte bajo
  appData[2] = int_pressure >> 8; // byte alto
  appData[3] = int_pressure; // byte bajo
  appData[4] = int_hum >> 8;
  appData[5] = int_hum;
  appData[6] = int_val_0 >> 8;
  appData[7] = int_val_0;
  appData[8] = int_val_1 >> 8;
  appData[9] = int_val_1;
  appData[10] = int_val_2 >> 8;
  appData[11] = int_val_2;
  appData[12] = int_val_3 >> 8;
  appData[13] = int_val_3;
  
 /* En esta parte vemos lo que tenemos que poner en ttn al decodificar el número que nos llega */
//  function decodeUplink(input) {
  
//     var temp = input.bytes[0] << 8 | input.bytes[1];
//     var hum = input.bytes[2] << 8 | input.bytes[3];
  
//     return {
//       data: {
//         temperature: temp/100,
//         humidity: hum/10
//       }
//     };

/**************************************************************************************************************************************************************/

}

RTC_DATA_ATTR bool firstrun = true;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin(41,42);
  ADS.begin();

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
    if (! aht.begin(&Wire1)) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }


  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  if(firstrun)
  {
    LoRaWAN.displayMcuInit();
    firstrun = false;
  }

}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_sDEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipenID();
#endif
			LoRaWAN.init(loraWanClass,loraWanRegion);
			//both set join DR and DR when ADR off 
			LoRaWAN.setDefaultDR(3);
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.displayJoining();
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
			LoRaWAN.displaySending();
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.displayAck();
			LoRaWAN.sleep(loraWanClass);
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}