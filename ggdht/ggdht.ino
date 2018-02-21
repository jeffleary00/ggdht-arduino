
/*********************************************************************
 Arduino code for custom Temperature/Humidity/CO2 sensor, based on
 Adafruit 32u4 Feather Bluefruit, with DHT22 and MH-Z19 sensors.

 Set "debug" to true to see serial messages. Must be set to false for production.

 Also, note long warmup delay required by MH-Z19 sensor!

 
 GATT INFO:
 
  OUR CUSTOM SERVICE UUID = B3F72C28-2618-4E2B-9075-1B17CCA4EC66
  
    TEMPERATURE GATT Characteristic UUID = E60A00E9-D6A9-430F-959C-872F07E64FCE
      value = 16bit signed int
      
    HUMIDITY GATT Characteristic UUID = 4DF3BB88-C7CB-47B5-B213-CEA3770DB9E8
      value = 16 bit unsigned int
    
    CO2 PPM GATT CharacteristicUUID = 6431AF8C-A5B4-47EB-BA73-B69495327E53
      value = 16 bit unsigned int


MIT License

Copyright (c) 2018 Jeff Leary

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*********************************************************************/


#include <Arduino.h>
#include <SPI.h>
#include <DHT.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEGatt.h"
#include "BluefruitConfig.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


#define DHTPIN 11   
#define DHTTYPE DHT22
#define SFW_UART_TX 9
#define SFW_UART_RX 10
#define SERVICE_UUID      {0xb3, 0xf7, 0x2c, 0x28, 0x26, 0x18, 0x4e, 0x2b, 0x90, 0x75, 0x1b, 0x17, 0xcc, 0xa4, 0xec, 0x66}
#define TEMPERATURE_UUID  {0xe6, 0x0a, 0x00, 0xe9, 0xd6, 0xa9, 0x43, 0x0f, 0x95, 0x9c, 0x87, 0x2f, 0x07, 0xe6, 0x4f, 0xce}
#define HUMIDITY_UUID     {0x4d, 0xf3, 0xbb, 0x88, 0xc7, 0xcb, 0x47, 0xb5, 0xb2, 0x13, 0xce, 0xa3, 0x77, 0x0d, 0xb9, 0xe8}
#define PPM_UUID          {0x64, 0x31, 0xaf, 0x8c, 0xa5, 0xb4, 0x47, 0xeb, 0xba, 0x73, 0xb6, 0x94, 0x95, 0x32, 0x7e, 0x53}
#define MSG_LEN 9
#define WARMUP_SECONDS 180


/* GLOBALS */
uint8_t enviroServiceId;
uint8_t tempMeasureCharId;
uint8_t humidMeasureCharId;
uint8_t ppmMeasureCharId;
int loop_delay = 20000;
bool debug = false;

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt gatt(ble);
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial sfwser(SFW_UART_RX, SFW_UART_TX);


void setup(void)
{
    boolean success;
    uint8_t my_svc[] = SERVICE_UUID;
    uint8_t my_temp[] = TEMPERATURE_UUID;
    uint8_t my_humid[] = HUMIDITY_UUID;
    uint8_t my_ppm[] = PPM_UUID;

    if (debug) 
    {
        Serial.begin(9600);
        while (!Serial);
        delay(500);
    }
    
    warmup();
    dht.begin();
    sfwser.begin(9600);

    if ( !ble.begin(VERBOSE_MODE) )
    {
        if (debug) { error(F("Couldn't find Bluefruit!")); }
    }
    
    if (! ble.factoryReset() )
    {
        if (debug) { error(F("Couldn't factory reset")); }
    }

    ble.echo(false);
    
    if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Greenery DHTCO2")) ) 
    {
        if (debug) { error(F("Could not set device name?")); }
    }

    
    /* Add our CUSTOM GATT Service */
    enviroServiceId = gatt.addService(my_svc);
    if (enviroServiceId == 0) 
    {
        if (debug) { error(F("Could not add Custom Environment Sensing service")); }
    }

    /* Add our GATT Characteristics */
    tempMeasureCharId = gatt.addCharacteristic(my_temp, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
    if (tempMeasureCharId == 0) {
        if (debug) { error(F("Could not add Temperature characteristic")); }
    }
    
    humidMeasureCharId = gatt.addCharacteristic(my_humid, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
    if (humidMeasureCharId == 0) {
        if (debug) { error(F("Could not add Humidity characteristic")); }
    }
    
    ppmMeasureCharId = gatt.addCharacteristic(my_ppm, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
    if (ppmMeasureCharId == 0) {
        if (debug) { error(F("Could not add CO2-PPM characteristic")); }
    }
    
    uint8_t advdata[] { 0x02, 0x01, 0x06, 0x05, 0x02, 0x00, 0x18};
    ble.setAdvData( advdata, sizeof(advdata) );
    ble.reset();
    if (debug) { Serial.println(); }
}


void loop(void)
{
    // fetch the sensor values
    float temp = dht.readTemperature();
    float humid = dht.readHumidity();
    uint32_t ppm = read_CO2();

    // convert values to 16bit short ints (signed and unsigned)
    int16_t nt = (int16_t) 10.0 * temp;
    uint16_t nh = (int16_t) 10.0 * humid;
    uint16_t np = (int16_t) ppm;
    
    // write values to GATT Charactersitics
    gatt.setChar(tempMeasureCharId, nt);
    gatt.setChar(humidMeasureCharId, nh);
    gatt.setChar(ppmMeasureCharId, np);
    
    delay(loop_delay);
}


void warmup()
{
    if (debug) { Serial.println( F("CO2 sensor warmup...") ); }
    
    int counter = 0;
    int timeout = WARMUP_SECONDS;
    int second = 1000;
    while (counter < timeout)
    {
        delay(second);
        counter ++;
    }
    if (debug) { Serial.println( F("Warmup complete") ); }
}


/*
 * Read CO2 ppm from software serial UART.
 * 
 * returns -1 on error
 */
uint16_t read_CO2()
{
    while (sfwser.read() != -1); 

    byte question[MSG_LEN] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    byte answer[MSG_LEN];
    
    sfwser.write(question, MSG_LEN); //request PPM CO2 

    // Then for 1 second listen for 9 bytes of data.
    sfwser.readBytes(answer, MSG_LEN);
   
    if (answer[0] != 0xFF) 
    {
        if (debug) { Serial.println("Wrong starting byte from co2 sensor! (should be FF)"); }
        return -1;
    }

    if (answer[1] != 0x86) 
    {
        if (debug) { Serial.println("Wrong command from co2 sensor! (should be 86)"); }
        return -1;
    }

    int responseHigh = (int) answer[2];
    int responseLow = (int) answer[3];
    uint16_t ppm = (256 * responseHigh) + responseLow;
    
    return ppm;
}


/*
 * print the bytes in the UART buffers from MH-Z19 sensor
 */
void print_cmd_buffer(byte *buf, unsigned int blen) 
{
    int i;
    for (i = 0; i <= blen; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(F(" "));
    }
    Serial.println(F(""));
}

void error(const __FlashStringHelper*err) {
    Serial.println(err);
    while (1);
}


