/*
  Processor - Atmega328p 5v, 16MHz
*/

#include "RF24.h"
#include "BME280.h"
#include "lowpower.h"
#include "Prescaler.h"

#include "Adafruit_GFX.h"
#include "Adafruit_PCD8544.h"

#define BME_SCK 11//2 /*SCL*/
#define BME_MISO 3 /*SDO*/
#define BME_MOSI 4 /*SDA/SDI*/
#define BME_CS 5 /*CSB??*/

#include <stdio.h> // for function sprintf

typedef struct
{
  int16_t temp;       // 10
  uint16_t humid;     // 7
  float pressure;
  uint16_t voltage;   // 10
  uint8_t counter;
} __attribute__((__packed__)) ClimatReading;

Adafruit_BME280<BME_MISO, BME_MOSI, BME_SCK, BME_CS> _bme;
//Adafruit_PCD8544 _display(7, 6, 5, 4, 3);

RF24 _radio (9, 10);
byte _addresses[][6] = {"1Node", "2Node"};
ClimatReading _data;

uint8_t _packetCounter;

void setup() {
  setClockPrescaler(CLOCK_PRESCALER_32);
  Serial.begin(1200 * getClockDivisionFactor());
  Serial.println(F("boot"));
  Serial.flush();
  auto status = _bme.begin(0x76);
  if (!status) {
    trueDelay(10);
    Serial.println(F("sensor error"));
    while (1);
  }

  Serial.println(F("boot ok, sensor init done"));
  Serial.flush();

  //Serial.end();

  _radio.begin();  // Start up the physical nRF24L01 Radio
  _radio.setChannel(108);  // Above most Wifi Channels
  _radio.setPALevel(RF24_PA_HIGH);
  _radio.setPayloadSize(sizeof(_data));
  _radio.openWritingPipe( _addresses[0]);
  _radio.openReadingPipe(1, _addresses[1]);

  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
  //ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS1);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
}

byte _intervalSelector = 3;

void loop() {

  //_display.begin();
  //_display.setContrast(50);
  
  auto state = digitalRead(BME_MOSI);
  digitalWrite(BME_MOSI, HIGH);

  switch (_intervalSelector)
  {
    case 1:
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
      break;
    case 2:
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      break;
    case 3:
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
    case 4:
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      break;
    case 5:
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
      break;
  }


  digitalWrite(BME_MOSI, state);
  printValues();
}

byte command[sizeof(_data)];

void setInterval(byte cmd);

void printValues() {

  Serial.print(F("reading..."));
  auto ms1 = micros();
  auto datata = _bme.readAll();
  _data.temp = datata.temp;
  _data.humid = datata.humid;
  _data.pressure = datata.pressure;

  auto ms2 = micros();
  auto dif = ms2 - ms1;
  Serial.print(" done in ");
  Serial.print(dif);  
  Serial.println(" us");
  
  Serial.print("t ");
  Serial.print(_data.temp);
  Serial.print("; h ");
  Serial.print(_data.humid);
  Serial.print("; p ");
  Serial.println(_data.pressure);
  
  Serial.flush();

  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  uint16_t res = ADC;
  _data.voltage = 108000 / res;
  _data.counter = _packetCounter++;

  _radio.write( &_data, sizeof(_data) );
  _radio.startListening();
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  if (_radio.available())
  {
    _radio.read(&command, sizeof(_data));
    byte cmd = command[0];
    setInterval(cmd);
  }
  _radio.stopListening();

  Serial.print("voltage ");
  Serial.print(1080000 / res);
  Serial.println(" mV");
  Serial.flush();
}

void setInterval(byte cmd)
{
  if (cmd >= 1 && cmd <= 5) {
    _intervalSelector = cmd;
  }
  else
  {
    _intervalSelector = 1;
  }
}
