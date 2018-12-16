/*
  Processor - Atmega328p 5v, 16MHz
*/

#pragma region includes/defines

#include "RF24.h"
#include "BME280.h"
#include "lowpower.h"
#include "Prescaler.h"

#include "Adafruit_GFX.h"
#include "Adafruit_PCD8544.h"

#include <stdio.h> // for function sprintf

#define BME_SCK 11//2 /*SCL*/
#define BME_MISO 3 /*SDO*/
#define BME_MOSI 4 /*SDA/SDI*/
#define BME_CS 5 /*CSB??*/

#define RF_CE 9
#define RF_CS 10

#pragma endregion


typedef struct
{
  int16_t temp;       // 10
  uint16_t humid;     // 7
  float pressure;
  uint16_t voltage;   // 10
  uint8_t counter;
} __attribute__((__packed__)) ClimatReading;


const uint32_t _internalRef = 1080000;

Adafruit_BME280_T<BME_MISO, BME_MOSI, BME_SCK, BME_CS> _bme;
RF24 _radio (RF_CE, RF_CS);
//Adafruit_PCD8544 _display(7, 6, 5, 4, 3);
byte _addresses[][6] = {"1Node", "2Node"};
ClimatReading _dataToSend;
BME280_Data _readData;
byte _command[sizeof(_dataToSend)];
uint8_t _packetCounter;
byte _intervalSelector = 3;
float _adcVal;


inline __attribute__((always_inline))
void waitAdc()
{
  while (ADCSRA & (1 << ADSC));
}

inline __attribute__((always_inline))
void startAdc() {
  ADCSRA |= (1 << ADSC);
}

void printValues();

void setup() {
  setClockPrescaler(CLOCK_PRESCALER_32);
  uint16_t divFactor = 32;
  Serial.begin(1200 * divFactor);
  Serial.println(F("boot"));
  Serial.flush();
  auto status = _bme.begin(0x76);
  if (!status) {
    trueDelay(10);
    Serial.println(F("sensor error"));
    while (1);
  }

  Serial.println(F("boot ok, sensor init done"));
  Serial.print(F("ckdiv "));
  Serial.println(divFactor);
  Serial.flush();

  //Serial.end();

  _radio.begin();  // Start up the physical nRF24L01 Radio
  _radio.setChannel(108);  // Above most Wifi Channels
  _radio.setPALevel(RF24_PA_HIGH);
  _radio.setPayloadSize(sizeof(_dataToSend));
  _radio.openWritingPipe( _addresses[0]);
  _radio.openReadingPipe(1, _addresses[1]);
  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
  //ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS1);
  startAdc();

  _bme.setSampling(Adafruit_BME280::sensor_mode::MODE_NORMAL,
                  /*tempSampling  */   Adafruit_BME280::sensor_sampling::SAMPLING_X16,
                  /*pressSampling */   Adafruit_BME280::sensor_sampling::SAMPLING_X16,
                  /*humSampling   */   Adafruit_BME280::sensor_sampling::SAMPLING_X16,
                  /*filter        */   Adafruit_BME280::sensor_filter::FILTER_X4,
                  /*standby_duration*/ Adafruit_BME280::standby_duration::STANDBY_MS_250);

  waitAdc();
  startAdc();
  waitAdc();
  startAdc();
  waitAdc();

  _adcVal = _internalRef / ADC;

  PRR |= 0b11001100;
}

void loop() {
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
  Serial.print(PRR, BIN);
  Serial.println();
  Serial.flush();
}



void printValues() {
  startAdc();

  auto ms1 = trueMicros();
  _readData = _bme.readAll();
  _dataToSend.temp = _readData.temp;
  _dataToSend.humid = _readData.humid;
  _dataToSend.pressure = _readData.pressure;

  waitAdc();
  startAdc();
  waitAdc();
  uint32_t val = _internalRef / ADC;

  _adcVal += (val - _adcVal) * 0.1f;

  _dataToSend.voltage = (uint16_t) _adcVal;
  _dataToSend.counter = _packetCounter++;

  _radio.write( &_dataToSend, sizeof(_dataToSend) );

  auto ms2 = trueMicros();
  auto dif = ms2 - ms1;
  Serial.print("done in ");
  Serial.print(dif);
  Serial.println(" us");

  Serial.print("t ");
  Serial.print(_dataToSend.temp);
  Serial.print("; h ");
  Serial.print(_dataToSend.humid);
  Serial.print("; p ");
  Serial.print(_dataToSend.pressure);
  Serial.print("; v ");
  Serial.println(_dataToSend.voltage);
  Serial.println(val);
  Serial.println(_adcVal);
  Serial.flush();

  // _radio.write( &_dataToSend, sizeof(_dataToSend) );
  // _radio.startListening();
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  // if (_radio.available())
  // {
  //   _radio.read(&_command, sizeof(_dataToSend));
  //   byte cmd = _command[0];
  //   setInterval(cmd);
  // }
  // _radio.stopListening();
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
