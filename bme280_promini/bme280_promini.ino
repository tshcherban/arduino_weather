/*

  Processos - Atmega328p 5v, 16MHz

*/

#include "RF24.h"
#include "BME280.h"
#include "lowpower.h"
#include "Prescaler.h"

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
} __attribute__((__packed__)) ClimatReading;

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
RF24 myRadio (9, 10);
byte addresses[][6] = {"1Node", "2Node"};
ClimatReading data;

void setup() {
  setClockPrescaler(CLOCK_PRESCALER_32);
  Serial.begin(1200 * getClockDivisionFactor());
  Serial.println(F("boot"));
  Serial.flush();
  auto status = bme.begin(0x76);
  if (!status) {
    trueDelay(10);
    Serial.println(F("sensor error"));
    while (1);
  }

  Serial.println(F("boot ok, sensor init done"));
  Serial.flush();

  //Serial.end();

  myRadio.begin();  // Start up the physical nRF24L01 Radio
  myRadio.setChannel(108);  // Above most Wifi Channels
  myRadio.setPALevel(RF24_PA_HIGH);
  myRadio.setPayloadSize(sizeof(data));
  myRadio.openWritingPipe( addresses[0]);
  myRadio.openReadingPipe(1, addresses[1]);

  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
  //ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS1);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
}

byte interval = 3;

void loop() {
  auto state = digitalRead(BME_MOSI);
  digitalWrite(BME_MOSI, HIGH);

  switch (interval)
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

byte command[sizeof(data)];

void setInterval(byte cmd);

void printValues() {

  Serial.print(F("reading..."));
  auto ms1 = micros();
  auto datata = bme.readAll();
  //data.temp = bme.readTemperatureInt16();
  //data.humid = bme.readHumidityUint16Unsafe();
  //data.pressure = bme.readPressureUnsafe();
  data.temp = datata.temp;
  data.humid = datata.humid;
  data.pressure = datata.pressure;

  auto ms2 = micros();
  auto dif = ms2 - ms1;
  Serial.print(" done in ");
  Serial.print(dif);  
  Serial.println(" us");
  
  Serial.print("t ");
  Serial.print(data.temp);
  Serial.print("; h ");
  Serial.print(data.humid);
  Serial.print("; p ");
  Serial.println(data.pressure);
  
  Serial.flush();

  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  uint16_t res = ADC;
  data.voltage = 108000 / res;

  myRadio.write( &data, sizeof(data) );
  myRadio.startListening();
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  if (myRadio.available())
  {
    myRadio.read(&command, sizeof(data));
    byte cmd = command[0];
    setInterval(cmd);
  }
  myRadio.stopListening();

  Serial.print("voltage ");
  Serial.print(1080000 / res);
  Serial.println(" mV");
  Serial.flush();
}

void setInterval(byte cmd)
{
  if (cmd >= 1 && cmd <= 5) {
    interval = cmd;
  }
  else
  {
    interval = 1;
  }
}
