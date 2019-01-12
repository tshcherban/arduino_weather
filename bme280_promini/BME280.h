#ifndef __BME280_H__
#define __BME280_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "dio\DigitalPin.h"

#include "Prescaler.h"


static inline __attribute__((always_inline))
void delayw(unsigned long ms)
{
  trueDelay(ms);
}

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BME280_ADDRESS                (0x77)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum
{
  BME280_REGISTER_DIG_T1              = 0x88,
  BME280_REGISTER_DIG_T2              = 0x8A,
  BME280_REGISTER_DIG_T3              = 0x8C,

  BME280_REGISTER_DIG_P1              = 0x8E,
  BME280_REGISTER_DIG_P2              = 0x90,
  BME280_REGISTER_DIG_P3              = 0x92,
  BME280_REGISTER_DIG_P4              = 0x94,
  BME280_REGISTER_DIG_P5              = 0x96,
  BME280_REGISTER_DIG_P6              = 0x98,
  BME280_REGISTER_DIG_P7              = 0x9A,
  BME280_REGISTER_DIG_P8              = 0x9C,
  BME280_REGISTER_DIG_P9              = 0x9E,

  BME280_REGISTER_DIG_H1              = 0xA1,
  BME280_REGISTER_DIG_H2              = 0xE1,
  BME280_REGISTER_DIG_H3              = 0xE3,
  BME280_REGISTER_DIG_H4              = 0xE4,
  BME280_REGISTER_DIG_H5              = 0xE5,
  BME280_REGISTER_DIG_H6              = 0xE7,

  BME280_REGISTER_CHIPID             = 0xD0,
  BME280_REGISTER_VERSION            = 0xD1,
  BME280_REGISTER_SOFTRESET          = 0xE0,

  BME280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

  BME280_REGISTER_CONTROLHUMID       = 0xF2,
  BME280_REGISTER_STATUS             = 0XF3,
  BME280_REGISTER_CONTROL            = 0xF4,
  BME280_REGISTER_CONFIG             = 0xF5,
  BME280_REGISTER_PRESSUREDATA       = 0xF7,
  BME280_REGISTER_TEMPDATA           = 0xFA,
  BME280_REGISTER_HUMIDDATA          = 0xFD
};

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
typedef struct
{
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
} bme280_calib_data;
/*=========================================================================*/

/*
  class Adafruit_BME280_Unified : public Adafruit_Sensor
  {
  public:
    Adafruit_BME280_Unified(int32_t sensorID = -1);

    bool  begin(uint8_t addr = BME280_ADDRESS);
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    void  getEvent(sensors_event_t*);
    void  getSensor(sensor_t*);

  private:
    uint8_t   _i2c_addr;
    int32_t   _sensorID;
  };

*/

typedef struct
{
  int16_t temp;
  uint16_t humid;
  float pressure;
} __attribute__((__packed__)) BME280_Data;

class Adafruit_BME280 {
  public:
    enum sensor_sampling {
        SAMPLING_NONE = 0b000,
        SAMPLING_X1   = 0b001,
        SAMPLING_X2   = 0b010,
        SAMPLING_X4   = 0b011,
        SAMPLING_X8   = 0b100,
        SAMPLING_X16  = 0b101
      };

      enum sensor_mode {
      MODE_SLEEP  = 0b00,
      MODE_FORCED = 0b01,
      MODE_NORMAL = 0b11
    };

    enum sensor_filter {
      FILTER_OFF = 0b000,
      FILTER_X2  = 0b001,
      FILTER_X4  = 0b010,
      FILTER_X8  = 0b011,
      FILTER_X16 = 0b100
    };

    // standby durations in ms
    enum standby_duration {
      STANDBY_MS_0_5  = 0b000,
      STANDBY_MS_10   = 0b110,
      STANDBY_MS_20   = 0b111,
      STANDBY_MS_62_5 = 0b001,
      STANDBY_MS_125  = 0b010,
      STANDBY_MS_250  = 0b011,
      STANDBY_MS_500  = 0b100,
      STANDBY_MS_1000 = 0b101
    };
};

template<uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin, uint8_t CsPin>
class Adafruit_BME280_T : Adafruit_BME280{
  public:
    Adafruit_BME280_T() {};

    bool begin(void)
    {
      _i2caddr = BME280_ADDRESS;
      return init();
    }

    bool begin(uint8_t addr)
    {
      _i2caddr = addr;
      return init();
    }
    bool init() {
      _pinCs.config(OUTPUT, HIGH);
      //digitalWrite(_cs, HIGH);
      //pinMode(_cs, OUTPUT);

      // software SPI
      _pinSck.config(OUTPUT);
      _pinMosi.config(OUTPUT);
      _pinMiso.config(INPUT);
      //pinMode(_sck, OUTPUT);
      //pinMode(_mosi, OUTPUT);
      //pinMode(_miso, INPUT);

      // check if sensor, i.e. the chip ID is correct
      if (read8(BME280_REGISTER_CHIPID) != 0x60)
        return false;

      // reset the device using soft-reset
      // this makes sure the IIR is off, etc.
      write8(BME280_REGISTER_SOFTRESET, 0xB6);

      // wait for chip to wake up.
      delayw(300);

      // if chip is still reading calibration, delay
      while (isReadingCalibration())
        delayw(100);

      readCoefficients(); // read trimming parameters, see DS 4.2.2

      setSampling(); // use defaults

      delayw(100);

      return true;
    }

    void setSampling(sensor_mode mode              = MODE_NORMAL,
                     sensor_sampling tempSampling  = SAMPLING_X16,
                     sensor_sampling pressSampling = SAMPLING_X16,
                     sensor_sampling humSampling   = SAMPLING_X16,
                     sensor_filter filter          = FILTER_OFF,
                     standby_duration duration     = STANDBY_MS_0_5
                    )
    {
      _measReg.mode     = mode;
      _measReg.osrs_t   = tempSampling;
      _measReg.osrs_p   = pressSampling;


      _humReg.osrs_h    = humSampling;
      _configReg.filter = filter;
      _configReg.t_sb   = duration;


      // you must make sure to also set REGISTER_CONTROL after setting the
      // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
      write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
      write8(BME280_REGISTER_CONFIG, _configReg.get());
      write8(BME280_REGISTER_CONTROL, _measReg.get());
    }

    void takeForcedMeasurement()
    {
      // If we are in forced mode, the BME sensor goes back to sleep after each
      // measurement and we need to set it to forced mode once at this point, so
      // it will take the next measurement and then return to sleep again.
      // In normal mode simply does new measurements periodically.
      if (_measReg.mode == MODE_FORCED) {
        // set to forced mode, i.e. "take next measurement"
        write8(BME280_REGISTER_CONTROL, _measReg.get());
        // wait until measurement has been completed, otherwise we would read
        // the values from the last measurement
        while (read8(BME280_REGISTER_STATUS) & 0x08)
          delayw(1);
      }
    }
    float readTemperature(void)
    {
      int32_t var1, var2;

      int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
      if (adc_T == 0x800000) // value in case temp measurement was disabled
        return NAN;
      adc_T >>= 4;

      var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
              ((int32_t)_bme280_calib.dig_T2)) >> 11;

      var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
                ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
              ((int32_t)_bme280_calib.dig_T3)) >> 14;

      t_fine = var1 + var2;

      float T = (t_fine * 5 + 128) >> 8;
      return T / 100;
    }
    int16_t readTemperatureInt16(void)
    {
      int32_t var1, var2;

      int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
      if (adc_T == 0x800000) // value in case temp measurement was disabled
        return -100;

      adc_T >>= 4;

      var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
              ((int32_t)_bme280_calib.dig_T2)) >> 11;

      var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
                ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
              ((int32_t)_bme280_calib.dig_T3)) >> 14;

      t_fine = var1 + var2;

      int32_t T = (t_fine * 5 + 128) >> 8;
      return (int16_t)(T / 10);
    }

    int16_t convertTemperature(int32_t adc_T)
    {
      int32_t var1, var2;

      if (adc_T == 0x800000) // value in case temp measurement was disabled
        return -100;

      adc_T >>= 4;

      var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
              ((int32_t)_bme280_calib.dig_T2)) >> 11;

      var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
                ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
              ((int32_t)_bme280_calib.dig_T3)) >> 14;

      t_fine = var1 + var2;

      int32_t T = (t_fine * 5 + 128) >> 8;
      return (int16_t)(T / 10);
    }

    float readPressure(void)
    {
      int64_t var1, var2, p;

      readTemperatureInt16(); // must be done first to get t_fine

      int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
      if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;

      adc_P >>= 4;

      var1 = ((int64_t)t_fine) - 128000;
      var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
      var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
      var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
      var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
             ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
      var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1) >> 33;

      if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
      }
      p = 1048576 - adc_P;
      p = (((p << 31) - var2) * 3125) / var1;
      var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
      var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

      p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
      return (float)p / 256;
    }
    float convertPressure(int32_t adc_P)
    {
      if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;

      int64_t var1, var2, p;

      adc_P >>= 4;

      var1 = ((int64_t)t_fine) - 128000;
      var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
      var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
      var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
      var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
             ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
      var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1) >> 33;

      if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
      }
      p = 1048576 - adc_P;
      p = (((p << 31) - var2) * 3125) / var1;
      var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
      var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

      p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
      return (float)p / 256;
    }
    float readHumidity(void)
    {
      readTemperatureInt16(); // must be done first to get t_fine

      int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
      if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;

      int32_t v_x1_u32r;

      v_x1_u32r = (t_fine - ((int32_t)76800));

      v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                      (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                   (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                        (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                      ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

      v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                 ((int32_t)_bme280_calib.dig_H1)) >> 4));

      v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
      v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
      float h = (v_x1_u32r >> 12);
      return  h / 1024.0;
    }
    uint8_t readHumidityUint8(void)
    {
      readTemperatureInt16(); // must be done first to get t_fine

      int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
      if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return 0;

      int32_t v_x1_u32r;

      v_x1_u32r = (t_fine - ((int32_t)76800));

      v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                      (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                   (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                        (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                      ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

      v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                 ((int32_t)_bme280_calib.dig_H1)) >> 4));

      v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
      v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
      return (uint8_t)(v_x1_u32r >> 22);
    }
    uint16_t convertHumidity(int32_t adc_H)
    {
      if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return 0;

      int32_t v_x1_u32r;

      v_x1_u32r = (t_fine - ((int32_t)76800));

      v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                      (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                   (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                        (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                      ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

      v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                 ((int32_t)_bme280_calib.dig_H1)) >> 4));

      v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
      v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

      return (uint16_t)(((v_x1_u32r >> 12) * 10) >> 10);
    }

    BME280_Data readAll(void)
    {
      BME280_Data data;

      _pinCs.low();
      // read command, bit 7 high (start burst read)
      spixfer(BME280_REGISTER_PRESSUREDATA | 0x80);

      int32_t adc_P = read24Burst(); // BME280_REGISTER_PRESSUREDATA  0xF7-0xF9
      int32_t adc_T = read24Burst(); // BME280_REGISTER_TEMPDATA      0xFA-0xFC
      int32_t adc_H = read16Burst(); // BME280_REGISTER_HUMIDDATA     0xFD-0xFE
      _pinCs.high();
      
      data.temp = convertTemperature(adc_T);
      data.humid = convertHumidity(adc_H);
      data.pressure = convertPressure(adc_P);
      
      return data;
    }

  private:
    void readCoefficients(void)
    {
      _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
      _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
      _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

      _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
      _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
      _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
      _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
      _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
      _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
      _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
      _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
      _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

      _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
      _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
      _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
      _bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
      _bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5 + 1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
      _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
    }

    bool isReadingCalibration(void)
    {
      uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

      return (rStatus & (1 << 0)) != 0;
    }

    /*uint8_t Adafruit_BME280::spixfer(uint8_t x) {
      uint8_t reply = 0;
      for (int i = 7; i >= 0; i--) {
        reply <<= 1;
        digitalWrite(_sck, LOW);
        digitalWrite(_mosi, x & (1 << i));
        digitalWrite(_sck, HIGH);
        if (digitalRead(_miso))
          reply |= 1;
      }
      return reply;
      }*/

    //inline __attribute__((always_inline))
    uint8_t spixfer(uint8_t x) {
      uint8_t reply = 0;
      reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 7)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;

      reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 6)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;

      reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 5)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;

      reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 4)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;

      reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 3)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;

              reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 2)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;

              reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 1)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;

              reply <<= 1;
      _pinSck.low();
      _pinMosi.write((x & (1 << 0)));
      _pinSck.high();
      if (_pinMiso.read())
        reply |= 1;
      
      return reply;
    }

    void write8(byte reg, byte value)
    {
      _pinCs.low();
      spixfer(reg & ~0x80); // write, bit 7 low
      spixfer(value);
      _pinCs.high();
    }
    uint8_t   read8(byte reg)
    {
      uint8_t value;

      _pinCs.low();
      spixfer(reg | 0x80); // read, bit 7 high
      value = spixfer(0);
      _pinCs.high();

      return value;
    }
    uint16_t  read16(byte reg)
    {
      uint16_t value;

      _pinCs.low();
      spixfer(reg | 0x80); // read, bit 7 high
      value = (spixfer(0) << 8) | spixfer(0);
      _pinCs.high();

      return value;
    }
    uint16_t  read16Burst()
    {
      uint16_t value;

      value = (spixfer(0) << 8) | spixfer(0);

      return value;
    }
    uint16_t  read16t(byte reg)
    {
      uint16_t value;

      spixfer(reg | 0x80); // read, bit 7 high
      value = (spixfer(0) << 8) | spixfer(0);

      return value;
    }
    uint32_t  read24(byte reg)
    {
      uint32_t value;

      _pinCs.write(LOW);
      //digitalWriteW(_cs, LOW);
      spixfer(reg | 0x80); // read, bit 7 high

      value = spixfer(0);
      value <<= 8;
      value |= spixfer(0);
      value <<= 8;
      value |= spixfer(0);

      _pinCs.write(HIGH);
      //digitalWriteW(_cs, HIGH);

      return value;
    }

    uint32_t  read24Burst()
    {
      uint32_t value;

      value = spixfer(0);
      value <<= 8;
      value |= spixfer(0);
      value <<= 8;
      value |= spixfer(0);

      return value;
    }
    uint32_t  read24t(byte reg)
    {
      uint32_t value;

      spixfer(reg | 0x80); // read, bit 7 high

      value = spixfer(0);
      value <<= 8;
      value |= spixfer(0);
      value <<= 8;
      value |= spixfer(0);

      return value;
    }
    int16_t   readS16(byte reg)
    {
      return (int16_t)read16(reg);
    }
    uint16_t  read16_LE(byte reg) // little endian
    {
      uint16_t temp = read16(reg);
      return (temp >> 8) | (temp << 8);
    }
    int16_t   readS16_LE(byte reg) // little endian
    {
      return (int16_t)read16_LE(reg);
    }

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t   t_fine;

    DigitalPin<CsPin> _pinCs;
    DigitalPin<MisoPin> _pinMiso;
    DigitalPin<MosiPin> _pinMosi;
    DigitalPin<SckPin> _pinSck;

    bme280_calib_data _bme280_calib;

    // The config register
    struct config {
      // inactive duration (standby time) in normal mode
      // 000 = 0.5 ms
      // 001 = 62.5 ms
      // 010 = 125 ms
      // 011 = 250 ms
      // 100 = 500 ms
      // 101 = 1000 ms
      // 110 = 10 ms
      // 111 = 20 ms
      unsigned int t_sb : 3;

      // filter settings
      // 000 = filter off
      // 001 = 2x filter
      // 010 = 4x filter
      // 011 = 8x filter
      // 100 and above = 16x filter
      unsigned int filter : 3;

      // unused - don't set
      unsigned int none : 1;
      unsigned int spi3w_en : 1;

      unsigned int get() {
        return (t_sb << 5) | (filter << 3) | spi3w_en;
      }
    };
    config _configReg;


    // The ctrl_meas register
    struct ctrl_meas {
      // temperature oversampling
      // 000 = skipped
      // 001 = x1
      // 010 = x2
      // 011 = x4
      // 100 = x8
      // 101 and above = x16
      unsigned int osrs_t : 3;

      // pressure oversampling
      // 000 = skipped
      // 001 = x1
      // 010 = x2
      // 011 = x4
      // 100 = x8
      // 101 and above = x16
      unsigned int osrs_p : 3;

      // device mode
      // 00       = sleep
      // 01 or 10 = forced
      // 11       = normal
      unsigned int mode : 2;

      unsigned int get() {
        return (osrs_t << 5) | (osrs_p << 3) | mode;
      }
    };
    ctrl_meas _measReg;


    // The ctrl_hum register
    struct ctrl_hum {
      // unused - don't set
      unsigned int none : 5;

      // pressure oversampling
      // 000 = skipped
      // 001 = x1
      // 010 = x2
      // 011 = x4
      // 100 = x8
      // 101 and above = x16
      unsigned int osrs_h : 3;

      unsigned int get() {
        return (osrs_h);
      }
    };
    ctrl_hum _humReg;
};

#endif
