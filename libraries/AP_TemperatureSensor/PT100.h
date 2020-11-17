/*
 * I2C driver for Measurement Specialties MEAS PT100 digital temperature sensor
 */
//jslee_1007: add

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

//#define TSYS01_ADDR 0x77  // not used
#define PT100_I2C_HUB 	0x70
#define PT100_ADDR 		0x28
#define PT100_DFR0558 	0x10

class I2C_Hub {
public:
    bool init(void);
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    bool select(char i);
};



class PT100 {
public:

	I2C_Hub m_hub;
    bool init(void);
    float temperature(void) { return _temperature; } // temperature in degrees C
    float temperature_1(void) { return _temperature_1; } // temperature in degrees C
    bool healthy(void) { // do we have a valid temperature reading?
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        return true;
#endif
        return _healthy;
    }

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    float _temperature = 42.42; // degrees C
#else
    float _temperature; // degrees C
    float _temperature_1; // degrees C
#endif
    bool _healthy; // we have a valid temperature reading to report
    bool m_toggle; // we have a valid temperature reading to report
    uint16_t _k[5]; // internal calibration for temperature calculation
    bool _reset(void); // reset device
    bool _read_prom(void); // read (relevant) internal calibration registers into _k
    bool _convert(void); // begin an ADC conversion (min:7.40ms typ:8.22ms max:9.04ms)

    uint32_t _read_adc(uint8_t* buffer);
    uint16_t _read_prom_word(uint8_t word);
    void _timer(void); // update the temperature, called at 20Hz
    void _calculate(uint32_t adc); // calculate temperature using adc reading and internal calibration
};



