/*
 * jslee_20117: Add
 * I2C driver for Measurement Specialties MEAS TSYS01 digital temperature sensor
 */

#include "PT100.h"

#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#define	DFR0558	1	// OK
#define	PT100_1	2	// OK
#define	PT100_2	3

#define	OPTION PT100_2

extern const AP_HAL::HAL &hal;

static const uint8_t PT100_CMD_RESET       = 0x1E;
static const uint8_t PT100_CMD_READ_PROM   = 0xA0;
static const uint8_t PT100_CMD_CONVERT     = 0x40;
static const uint8_t PT100_CMD_READ_ADC    = 0x00;


PT100* gPT100;
float gTemper_1=0;
float gTemper_2=0;


bool PT100::init()
{

	m_toggle = false;
	gPT100 = this;

//	gTemper_1 = 1001;
//	gTemper_2 = 2001;

#if OPTION == DFR0558

    _dev = std::move(hal.i2c_mgr->get_device(0, PT100_DFR0558,100000));

    if (!_dev) {
    	_temperature = 100;
        printf("PT100 device is null!");
        return false;
    }

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
    	_temperature = 200;
        AP_HAL::panic("PANIC: PT100: failed to take serial semaphore for init");
    }

    _dev->set_retries(10);
    hal.scheduler->delay(4);
    // lower retries for run
    _dev->set_retries(3);
    _dev->get_semaphore()->give();

    /* Request 20Hz update */
    // Max conversion time is 9.04 ms
    _dev->register_periodic_callback(500 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&PT100::_timer, void));
#else

	#if	OPTION == PT100_1 // chackable one engine
		_dev = std::move(hal.i2c_mgr->get_device(0, PT100_ADDR,100000));

		 if (!_dev) {
			_temperature = 100;
			 printf("PT100 device is null!");
			 return false;
		 }

		 if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
			_temperature = 200;
			 AP_HAL::panic("PANIC: PT100: failed to take serial semaphore for init");
		 }

		 _dev->set_retries(10);
		 hal.scheduler->delay(4);
		 // lower retries for run
		 _dev->set_retries(3);
		 _dev->get_semaphore()->give();

		 /* Request 20Hz update */
		 // Max conversion time is 9.04 ms
		 _dev->register_periodic_callback(500 * AP_USEC_PER_MSEC,
										  FUNCTOR_BIND_MEMBER(&PT100::_timer, void));
	#else
        m_hub.init();

        _dev = std::move(hal.i2c_mgr->get_device(0, PT100_ADDR,100000));

         if (!_dev) {
            _temperature = 100;
             printf("PT100 device is null!");
             return false;
         }

         if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            _temperature = 200;
             AP_HAL::panic("PANIC: PT100: failed to take serial semaphore for init");
         }

         _dev->set_retries(10);
         hal.scheduler->delay(4);
         // lower retries for run
         _dev->set_retries(3);
         _dev->get_semaphore()->give();

         /* Request 20Hz update */
         // Max conversion time is 9.04 ms
         _dev->register_periodic_callback(500 * AP_USEC_PER_MSEC,
                                          FUNCTOR_BIND_MEMBER(&PT100::_timer, void));

	#endif

#endif

    return true;
}

bool PT100::_reset()
{
	return true;
}


bool PT100::_convert()
{
    return _dev->transfer(&PT100_CMD_CONVERT, 1, nullptr, 0);
}

uint32_t PT100::_read_adc(uint8_t* buffer)
{
    uint8_t val[4];

#if OPTION == DFR0558
    uint8_t addr;

    addr = 0;
	if (!_dev->transfer(&addr, 1, &val[0], 1)) {
		return 0;
	}
	buffer[0] = val[0];

    addr = 1;
	if (!_dev->transfer(&addr, 1,  &val[1], 1)) {
		return 0;
	}
	buffer[1] = val[1];

  //  addr = 2;
//	if (!_dev->transfer(&addr, 1,  &val[2], 1)) {
//		return 0;
//	}
//	buffer[2] = val[2];

    addr = 3;
	if (!_dev->transfer(&addr, 1,  &val[3], 1)) {
		return 0;
	}
	buffer[3] = val[3];

	return (val[0] << 16) | (val[1] << 8) | val[2];

#else

	#if	 OPTION == PT100_1
		if (!_dev->transfer(nullptr, 0, val, 3)) {
			return 0;
		}

		return (val[0] << 16) | (val[1] << 8) | val[2];
	#else
		if (!_dev->transfer(nullptr, 0, val, 3)) {
			return 0;
		}

		return (val[0] << 16) | (val[1] << 8) | val[2];

	#endif

#endif

}

void PT100::_timer(void)
{

#if OPTION == DFR0558

	uint8_t adcValue[5];
	_read_adc(adcValue);

	if(adcValue[3]&0x7){
		_temperature = 0;
		_healthy = false;
	}
	else
	{//adcValue[0]; 가 상위 바이트이고, [1] 하위 바이트이다.
		_healthy = true;

		if(adcValue[0] & 0x80){
			adcValue[0] = 0xff - adcValue[0];
			adcValue[1] = 0xff - adcValue[1];
			_temperature =-((((adcValue[0] << 8)|(adcValue[1] & 0xfc)) >> 2) + 1) * 0.25;

//			gTemper_1 = adcValue[0];
//			gTemper_2 = adcValue[1];

		}
		else
		{
			_temperature =  (((adcValue[0] << 8 )| (adcValue[1] & 0xfc)) >> 2)*0.25;

//			gTemper_1 = adcValue[0];
//			gTemper_2 = adcValue[1];
		}
	}
#else
	uint32_t adc;
	float adc16;

	#if OPTION == PT100_1
		adc = _read_adc(nullptr);
		_healthy = adc != 0;

		if (_healthy) {
			adc16 = adc/100;
			_temperature = adc16*100;
		} else {
			_temperature = 0;
		}

	#else

		if(m_toggle == true)
		{
			adc = _read_adc(nullptr);
			_healthy = adc != 0;

			if (_healthy) {
				adc16 = adc/100;        // 소수점 아래 버릴 목적으로 추측
				_temperature = adc16*100;

				// jslee_0821: Modify. xxx00.00℃ 출력
				//gTemper_1 = adc16*100;
				gTemper_1 = adc16;

			} else {
				_temperature = 0;
				gTemper_1 = 654;
			}
			m_toggle = false;
			m_hub.select(7);
		}
		else
		{
			adc = _read_adc(nullptr);
			_healthy = adc != 0;
			if (_healthy) {
				adc16 = adc/100;        // 소수점 아래 버릴 목적으로 추측

				// jslee_0821: Modify. xxx00.00℃ 출력
				//gTemper_2 = adc16*100;
				gTemper_2 = adc16;

			} else {
				gTemper_2 = 987;
			}
			m_toggle = true;
			m_hub.select(2);
		}
	#endif

#endif

}

void PT100::_calculate(uint32_t adc)
{
    float adc16 = adc/100;
    _temperature = adc16*100;
}


bool I2C_Hub::init(void)
{
  _dev = std::move(hal.i2c_mgr->get_device(0, PT100_I2C_HUB,100000));
	if (!_dev) {
		return false;
	}

	if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
		AP_HAL::panic("PANIC: PT100: failed to take serial semaphore for init");
	}

	_dev->set_retries(10);
	hal.scheduler->delay(4);
	// lower retries for run
	_dev->set_retries(3);
	_dev->get_semaphore()->give();
	/* Request 20Hz update */
	// Max conversion time is 9.04 ms

	return true;
}

bool I2C_Hub::select(char i)
{
	uint8_t val;
	val = 1 << i;

	if (!_dev->transfer(&val, 1, nullptr, 0)) {
		return false;
	}
	return true;
}


