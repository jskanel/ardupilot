/*
 * Seeed_MCP9600.cpp
 * Driver for MCP9600
 *  
 * Copyright (c) 2018 Seeed Technology Co., Ltd.
 * Website    : www.seeed.cc
 * Author     : downey
 * Create Time: May 2018
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
// jslee_201116: Add

#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "mcp9600.h"


extern const AP_HAL::HAL &hal;

MCP9600* gPT100;
float gTemper_1=0;
float gTemper_2=0;


MCP9600::MCP9600(uint8_t IIC_ADDR)
{
    set_iic_addr(IIC_ADDR);
}


/**@brief set type of thermocouple.read version.
 * @param therm_type
 * @return 0 if successed.
 * */
err_t MCP9600::init(uint8_t therm_type)
{
    err_t ret=NO_ERROR;
    IIC_begin();

    // jslee_0813
#if 0
    ret=set_therm_type(therm_type);
    set_filt_coefficients(FILT_MID);
    set_cold_junc_resolution(COLD_JUNC_RESOLUTION_0_25);
    set_ADC_meas_resolution(ADC_14BIT_RESOLUTION);
    set_burst_mode_samp(BURST_32_SAMPLE);
    set_sensor_mode(NORMAL_OPERATION);
#else
	ret=set_therm_type(therm_type);
	set_filt_coefficients(FILT_MID);
	set_cold_junc_resolution(COLD_JUNC_RESOLUTION_0_25);
	set_ADC_meas_resolution(ADC_12BIT_RESOLUTION);
	set_burst_mode_samp(BURST_4_SAMPLE);
	set_sensor_mode(NORMAL_OPERATION);
#endif
    return ret;
}   




/**@brief read version.
 * @param ver.
 * @return 0 if successed.
 * */
err_t MCP9600::read_version(uint16_t *ver)
{
    if(IIC_read_16bit(VERSION_ID_REG_ADDR,ver))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief read hot-junction,the temperature result.
 * @param value: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_hot_junc(float *value)
{
    *value=0;
    uint16_t read_value=0;

    if( IIC_read_16bit(HOT_JUNCTION_REG_ADDR,&read_value) != NO_ERROR)
    {
        gTemper_1 = 1007;
        return ERROR_COMM;
    }


    if(read_value&0x8000)
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0-4096.0;
    }
    else
    {
        *value = read_value;//(read_value>>8)*16.0+(read_value&0x00ff)/16.0;
    }
    return NO_ERROR;
}


err_t MCP9600::read_hot_junc(uint16_t *value)
{
    *value=0;
    uint16_t read_value=0;

    if( IIC_read_16bit(HOT_JUNCTION_REG_ADDR,&read_value) != NO_ERROR)
    {
        gTemper_1 = 1007;
        return ERROR_COMM;
    }


    if(read_value&0x8000)
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0-4096.0;
    }
    else
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0;
    }
    return NO_ERROR;
}


/**@brief read junction delta.
 * @param value: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_junc_temp_delta(float *value)
{
    *value=0;
    uint16_t read_value=0;
    if(IIC_read_16bit(JUNCTION_TEMP_DELTA_REG_ADDR,&read_value))
    {
        return ERROR_COMM;
    }
    if(read_value&0x8000)
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0-4096.0;
    }
    else
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0;
    }
    return NO_ERROR;
}

/**@brief read cold-junction.
 * @param value: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_cold_junc(float *value)
{
    *value=0;
    uint16_t read_value=0;
    if(IIC_read_16bit(COLD_JUNCTION_TEMP_REG_ADDR,&read_value))
    {
        return ERROR_COMM;
    }

    
    if(read_value&0x8000)
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0-4096;
    }
    else
    {
        *value = (read_value>>8)*16.0+(read_value&0x00ff)/16.0;
    }
    return NO_ERROR;
}

/**@brief read raw ADC value.
 * @param data
 * @param data_len
 * @return 0 if successed.
 * */
err_t MCP9600::read_ADC_data(uint8_t* data,uint32_t data_len)
{
    if(IIC_read_bytes(RAW_ADC_DATA_REG_ADDR,data,data_len))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}   

/**@brief read sensor status.
 * @param byte: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_status(uint8_t* byte)
{
    *byte=0;
    if(IIC_read_byte(STAT_REG_ADDR,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_therm_cfg
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_therm_cfg(uint8_t set_byte)
{
    if(IIC_write_byte(THERM_SENS_CFG_REG_ADDR,set_byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}


/**@brief read thermocouple status.
 * @param byte: result.
 * @return 0 if successed.
 * */
err_t MCP9600::read_therm_cfg(uint8_t* byte)
{
    if(IIC_read_byte(THERM_SENS_CFG_REG_ADDR,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_therm_type
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_therm_type(uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(THERM_SENS_CFG_REG_ADDR,&therm_cfg_data))
    {
        gTemper_1 = 1007;
        return ERROR_COMM;
    }


    byte_to_set=(therm_cfg_data&0x8f)|set_byte;

    return IIC_write_byte(THERM_SENS_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_filt_coefficients
 * The  content of filt-coefficients register are:code-junction-resolution/ADC measurement resolution/burst mode temp samples/sensor mode
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_filt_coefficients(uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(THERM_SENS_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }

    byte_to_set=(therm_cfg_data&0xf8)|set_byte;

    return IIC_write_byte(THERM_SENS_CFG_REG_ADDR,byte_to_set);
}


/**@brief set_dev_cfg
 * The content of device configuration register are:interrupt clear/monitor TH or TC/inerrupt pin rise or FALL/
 *                                                  Active-high or low/compare mode or int mode /alert enable or not
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_dev_cfg(uint8_t set_byte)
{
    if(IIC_write_byte(DEVICE_CFG_REG_ADDR,set_byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief read_dev_cfg
 * @param byte: the byte to be read in.
 * @return 0 if successed.
 * */
err_t MCP9600::read_dev_cfg(uint8_t* byte)
{
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_sensor_mode
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_sensor_mode(uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xfc)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_burst_mode_samp
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_burst_mode_samp(uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xe3)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}


/**@brief set_ADC_meas_resolution
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_ADC_meas_resolution(uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0x9f)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_cold_junc_resolution
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_cold_junc_resolution(uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(DEVICE_CFG_REG_ADDR,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0x7f)|set_byte;
    return IIC_write_byte(DEVICE_CFG_REG_ADDR,byte_to_set);
}

/**@brief set_alert_limitation
 * @param alert num the channel of alert
 * @param value: the 16bit value to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_limit(uint8_t alert_num,uint16_t value)
{
    if(IIC_write_16bit(TEMP_ALERT1_LIMIT_REG_ADDR+alert_num,value))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_alert_hysteresis
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set..
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_hys(uint8_t alert_num,uint16_t value)
{
    if(IIC_write_byte(ALERT1_HYS_REG_ADDR+alert_num,value))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief set_alert_cfg
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_cfg(uint8_t alert_num,uint8_t set_byte)
{
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,set_byte);
}

/**@brief read_alert_cfg
 * @param alert num the channel of alert
 * @param byte: the byte to be read in.
 * @return 0 if successed.
 * */
err_t MCP9600::read_alert_cfg(uint8_t alert_num,uint8_t *byte)
{
    *byte=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,byte))
    {
        return ERROR_COMM;
    }
    return NO_ERROR;
}

/**@brief clear_int_flag
 * @param alert num the channel of alert
 * @return 0 if successed.
 * */
err_t MCP9600::clear_int_flag(uint8_t alert_num)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data|0x80);
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}


/**@brief set_alert_for_TH_or_TC
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_for_TH_or_TC(uint8_t alert_num,uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xef)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_limit_direction
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_limit_direction(uint8_t alert_num,uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xf7)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_bit
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_bit(uint8_t alert_num,uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }

    byte_to_set=(therm_cfg_data&0xfb)|set_byte;

    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_mode_bit
 * set alert mode:comparator mode or INT mode.
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_mode_bit(uint8_t alert_num,uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xfd)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}

/**@brief set_alert_enable
 * Eable alert pin or not.
 * @param alert num the channel of alert
 * @param set_byte: the byte to be set.
 * @return 0 if successed.
 * */
err_t MCP9600::set_alert_enable(uint8_t alert_num,uint8_t set_byte)
{
    uint8_t therm_cfg_data=0;
    uint8_t byte_to_set=0;
    if(IIC_read_byte(ALERT1_CFG_REG_ADDR+alert_num,&therm_cfg_data))
    {
        return ERROR_COMM;
    }
    byte_to_set=(therm_cfg_data&0xfe)|set_byte;
    return IIC_write_byte(ALERT1_CFG_REG_ADDR+alert_num,byte_to_set);
}


/**@brief check_data_update
 * check if data ready.
 * @param stat :indicate if data ready
 * @return 0 if successed.
 * */
err_t MCP9600::check_data_update(bool *stat)
{
    *stat=0;

    uint8_t byte=0;
    CHECK_RESULT(ret,read_status(&byte));
    if(byte&0x40)
    {
        *stat=true;
    }
    else
    {
        *stat=false;
    }
    return NO_ERROR;
}

/**@brief read_INT_stat
 * check if any interruption is generated.
 * @param stat :indicate if any interruption is generated
 * @return 0 if successed.
 * */
err_t MCP9600::read_INT_stat(uint8_t *stat)
{
    *stat=0;
    uint8_t byte=0;
    CHECK_RESULT(ret,read_status(&byte));
    for(int i=0;i<4;i++)
    {
        if(byte & 1<<i)
        {
//            Serial.print("channel ");
 //           Serial.print(i);
   //         Serial.println("generate interruption!!!");
        }
    }
    *stat=byte;
    return NO_ERROR;
}


/******************************************************************************************************/
uint16_t MCP9600::covert_temp_to_reg_form(float temp)
{
    uint8_t negetive=0;
    if(temp<0) negetive=1;
    temp=abs(temp);
    uint16_t dest_temp=0;
    uint8_t temp_H=0,temp_L=0;
    uint16_t interger=(uint16_t)temp;
    float decimal=temp-interger;
    temp_H=interger/16;
    temp_L|=(interger%16)<<4;
    temp_L|=(uint8_t)(decimal/0.25)<<2;
    if(negetive)
        temp_H|=0x80;
    dest_temp=(uint16_t)temp_H<<8|temp_L;
    return dest_temp;
}




/**********************************************************************************************************/
/************************************************IIC PART************************************************/
/**********************************************************************************************************/




/**@brief I2C write byte
 * @param reg :Register address of operation object
 * @param byte :The byte to be wrote.
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600::IIC_write_byte(uint8_t reg,uint8_t byte)
{
	uint8_t addr[2];

	addr[0] = reg;
	addr[1] = byte;

	if (!_dev->transfer(addr, 2, nullptr, 0) )
	{
        gTemper_1 = 1008;
		return ERROR_COMM;
	}

	return NO_ERROR;
}

/**@brief I2C write 16bit value
 * @param reg: Register address of operation object
 * @param value: The 16bit value to be wrote .
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600::IIC_write_16bit(uint8_t reg,uint16_t value)
{
	uint8_t addr[3];

	addr[0] = reg;
	addr[1] = (uint8_t)(value>>8);
	addr[2] = value;

	if (!_dev->transfer(addr, 3, nullptr, 0)) {
		return ERROR_COMM;
	}
	return NO_ERROR;
}



/**@brief I2C read byte
 * @param reg: Register address of operation object
 * @param byte: The byte to be read in.
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600::IIC_read_byte(uint8_t reg,uint8_t* byte)
{
    uint8_t addr;
    uint8_t val[4];

    hal.scheduler->delay(600);

    addr = reg;
	if (!_dev->transfer(&addr, 1,  nullptr, 0)) {
		gTemper_1 = 1011;
		return ERROR_COMM;
	}

	hal.scheduler->delay(600);
	if (!_dev->transfer(nullptr, 0,  &val[0], 1)) {
		gTemper_1 = 1012;
		return ERROR_COMM;
	}

	*byte = val[0];
    return NO_ERROR;
}

/**@brief I2C read 16bit value
 * @param reg: Register address of operation object
 * @param byte: The 16bit value to be read in.
 * @return result of operation,non-zero if failed.
 * */
err_t MCP9600::IIC_read_16bit(uint8_t start_reg,uint16_t *value)
{
    uint8_t addr;
    uint8_t val[4];

    hal.scheduler->delay(600);

    addr = start_reg;
	if (!_dev->transfer(&addr, 1,  nullptr, 0)) {
		gTemper_1 = 1011;
		return ERROR_COMM;
	}

	hal.scheduler->delay(600);
	if (!_dev->transfer(nullptr, 0,  &val[0], 2)) {
		gTemper_1 = 1012;
		return ERROR_COMM;
	}

	*value = (uint16_t) val[0] << 8 | val[1];
	return NO_ERROR;
}


err_t MCP9600::IIC_read_bytes(uint8_t start_reg,uint8_t *data,uint32_t data_len)
{
    uint8_t index;
    uint8_t addr;
    uint8_t val[40];

    hal.scheduler->delay(600);

    addr = start_reg;
	if (!_dev->transfer(&addr, 1,  nullptr, 0)) {
		gTemper_1 = 1011;
		return ERROR_COMM;
	}

	hal.scheduler->delay(600);

	if (!_dev->transfer(nullptr, 0,  &val[0], data_len)) {
		gTemper_1 = 1012;
		return ERROR_COMM;
	}

	for(index = 0; index < data_len;index++)
	{
		*data++ = val[index];
	}

    return NO_ERROR;
}

void MCP9600::set_iic_addr(uint8_t IIC_ADDR)
{
    _IIC_ADDR=IIC_ADDR;
}

bool MCP9600::IIC_begin()
{
	gPT100 = this;

	_dev = std::move(hal.i2c_mgr->get_device(0, _IIC_ADDR, 10000));

	if (!_dev) {

		gTemper_1 = 1002;
		_temperature = 100;
        printf("PT100 device is null!");
        return false;
    }

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
    	_temperature = 200;
        AP_HAL::panic("PANIC: PT100: failed to take serial semaphore for init");
    	gTemper_1 = 1003;
    }

    _dev->set_retries(10);
    hal.scheduler->delay(4);
    // lower retries for run
    _dev->set_retries(3);
    _dev->get_semaphore()->give();

    /* Request 20Hz update */
    // Max conversion time is 9.04 ms
    _dev->register_periodic_callback(500 * AP_MSEC_PER_SEC, FUNCTOR_BIND_MEMBER(&MCP9600::_timer, void));
    return true;
}


void MCP9600::_timer(void)
{
	err_t retError;
	uint16_t hot_junc=0;

	retError = read_hot_junc(&hot_junc);
	if( retError == NO_ERROR)
	{
		_healthy = hot_junc;
		_temperature = hot_junc;

		if(_IIC_ADDR == 0x60)
		{
			gTemper_1 = hot_junc;
		}
		else if(_IIC_ADDR == 0x67)
		{
			gTemper_2 = hot_junc;
		}
	}
	else
	{
		_healthy = 0;
		_temperature = 0;
		if(_IIC_ADDR == 0x60)
		{
			gTemper_1 = 654;   //error test value
		}
		else if(_IIC_ADDR == 0x67)
		{
		    gTemper_2 = 987;   //error test value
		}
	}
}
