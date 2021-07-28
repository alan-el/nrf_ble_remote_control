#ifndef __BATTERY_H
#define __BATTERY_H

#include "nrf_drv_saadc.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define BATTERY_TEST
#define BATTERY_INPUT_AIN_PIN 	NRF_SAADC_INPUT_AIN3

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                   /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                     /**< The ADC is configured to use VDD with 1/6 gain as input. And hence the result of conversion is to be multiplied by 6 to get the actual value of the battery voltage.*/
#define ADC_RES_10BIT                   1024
#define ADC_RES_12BIT                   4096

void battery_voltage_sample(void *p_context);
void battery_vol_detect_timer_create(void);
void battery_vol_detect_timer_start(void);
void battery_vol_detect_timer_stop(void);
#endif //__BATTERY_H

