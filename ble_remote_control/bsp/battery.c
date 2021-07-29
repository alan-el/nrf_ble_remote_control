
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "battery.h"
#include "app_timer.h"

#define BATTERY_VOLTAGE_DETECT_INTERVAL_MS  (1000)
#define PARTIAL_VOLTAGE_RATIO               (2 * 1.0366)
APP_TIMER_DEF(m_bat_dtct_tmr);

int16_t battery_voltage = 0;

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / (ADC_RES_10BIT - 1)) * ADC_PRE_SCALING_COMPENSATION)

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event){NRF_LOG_INFO("ADC event");}

static void start_battery_adc(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BATTERY_INPUT_AIN_PIN);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

}

static void capture_battery_voltage(void)
{
    ret_code_t err_code = nrf_drv_saadc_sample_convert(0, &battery_voltage);
    APP_ERROR_CHECK(err_code);
    battery_voltage = PARTIAL_VOLTAGE_RATIO * ADC_RESULT_IN_MILLI_VOLTS(battery_voltage);
    //SET_BATTERY_VOLTAGE(&battery_voltage);
}

static void stop_battery_adc(void)
{
    nrf_drv_saadc_uninit(); 
}

void battery_voltage_sample(void *p_context)
{
    start_battery_adc();
    capture_battery_voltage();
    stop_battery_adc();
}

void battery_vol_detect_timer_create(void)
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_bat_dtct_tmr, APP_TIMER_MODE_REPEATED, battery_voltage_sample);
    APP_ERROR_CHECK(err_code);
}    

void battery_vol_detect_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_bat_dtct_tmr, APP_TIMER_TICKS(BATTERY_VOLTAGE_DETECT_INTERVAL_MS), NULL);
    APP_ERROR_CHECK(err_code);
}

void battery_vol_detect_timer_stop(void)
{
    ret_code_t err_code;
    err_code = app_timer_stop(m_bat_dtct_tmr);
    APP_ERROR_CHECK(err_code);
}
