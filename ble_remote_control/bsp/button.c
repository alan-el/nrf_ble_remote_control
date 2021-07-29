#include "button.h"
#include "app_timer.h"
#include "app_error.h"
#include "ble_rcs.h"
#include "main.h"
#include "ble_gap.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


APP_TIMER_DEF(m_btn_long_push_tmr);
APP_TIMER_DEF(m_start_up_cmd_tmr);

button_event_t start_up_evt = BUTTON_EVENT_NOTHING;

static const app_button_cfg_t m_buttons[BUTTONS_NUMBER] = 
{
    BUTTON_CFG(BUTTON1_PIN_NO),
    BUTTON_CFG(BUTTON2_PIN_NO),
    BUTTON_CFG(BUTTON3_PIN_NO),
    BUTTON_CFG(BUTTON4_PIN_NO),
//    BUTTON_CFG(BUTTON5_PIN_NO),
    BUTTON_CFG(BUTTON10_PIN_NO),
    BUTTON_CFG(BUTTON11_PIN_NO),
    BUTTON_CFG(BUTTON12_PIN_NO)
};

static uint8_t m_btn_list[BUTTONS_NUMBER] = 
{   
    BUTTON1_PIN_NO,
    BUTTON2_PIN_NO,
    BUTTON3_PIN_NO, 
    BUTTON4_PIN_NO, 
//    BUTTON5_PIN_NO, 
    BUTTON10_PIN_NO, 
    BUTTON11_PIN_NO, 
    BUTTON12_PIN_NO
};

static button_event_cfg_t m_btn_event_list[BUTTONS_NUMBER] = 
{
    // button 1
    {
        BUTTON_EVENT_NOTHING,
        BUTTON_EVENT_NEXT_PARA,
        BUTTON_EVENT_NOTHING      
    },
    // button 2
    {
        BUTTON_EVENT_NOTHING,
        BUTTON_EVENT_PHOTO,
        BUTTON_EVENT_NOTHING
    },
    // button 3
    {
        BUTTON_EVENT_NOTHING,
        BUTTON_EVENT_PREV_PARA,
        BUTTON_EVENT_NOTHING
    },
    // button 4
    {
        BUTTON_EVENT_NOTHING,
        BUTTON_EVENT_PAUSE,
        BUTTON_EVENT_NOTHING
    },
    // button 10
    {
        BUTTON_EVENT_NOTHING,
        BUTTON_EVENT_MINUS,
        BUTTON_EVENT_NOTHING
    },
    // button 11
    {
        BUTTON_EVENT_NOTHING,
        BUTTON_EVENT_PLUS,
        BUTTON_EVENT_NOTHING
    },
    // button 12
    {
        BUTTON_EVENT_NOTHING,
        BUTTON_EVENT_MODE,
        BUTTON_EVENT_ERASE_BONDS
    }
};

static void button_command_process(button_event_t btn_evt);

static uint32_t button_pin_to_idx(uint8_t pin_no)
{
    uint32_t i;
    uint32_t ret = 0xFFFFFFFF;
    for (i = 0; i < BUTTONS_NUMBER; ++i)
    {
        if (m_btn_list[i] == pin_no)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

static void button_long_push_detect(void *p_context)
{
    uint8_t pin_no = *((uint8_t *)p_context);

    button_evt_handler(pin_no, BUTTON_ACTION_LONG_PUSH);
}

static uint32_t button_long_push_timer_create(void)
{
    uint32_t ret_code;
    ret_code = app_timer_create(&m_btn_long_push_tmr, APP_TIMER_MODE_SINGLE_SHOT, button_long_push_detect);
    return ret_code;
}

static void start_up_command_notification(void *p_context)
{
    button_event_t start_cmd = *((button_event_t *)p_context);

    button_command_process(start_cmd);
    
    start_up_evt = BUTTON_EVENT_NOTHING;
}

void start_up_command_notification_timer_start(button_event_t *p_evt)
{
    app_timer_start(m_start_up_cmd_tmr, APP_TIMER_TICKS(START_CMD_NTF_TIMEOUT_MS), (void*)p_evt);
}

static uint32_t start_up_command_notification_timer_create(void)
{
    uint32_t ret_code;
    ret_code = app_timer_create(&m_start_up_cmd_tmr, APP_TIMER_MODE_SINGLE_SHOT, start_up_command_notification);
    return ret_code;
}

static void button_command_process(button_event_t btn_evt)
{
    switch(btn_evt)
    {
        case BUTTON_EVENT_NEXT_PARA:
            NRF_LOG_DEBUG("Next paragraph command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_NEXT_PARA);
            break;
        case BUTTON_EVENT_PHOTO:
            NRF_LOG_DEBUG("Photograph command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_PHOTO);
            break;
        case BUTTON_EVENT_PREV_PARA:
            NRF_LOG_DEBUG("Previous paragraph command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_PREV_PARA);
            break;
        case BUTTON_EVENT_PAUSE:
            NRF_LOG_DEBUG("Pause command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_PAUSE);
            break;
        case BUTTON_EVENT_MINUS:
            NRF_LOG_DEBUG("Minus command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_MINUS);
            break;
        case BUTTON_EVENT_PLUS:
            NRF_LOG_DEBUG("Plus command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_PLUS);
            break;
        case BUTTON_EVENT_MODE:
            NRF_LOG_DEBUG("Mode command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_MODE);
            break;   
        case BUTTON_EVENT_ERASE_BONDS:
            NRF_LOG_DEBUG("Erase bonds command get.");
            delete_bonds();
            break;
        case BUTTON_EVENT_DEBUG:
            NRF_LOG_DEBUG("Debug command get.");
            ble_rcs_command_notification(BLE_RCS_CMD_INVALID);
            break;
        default:
            break;
    }
}

void button_evt_handler(uint8_t pin_no, uint8_t button_action)
{
    button_event_t     event  = BUTTON_EVENT_NOTHING;
    uint32_t           button = 0;
    uint32_t           err_code;
    static uint8_t     current_long_push_pin_no;              /**< Pin number of a currently pushed button, that could become a long push if held long enough. */
    static button_event_t release_event_at_push[BUTTONS_NUMBER]; /**< Array of what the release event of each button was last time it was pushed, so that no release event is sent if the event was bound after the push of the button. */

    button = button_pin_to_idx(pin_no);

    if (button < BUTTONS_NUMBER)
    {
        switch (button_action)
        {
            case APP_BUTTON_PUSH:
                event = m_btn_event_list[button].push_event;
                if (m_btn_event_list[button].long_push_event != BUTTON_EVENT_NOTHING)
                {
                    err_code = app_timer_start(m_btn_long_push_tmr, APP_TIMER_TICKS(BUTTON_LONG_PUSH_TIMEOUT_MS), (void*)&current_long_push_pin_no);
                    if (err_code == NRF_SUCCESS)
                    {
                        current_long_push_pin_no = pin_no;
                    }
                }
                release_event_at_push[button] = m_btn_event_list[button].release_event;
                break;
            case APP_BUTTON_RELEASE:
                (void)app_timer_stop(m_btn_long_push_tmr);
                if (release_event_at_push[button] == m_btn_event_list[button].release_event)
                {
                    event = m_btn_event_list[button].release_event;
                }
                break;
            case BUTTON_ACTION_LONG_PUSH:
                event = m_btn_event_list[button].long_push_event;
                /* After a button long push trigger, discard its release event */
                release_event_at_push[button] = BUTTON_EVENT_NOTHING;
                break;
        }
    }

    if (event != BUTTON_EVENT_NOTHING)
    {
        button_command_process(event);
    }
}

static void extract_cmd_startup_button_pressed(void)
{
    for(int i = 0; i < BUTTONS_NUMBER; i++)
    {
        bool pin_set = nrf_gpio_pin_read(m_btn_list[i]) ? true : false;
        
        if(pin_set == (BUTTON_ACTIVE_STATE ? true : false))
        {
            start_up_evt = m_btn_event_list[i].release_event;
            return ;
        }
            
    }
}

void button_init(void)
{
    uint32_t err_code;
    err_code = app_button_init(m_buttons, BUTTONS_NUMBER, APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
    
    err_code = button_long_push_timer_create();
    APP_ERROR_CHECK(err_code);

    err_code = start_up_command_notification_timer_create();
    APP_ERROR_CHECK(err_code);
    
    extract_cmd_startup_button_pressed();
}

void prepare_button_wake_up_from_power_down_mode(void)
{
    for(int i = 0; i < BUTTONS_NUMBER; i++)
    {
        nrf_gpio_cfg_sense_set(m_btn_list[i], NRF_GPIO_PIN_SENSE_HIGH);
    }
}
