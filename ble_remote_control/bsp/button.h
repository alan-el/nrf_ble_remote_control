#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_button.h"

#define BUTTONS_NUMBER (7)

#define BUTTON1_PIN_NO  (12)
#define BUTTON2_PIN_NO  (14)
#define BUTTON3_PIN_NO  (15)
#define BUTTON4_PIN_NO  (16)
//#define BUTTON5_PIN_NO  (18)    // reserve for future use, not implement
#define BUTTON10_PIN_NO (6)
#define BUTTON11_PIN_NO (9)
#define BUTTON12_PIN_NO (10)

#define BUTTON_LONG_PUSH_TIMEOUT_MS (2000)
#define START_CMD_NTF_TIMEOUT_MS (500)
#define BUTTON_ACTION_LONG_PUSH (2)
#define BUTTON_ACTIVE_STATE     APP_BUTTON_ACTIVE_HIGH

#define BUTTON_CFG(_pin_no) \
{ \
    .pin_no = _pin_no, \
    .active_state = BUTTON_ACTIVE_STATE, \
    .pull_cfg = NRF_GPIO_PIN_PULLDOWN, \
    .button_handler = button_evt_handler, \
}

typedef enum
{
    BUTTON_EVENT_NOTHING = 0,
    BUTTON_EVENT_NEXT_PARA,
    BUTTON_EVENT_PHOTO,
    BUTTON_EVENT_PREV_PARA,
    BUTTON_EVENT_PAUSE,
    BUTTON_EVENT_MINUS,
    BUTTON_EVENT_PLUS,
    BUTTON_EVENT_MODE,

    BUTTON_EVENT_ERASE_BONDS,

    BUTTON_EVENT_DEBUG,
    
}button_event_t;

typedef struct
{
    button_event_t push_event;      /**< The event to fire on regular button press. */
    button_event_t release_event;   /**< The event to fire on button release. */
    button_event_t long_push_event; /**< The event to fire on long button press. */
} button_event_cfg_t;

void start_up_command_notification_timer_start(button_event_t *p_evt);
void button_evt_handler(uint8_t pin_no, uint8_t button_action);
void button_init(void);
void prepare_button_wake_up_from_power_down_mode(void);

#endif

