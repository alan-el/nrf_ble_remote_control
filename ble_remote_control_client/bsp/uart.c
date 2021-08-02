#include "app_uart.h"
#include "uart.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "uart_nrf52_rv1126.h"
#include <string.h>

extern uint8_t m_rx_buffer[64];
extern uint32_t rx_idx;

void uart_error_handle(app_uart_evt_t * p_event)
{
    switch(p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            // TODO
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
        case APP_UART_DATA_READY:
            uart_nrf52_rv1126_get_cmd(m_rx_buffer, &rx_idx);
            break;
        case APP_UART_TX_EMPTY:
            break;
        default:
            break;
    }
}

void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
      UART_RX_PIN_NUM,
      UART_TX_PIN_NUM,
      UART_RTS_PIN_NUM,
      UART_CTS_PIN_NUM,
      UART_HWFC,
      false,
#if defined (UART_PRESENT)
      NRF_UART_BAUDRATE_9600
#else
      NRF_UARTE_BAUDRATE_9600
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                     UART_RX_BUF_SIZE,
                     UART_TX_BUF_SIZE,
                     uart_error_handle,
                     APP_IRQ_PRIORITY_LOWEST,
                     err_code);

    APP_ERROR_CHECK(err_code);
    
}

uint32_t uart_deinit(void)
{
    uint32_t ret;
    ret = app_uart_close();
    return ret;
}

uint32_t uart_receive(uint8_t * r_buf)
{
    uint8_t cr; 
    uint32_t len = 0;
    while(app_uart_get(&cr) == NRF_SUCCESS)
    {
        r_buf[len++] = cr;
    }
    if(len != 0)
        return len;
    else
        return UART_GET_FAILED;
}
 
uint32_t uart_send(uint8_t* p_payload, uint32_t payload_len)
{
    uint32_t err_code;   
    for(int i = 0; i < payload_len; i++)
    {
        err_code = app_uart_put(*(p_payload + i));
        if (err_code != NRF_SUCCESS)
        {
            return UART_PUT_FAILED;
        }
    }
    return UART_PUT_SUCCESS;
}

