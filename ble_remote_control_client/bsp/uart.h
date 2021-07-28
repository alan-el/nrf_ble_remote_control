#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#ifdef USE_PCA10040_SIMULATE_IDCARD
#define UART_TX_PIN_NUM      6
#define UART_RX_PIN_NUM      8
#else
#define UART_TX_PIN_NUM      10
#define UART_RX_PIN_NUM      9
#endif
#define UART_CTS_PIN_NUM    (NULL)
#define UART_RTS_PIN_NUM    (NULL)
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define MAX_PAYLOAD_LEN (128)
#define MAX_DEBUG_INFO_LEN (4 + 1 + 1 + MAX_PAYLOAD_LEN)

#define UART_GET_FAILED     (0x00000000)
#define UART_GET_SUCCESS    (0x00000001)
#define UART_PUT_FAILED     (0x00000000)
#define UART_PUT_SUCCESS    (0x00000001)
typedef struct
{
    uint8_t data[MAX_DEBUG_INFO_LEN];
    uint8_t len;
}uart_buff_t;

void uart_init(void);
uint32_t uart_deinit(void);
uint32_t uart_receive(uint8_t * r_buf);
uint32_t uart_send(uint8_t* p_payload, uint32_t payload_len);
#endif // UART_H_

