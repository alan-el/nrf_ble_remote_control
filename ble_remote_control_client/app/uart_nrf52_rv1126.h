#ifndef _UART_NRF52_RV1126_H_
#define _UART_NRF52_RV1126_H_

#include<stdint.h>

// NRU_START_CODE - The symbol of nrf52 rv1126 uart frame begin
#ifndef NRU_START_CODE
#define NRU_START_CODE  0xFFFFFFFF
#endif
// NRC_LEN - The length of nrf52 rv1126 command in bytes
#ifndef NRC_LEN
#define NRC_LEN  sizeof(nrf_rv1126_uart_t)
#endif
// CRC_LEN - The length of crc field in bytes
#ifndef CRC_LEN
#define CRC_LEN  sizeof(uint16_t)
#endif

/* nRF52 与 rv1126 UART 通信命令枚举类型定义 */
typedef enum 
{
    /* nRF52832 to rv1126 command type */
    N2R_CMD_TYPE_NEXT_PARA = 0, // 下一段
    N2R_CMD_TYPE_PHOTO,         // 拍照
    N2R_CMD_TYPE_PREV_PARA,     // 上一段
    N2R_CMD_TYPE_PAUSE,         // 暂停
    N2R_CMD_TYPE_MINUS,         // "-"
    N2R_CMD_TYPE_PLUS,          // "+"
    N2R_CMD_TYPE_MODE,          // 模式

    /* rv1126 to nRF52832 command type */
    R2N_CMD_TYPE_DELETE_BONDS = 0x80, // 擦除广角板蓝牙SoC的BLE bonds信息
}nrf_rv1126_cmd_t;

/* nRF52 与 rv1126 UART 通信结构体定义 */
typedef struct 
{
    uint32_t start_code;
    uint16_t cmd;
    uint16_t crc;
}nrf_rv1126_uart_t;

typedef nrf_rv1126_uart_t * nrf_rv1126_uart_handle;

uint32_t uart_nrf52_rv1126_send_cmd(uint8_t *p_tx_buf, nrf_rv1126_cmd_t cmd_type);
uint32_t uart_nrf52_rv1126_get_cmd(uint8_t *p_rx_buf, uint32_t *p_idx);

#endif


