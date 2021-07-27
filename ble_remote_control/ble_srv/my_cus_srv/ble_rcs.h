/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef _BLE_RCS_H_
#define _BLE_RCS_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ble.h"
#include "ble_srv_common.h"



#define BLE_REMOTE_CONTROL_BASE_UUID            {0xCA, 0xE6, 0x6F, 0xDF,\
                                                 0xD6, 0x0D, 0xFC, 0x9E,\
                                                 0x9F, 0x42, 0xC2, 0x85,\
                                                 0x00, 0x00, 0x1A, 0xA1} // 128-bit base UUID

#define BLE_REMOTE_CONTROL_SERVICE_UUID         0xD497

#define CHARACTERISTICS_NUM 1

#define BLE_CHARACTERISTICS_BASE_UUID               0x0001

#define BLE_CHARACTERISTICS_COMMAND_UUID            0x0001  

typedef enum
{
    BLE_RCS_CMD_NEXT_PARA = 0,
    BLE_RCS_CMD_PHOTO,
    BLE_RCS_CMD_PREV_PARA,
    BLE_RCS_CMD_PAUSE,
    BLE_RCS_CMD_MINUS,
    BLE_RCS_CMD_PLUS,
    BLE_RCS_CMD_MODE,

    BLE_RCS_CMD_INVALID = 0xFF,
    
}ble_rcs_cmd_t;

typedef struct
{
    uint16_t conn_handle;      
    // OUR_JOB: Step 2.D, Add handles for our characteristic
    ble_gatts_char_handles_t command_char_handles;
    
    uint16_t service_handle;     /**< Handle of bs Service (as provided by the BLE stack). */
}ble_rcs_t;


/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_rc_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void remote_control_service_init(ble_rcs_t * p_rc_service);

void ble_rcs_command_notification(ble_rcs_cmd_t cmd);


#endif

