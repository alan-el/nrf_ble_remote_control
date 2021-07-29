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

#include "ble_rcs.h"
#include "app_error.h"
#include "nrf_log.h"
extern ble_rcs_t m_rcs;
// ALREADY_DONE_FOR_YOU: Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_rc_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  	ble_rcs_t * p_rc_service =(ble_rcs_t *) p_context;  
		// OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
		switch (p_ble_evt->header.evt_id)
		{
				case BLE_GAP_EVT_CONNECTED:
						p_rc_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						break;
				case BLE_GAP_EVT_DISCONNECTED:
						p_rc_service->conn_handle = BLE_CONN_HANDLE_INVALID;
						break;
				default:
						// No implementation needed.
						break;
		}
	
}

/**@brief Function for adding our new characterstic to "beacon params set service"
 * @param[in]   p_bpss_service        Our Service structure.
 *
 */

static uint32_t remote_control_service_char_add(ble_rcs_t * p_rc_service)
{
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID，设置特性的uuid
	int i = 0;
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_REMOTE_CONTROL_BASE_UUID;
    ble_gatts_attr_md_t cccd_md;
    for(; i < CHARACTERISTICS_NUM; i++)
    {

        char_uuid.uuid = BLE_CHARACTERISTICS_BASE_UUID + i;
        
        err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
        APP_ERROR_CHECK(err_code);

        // OUR_JOB: Step 2.F Add read/write properties to our characteristic
        ble_gatts_char_md_t char_md;
        memset(&char_md, 0, sizeof(char_md));
        char_md.char_props.read = 1;
        char_md.char_props.write = 0;

        // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
        if(i == 0 )
        {
            
            memset(&cccd_md, 0, sizeof(cccd_md));
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
            BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cccd_md.write_perm);
            cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
            char_md.p_cccd_md           = &cccd_md;
            char_md.char_props.notify   = 1;
        }

        // OUR_JOB: Step 2.B, Configure the attribute metadata
        ble_gatts_attr_md_t attr_md;
        memset(&attr_md, 0, sizeof(attr_md));  
        attr_md.vloc = BLE_GATTS_VLOC_STACK;

        // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
        BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        // OUR_JOB: Step 2.C, Configure the characteristic value attribute
        ble_gatts_attr_t    attr_char_value;
        memset(&attr_char_value, 0, sizeof(attr_char_value));
        attr_char_value.p_uuid      = &char_uuid;
        attr_char_value.p_attr_md   = &attr_md;

        // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
        if(i == 0)	// 命令																								
        {	
            attr_char_value.max_len     = 1;
            attr_char_value.init_len    = 1;
            uint8_t value = BLE_RCS_CMD_INVALID;
            attr_char_value.p_value = (uint8_t *)&value;
        }
        //选择特征值句柄
        ble_gatts_char_handles_t* p_char_handle;
        p_char_handle = &(p_rc_service->command_char_handles) + i;

        // OUR_JOB: Step 2.E, Add our new characteristic to the service
        err_code = sd_ble_gatts_characteristic_add( p_rc_service->service_handle,
                                                    &char_md,
                                                    &attr_char_value,
                                                    p_char_handle);
        APP_ERROR_CHECK(err_code);
    }
    
    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void remote_control_service_init(ble_rcs_t * p_rc_service)
{

    // STEP 3: Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table     
    uint32_t err_code;
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = BLE_REMOTE_CONTROL_BASE_UUID;
    service_uuid.uuid       = BLE_REMOTE_CONTROL_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);
	
  // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    p_rc_service->conn_handle = BLE_CONN_HANDLE_INVALID;

// STEP 4: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                            &service_uuid,
                            &p_rc_service->service_handle);
    APP_ERROR_CHECK(err_code);
// Print messages to Segger Real Time Terminal
// UNCOMMENT THE FOUR LINES BELOW AFTER INITIALIZING THE SERVICE OR THE EXAMPLE WILL NOT COMPILE.
//    SEGGER_RTT_WriteString(0, "Executing bpss_service_init().\n"); // Print message to RTT to the application flow
//    SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_OUR_SERVICE
//    SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
//    SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_rc_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values

// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    remote_control_service_char_add(p_rc_service);
}

void ble_rcs_command_notification(ble_rcs_cmd_t cmd)
{
    uint32_t err_code;
    
    // 更新特征值
    ble_gatts_value_t gatts_value;
    gatts_value.len = 1;
    gatts_value.offset = 0;
    gatts_value.p_value = (uint8_t*)&cmd;
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, m_rcs.command_char_handles.value_handle, &gatts_value);
    APP_ERROR_CHECK(err_code);

    
    
    // 检查当前是否有连接
    if (m_rcs.conn_handle != BLE_CONN_HANDLE_INVALID )
    {
        // 检查当前连接的Client是否使能了命令特征值的Notification or indication
        uint16_t cccd_value;
        // Pupulate ble_gatts_value_t structure to hold received data and metadata.
        ble_gatts_value_t cccd_data;
        cccd_data.len = BLE_CCCD_VALUE_LEN;
        cccd_data.offset = 0;
        cccd_data.p_value = (uint8_t*)&cccd_value;
        sd_ble_gatts_value_get(m_rcs.conn_handle, m_rcs.command_char_handles.cccd_handle, &cccd_data);
        NRF_LOG_DEBUG("CCCD value: %d", cccd_value);
        
        if(cccd_value == 1 || cccd_value == 2)
        {
            uint16_t len;
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            
            hvx_params.handle = m_rcs.command_char_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            len = sizeof(uint8_t);
            hvx_params.p_len  = &len;
            hvx_params.p_data = (uint8_t*)&cmd;
      
            err_code = sd_ble_gatts_hvx(m_rcs.conn_handle, &hvx_params); 
            APP_ERROR_CHECK(err_code);
        }
    } 
}
