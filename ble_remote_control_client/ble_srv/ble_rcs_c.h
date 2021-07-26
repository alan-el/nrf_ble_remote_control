/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
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
/**@file
 *
 * @defgroup ble_rcs_c Nordic UART Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic UART Service Client module.
 *
 * @details  This module contains the APIs and types exposed by the Nordic UART Service Client
 *           module. The application can use these APIs and types to perform the discovery of
 *           the Nordic UART Service at the peer and to interact with it.
 *
 * @note    The application must register this module as the BLE event observer by using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_rcs_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_RCS_C_BLE_OBSERVER_PRIO,
 *                                   ble_rcs_c_on_ble_evt, &instance);
 *          @endcode
 *
 */


#ifndef BLE_RCS_C_H__
#define BLE_RCS_C_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"

#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_rcs_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_RCS_C_DEF(_name)                                                                        \
static ble_rcs_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_RCS_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_rcs_c_on_ble_evt, &_name)

/** @brief Macro for defining multiple ble_rcs_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_RCS_C_ARRAY_DEF(_name, _cnt)                 \
static ble_rcs_c_t _name[_cnt];                          \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
                      BLE_RCS_C_BLE_OBSERVER_PRIO,       \
                      ble_rcs_c_on_ble_evt, &_name, _cnt)

#define RCS_BASE_UUID                   {{0xCA, 0xE6, 0x6F, 0xDF, 0xD6, 0x0D, 0xFC, 0x9E, 0x9F, 0x42, 0xC2, 0x85, 0x00, 0x00, 0x1A, 0xA1}} /**< Used vendor-specific UUID. */

#define BLE_UUID_RCS_SERVICE                0xD497                      /**< The UUID of the Remote Control Service. */
#define BLE_UUID_CHARACTERISTICS_COMMAND    0x0001                      /**< The UUID of the command Characteristic. */

/**@brief RCS Client event type. */
typedef enum
{
    BLE_RCS_C_EVT_DISCOVERY_COMPLETE,   /**< Event indicating that the RCS service and its characteristics were found. */
    BLE_RCS_C_EVT_RCS_COMMAND_EVT,      /**< Event indicating that the central received a command. */
    BLE_RCS_C_EVT_DISCONNECTED          /**< Event indicating that the RCS server disconnected. */
} ble_rcs_c_evt_type_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t rcs_command_handle;      /**< Handle of the RCS command characteristic, as provided by a discovery. */
    uint16_t rcs_command_cccd_handle; /**< Handle of the CCCD of the RCS command characteristic, as provided by a discovery. */
} ble_rcs_c_handles_t;

/**@brief Structure containing the RCS event data received from the peer. */
typedef struct
{
    ble_rcs_c_evt_type_t evt_type;
    uint16_t             conn_handle;
    //uint8_t            * p_data;
    //uint16_t             data_len;
    uint8_t* p_command;     // size = two bytes( 16 bits )
    ble_rcs_c_handles_t  handles;     /**< Handles on which the Nordic UART service characteristics were discovered on the peer device. This is filled if the evt_type is @ref BLE_RCS_C_EVT_DISCOVERY_COMPLETE.*/
} ble_rcs_c_evt_t;

// Forward declaration of the ble_rcs_t type.
typedef struct ble_rcs_c_s ble_rcs_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that is to be provided by the application
 *          of this module to receive events.
 */
typedef void (* ble_rcs_c_evt_handler_t)(ble_rcs_c_t * p_ble_rcs_c, ble_rcs_c_evt_t const * p_evt);

/**@brief RCS Client structure. */
struct ble_rcs_c_s
{
    uint8_t                   uuid_type;      /**< UUID type. */
    uint16_t                  conn_handle;    /**< Handle of the current connection. Set with @ref ble_rcs_c_handles_assign when connected. */
    ble_rcs_c_handles_t       handles;        /**< Handles on the connected peer device needed to interact with it. */
    ble_rcs_c_evt_handler_t   evt_handler;    /**< Application event handler to be called when there is an event related to the RCS. */
    ble_srv_error_handler_t   error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t            * p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
};

/**@brief RCS Client initialization structure. */
typedef struct
{
    ble_rcs_c_evt_handler_t   evt_handler;    /**< Application event handler to be called when there is an event related to the RCS. */
    ble_srv_error_handler_t   error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t            * p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
} ble_rcs_c_init_t;


/**@brief     Function for initializing the Remote Control client module.
 *
 * @details   This function registers with the Database Discovery module
 *            for the RCS. The Database Discovery module looks for the presence
 *            of a RCS instance at the peer when a discovery is started.
 *            
 * @param[in] p_ble_rcs_c      Pointer to the RCS client structure.
 * @param[in] p_ble_rcs_c_init Pointer to the RCS initialization structure that contains the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS If the module was initialized successfully.
 * @retval    err_code    Otherwise, this function propagates the error code
 *                        returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_rcs_c_init(ble_rcs_c_t * p_ble_rcs_c, ble_rcs_c_init_t * p_ble_rcs_c_init);


/**@brief Function for handling events from the Database Discovery module.
 *
 * @details This function handles an event from the Database Discovery module, and determines
 *          whether it relates to the discovery of RCS at the peer. If it does, the function
 *          calls the application's event handler to indicate that RCS was
 *          discovered at the peer. The function also populates the event with service-related
 *          information before providing it to the application.
 *
 * @param[in] p_ble_rcs_c Pointer to the RCS client structure.
 * @param[in] p_evt       Pointer to the event received from the Database Discovery module.
 */
 void ble_rcs_c_on_db_disc_evt(ble_rcs_c_t * p_ble_rcs_c, ble_db_discovery_evt_t * p_evt);


/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function handles the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the RCS module, the function uses the event's data to update
 *            internal variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the RCS client structure.
 */
void ble_rcs_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for requesting the peer to start sending notification of TX characteristic.
 *
 * @details This function enables notifications of the RCS TX characteristic at the peer
 *          by writing to the CCCD of the RCS TX characteristic.
 *
 * @param   p_ble_rcs_c Pointer to the RCS client structure.
 *
 * @retval  NRF_SUCCESS If the operation was successful. 
 * @retval  err_code 	Otherwise, this API propagates the error code returned by function @ref nrf_ble_gq_item_add.
 */
uint32_t ble_rcs_c_cmd_notif_enable(ble_rcs_c_t * p_ble_rcs_c);


/**@brief Function for sending a string to the server.
 *
 * @details This function writes the RX characteristic of the server.
 *
 * @param[in] p_ble_rcs_c Pointer to the RCS client structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. 
 * @retval err_code    Otherwise, this API propagates the error code returned by function @ref nrf_ble_gq_item_add.
 */
//uint32_t ble_rcs_c_string_send(ble_rcs_c_t * p_ble_rcs_c, uint8_t * p_string, uint16_t length);


/**@brief Function for assigning handles to this instance of rcs_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate the link to this instance of the module. This makes it
 *          possible to handle several links and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles are
 *          provided from the discovery event @ref BLE_RCS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_rcs_c    Pointer to the RCS client structure instance to associate with these
 *                           handles.
 * @param[in] conn_handle    Connection handle to associated with the given RCS Instance.
 * @param[in] p_peer_handles Attribute handles on the RCS server that you want this RCS client to
 *                           interact with.
 *
 * @retval    NRF_SUCCESS    If the operation was successful.
 * @retval    NRF_ERROR_NULL If a p_rcs was a NULL pointer.
 * @retval    err_code       Otherwise, this API propagates the error code returned 
 *                           by function @ref nrf_ble_gq_item_add.
 */
uint32_t ble_rcs_c_handles_assign(ble_rcs_c_t *               p_ble_rcs_c,
                                  uint16_t                    conn_handle,
                                  ble_rcs_c_handles_t const * p_peer_handles);


#ifdef __cplusplus
}
#endif

#endif // BLE_RCS_C_H__

/** @} */
