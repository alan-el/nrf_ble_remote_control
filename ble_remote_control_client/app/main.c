/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
/**
 * @brief BLE Heart Rate Collector application main file.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_util.h"
#include "app_error.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_util.h"
#include "app_timer.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_scan.h"

#include "main.h"

#include "ble_rcs_c.h"
#include "uart.h"

#define APP_BLE_CONN_CFG_TAG        1                                   /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_BLE_OBSERVER_PRIO       3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< Applications' SoC observer priority. You shouldn't need to modify this value. */

#define LESC_DEBUG_MODE             0                                   /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND              1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM              0                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC              0                                   /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS          0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE                /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                   /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                  /**< Maximum encryption key size in octets. */

#define SCAN_DURATION_WITELIST      0                                   /**< Duration of the scanning in units of 10 milliseconds. */
#define RCS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN
/**@brief RCS UUID. */
static ble_uuid_t const m_rcs_uuid =
{
    .uuid = BLE_UUID_RCS_SERVICE,
    .type = RCS_SERVICE_UUID_TYPE
};

BLE_RCS_C_DEF(m_ble_rcs_c);
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                    /**< DB discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                           /**< Scanning module instance. */

static uint16_t m_conn_handle;                                      /**< Current connection handle. */
static bool     m_whitelist_disabled;                               /**< True if whitelist has been temporarily disabled. */
static bool     m_memory_access_in_progress;                        /**< Flag to keep track of ongoing operations on persistent memory. */
static bool     erasing_bonds = false;
bool is_scanning = false;
/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param =
{
    .active        = 0x01,
#if (NRF_SD_BLE_API_VERSION > 7)
    .interval_us   = NRF_BLE_SCAN_SCAN_INTERVAL * UNIT_0_625_MS,
    .window_us     = NRF_BLE_SCAN_SCAN_WINDOW * UNIT_0_625_MS,
#else
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,    // 100ms
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,      // 50ms
#endif // (NRF_SD_BLE_API_VERSION > 7)
    .filter_policy = BLE_GAP_SCAN_FP_WHITELIST,
    .timeout       = SCAN_DURATION_WITELIST,        // infinite
    .scan_phys     = BLE_GAP_PHY_1MBPS,
};


/**@brief Names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static char const m_target_periph_name[] = "";      /**< If you want to connect to a peripheral using a given advertising name, type its name here. */
static bool is_connect_per_addr = false;            /**< If you want to connect to a peripheral with a given address, set this to true and put the correct address in the variable below. */

static ble_gap_addr_t const m_target_periph_addr =
{
    /* Possible values for addr_type:
       BLE_GAP_ADDR_TYPE_PUBLIC,
       BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    .addr      = {0x8D, 0xFE, 0x23, 0x86, 0x77, 0xD9}
};


static void scan_start(void);


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_rcs_c_on_db_disc_evt(&m_ble_rcs_c, p_evt);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            // Bonds are deleted. Start scanning.
            scan_start();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");

            // Discover peer's services.
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
        } break;

        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected, reason 0x%x.",
                         p_gap_evt->params.disconnected.reason);

            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            break;
    }
}


/**@brief SoftDevice SoC event handler.
 *
 * @param[in]   evt_id      SoC event.
 * @param[in]   p_context   Context.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    switch (evt_id)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}



/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage
 */
void delete_bonds(void)
{
    erasing_bonds = true;
    ret_code_t err_code;
    if(m_ble_rcs_c.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle,
                                   BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
    
    erasing_bonds = false;
}


/**@brief Function for disabling the use of whitelist for scanning.
 */
// TODO this function may need to be used
static void whitelist_disable(void)
{
    if (!m_whitelist_disabled)
    {
        NRF_LOG_INFO("Whitelist temporarily disabled.");
        m_whitelist_disabled = true;
        nrf_ble_scan_stop();
        is_scanning = false;
        scan_start();
    }
}

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);

    APP_ERROR_CHECK(err_code);
}


/**@brief Retrieve a list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


static void whitelist_load()
{
    ret_code_t   ret;
    pm_peer_id_t peers[8];
    uint32_t     peer_cnt;

    memset(peers, PM_PEER_ID_INVALID, sizeof(peers));
    peer_cnt = (sizeof(peers) / sizeof(pm_peer_id_t));

    // Load all peers from flash and whitelist them.
    peer_list_get(peers, &peer_cnt);

    ret = pm_whitelist_set(peers, peer_cnt);
    APP_ERROR_CHECK(ret);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(peers, peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(ret);
    }
}


static void on_whitelist_req(void)
{
    ret_code_t err_code;

    // Whitelist buffers.
    ble_gap_addr_t whitelist_addrs[8];
    ble_gap_irk_t  whitelist_irks[8];

    memset(whitelist_addrs, 0x00, sizeof(whitelist_addrs));
    memset(whitelist_irks,  0x00, sizeof(whitelist_irks));

    uint32_t addr_cnt = (sizeof(whitelist_addrs) / sizeof(ble_gap_addr_t));
    uint32_t irk_cnt  = (sizeof(whitelist_irks)  / sizeof(ble_gap_irk_t));

    // Reload the whitelist and whitelist all peers.
    whitelist_load();

    // Get the whitelist previously set using pm_whitelist_set().
    err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                whitelist_irks,  &irk_cnt);

    if (((addr_cnt == 0) && (irk_cnt == 0)) ||
        (m_whitelist_disabled))
    {
        // Don't use whitelist.
        err_code = nrf_ble_scan_params_set(&m_scan, NULL);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    if(erasing_bonds == true || is_scanning == true)
        return;
    ret_code_t err_code;

    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    NRF_LOG_INFO("Starting scan.");

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    is_scanning = true;
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the power management module. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                         p_evt->conn_handle,
                         p_evt->params.att_mtu_effective);
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
                         p_evt->conn_handle,
                         p_evt->params.data_length);
        } break;

        default:
            break;
    }
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        {
            on_whitelist_req();
            m_whitelist_disabled = false;
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;
        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
            ble_gap_evt_connected_t const * p_connected =
            p_scan_evt->params.connected.p_connected;
            // Scan is automatically stopped by the connection.
            NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                        p_connected->peer_addr.addr[0],
                        p_connected->peer_addr.addr[1],
                        p_connected->peer_addr.addr[2],
                        p_connected->peer_addr.addr[3],
                        p_connected->peer_addr.addr[4],
                        p_connected->peer_addr.addr[5]
                        );
        } break;
        
        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            NRF_LOG_INFO("Scan filter match.");
            break;
        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
            break;

        default:
          break;
    }
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initialization scanning and setting filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param     = &m_scan_param;
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan,
                                       SCAN_UUID_FILTER,
                                       &m_rcs_uuid);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan,
                                           SCAN_NAME_FILTER,
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }

    if (is_connect_per_addr)
    {
       err_code = nrf_ble_scan_filter_set(&m_scan,
                                          SCAN_ADDR_FILTER,
                                          m_target_periph_addr.addr);
       APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_ble_scan_filters_enable(&m_scan,
                                           NRF_BLE_SCAN_ALL_FILTER,
                                           false);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}

APP_TIMER_DEF(m_notify_timer);

void command_char_notify_enable(void *p_context)
{
    ble_rcs_c_t *p_ble_rcs_c  = (ble_rcs_c_t *)p_context;
    uint32_t err_code;
    err_code = ble_rcs_c_cmd_notif_enable(p_ble_rcs_c);
    APP_ERROR_CHECK(err_code);
    
}

uint32_t command_char_notify_timer_create(void)
{
    uint32_t ret_code;
    ret_code = app_timer_create(&m_notify_timer, APP_TIMER_MODE_SINGLE_SHOT, command_char_notify_enable);
    return ret_code;
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of RCS client events.
 *
 * @param[in]   p_ble_rcs_c   RCS client handle. This identifies the RCS client.
 * @param[in]   p_ble_rcs_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_rcs_c module] */
static void ble_rcs_c_evt_handler(ble_rcs_c_t * p_ble_rcs_c, ble_rcs_c_evt_t const * p_ble_rcs_evt)
{
    ret_code_t err_code;

    switch (p_ble_rcs_evt->evt_type)
    {
        case BLE_RCS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_rcs_c_handles_assign(p_ble_rcs_c, p_ble_rcs_evt->conn_handle, &p_ble_rcs_evt->handles);
            APP_ERROR_CHECK(err_code);
            
            // Initiate bonding.
            err_code = pm_conn_secure(p_ble_rcs_evt->conn_handle, false);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
            err_code = app_timer_start(m_notify_timer, APP_TIMER_TICKS(250), (void*)p_ble_rcs_c);
            APP_ERROR_CHECK(err_code);       
            
            NRF_LOG_INFO("Connected to device with Remote Control Service.");
            break;

        case BLE_RCS_C_EVT_RCS_COMMAND_EVT:
            // TODO Send command to host control by UART.
            ble_rcs_command_notify_debug((ble_rcs_cmd_t *)p_ble_rcs_evt->p_command);
            break;

        case BLE_RCS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void rcs_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void rcs_c_init(void)
{
    ret_code_t err_code;
    ble_rcs_c_init_t init;

    init.evt_handler   = ble_rcs_c_evt_handler;
    init.error_handler = rcs_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_rcs_c_init(&m_ble_rcs_c, &init);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    peer_manager_init();
    db_discovery_init();
    rcs_c_init();
    scan_init();
    command_char_notify_timer_create();
    
    uart_init();
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    // Start execution.
    NRF_LOG_INFO("BLE remote control collector example started.");
    scan_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


