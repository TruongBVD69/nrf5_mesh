
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"


#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ble.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

#include "log.h"
#include "advertiser.h"
#include "ble_softdevice_support.h"

#include "nrf_mesh_config_examples.h"
#include "app_config.h"
#include "example_common.h"

#include "ad_type_filter.h"

#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
#include "stack_depth.h"
#endif

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define ADVERTISER_BUFFER_SIZE  (64)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
//static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
//static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
//                                               const access_message_rx_meta_t * p_meta,
//                                               const generic_onoff_status_params_t * p_in);
//static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
//                                                       void * p_args,
//                                                       access_reliable_status_t status);


/*****************************************************************************
 * Static variables
 *****************************************************************************/
static advertiser_t m_advertiser;

static uint8_t      m_adv_buffer[ADVERTISER_BUFFER_SIZE];
static bool         m_device_provisioned;

static void rx_cb(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    LEDS_OFF(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
    char msg[128];
    (void) sprintf(msg, "RX [@%u]: RSSI: %3d ADV TYPE: %x ADDR: [%02x:%02x:%02x:%02x:%02x:%02x]",
                   p_rx_data->p_metadata->params.scanner.timestamp,
                   p_rx_data->p_metadata->params.scanner.rssi,
                   p_rx_data->adv_type,
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[0],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[1],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[2],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[3],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[4],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[5]);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, msg, p_rx_data->p_payload, p_rx_data->length);
    LEDS_ON(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
}

static void adv_init(void)
{
    advertiser_instance_init(&m_advertiser, NULL, m_adv_buffer, ADVERTISER_BUFFER_SIZE);
}

static void adv_start(void)
{
    /* Let scanner accept Complete Local Name AD Type. */
    bearer_adtype_add(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);

    advertiser_enable(&m_advertiser);
    static const uint8_t adv_data[] =
    {
        0x13, /* AD data length (including type, but not itself) */
        BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, /* AD data type (Complete local name) */
        'N',  /* AD data payload (Name of device) */
        'o',
        'r',
        'd',
        'i',
        'c',
        ' ',
        'S',
        'e',
        'm',
        'i',
        ' ',
        'M',
        'e',
        's',
        'h',
        ' ',
        '2'
    };

    /* Allocate packet */
    adv_packet_t * p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if (p_packet)
    {
        /* Construct packet contents */
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));
        /* Repeat forever */
        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;

        advertiser_packet_send(&m_advertiser, p_packet);
    }

}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(HAL_LED_MASK, false);
    hal_led_blink_ms(HAL_LED_MASK_HALF,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}


/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}


#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t------------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) Send a message to the odd group (address: 0xC003) to turn on LED 1.\n"
    "\t\t Button/RTT 2) Send a message to the odd group (address: 0xC003) to turn off LED 1.\n"
    "\t\t Button/RTT 3) Send a message to the even group (address: 0xC002) to turn on LED 1.\n"
    "\t\t Button/RTT 4) Send a message to the even group (address: 0xC002) to turn off LED 1.\n"
    "\t\t------------------------------------------------------------------------------------\n";
#endif


static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc     = DEV_BOARD_LF_CLK_CFG,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reboot device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }

    /* Start listening for incoming packets */
    nrf_mesh_rx_cb_set(rx_cb);

    /* Initialize the advertiser */
    adv_init();
}

static void initialize(void)
{
#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
    stack_depth_paint_stack();
#endif

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Beacon Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();

     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initialization complete!\n");
}

static void start(void)
{

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_BEACON
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

  /* Start advertising own beacon */
    /* Note: If application wants to start beacons at later time, adv_start() API must be called
     * from the same IRQ priority context same as that of the Mesh Stack. */
    adv_start();

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
