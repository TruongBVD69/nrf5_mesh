
#include <stdint.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

#include "nrf_drv_clock.h"
#include "timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "mesh_config.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_light_ctl.h"
#include "app_light_lightness.h"
#include "app_level.h"
#include "app_config.h"
#include "example_common.h"
#include "light_lightness_utils.h"
#include "light_ctl_utils.h"
#include "ble_softdevice_support.h"
#include "app_timer.h"
#include "model_config_file.h"
#include "mesh_config_listener.h"
#include "light_ctl_mc.h"
#include "app_scene.h"
#include "generic_level_mc.h"
#include "generic_level_server.h"
#include "app_dtt.h"

#include "nrf_delay.h"
#include "ds18b20.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// ADC
#include "nrf_drv_saadc.h"


#include "mesh_friend.h"
#include "heartbeat.h"

#include "display.h"
#include "74hc595.h"

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */



/*****************************************************************************
 * Definitions
 *****************************************************************************/
/* Client lightness parameter step size */
#define APP_CTL_LIGHTNESS_STEP_SIZE     (16384)
/* Client temperature parameter step size */
#define APP_CTL_TEMPERATURE_STEP_SIZE   (8000)
/* Client delta uv parameter step size */
#define APP_CTL_DELTA_UV_STEP_SIZE      (16384)

#define APP_LEVEL_STEP_SIZE         (16384L)
#define APP_LEVEL_ELEMENT_INDEX     (2)



/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION          (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                    (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Gives the light CTL element index. */
#define APP_CTL_ELEMENT_INDEX           (0)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void mesh_events_handle(const nrf_mesh_evt_t * p_evt);

static void app_level_server_set_cb(const app_level_server_t * p_server, int16_t present_level);
static void app_level_server_get_cb(const app_level_server_t * p_server, int16_t * p_present_level);
static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                                uint32_t transition_time_ms, uint16_t target_level,
                                                app_transition_type_t transition_type);

static void light_ctl_set_cb(const app_light_ctl_setup_server_t * p_app, app_light_ctl_temperature_duv_hw_state_t * present_ctl_state);
static void light_ctl_get_cb(const app_light_ctl_setup_server_t * p_app, app_light_ctl_temperature_duv_hw_state_t * present_ctl_state);
static void ligth_ctl_transition_cb(const app_light_ctl_setup_server_t * p_server,
                                         uint32_t transition_time_ms,
                                         app_light_ctl_temperature_duv_hw_state_t target_ctl_state);

static void light_ctl_lightness_set_cb(const app_light_lightness_setup_server_t * p_app, uint16_t lightness);
static void light_ctl_lightness_get_cb(const app_light_lightness_setup_server_t * p_app, uint16_t * p_lightness);
static void temperature32_range_listener_cb(mesh_config_change_reason_t reason,
                                            mesh_config_entry_id_t id,
                                            const void * p_entry);

static void ligth_ctl_lightness_transition_cb(const app_light_lightness_setup_server_t * p_server,
                                                   uint32_t transition_time_ms, uint16_t target_lightness);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void scene_transition_lightness_cb(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene);

static void app_level_scene_transition_cb(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene);
#endif

static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static bool m_device_provisioned;
static nrf_mesh_evt_handler_t m_event_handler =
{
    .evt_cb = mesh_events_handle,
};

float temp = 0;
float volts = 0;

/* CTL Setup Server and associated model structures' definition and initialization */
APP_LIGHT_LIGHTNESS_SETUP_SERVER_DEF(m_light_ctl_server_0_ll,
                                     APP_FORCE_SEGMENTATION,
                                     APP_MIC_SIZE,
                                     light_ctl_lightness_set_cb,
                                     light_ctl_lightness_get_cb,
                                     ligth_ctl_lightness_transition_cb)
APP_LIGHT_CTL_SETUP_SERVER_DEF(m_light_ctl_server_0,
                               APP_FORCE_SEGMENTATION,
                               APP_MIC_SIZE,
                               light_ctl_set_cb,
                               light_ctl_get_cb,
                               ligth_ctl_transition_cb)
/* Application level generic level server structure definition and initialization */
APP_LEVEL_SERVER_DEF(m_level_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     NULL,
                     app_level_server_set_cb,
                     app_level_server_get_cb,
                     app_level_server_transition_cb)

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
/* Scene Setup server structure definition and initialization */
APP_SCENE_SETUP_SERVER_DEF(m_scene_server_0,
                           APP_FORCE_SEGMENTATION,
                           APP_MIC_SIZE,
                           scene_transition_lightness_cb,
                           &m_light_ctl_server_0_ll.light_lightness_setup_server.generic_ponoff_setup_srv.generic_dtt_srv);
APP_DTT_SERVER_DEF(m_dtt_server_0,
                   APP_FORCE_SEGMENTATION,
                   APP_MIC_SIZE,
                   NULL)

/* Scene Setup server structure definition and initialization */
APP_SCENE_SETUP_SERVER_DEF(m_scene_server_1,
                          APP_FORCE_SEGMENTATION,
                          APP_MIC_SIZE,
                           app_level_scene_transition_cb,
                           &m_dtt_server_0.server);
#endif

/* Application variables for holding instantaneous CTL state values */
static app_light_ctl_temperature_duv_hw_state_t m_pwm0_present_ctl_value;
static uint16_t Lightness_to_Temp = 0;

/* Application variable for holding instantaneous level value */
static int32_t Level_to_Volts = 0;


/* Callback for updating the hardware state */
static void app_level_server_set_cb(const app_level_server_t * p_server, int16_t present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    Level_to_Volts = present_level;

}

/* Callback for reading the hardware state */
static void app_level_server_get_cb(const app_level_server_t * p_server, int16_t * p_present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    *p_present_level = Level_to_Volts;
}

/* Callback for updateing according to transition time. */
static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                                uint32_t transition_time_ms, uint16_t target_level,
                                                app_transition_type_t transition_type)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target level: %d, Transition type: %d\n",
                                       transition_time_ms, target_level, transition_type);
}


/* Callback for updating the hardware state */
static void light_ctl_set_cb(const app_light_ctl_setup_server_t * p_app,
                             app_light_ctl_temperature_duv_hw_state_t * p_present_ctl_state)
{
    m_pwm0_present_ctl_value.temperature32 = 0;
    m_pwm0_present_ctl_value.delta_uv = 0;
}

/* Callback for reading the hardware state */
static void light_ctl_get_cb(const app_light_ctl_setup_server_t * p_app,
                             app_light_ctl_temperature_duv_hw_state_t * p_present_ctl_state)
{
    p_present_ctl_state->temperature32 = m_pwm0_present_ctl_value.temperature32;
    p_present_ctl_state->delta_uv = m_pwm0_present_ctl_value.delta_uv;
    int *temppp = &(p_present_ctl_state->temperature32);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "actual temp = %d \n", p_present_ctl_state->temperature32);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "actual tempppppp = %d \n", *temppp);
}

/* Callback for implementing custom HW interface using given transition time */
static void ligth_ctl_transition_cb(const app_light_ctl_setup_server_t * p_server,
                                         uint32_t transition_time_ms,
                                         app_light_ctl_temperature_duv_hw_state_t target_ctl_state)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target temperature: %d, Target delta_uv: %d\n",
          transition_time_ms, light_ctl_utils_temperature32_to_temperature(target_ctl_state.temperature32),
          target_ctl_state.delta_uv);
}

/* Callback for updating the hardware state for lightness */
static void light_ctl_lightness_set_cb(const app_light_lightness_setup_server_t * p_app, uint16_t lightness)
{
    Lightness_to_Temp = lightness;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "actual lightness = %d \n", Lightness_to_Temp);

}

/* Callback for reading the hardware state */
static void light_ctl_lightness_get_cb(const app_light_lightness_setup_server_t * p_app, uint16_t * p_lightness)
{
    *p_lightness = Lightness_to_Temp;
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "actual lightness = %d \n", *p_lightness);

}

/* Callback for receiveing transition time. */
static void ligth_ctl_lightness_transition_cb(const app_light_lightness_setup_server_t * p_server,
                                                   uint32_t transition_time_ms, uint16_t target_lightness)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target lightness: %d\n",
                                       transition_time_ms, target_lightness);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void scene_transition_lightness_cb(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target Scene: %d\n",
                                       transition_time_ms, target_scene);
}

static void app_level_scene_transition_cb(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target Scene: %d\n",
                                       transition_time_ms, target_scene);
}
#endif

static void models_init_cb(void)
{
    /* Initialize the CTL Setup Server and associated models */
    /* Initialize the Light Lightness Setup Server - the CTL main element is shared with light lightness */
    APP_ERROR_CHECK(app_light_lightness_model_init(&m_light_ctl_server_0_ll, APP_CTL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Light Lightness Model handle: %d, Element index: %d\n",
          m_light_ctl_server_0_ll.light_lightness_setup_server.model_handle,
          m_light_ctl_server_0_ll.light_lightness_setup_server.settings.element_index);
    /* Initialize the CTL Setup Server */
    APP_ERROR_CHECK(app_light_ctl_model_init(&m_light_ctl_server_0, APP_CTL_ELEMENT_INDEX, &m_light_ctl_server_0_ll));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App CTL Model Handle: %d, Element index: %d\n",
          m_light_ctl_server_0.light_ctl_setup_srv.model_handle,
          m_light_ctl_server_0.light_ctl_setup_srv.settings.element_index);

        /* Instantiate level server on element index APP_LEVEL_ELEMENT_INDEX */
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_LEVEL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Level Model handle: %d\n",
          m_level_server_0.server.model_handle);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    /* Instantiate scene server and register light CTL server to have scene support */
    ERROR_CHECK(app_scene_model_init(&m_scene_server_0, APP_CTL_ELEMENT_INDEX));

    ERROR_CHECK(app_scene_model_add(&m_scene_server_0, &m_light_ctl_server_0_ll.scene_if));
    ERROR_CHECK(app_light_lightness_scene_context_set(&m_light_ctl_server_0_ll, &m_scene_server_0));

    ERROR_CHECK(app_scene_model_add(&m_scene_server_0, &m_light_ctl_server_0.scene_if));
    ERROR_CHECK(app_light_ctl_scene_context_set(&m_light_ctl_server_0, &m_scene_server_0));

    /* Instantiate Generic Default Transition Time server as needed by Scene models */
    ERROR_CHECK(app_dtt_init(&m_dtt_server_0, APP_LEVEL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App DTT Model handle: %d\n",
          m_dtt_server_0.server.model_handle);

  /* Instantiate scene server and register onoff server to have scene support */
   ERROR_CHECK(app_scene_model_init(&m_scene_server_1, APP_LEVEL_ELEMENT_INDEX));
   ERROR_CHECK(app_scene_model_add(&m_scene_server_1, &m_level_server_0.scene_if));
   ERROR_CHECK(app_level_scene_context_set(&m_level_server_0, &m_scene_server_1));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Scene Model handle: %d, Element index: %d\n",
          m_scene_server_0.scene_setup_server.model_handle,
          m_scene_server_0.scene_setup_server.settings.element_index);
#endif
}

/*************************************************************************************************/

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_ENABLED)
    {

        /* The onpowerup/last/actual binding is required at boot time to restore the correct state
         * of the lightness model. */
        APP_ERROR_CHECK(app_light_lightness_binding_setup(&m_light_ctl_server_0_ll));

        /* Note that the mid app is expecting the lightness binding setup first - when the publish
         * occurs for the ctl binding, lightness will have already been set. */
        APP_ERROR_CHECK(app_light_ctl_binding_setup(&m_light_ctl_server_0));

        APP_ERROR_CHECK(app_level_value_restore(&m_level_server_0));
    }
#if NRF_MESH_LOG_ENABLE
    else if (p_evt->type == NRF_MESH_EVT_CONFIG_LOAD_FAILURE)
    {
        const nrf_mesh_evt_config_load_failure_t * p_details = &p_evt->params.config_load_failure;
        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Corrupted entry: file:%d record:%d reason:%d\n",
              p_details->id.file, p_details->id.record, p_details->reason);
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DBG1, "Raw data:", (const uint8_t *)p_details->p_data, p_details->data_len);
    }
#endif

    //switch (p_evt->type)
    //{
    //  case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
    //   {
    //      const nrf_mesh_evt_friendship_established_t *p_establish = &p_evt->params.friendship_established;

    //      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ESTABLISH FRIENDSHIP from 0x%04x\n", p_establish->friend_src);

    //      uint32_t status = friend_friendship_established(p_establish);
    //    break;
    //    }
       
    //  //case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
    //  //{
    //  //  const nrf_mesh_evt_friendship_terminated_t *p_term = &p_evt->params.friendship_terminated;

    //   default:
    //      break;
    // }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");

    /* This function may return if there are ongoing flash operations. */
    model_config_file_clear();
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
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
    "\t\t------------------------------------------------------------------------------------------\n"
    "\t\t RTT 1)                                                                                   \n"
    "\t\t RTT 2)                        ??????                                                           \n"
    "\t\t RTT 3)                                                                                   \n"
    "\t\t------------------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        
        case 1:
        {
          displayFloat(Lightness_to_Temp/100.0);
            nrf_gpio_pin_write(LED_1, 0);
          
            break;
        }

        case 2:
        {
          displayFloat(Level_to_Volts/1000.0);
            nrf_gpio_pin_write(LED_2, 0);
          
            break;
        }

        case 3:
        {
          dsm_local_unicast_address_t node_address;
          dsm_local_unicast_addresses_get(&node_address);
          displayInt(node_address.address_start);
            nrf_gpio_pin_write(LED_3, 0);
            break;
        }

        /* Initiate node reset */
        case 4:
        {
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
#endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }

}

static void app_rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
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
    /* Restores the application parameters after switching from the Provisioning service to the
     * Proxy */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();

    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);

}

static void mesh_init(void)
{
    /* Initialize the application storage for models */
    model_config_file_init();

    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);

    if (status == NRF_SUCCESS)
    {
        /* Check if application stored data is valid, if not clear all data and use default values. */
        status = model_config_file_config_apply();
    }

    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            /* Clear model config file as loading failed */
            model_config_file_clear();
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reboot device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            APP_ERROR_CHECK(status);
    }

       
}

static void initialize(void)
{

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Sensor Setup Server Demo -----\n");

    APP_ERROR_CHECK(app_timer_init());
    hal_leds_init();
#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();


}

static void start(void)
{

    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = NULL,
            .p_device_uri = EX_URI_CTL_SERVER
        };
        APP_ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    /* NRF_MESH_EVT_ENABLED is triggered in the mesh IRQ context after the stack is fully enabled.
     * This event is used to call Model APIs for establishing bindings and publish a model state information. */
    nrf_mesh_evt_handler_add(&m_event_handler);
    APP_ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}
void saadc_callback_handler(nrf_drv_saadc_evt_t * p_event)
{

}
void saadc_init(void)
{
  ret_code_t err_code;
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);

  err_code = nrf_drv_saadc_init(NULL, saadc_callback_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

}



/* Entry-point */

int main(void)
{
    initialize();
    start();

    saadc_init();

    displayInit();
    displayInt(8888);

    nrf_saadc_value_t adc_val = 0;

    ds18b20_setResolution(12);

    for (;;)
    {
        (void)sd_app_evt_wait();

        nrfx_saadc_sample_convert(0, &adc_val);

        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sample value read: %d \n", adc_val);
        volts = adc_val * 4.65 / 16384;

        temp = ds18b20_get_temp_method_2();

        Level_to_Volts = volts * 1000;
        Lightness_to_Temp = temp*100;

        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data pin=%d\n", Level_to_Volts);
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data temperature=%d\n", Lightness_to_Temp);

        displayStop();
        

        nrf_delay_ms(2000);

    }
}
