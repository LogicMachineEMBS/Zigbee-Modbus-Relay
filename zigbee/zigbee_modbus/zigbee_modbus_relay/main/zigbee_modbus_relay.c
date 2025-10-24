// Copyright (c) 2025 LogicMachine
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

// This file is based on or incorporates material from the Espressif IoT Development Framework (esp-idf),
// which is licensed under the Apache License, Version 2.0.
// A copy of the original license can be found in the LICENSE-3RD-PARTY.txt file.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "zcl_utility.h"
#include "esp_system.h"
#include "zcl/esp_zigbee_zcl_basic.h"

#include "esp_err.h"
#include "mbcontroller.h"
#include "modbus_params.h"
#include "sdkconfig.h"

#define MB_PORT_NUM 1
#define MB_SLAVE_ADDR 1
#define MB_DEV_SPEED 9600

static const char *NVS_NAMESPACE = "storage";
static const char *NVS_MODBUS_ID_KEY = "modbus_id";
static const char *NVS_MODBUS_BAUDRATE_KEY = "modbus_baudrate";
static const char *NVS_MODBUS_PARITY_KEY = "modbus_parity";

nvs_handle my_handle;

// Defines below are used to define register start address for each type of Modbus registers
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_COILS_START                  (0x0000)
#define MB_REG_INPUT_START_AREA0            (INPUT_OFFSET(input_data0)) // register offset input area 0
#define MB_REG_INPUT_START_AREA1            (INPUT_OFFSET(input_data4)) // register offset input area 1
#define MB_REG_HOLDING_START_AREA0          (HOLD_OFFSET(holding_data0))
#define MB_REG_HOLDING_START_AREA1          (HOLD_OFFSET(holding_data4))

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_CHAN_DATA_MAX_VAL                (6)
#define MB_CHAN_DATA_OFFSET                 (0.2f)
#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)


uint8_t modbus_id = 1;
uint32_t modbus_baudrate = 9600;
uint8_t modbus_parity = 0;

bool saved_modbus_id = false;
bool saved_modbus_baudrate = false;
bool saved_modbus_parity = false;

uint8_t button_holding_time = 0;

/* 
  MB_PARITY_NONE = 0
  MB_PARITY_EVEN = 1
  MB_PARITY_EVEN = 2
  MB_PARITY_ODD = 3
*/

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false                                /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN        /* aging timeout of device */
#define ED_KEEP_ALIVE                   3000                                 /* 3000 millisecond */
#define HA_ESP_SOCKET_ENDPOINT          10                                   /* esp socket device endpoint, used to process socket controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */
#define SOCKET_CONTROL_PIN              GPIO_NUM_0                           // RELAY A1  ESPZ2Mv1.0
#define BUTTON_PIN                      GPIO_NUM_9                           // GPIO pin for button ESPZ2Mv1.0
#define GREEN_LED                       GPIO_NUM_22                          // GPIO pin Green Led ESPZ2Mv1.0
#define RED_LED                         GPIO_NUM_23                          // GPIO pin Red Led ESPZ2Mv1.0


/* Enhanced compatibility and optimization settings */
#define SOCKET_MIN_TOGGLE_INTERVAL     500                                 /* Minimum time (ms) between state changes */
#define ZB_ENHANCED_KEEP_ALIVE         2000                                /* Optimized keep-alive interval (ms) */
#define ZB_REJOIN_INTERVAL             3600                                /* Rejoin attempt interval (seconds) */
#define ZB_KEEP_ALIVE_ATTEMPTS         3                                   /* Number of keep-alive attempts before rejoin */

#define ESP_MANUFACTURER_NAME "\x04""EMBS"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07""relay_1" /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                                 \
{                                                           \
  .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                     \
  .install_code_policy = INSTALLCODE_POLICY_ENABLE,         \
  .nwk_cfg.zed_cfg = {                                      \
    .ed_timeout = ED_AGING_TIMEOUT,                         \
    .keep_alive = ZB_ENHANCED_KEEP_ALIVE,                   \
  },                                                        \
}

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                       \
{                                                           \
  .radio_mode = ZB_RADIO_MODE_NATIVE,                       \
}

#define ESP_ZB_DEFAULT_HOST_CONFIG()                        \
{                                                           \
  .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,     \
}

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile socket (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_ON_OFF_SOCKET";

/* Socket state tracking */
static struct {
  bool current_state;
  int64_t last_toggle_time;
  uint8_t keep_alive_fails;
} socket_state = {
  .current_state = false,
  .last_toggle_time = 0,
  .keep_alive_fails = 0
};


esp_err_t save_modbus_id_to_nvs(uint8_t id){
  return nvs_set_u8(my_handle, NVS_MODBUS_ID_KEY, id);
}

void load_modbus_id_from_nvs(){
  uint8_t id;
  nvs_get_u8(my_handle, NVS_MODBUS_ID_KEY, &id);
  if(id >= 1 && id <= 247){
    modbus_id = id;
  }
}

esp_err_t save_modbus_baudrate_to_nvs(uint32_t baudrate){
  return nvs_set_u32(my_handle, NVS_MODBUS_BAUDRATE_KEY, baudrate);
}

void load_modbus_baudrate_from_nvs(){
  uint32_t baudrate;
  nvs_get_u32(my_handle, NVS_MODBUS_BAUDRATE_KEY, &baudrate);
  if(baudrate >= 9600 && baudrate <= 115200){
    modbus_baudrate = baudrate;
  }
}

esp_err_t save_modbus_parity_to_nvs(uint8_t parity){
  return nvs_set_u8(my_handle, NVS_MODBUS_PARITY_KEY, parity);
}

void load_modbus_parity_from_nvs(){
  uint8_t parity;
  nvs_get_u8(my_handle, NVS_MODBUS_PARITY_KEY, &parity);
  if(parity >= 0 && parity <= 3){
    modbus_parity = parity;
  }
}

static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

mb_param_info_t reg_info; // keeps the Modbus registers access information
mb_communication_info_t comm_info; // Modbus communication parameters
mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

static void setup_reg_data(void){
  coil_reg_params.coils_port0 = 0;
}


static esp_err_t socket_driver_init(void){
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << SOCKET_CONTROL_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret = gpio_config(&io_conf);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to configure GPIO");
  gpio_set_level(SOCKET_CONTROL_PIN, 0);  // Initialize to OFF state

  gpio_config_t button_conf = {
    .pin_bit_mask = (1ULL << BUTTON_PIN),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE, // enable pull-up resistor
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE, // without interupts
  };
  esp_err_t ret_button = gpio_config(&button_conf);
  ESP_RETURN_ON_ERROR(ret_button, TAG, "Failed to configure BUTTON GPIO");

  gpio_config_t red_led_conf = {
    .pin_bit_mask = (1ULL << RED_LED),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret_red_led = gpio_config(&red_led_conf);
  ESP_RETURN_ON_ERROR(ret_red_led, TAG, "Failed to configure RED LED GPIO");
  gpio_set_level(RED_LED, 0);

  gpio_config_t green_led_conf = {
    .pin_bit_mask = (1ULL << GREEN_LED),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret_green_led = gpio_config(&green_led_conf);
  ESP_RETURN_ON_ERROR(ret_green_led, TAG, "Failed to configure GREEN LED GPIO");
  gpio_set_level(GREEN_LED, 0);

  return ESP_OK;
}

static void socket_set_power(bool state){
  if (state != socket_state.current_state) {
    int64_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - socket_state.last_toggle_time < SOCKET_MIN_TOGGLE_INTERVAL) {
      ESP_LOGW(TAG, "Ignoring too frequent toggle request");
      return;
    }
        
    gpio_set_level(SOCKET_CONTROL_PIN, state ? 1 : 0);
        
    if(state){
      coil_reg_params.coils_port0 |= 0x01;
    }
    else{
      coil_reg_params.coils_port0 &= 0xFE;
    }
        
    socket_state.current_state = state;
    socket_state.last_toggle_time = current_time;
  }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask){
  ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct){
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "Initialize Zigbee stack");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        ESP_LOGI(TAG, "Deferred driver initialization %s", socket_driver_init() ? "failed" : "successful");
        ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
          if (esp_zb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
          } else {
            ESP_LOGI(TAG, "Device rebooted");
          }
      } else {
        /* commissioning failed */
        ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
      }
      socket_state.keep_alive_fails = 0; // Reset counter on reboot
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
          extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
          extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
          esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
      } else {
        ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
    case ESP_ZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED:
      ESP_LOGW(TAG, "PAN ID conflict detected, attempting network rejoin");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
      ESP_LOGI(TAG, "Left network, scheduling rejoin");
      socket_state.keep_alive_fails = 0;
      esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
        ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      break;
    default:
      ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
        esp_err_to_name(err_status));
      break;
  }
}


static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message){
  esp_err_t ret = ESP_OK;
  bool socket_state = 0;

  ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
  ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
    message->info.status);
  ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
    message->attribute.id, message->attribute.data.size);
  if (message->info.dst_endpoint == HA_ESP_SOCKET_ENDPOINT) {
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
      if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
        socket_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : socket_state;
        ESP_LOGI(TAG, "Socket sets to %s", socket_state ? "On" : "Off");
        socket_set_power(socket_state);
      }
    } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
      if (message->attribute.id == 0xF000 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
        modbus_id = *(uint8_t *)message->attribute.data.value;
        ESP_LOGI(TAG, "Modbus ID set to %d", modbus_id);
        save_modbus_id_to_nvs(modbus_id);
        saved_modbus_id = true;
      }
      else if(message->attribute.id == 0xF001 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U32) {
        modbus_baudrate = *(uint32_t *)message->attribute.data.value;
        ESP_LOGI(TAG, "Modbus Baudrate set to %u ", (unsigned int) modbus_baudrate);
        save_modbus_baudrate_to_nvs(modbus_baudrate);
        saved_modbus_baudrate = true;
      }
      else if(message->attribute.id == 0xF002 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
        modbus_parity = *(uint8_t *)message->attribute.data.value;
        ESP_LOGI(TAG, "Modbus Parity set to %d ", modbus_parity);
        save_modbus_parity_to_nvs(modbus_parity);
        saved_modbus_parity = true;
      }
    }
  }
  return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message){
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
      ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
      break;
    default:
      ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
      break;
  }
  return ret;
}


void modbus_loop(){
  // The cycle below will be terminated when parameter holdingRegParams.dataChan0
  // incremented each access cycle reaches the CHAN_DATA_MAX_VAL value.
  for(;holding_reg_params.holding_data0 < MB_CHAN_DATA_MAX_VAL;) {
    // Check for read/write events of Modbus master for certain events
    mb_event_group_t event = mbc_slave_check_event(MB_READ_WRITE_MASK);
    const char* rw_str = (event & MB_READ_MASK) ? "READ" : "WRITE";

    // Filter events and process them accordingly
    if(event & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD)) {
      // Get parameter information from parameter queue
      ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
      
      if (reg_info.address == (uint8_t*)&holding_reg_params.holding_data0){
        portENTER_CRITICAL(&param_lock);
        holding_reg_params.holding_data0 += MB_CHAN_DATA_OFFSET;
        if (holding_reg_params.holding_data0 >= (MB_CHAN_DATA_MAX_VAL - MB_CHAN_DATA_OFFSET)) {
          //coil_reg_params.coils_port1 = 0xFF;
        }
        portEXIT_CRITICAL(&param_lock);
      }
    } else if (event & MB_EVENT_INPUT_REG_RD) {
      ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
      
    } else if (event & MB_EVENT_DISCRETE_RD) {
      ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
      
    } else if (event & (MB_EVENT_COILS_RD | MB_EVENT_COILS_WR)) {
      ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));    

      uint8_t my_coils = coil_reg_params.coils_port0;
      
      bool new_state = (0x01 & my_coils) ? true : false;

      if (new_state != socket_state.current_state){
        socket_state.current_state = new_state;
        socket_state.last_toggle_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ESP_LOGI(TAG, "Socket state changed by Modbus to %s", new_state ? "ON" : "OFF");
        // notify zigbee state changed
        esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(HA_ESP_SOCKET_ENDPOINT,
          ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
          ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, (void*)&socket_state.current_state, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS){
          ESP_LOGE(TAG, "Error reporting the zigbee status");
        }              
      }

      gpio_set_level(SOCKET_CONTROL_PIN, new_state ? 1 : 0);
      //
    }
  }
    // Destroy of Modbus controller on alarm
  ESP_LOGI(TAG,"Modbus controller destroyed.");
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(mbc_slave_destroy());
}


static void esp_zb_task(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(500));
  /* initialize Zigbee stack */
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
  esp_zb_init(&zb_nwk_cfg);
  esp_zb_on_off_light_cfg_t socket_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
  esp_zb_ep_list_t *esp_zb_on_off_socket_ep = esp_zb_on_off_light_ep_create(HA_ESP_SOCKET_ENDPOINT, &socket_cfg);
  zcl_basic_manufacturer_info_t info = {
    .manufacturer_name = ESP_MANUFACTURER_NAME,
    .model_identifier = ESP_MODEL_IDENTIFIER,
    .modbus_id = modbus_id,
    .modbus_baudrate = modbus_baudrate,
    .modbus_parity = modbus_parity
  };

  esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_on_off_socket_ep, HA_ESP_SOCKET_ENDPOINT, &info);

  esp_zb_device_register(esp_zb_on_off_socket_ep);
  esp_zb_core_action_handler_register(zb_action_handler);
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_stack_main_loop();
}



void app_main(void){
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
  load_modbus_id_from_nvs();
  load_modbus_baudrate_from_nvs();
  load_modbus_parity_from_nvs();

  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

  void* mbc_slave_handler = NULL;
  ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave_handler)); // Initialization of Modbus controller 

  // Setup communication parameters and start stack

  comm_info.mode = MB_MODE_RTU,
  //comm_info.slave_addr = MB_SLAVE_ADDR;
  comm_info.slave_addr = modbus_id;
  comm_info.port = MB_PORT_NUM;
  //comm_info.baudrate = MB_DEV_SPEED;
  comm_info.baudrate = modbus_baudrate;
  //comm_info.parity = MB_PARITY_NONE;
  comm_info.parity = modbus_parity;
  
  /* 
  MB_PARITY_NONE = 0
  MB_PARITY_EVEN = 1
  MB_PARITY_EVEN = 2
  MB_PARITY_ODD = 3
  */

  ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

  // Initialization of Coils register area
  reg_area.type = MB_PARAM_COIL;
  reg_area.start_offset = MB_REG_COILS_START;
  reg_area.address = (void*)&coil_reg_params;
  reg_area.size = sizeof(coil_reg_params);
  ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));


  setup_reg_data(); // Set values into known state

  // Starts of modbus controller and stack
  ESP_ERROR_CHECK(mbc_slave_start());

  // Set UART pin numbers
  /*
  ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD,
                          CONFIG_MB_UART_RXD, CONFIG_MB_UART_RTS,
                          UART_PIN_NO_CHANGE));
  */
  ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, 21,
                          20, 19,
                          UART_PIN_NO_CHANGE)); // CONFIG_MB_UART_RTS is not connected

  // Set UART driver mode to Half Duplex
  ESP_ERROR_CHECK(uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));

  ESP_LOGI(TAG, "Modbus slave stack initialized.");
  ESP_LOGI(TAG, "Start modbus test...");
  
  xTaskCreate(modbus_loop, "modbus_loop", 4096, NULL, 5, NULL);

  while(1){
    if(saved_modbus_id && saved_modbus_baudrate && saved_modbus_parity){
      vTaskDelay(pdMS_TO_TICKS(100));
      esp_restart();
    }

    if(gpio_get_level((gpio_num_t) BUTTON_PIN) == 0){
      button_holding_time++;
      gpio_set_level(GREEN_LED, 1);
      //button_holding_time
      while( gpio_get_level((gpio_num_t) BUTTON_PIN) == 0){
        button_holding_time++;
        if(button_holding_time >= 40){
          gpio_set_level(RED_LED, button_holding_time % 2);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }

      if(button_holding_time > 0 && button_holding_time < 40){
        esp_restart();
      }
      else if(button_holding_time >= 40){
        // zigbee config reset
        gpio_set_level(GREEN_LED, 1);
        gpio_set_level(RED_LED, 1);
        esp_zb_bdb_reset_via_local_action();

        save_modbus_id_to_nvs(1);
        save_modbus_baudrate_to_nvs(9600);
        save_modbus_parity_to_nvs(0);

        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

}