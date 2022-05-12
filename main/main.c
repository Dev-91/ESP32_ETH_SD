#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#if CONFIG_ETH_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif // CONFIG_ETH_USE_SPI_ETHERNET

// #include <stdio.h>
#include <stdint.h>
#include <stddef.h>
// #include <string.h>
// #include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "protocol_examples_common.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

// #include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG_ETH = "ethernet";
static const char *TAG_MQTT = "mqtt";

// #define CONFIG_USER_SPI_ETHERNETS_NUM 1
// #define CONFIG_USER_ETH_SPI_HOST 1
// #define CONFIG_USER_ETH_SPI_CLOCK_MHZ 12

// #define CONFIG_USER_ETH_SPI_MISO_GPIO 12
// #define CONFIG_USER_ETH_SPI_MOSI_GPIO 13
// #define CONFIG_USER_ETH_SPI_SCLK_GPIO 14

// #define CONFIG_USER_ETH_SPI_CS0_GPIO 15
// #define CONFIG_USER_ETH_SPI_INT0_GPIO 1
// #define CONFIG_USER_ETH_SPI_PHY_RST0_GPIO -1
// #define CONFIG_USER_ETH_SPI_PHY_ADDR0 1

// #define CONFIG_USER_USE_SPI_ETHERNET true
// #define CONFIG_USER_USE_W5500 true

#if CONFIG_USER_USE_SPI_ETHERNET
#define INIT_SPI_ETH_MODULE_CONFIG(eth_module_config, num)                                  \
    do {                                                                                    \
        eth_module_config[num].spi_cs_gpio = CONFIG_USER_ETH_SPI_CS ##num## _GPIO;          \
        eth_module_config[num].int_gpio = CONFIG_USER_ETH_SPI_INT ##num## _GPIO;            \
        eth_module_config[num].phy_reset_gpio = CONFIG_USER_ETH_SPI_PHY_RST ##num## _GPIO;  \
        eth_module_config[num].phy_addr = CONFIG_USER_ETH_SPI_PHY_ADDR ##num;               \
    } while(0)

typedef struct{
    uint8_t spi_cs_gpio;
    uint8_t int_gpio;
    int8_t phy_reset_gpio;
    uint8_t phy_addr;
} spi_eth_module_config_t;
#endif

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG_ETH, "Ethernet Link Up!!");
        ESP_LOGI(TAG_ETH, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_ETH, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG_ETH, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG_ETH, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG_ETH, "Ethernet Got IP Address");
    ESP_LOGI(TAG_ETH, "=========================");
    ESP_LOGI(TAG_ETH, "ETH-IP   : " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG_ETH, "ETH-MASK : " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG_ETH, "ETH-GW   : " IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG_ETH, "=========================");
}

void ethernet_connect(void)
{
    // MQTT example init
    ESP_ERROR_CHECK(nvs_flash_init());
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_USER_USE_SPI_ETHERNET
    // Create instance(s) of esp-netif for SPI Ethernet(s)
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg_spi = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    esp_netif_t *eth_netif_spi[CONFIG_USER_SPI_ETHERNETS_NUM] = { NULL };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < CONFIG_USER_SPI_ETHERNETS_NUM; i++) {
        itoa(i, num_str, 10);
        strcat(strcpy(if_key_str, "ETH_SPI_"), num_str);
        strcat(strcpy(if_desc_str, "eth"), num_str);
        esp_netif_config.if_key = if_key_str;
        esp_netif_config.if_desc = if_desc_str;
        esp_netif_config.route_prio = 30 - i;
        eth_netif_spi[i] = esp_netif_new(&cfg_spi);
    }

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config_spi = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config_spi = ETH_PHY_DEFAULT_CONFIG();

    // Install GPIO ISR handler to be able to service SPI Eth modlues interrupts
    gpio_install_isr_service(0);

    // Init SPI bus
    spi_device_handle_t spi_handle[CONFIG_USER_SPI_ETHERNETS_NUM] = { NULL };
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_USER_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_USER_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_USER_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_USER_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Init specific SPI Ethernet module configuration from Kconfig (CS GPIO, Interrupt GPIO, etc.)
    spi_eth_module_config_t spi_eth_module_config[CONFIG_USER_SPI_ETHERNETS_NUM];
    INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 0);

// #if CONFIG_USER_SPI_ETHERNETS_NUM > 1
//     INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 1);
// #endif
    // Configure SPI interface and Ethernet driver for specific SPI module
    esp_eth_mac_t *mac_spi[CONFIG_USER_SPI_ETHERNETS_NUM];
    esp_eth_phy_t *phy_spi[CONFIG_USER_SPI_ETHERNETS_NUM];
    esp_eth_handle_t eth_handle_spi[CONFIG_USER_SPI_ETHERNETS_NUM] = { NULL };

#if CONFIG_USER_USE_W5500
    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
        .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = CONFIG_USER_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20
    };

    for (int i = 0; i < CONFIG_USER_SPI_ETHERNETS_NUM; i++) {
        // Set SPI module Chip Select GPIO
        devcfg.spics_io_num = spi_eth_module_config[i].spi_cs_gpio;

        ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_USER_ETH_SPI_HOST, &devcfg, &spi_handle[i]));
        // w5500 ethernet driver is based on spi driver
        eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle[i]);

        // Set remaining GPIO numbers and configuration used by the SPI module
        w5500_config.int_gpio_num = spi_eth_module_config[i].int_gpio;
        phy_config_spi.phy_addr = spi_eth_module_config[i].phy_addr;
        phy_config_spi.reset_gpio_num = spi_eth_module_config[i].phy_reset_gpio;

        // ESP_LOGI(TAG_ETH, "%d", spi_eth_module_config[i].spi_cs_gpio);
        // ESP_LOGI(TAG_ETH, "%d", spi_eth_module_config[i].int_gpio);
        // ESP_LOGI(TAG_ETH, "%d", spi_eth_module_config[i].phy_addr);
        // ESP_LOGI(TAG_ETH, "%d", spi_eth_module_config[i].phy_reset_gpio);

        mac_spi[i] = esp_eth_mac_new_w5500(&w5500_config, &mac_config_spi);
        phy_spi[i] = esp_eth_phy_new_w5500(&phy_config_spi);
    }
#endif //CONFIG_USER_USE_W5500

    for (int i = 0; i < CONFIG_USER_SPI_ETHERNETS_NUM; i++) {
        esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac_spi[i], phy_spi[i]);
        ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config_spi, &eth_handle_spi[i]));

        /* The SPI Ethernet module might not have a burned factory MAC address, we cat to set it manually.
       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
        */
        ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle_spi[i], ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x02, 0x00, 0x00, 0x12, 0x34, 0x56 + i
        }));

        // attach Ethernet driver to TCP/IP stack
        ESP_ERROR_CHECK(esp_netif_attach(eth_netif_spi[i], esp_eth_new_netif_glue(eth_handle_spi[i])));
    }
#endif // CONFIG_USE_SPI_ETHERNET

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* start Ethernet driver state machine */
#if CONFIG_USER_USE_SPI_ETHERNET
    for (int i = 0; i < CONFIG_USER_SPI_ETHERNETS_NUM; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handle_spi[i]));
    }
#endif // CONFIG_USER_USE_SPI_ETHERNET
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG_MQTT, "Last error %s: 0x%x", message, error_code);
    }
}
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, "esp32/dev91", 0);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);
        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG_MQTT, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        // ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG_MQTT, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG_MQTT, "DATA=%.*s", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = CONFIG_USER_BROKER_HOST,
        .port = CONFIG_USER_BROKER_PORT,
        .username = CONFIG_USER_BROKER_USER,
        .password = CONFIG_USER_BROKER_PASS,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ethernet_connect();
    // vTaskDelay(2000, portTICK_PERIOD_MS);
    mqtt_app_start();
}