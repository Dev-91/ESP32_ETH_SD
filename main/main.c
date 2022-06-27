#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#include "math.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <sys/unistd.h>
#include <sys/stat.h>

#include <esp_http_client.h>

static const char *TAG_ETH = "ETHERNET";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_SD = "SD CARD";
static const char *TAG_HTTP = "HTTP_CLIENT";
static const char *TAG_RS485 = "RS485";

int countX = 0;

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define USER_BLINK_GPIO 23

#define USER_RS485_RX_GPIO 16
#define USER_RS485_TX_GPIO 17
#define USER_RS485_RTS -1
#define USER_RS485_CTS -1
#define USER_RS485_BAUD_RATE 115200
#define UART_PORT_NUM 2
#define READ_TIME_OUT 3
#define BUF_SIZE 127
#define PACKET_READ_TICS (100 / portTICK_RATE_MS)

#define USER_SPI_MISO_GPIO 25
#define USER_SPI_MOSI_GPIO 26
#define USER_SPI_SCLK_GPIO 27

#define USER_SD_SPI_CS_GPIO 18

#define USER_ETH_SPI_CS_GPIO 19
#define USER_ETH_SPI_INT_GPIO 4
#define USER_ETH_SPI_PHY_RST_GPIO -1
#define USER_ETH_SPI_PHY_ADDR_GPIO 1

#define SPI_DMA_CHAN 1

#define MOUNT_POINT "/sdcard"

sdmmc_card_t *card;
const char mount_point[] = MOUNT_POINT;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} timer_info_t;

typedef struct {
    timer_info_t info;
    uint64_t timer_counter_value;
} timer_event_t;

static xQueueHandle sd_timer_queue;
static xQueueHandle http_timer_queue;
static xQueueHandle mqtt_timer_queue;

#define USER_SPI_ETHERNETS_NUM 1
#define USER_ETH_SPI_HOST 1
#define USER_ETH_SPI_CLOCK_MHZ 12

#define USER_USE_SPI_ETHERNET true
#define USER_USE_W5500 true

esp_mqtt_client_handle_t client_obj;

bool esp_ethernet_ready = false;
bool esp_mqtt_ready = false;

#if USER_USE_SPI_ETHERNET
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
        esp_ethernet_ready = false;
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

    esp_ethernet_ready = true;
}

void ethernet_connect(void)
{
    // MQTT example init
    ESP_ERROR_CHECK(nvs_flash_init());
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if USER_USE_SPI_ETHERNET
    // Create instance(s) of esp-netif for SPI Ethernet(s)
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg_spi = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    esp_netif_t *eth_netif_spi[USER_SPI_ETHERNETS_NUM] = { NULL };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < USER_SPI_ETHERNETS_NUM; i++) {
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
    spi_device_handle_t spi_handle[USER_SPI_ETHERNETS_NUM] = { NULL };
    spi_bus_config_t buscfg = {
        .miso_io_num = USER_SPI_MISO_GPIO,
        .mosi_io_num = USER_SPI_MOSI_GPIO,
        .sclk_io_num = USER_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(USER_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Init specific SPI Ethernet module configuration from Kconfig (CS GPIO, Interrupt GPIO, etc.)
    spi_eth_module_config_t spi_eth_module_config[USER_SPI_ETHERNETS_NUM];
    // INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 0);

    spi_eth_module_config[0].spi_cs_gpio = USER_ETH_SPI_CS_GPIO;
    spi_eth_module_config[0].int_gpio = USER_ETH_SPI_INT_GPIO;
    spi_eth_module_config[0].phy_reset_gpio = USER_ETH_SPI_PHY_RST_GPIO;
    spi_eth_module_config[0].phy_addr = USER_ETH_SPI_PHY_ADDR_GPIO;

    // #if USER_SPI_ETHERNETS_NUM > 1
    //     INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 1);
    // #endif
    // Configure SPI interface and Ethernet driver for specific SPI module
    esp_eth_mac_t *mac_spi[USER_SPI_ETHERNETS_NUM];
    esp_eth_phy_t *phy_spi[USER_SPI_ETHERNETS_NUM];
    esp_eth_handle_t eth_handle_spi[USER_SPI_ETHERNETS_NUM] = { NULL };

#if USER_USE_W5500
    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
        .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = USER_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20
    };

    for (int i = 0; i < USER_SPI_ETHERNETS_NUM; i++) {
        // Set SPI module Chip Select GPIO
        devcfg.spics_io_num = spi_eth_module_config[i].spi_cs_gpio;

        ESP_ERROR_CHECK(spi_bus_add_device(USER_ETH_SPI_HOST, &devcfg, &spi_handle[i]));
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
#endif //USER_USE_W5500

    for (int i = 0; i < USER_SPI_ETHERNETS_NUM; i++) {
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
#if USER_USE_SPI_ETHERNET
    for (int i = 0; i < USER_SPI_ETHERNETS_NUM; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handle_spi[i]));
    }
#endif // USER_USE_SPI_ETHERNET
}

void mqtt_data_parser(esp_mqtt_event_handle_t event) {
    ESP_LOGI(TAG_MQTT, "TOPIC=%.*s", event->topic_len, event->topic);
    ESP_LOGI(TAG_MQTT, "DATA=%.*s", event->data_len, event->data);
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

        esp_mqtt_ready = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        esp_mqtt_ready = false;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        mqtt_data_parser(event);
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

    // esp_mqtt_client_handle_t 
    client_obj = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client_obj, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client_obj);
}

void dev_info(void) {
    /* Print chip information */
    static const char *TAG_INFO = "chip_info";
    esp_chip_info_t chip_info;
    
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG_INFO, "This is %s chip with %d CPU core(s)",
            CONFIG_IDF_TARGET, chip_info.cores); 

    ESP_LOGI(TAG_INFO, "WiFi%s%s",
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", 
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : ""); 

    ESP_LOGI(TAG_INFO, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG_INFO, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG_INFO, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG_INFO, "Number of Core: %d", chip_info.cores);
    ESP_LOGI(TAG_INFO, "CHIP Revision Number: %d", chip_info.revision); 
     
    ESP_LOGI(TAG_INFO, "ESP-IDF version = %s", esp_get_idf_version());
     
    uint8_t mac0[6];
    esp_efuse_mac_get_default(mac0);
    ESP_LOGI(TAG_INFO, "Default Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac0[0], mac0[1], mac0[2], mac0[3], mac0[4], mac0[5]);
     
    uint8_t mac3[6];
    esp_read_mac(mac3, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG_INFO, "[Wi-Fi Station] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]);
     
    uint8_t mac4[7];
    esp_read_mac(mac4, ESP_MAC_WIFI_SOFTAP);
    ESP_LOGI(TAG_INFO, "[Wi-Fi SoftAP] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac4[0], mac4[1], mac4[2], mac4[3], mac4[4], mac4[5]);
     
    uint8_t mac5[6];
    esp_read_mac(mac5, ESP_MAC_BT);
    ESP_LOGI(TAG_INFO, "[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac5[0], mac5[1], mac5[2], mac5[3], mac5[4], mac5[5]);
     
    uint8_t mac6[6];
    esp_read_mac(mac6, ESP_MAC_ETH);
    ESP_LOGI(TAG_INFO, "[Ethernet] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac6[0], mac6[1], mac6[2], mac6[3], mac6[4], mac6[5]);
}

void sd_card_mount(void) {

    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  // true인 경우 마운트 실패시 SD카드 포맷해버림
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    // sdmmc_card_t *card;
    // const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG_SD, "Initializing SD card %d", mount_config.format_if_mount_failed);

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG_SD, "Using SPI peripheral");

    // sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    /* ethernet 설정하면서 이미 SPI핀을 초기화함 
     * 그래서 SD 카드 부분의 SPI 초기화를 제거하고 같은 핀을 사용하도록함
     * 이미 같은 HSPI를 사용하기 때문에 다시 초기화시 에러가 발생함
     * 여기서는 따로 초기화 하지 않고, CS만 설정해줌 (maybe...)
     */

    // spi_bus_config_t bus_cfg = {
    //     .mosi_io_num = USER_SD_SPI_MOSI_GPIO,
    //     .miso_io_num = USER_SD_SPI_MISO_GPIO,
    //     .sclk_io_num = USER_SD_SPI_CLK_GPIO,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    //     .max_transfer_sz = 4000,
    // };
    // ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG_SD, "Failed to initialize bus.");
    //     return;
    // }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = USER_SD_SPI_CS_GPIO;  // 사용자 정의 SD_CS핀
    slot_config.host_id = host.slot;  // HSPI_HOST

    ESP_LOGI(TAG_SD, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_SD, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG_SD, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG_SD, "Filesystem mounted");

    // Card has been initialized, print its properties
    // sdmmc_card_print_info(stdout, card);

}

void sd_card_write(void) {
    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";

    ESP_LOGI(TAG_SD, "Opening file %s", file_hello);
    FILE *f = fopen(file_hello, "a");
    if (f == NULL) {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello %s!\n", card->cid.name);
    fclose(f);
    ESP_LOGI(TAG_SD, "File written");
}

void sd_card_unmount(void) {

    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG_SD, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}

esp_err_t _http_event_handle(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_ON_HEADER");
            ESP_LOGI(TAG_HTTP, "%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                ESP_LOGI(TAG_HTTP, "%.*s\n", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_HTTP, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

static bool IRAM_ATTR timer_group_isr_callback(void *args) {
    // timer_spinlock_take(TIMER_GROUP_0);
    BaseType_t high_task_awoken = pdFALSE;
    timer_info_t *info = (timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    /* Prepare basic event data that will be then sent back to task */
    timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    if (!info->auto_reload) { // auto_reload가 아닐때
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */
    if (info->timer_group == TIMER_GROUP_0) {
        if (info->timer_idx == TIMER_0) {
            xQueueSendFromISR(sd_timer_queue, &evt, &high_task_awoken);
        } else if (info->timer_idx == TIMER_1) {
            
        }
    } else if (info->timer_group == TIMER_GROUP_1) {
        if (info->timer_idx == TIMER_0) {
            xQueueSendFromISR(http_timer_queue, &evt, &high_task_awoken);
        } else if (info->timer_idx == TIMER_1) {
            xQueueSendFromISR(mqtt_timer_queue, &evt, &high_task_awoken);
        }
    }

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static void tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(group, timer);

    timer_info_t *timer_info = calloc(1, sizeof(timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);
}

static void inline print_timer_counter(uint64_t counter_value) {
    printf("Counter: 0x%08x%08x\r\n", (uint32_t) (counter_value >> 32),
           (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}

static void echo_send(const int port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(TAG_RS485, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

static void blink_task(void *pvParameter) {
    gpio_pad_select_gpio(USER_BLINK_GPIO);

    gpio_set_direction(USER_BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL<<USER_BLINK_GPIO;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    int cnt = 0;

    while(1) {
        gpio_set_level(USER_BLINK_GPIO, cnt % 2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        cnt++;
        if (cnt == 5) {
            sd_card_write();
            cnt = 0;
        }
    }
}

static void sd_timer_task(void *pvParameter) {
    gpio_pad_select_gpio(USER_BLINK_GPIO);

    gpio_set_direction(USER_BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL<<USER_BLINK_GPIO;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    int cnt = 0;

    while(1) {
        timer_event_t evt;
        xQueueReceive(sd_timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        // if (evt.info.auto_reload) {
        //     printf("Timer Group with auto reload\n");
        // } else {
        //     printf("Timer Group without auto reload\n");
        // }
        // printf("Group[%d], timer[%d] alarm event\n", evt.info.timer_group, evt.info.timer_idx);

        /* Print the timer values passed by event */
        // printf("------- EVENT TIME --------\n");
        // print_timer_counter(evt.timer_counter_value);
        
        gpio_set_level(USER_BLINK_GPIO, 1);
        sd_card_write();
        gpio_set_level(USER_BLINK_GPIO, 0);

        /* Print the timer values as visible by this task */
        // printf("-------- TASK TIME --------\n");
        // uint64_t task_counter_value;
        // timer_get_counter_value(evt.info.timer_group, evt.info.timer_idx, &task_counter_value);
        // print_timer_counter(task_counter_value);
    }
}

static void mqtt_timer_task(void *pvParameter) {
    gpio_pad_select_gpio(USER_BLINK_GPIO);

    gpio_set_direction(USER_BLINK_GPIO, GPIO_MODE_OUTPUT);

    mqtt_app_start();
    while (!esp_mqtt_ready);  // mqtt가 준비되기까지 잠시 대기
    
    int cnt = 0;
    while(1) {
        timer_event_t evt;
        xQueueReceive(mqtt_timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        // if (evt.info.auto_reload) {
        //     printf("Timer Group with auto reload\n");
        // } else {
        //     printf("Timer Group without auto reload\n");
        // }
        // printf("Group[%d], timer[%d] alarm event\n", evt.info.timer_group, evt.info.timer_idx);

        /* Print the timer values passed by event */
        // printf("------- MQTT EVENT TIME --------\n");
        // print_timer_counter(evt.timer_counter_value);

        if (esp_mqtt_ready) {  // mqtt 연결 문제 발생시 전송 일시 중단
            cnt++;
            char publish_data[100] = { 0x00, };
            sprintf(publish_data, "publish data cnt : %d", cnt);
            
            gpio_set_level(USER_BLINK_GPIO, 1);
            
            esp_mqtt_client_publish(client_obj, "/topic/haha", publish_data, 0, 1, 0);
            gpio_set_level(USER_BLINK_GPIO, 0);
        }

        /* Print the timer values as visible by this task */
        // printf("-------- MQTT TASK TIME --------\n");
        // uint64_t task_counter_value;
        // timer_get_counter_value(evt.info.timer_group, evt.info.timer_idx, &task_counter_value);
        // print_timer_counter(task_counter_value);
    }
}

static void http_timer_task(void *pvParameter) {
    while(1) {
        timer_event_t evt;
        xQueueReceive(http_timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        // if (evt.info.auto_reload) {
        //     printf("Timer Group with auto reload\n");
        // } else {
        //     printf("Timer Group without auto reload\n");
        // }
        // printf("Group[%d], timer[%d] alarm event\n", evt.info.timer_group, evt.info.timer_idx);

        // /* Print the timer values passed by event */
        // printf("------- HTTP EVENT TIME --------\n");
        // print_timer_counter(evt.timer_counter_value);

        esp_http_client_config_t config = {
            .url = "http://192.168.0.9:8000/post/",
            // .url = "http://www.google.com/",
            .event_handler = _http_event_handle,
        };

        char* post_data = "{\"str_data\" : \"string_data\",\"int_data\" : 12,\"float_data\" : 3.4} ";

        esp_http_client_handle_t http_client = esp_http_client_init(&config);
        // esp_http_client_set_method(http_client, HTTP_METHOD_GET);
        esp_http_client_set_method(http_client, HTTP_METHOD_POST);
        esp_http_client_set_header(http_client, "Content-Type", "application/json");
        esp_http_client_set_post_field(http_client, post_data, strlen(post_data));
        
        esp_err_t err = esp_http_client_perform(http_client);

        if (err == ESP_OK) {
            ESP_LOGI(TAG_HTTP, "Status = %d, content_length = %d",
            esp_http_client_get_status_code(http_client),
            esp_http_client_get_content_length(http_client));
        }

        esp_http_client_cleanup(http_client);

        /* Print the timer values as visible by this task */
        // printf("-------- HTTP TASK TIME --------\n");
        // uint64_t task_counter_value;
        // timer_get_counter_value(evt.info.timer_group, evt.info.timer_idx, &task_counter_value);
        // print_timer_counter(task_counter_value);
    }
}

static void rs485_recv_task(void *arg) {
    const int uart_num = UART_PORT_NUM;
    uart_config_t uart_config = {
        .baud_rate = USER_RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    // Set UART log level
    esp_log_level_set(TAG_RS485, ESP_LOG_INFO);

    ESP_LOGI(TAG_RS485, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_LOGI(TAG_RS485, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, USER_RS485_TX_GPIO, USER_RS485_RX_GPIO, USER_RS485_RTS, USER_RS485_CTS));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, READ_TIME_OUT));

    // Allocate buffers for UART
    uint8_t* rs485_read_data = (uint8_t*) malloc(BUF_SIZE);

    ESP_LOGI(TAG_RS485, "UART start recieve loop.\r\n");
    // echo_send(uart_num, "Start RS485 UART test.\r\n", 24);

    while(1) {
        //Read data from UART
        int len = uart_read_bytes(uart_num, rs485_read_data, BUF_SIZE, PACKET_READ_TICS);

        //Write data back to UART
        if (len > 0) {
            echo_send(uart_num, "\r\n", 2);
            char prefix[] = "RS485 Received: [";
            echo_send(uart_num, prefix, (sizeof(prefix) - 1));
            ESP_LOGI(TAG_RS485, "Received %u bytes:", len);
            printf("[ ");
            for (int i = 0; i < len; i++) {
                printf("0x%.2X ", (uint8_t)rs485_read_data[i]);
                echo_send(uart_num, (const char*)&rs485_read_data[i], 1);
                // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
                if (rs485_read_data[i] == '\r') {
                    echo_send(uart_num, "\n", 1);
                }
            }
            printf("] \n");
            echo_send(uart_num, "]\r\n", 3);
        } else {
            // Echo a "." to show we are alive while we wait for input
            // echo_send(uart_num, ".", 1);
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
        }
    }
    vTaskDelete(NULL);
}


void app_main(void) {
    dev_info();

    ethernet_connect();
    while (!esp_ethernet_ready) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // task_wdt 
    };  // ethernet이 준비되기까지 잠시 대기 

    sd_card_mount();

    sd_timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    http_timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    mqtt_timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 10);
    tg_timer_init(TIMER_GROUP_1, TIMER_0, true, 5);
    tg_timer_init(TIMER_GROUP_1, TIMER_1, true, 2);

    // xTaskCreatePinnedToCore(&blink_task, "blink_task", 2048, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&sd_timer_task, "sd_timer_task", 2048, NULL, 4, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&rs485_recv_task, "rs485_recv_task", 2048, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&mqtt_timer_task, "mqtt_timer_task", 2048, NULL, 1, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(&http_timer_task, "http_timer_task", 8192, NULL, 3, NULL, PRO_CPU_NUM);
}
