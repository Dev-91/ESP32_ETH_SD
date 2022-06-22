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

static const char *TAG_ETH = "ethernet";
static const char *TAG_MQTT = "mqtt";
static const char *TAG_SD = "sd";
static const char *TAG_HTTP = "http_client";

int countX = 0;

#define USER_BLINK_GPIO 23

// #define USER_SD_SPI_MISO_GPIO 25
// #define USER_SD_SPI_MOSI_GPIO 26
// #define USER_SD_SPI_CLK_GPIO 27
#define USER_SD_SPI_CS_GPIO 18

#define SPI_DMA_CHAN    1

#define MOUNT_POINT "/sdcard"

sdmmc_card_t *card;
const char mount_point[] = MOUNT_POINT;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
// #define CONFIG_USER_SPI_ETHERNETS_NUM 1
// #define CONFIG_USER_ETH_SPI_HOST 1
// #define CONFIG_USER_ETH_SPI_CLOCK_MHZ 12

// #define CONFIG_USER_ETH_SPI_CS0_GPIO 15
// #define CONFIG_USER_ETH_SPI_INT0_GPIO 1
// #define CONFIG_USER_ETH_SPI_PHY_RST0_GPIO -1
// #define CONFIG_USER_ETH_SPI_PHY_ADDR0 1

// #define CONFIG_USER_USE_SPI_ETHERNET true
// #define CONFIG_USER_USE_W5500 true

esp_mqtt_client_handle_t client_obj;

bool esp_ethernet_ready = false;
bool esp_mqtt_ready = false;

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
        .max_transfer_sz = 4000,
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

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
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
    sdmmc_card_print_info(stdout, card);

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

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
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

void blink_task(void *pvParameter) {
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

void mqtt_period_task(void *pvParameter) {
    gpio_pad_select_gpio(USER_BLINK_GPIO);

    gpio_set_direction(USER_BLINK_GPIO, GPIO_MODE_OUTPUT);

    mqtt_app_start();
    while (!esp_mqtt_ready);  // mqtt가 준비되기까지 잠시 대기
    
    int cnt = 0;
    while (1) {
        if (esp_mqtt_ready) {  // mqtt 연결 문제 발생시 전송 일시 중단
            cnt++;
            char publish_data[100] = { 0x00, };
            sprintf(publish_data, "publish data cnt : %d", cnt);
            
            gpio_set_level(USER_BLINK_GPIO, 1);
            
            esp_mqtt_client_publish(client_obj, "/topic/haha", publish_data, 0, 1, 0);
            gpio_set_level(USER_BLINK_GPIO, 0);
            
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        } else {
            // ESP_LOGI(TAG_MQTT, "MQTT Not Ready");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void http_period_task(void *pvParameter) {
    while(1) {
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

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    dev_info();

    ethernet_connect();
    while (!esp_ethernet_ready);  // ethernet이 준비되기까지 잠시 대기 

    sd_card_mount();

    xTaskCreatePinnedToCore(&blink_task, "blink_task", 2048, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&mqtt_period_task, "mqtt_period_task", 2048, NULL, 5, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(&http_period_task, "http_period_task", 8192, NULL, 5, NULL, PRO_CPU_NUM);

    while(1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
