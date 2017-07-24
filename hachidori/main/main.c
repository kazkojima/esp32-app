/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "ringbuf.h"

#include "rgbled.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    system_event_sta_disconnected_t *disconn;
    wifi_mode_t mode;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
        disconn = &event->event_info.disconnected;
        switch (disconn->reason) {
        case WIFI_REASON_AUTH_FAIL:
            printf("WiFi: desconntcted after auth fail\r\n");
            break;
        default:
            // try to reconnect
            if (esp_wifi_get_mode(&mode) == ESP_OK) {
                if (mode & WIFI_MODE_STA) {
                    printf("WiFi: try to reconnect...\r\n");
                    esp_wifi_connect();
                }
            }
            break;
        }
    default:
        break;
    }
    return ESP_OK;
}

#define UDP_SERVER "192.168.11.1"
#define UDP_PORT 5790

int sockfd = -1;

static void udp_task(void *arg)
{
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    printf("UDP client task starting...\r\n");

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated socket\r\n");

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    caddr.sin_port = htons(UDP_PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (UDP_SERVER);
    saddr.sin_port = htons(UDP_PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }
    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    sockfd = s;
    while (true) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

spi_device_handle_t spi_baro, spi_a, spi_m;

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS    5
#define PIN_NUM_CS_A 21

static void spi_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };
    spi_device_interface_config_t devcfg={
        .command_bits=8,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=400000,                 //Clock out at 400KHz
        .duty_cycle_pos=128,
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=1,                          //queue size
        .flags=SPI_DEVICE_HALFDUPLEX,
    };
    spi_device_interface_config_t devcfg_a={
        .command_bits=8,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=1000000,                //Clock out at 1 MHz
        .duty_cycle_pos=128,
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS_A,             //CS pin
        .queue_size=1,                          //queue size
        .flags=SPI_DEVICE_HALFDUPLEX,
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the slave devices to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi_baro);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(VSPI_HOST, &devcfg_a, &spi_a);
    assert(ret==ESP_OK);
}

#define I2C_MASTER_SCL_IO               26
#define I2C_MASTER_SDA_IO               25
#define I2C_MASTER_NUM                  I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

static void i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        const esp_partition_t *pa;
        pa = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        assert(pa && "partition table must have an NVS partition");
        ESP_ERROR_CHECK(esp_partition_erase_range(pa, 0, pa->size));
        // Retry nvs_flash_init
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

struct ringbuf ubloxbuf;

SemaphoreHandle_t ringbuf_sem;
SemaphoreHandle_t send_sem;
SemaphoreHandle_t pwm_sem;
SemaphoreHandle_t i2c_sem;
SemaphoreHandle_t nvs_sem;

volatile uint8_t rgb_led_red;
volatile uint8_t rgb_led_green;
volatile uint8_t rgb_led_blue;

extern void baro_task(void *arg);
extern void baro2_task(void *arg);
extern void imu_task(void *arg);
extern void pwm_task(void *arg);
extern void bat_task(void *arg);
extern void fs_task(void *arg);
extern void rn_task(void *arg);
extern void gps_task(void *arg);

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "hachidori_ap",
            .password = "e15f44ecdff3a",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    spi_init();
    i2c_init();
    nvs_init();

    vSemaphoreCreateBinary(send_sem);
    vSemaphoreCreateBinary(pwm_sem);
    vSemaphoreCreateBinary(i2c_sem);
    vSemaphoreCreateBinary(nvs_sem);

    // Initialize ring buffer
    vSemaphoreCreateBinary(ringbuf_sem);
    ringbuf_init (&ubloxbuf);

    xTaskCreate(udp_task, "udp_task", 2048, NULL, 11, NULL);
    xTaskCreate(imu_task, "imu_task", 2048, NULL, 10, NULL);
    xTaskCreate(baro_task, "baro_task", 2048, NULL, 9, NULL);
    //xTaskCreate(baro2_task, "baro2_task", 2048, NULL, 9, NULL);
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 8, NULL);
    xTaskCreate(rn_task, "rn_task", 2048, NULL, 7, NULL);
    xTaskCreate(gps_task, "gps_task", 2048, NULL, 6, NULL);
    xTaskCreate(bat_task, "bat_task", 2048, NULL, 5, NULL);
    xTaskCreate(fs_task, "fs_task", 2048, NULL, 5, NULL);

#if 1
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1<<GPIO_LED_RED)|(1<<GPIO_LED_GREEN)
                            |(1<<GPIO_LED_BLUE));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1<<GPIO_NUM_22);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_LED_RED, RGBLED_OFF);
    gpio_set_level(GPIO_LED_GREEN, RGBLED_OFF);
    gpio_set_level(GPIO_LED_BLUE, RGBLED_OFF);

    rgb_led_green = 1;

    uint8_t red = 0, green = 0, blue = 0;
    while (true) {
        if (red != rgb_led_red) {
            red = rgb_led_red;
            gpio_set_level(GPIO_LED_RED, (red ? RGBLED_ON : RGBLED_OFF));
        }
        if (green != rgb_led_green) {
            green = rgb_led_green;
            gpio_set_level(GPIO_LED_GREEN, (green ? RGBLED_ON : RGBLED_OFF));
        }
        if (blue != rgb_led_blue) {
            blue = rgb_led_blue;
            gpio_set_level(GPIO_LED_BLUE, (blue ? RGBLED_ON : RGBLED_OFF));
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
#else
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif
}

