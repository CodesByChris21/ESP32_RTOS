/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "led_strip.h"
#include "driver/uart.h"
#include "driver/temperature_sensor.h"
#include "config.h"

// Wifi Variables
static temperature_sensor_handle_t temp_sensor = NULL;
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "example";
static int num_retry = 0;


static uint8_t led_status = 1;
static led_strip_handle_t led_strip;

// Writes Message "ESP32\r\n"
static uint8_t esp[7] = {0x45, 0x53, 0x50, 0x33, 0x32, 0x0D, 0x0A};
static uint8_t tx_buff[UART_BUFFER_1K];

tx_buff_t dout = { tx_buff, sizeof(tx_buff), esp, sizeof(esp)};

QueueHandle_t wifi_queue;
QueueHandle_t uart_queue;


static void config_led(void)
{
    ESP_LOGI(TAG, "Configuring LEDs...");
    // RMT configuration
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_PIN48,
        .max_leds = LED_STRIP_LEN,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
    };

    // Create the LED strip object
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

static void config_tempsensor(void)
{
    ESP_LOGI(TAG, "Initializing Temperature Sensor with ranges between 10 - 65 ℃....");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 65);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}

static void config_uart(tx_buff_t *buffer){
    memcpy(buffer -> tx_ptr, esp, buffer -> dlen);
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
    

    ESP_LOGI(TAG, "Configuring UART...");

    ESP_ERROR_CHECK(uart_driver_install(UART0_PORT, UART_BUFFER_1K * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART0_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART0_PORT, UART0_TX_PIN, UART0_RX_PIN, UART0_RTS_PIN, UART0_CTS_PIN));
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (num_retry < MAXIMUM_RETRY) {
            esp_wifi_connect();
            num_retry++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        num_retry = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void config_wifi(void)
{
    //Initialize NVS - Checks for Errors or Updates
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
	
	
	// Event Loop so that I know I'm connected
	
	s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Wifi Initialization
	
	wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "WiFi Intialized ! ");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void led_task(void *arg)
{
    while (1) {
    /* If the addressable LED is enabled */
        ESP_LOGI(TAG, "Executing LED toggling task");
        if (led_status) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            led_strip_set_pixel(led_strip, 0, 16, 16, 16);
            /* Refresh the strip to send data */
            led_strip_refresh(led_strip);
            ESP_LOGI(TAG, "Toggling On LED");
        } else {
            /* Set all LED off to clear all pixels */
            led_strip_clear(led_strip);
            ESP_LOGI(TAG, "Toggling Off LED");
        }
        led_status = !led_status;
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD));
    }
}

static void monitortemp_task(void *pvParameters){
    temperature_sensor_handle_t *temp = (temperature_sensor_handle_t *) pvParameters;
    float temp_val;
    while(1){
        ESP_LOGI(TAG, "Reading temperature...");
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(*temp, &temp_val));
        xQueueSend(wifi_queue, &temp_val, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void uart_task(void *pvParameters){

    const char *msg = "Packets Successfully Sent:";
    uint16_t packets;
    char uart_buff[8];
    while(1){
        if(xQueueReceive(uart_queue, &packets, portMAX_DELAY)){
            // Use snprintf to format packet counter to readable ASCII
            snprintf(uart_buff, sizeof(uart_buff),"%u\r\n",packets);
            ESP_LOGI(TAG, "Packet statistics updated on UART monitor");
            uart_write_bytes(UART0_PORT, msg, strlen(msg));
            uart_write_bytes(UART0_PORT, &uart_buff, sizeof(uart_buff));
        }
    }
}

void udp_task(void *pvParameters){
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    tx_buff_t *udp = (tx_buff_t *) pvParameters;
    float temp_val;
    uint16_t wifi_packets = 0;
    size_t bytes_to_send;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
        
	while (1) {
        if(xQueueReceive(wifi_queue, &temp_val, portMAX_DELAY)){
            // Use snprintf to format floating point data to readable ASCII
            snprintf((char*)(udp->tx_ptr + udp -> dlen ), TEMP_FIELD_SIZE,"%.2f C",temp_val);
            bytes_to_send = udp -> dlen + sizeof(float);
            int err = sendto(sock, (const char *) udp -> tx_ptr, bytes_to_send, 0,
                (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            } else {
                ESP_LOGI(TAG, "Message sent");
                wifi_packets++;
                xQueueSend(uart_queue, &wifi_packets, portMAX_DELAY);
            }
        }

    }
    close(sock);
    vTaskDelete(NULL);
}


void app_main(void)
{
    // Configure Peripherals
    config_led();
    config_tempsensor();
    config_uart(&dout);
    config_wifi();

    // Create Queues for data to be passed around
    wifi_queue = xQueueCreate(10, sizeof(float));
    uart_queue = xQueueCreate(10, sizeof(uint16_t));

    // Create Tasks
    xTaskCreate(led_task, "Blink LED", TASK_STACK_SIZE, NULL, 4, NULL);
    xTaskCreate(monitortemp_task, "Monitor Temp", TASK_STACK_SIZE, (void *)&temp_sensor, 5, NULL);
    xTaskCreate(udp_task, "Send UDP", TASK_STACK_SIZE, (void *)&dout, 5, NULL);
    xTaskCreate(uart_task, "UART TX", TASK_STACK_SIZE, NULL, 5, NULL);
}