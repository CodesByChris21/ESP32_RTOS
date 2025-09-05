#ifndef CONFIG_H_
#define CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// GPIO Definitions for LED
#define TASK_STACK_SIZE         4096
#define GPIO_PIN48              48
#define BLINK_PERIOD            1000
#define LED_STRIP_LEN           1

// UART Definitions
#define UART_BAUD_RATE          9600
#define UART_BUFFER_1K          (1024)
#define UART0_PORT              0
#define UART0_TX_PIN            37
#define UART0_RX_PIN            36
#define UART0_RTS_PIN           UART_PIN_NO_CHANGE         
#define UART0_CTS_PIN           UART_PIN_NO_CHANGE

// WiFi Definitions
#define PORT 5005     
#define HOST_IP_ADDR            "192.168.0.255"   
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define WIFI_SSID               "RedactedID"
#define WIFI_PASS               "RedactedPW"
#define MAXIMUM_RETRY           5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

#define TEMP_FIELD_SIZE         10

typedef struct {
    uint8_t *tx_ptr;
    size_t  tx_len;
    uint8_t *dptr;
    size_t  dlen;
} tx_buff_t;

static void config_led(void);
static void config_uart(tx_buff_t *buffer);
static void config_tempsensor(void);
static void config_wifi(void);


#ifdef __cplusplus
}
#endif

#endif