#include "pn532_types.h"

#define MAX_READING_FAIL 100 // Maximum consecutive UID reading failures allowed

#define UART_RST_PIN 33
#define UART_IRQ_PIN 25
#define UART_TX_PIN 22
#define UART_RX_PIN 21
#define UART_PORT UART_NUM_1
#define UART_BAUD_RATE 115200

#define I2C_RST_PIN 33
#define I2C_IRQ_PIN 25
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_PORT I2C_NUM_0
#define I2C_CLK_SPEED 100000

#define PN532_HOST SPI2_HOST
#define SPI_RST_PIN 33
#define SPI_IRQ_PIN 25
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_CLK_PIN 18
#define SPI_CS_PIN 5
#define SPI_CLOCK_SPEED 1000000 // 1 MHz

volatile uint16_t fail_count = 0; // Tracks consecutive UID reading failures

static const char *TAG = "PN532_Example"; // Logging tag for debugging

/**
 * @brief Example task for interacting with PN532
 *
 * This task initializes the PN532, retrieves the firmware version,
 * configures it, and continuously reads the UID of nearby cards.
 *
 * @param pvParameters Pointer to the PN532 handle
 */
void example_task(void *pvParameters)
{
    pn532_handle_t pn532 = (pn532_handle_t)pvParameters;

    // Start PN532 and check firmware version
    ESP_ERROR_CHECK(pn532_start(pn532));
    uint8_t version[4] = {0}; // Firmware version buffer
    ESP_LOGI(TAG, "Getting firmware version...");
    ESP_ERROR_CHECK(pn532_get_firmware_version(pn532, version));

    ESP_LOGI(TAG, "Detected PN5%02X", version[0]);
    ESP_LOGI(TAG, "Firmware version: %02X.%02X", version[1], version[2]);

    // Configure PN532 for card detection
    ESP_ERROR_CHECK(pn532_SAM_configuration(pn532));
    ESP_ERROR_CHECK(pn532_set_passive_activation_retries(pn532, 0x01)); // Set retries to 1
    vTaskDelay(PN532_DELAY_DEFAULT); // Delay for stability

    uint8_t uid[8] = {0}; // Buffer for the card UID
    uint8_t uid_len = sizeof(uid);
    while (true)
    {
        // Attempt to read the card UID
        esp_err_t err = pn532_read_passive_target_id(pn532, (uint8_t)PN532_MIFARE_ISO14443A, uid, &uid_len);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Card UID:");
            ESP_LOG_BUFFER_HEX(TAG, uid, uid_len);
            fail_count = 0; // Reset failure counter on success
        }
        else if (err == ESP_ERR_NOT_FOUND)
        {
            if (++fail_count >= MAX_READING_FAIL)
            {
                ESP_LOGE(TAG, "Exceeded maximum failure count");
                break;
            }
            ESP_LOGI(TAG, "UID read failed, count: %d", fail_count);
        }
        else
        {
            ESP_LOGE(TAG, "error: %s", esp_err_to_name(err));
            ESP_LOGE(TAG, "Error reading UID");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay between read attempts
    }

    ESP_LOGI(TAG, "Example task completed");

    pn532_free(pn532); // Free PN532 resources
    vTaskDelete(NULL); // Terminate the task
}

/**
 * @brief Application entry point
 *
 * Initializes the PN532 configuration based on the defined protocol
 * and starts the example task.
 */
void app_main()
{
    pn532_config_t pn532_config = {0};

    // Configure the PN532 based on the selected protocol
#if 0
    pn532_config.protocol = PN532_UART_PROTOCOL;
    pn532_config.uart.rst_pin = UART_RST_PIN;
    pn532_config.uart.irq_pin = UART_IRQ_PIN;
    pn532_config.uart.tx_pin = UART_TX_PIN;
    pn532_config.uart.rx_pin = UART_RX_PIN;
    pn532_config.uart.port = UART_PORT;
    pn532_config.uart.baud_rate = UART_BAUD_RATE;
#elif 0
    pn532_config.protocol = PN532_I2C_PROTOCOL;
    pn532_config.i2c.clk_speed = I2C_CLK_SPEED;
    pn532_config.i2c.port = I2C_PORT;
    pn532_config.i2c.scl_pin = I2C_SCL_PIN;
    pn532_config.i2c.sda_pin = I2C_SDA_PIN;
    pn532_config.i2c.rst_pin = I2C_RST_PIN;
    pn532_config.i2c.irq_pin = I2C_IRQ_PIN;
#elif 1
    pn532_config.protocol = PN532_SPI_PROTOCOL;
    pn532_config.spi.host = SPI2_HOST;
    pn532_config.spi.clk_speed = SPI_CLOCK_SPEED;
    pn532_config.spi.sclk_pin = SPI_CLK_PIN;
    pn532_config.spi.miso_pin = SPI_MISO_PIN;
    pn532_config.spi.mosi_pin = SPI_MOSI_PIN;
    pn532_config.spi.cs_pin = SPI_CS_PIN;
    pn532_config.spi.rst_pin = SPI_RST_PIN;
    pn532_config.spi.irq_pin = SPI_IRQ_PIN;
#endif

    // Allocate memory for PN532 handle
    pn532_handle_t pn532 = malloc(sizeof(struct pn532_t));
    if (!pn532)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for PN532");
        return;
    }
    memset(pn532, 0, sizeof(struct pn532_t)); // Clear allocated memory

    // Initialize PN532
    ESP_ERROR_CHECK(pn532_init(pn532, &pn532_config));

    ESP_LOGI(TAG, "Starting example task");
    xTaskCreate(example_task, "example_task", 8192, pn532, 5, NULL);

    // Keep the main task alive
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
