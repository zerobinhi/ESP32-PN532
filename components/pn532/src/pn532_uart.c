#include "pn532_types.h"

#define PN532_UART_RX_BUF_SIZE 256
#define PN532_UART_TX_BUF_SIZE 256

// Logger tag for debugging
static const char *TAG = "pn532_uart";
// Write a command to the PN532 via UART
static esp_err_t pn532_uart_write_command(pn532_t *pn532, uint8_t *cmd, uint8_t cmd_len)
{
    if (!cmd || cmd_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uart_flush(pn532->handle.uart.port);

    uint8_t data_len = cmd_len + 1;
    uint8_t frame[data_len + 7]; // PREAMBLE ~ POSTAMBLE

    // Build the command frame
    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;

    frame[3] = data_len;
    frame[4] = (uint8_t)(~data_len + 1);

    frame[5] = PN532_HOSTTOPN532;

    memcpy(&frame[6], cmd, cmd_len);

    uint8_t checksum = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmd_len; i++)
    {
        checksum += cmd[i];
    }
    frame[6 + cmd_len] = (uint8_t)(~checksum + 1);
    frame[7 + cmd_len] = PN532_POSTAMBLE;

#ifdef PN532_DEBUG
    ESP_LOG_BUFFER_HEX(TAG, frame, sizeof(frame));
#endif

    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }

    int len = uart_write_bytes(pn532->handle.uart.port, (const char *)frame, sizeof(frame));

    xSemaphoreGive(pn532->mutex);

    return (len == sizeof(frame)) ? ESP_OK : ESP_FAIL;
}
// Read a response from the PN532 via UART
static esp_err_t pn532_uart_read_response(pn532_t *pn532, uint8_t *response, uint8_t response_len)
{
    if (!response || response_len == 0)
        return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }

    vTaskDelay(PN532_DELAY_DEFAULT);

    int len = uart_read_bytes(pn532->handle.uart.port, response, response_len, PN532_DELAY_DEFAULT);

    xSemaphoreGive(pn532->mutex);

    if (len <= 0)
        return ESP_FAIL;

#ifdef PN532_DEBUG
    ESP_LOG_BUFFER_HEX(TAG, response, len);
#endif

    return ESP_OK;
}

/**
 * @brief Free resources allocated for the PN532 UART interface
 *
 * @param[in] pn532 Pointer to the PN532 structure
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t pn532_uart_free(pn532_t *pn532)
{
    esp_err_t err = uart_driver_delete(pn532->handle.uart.port);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to delete UART driver: %d", err);
        return err;
    }

    vSemaphoreDelete(pn532->mutex);
    return ESP_OK;
}

/**
 * @brief Initialize the PN532 UART interface
 *
 * @param[in,out] pn532 Pointer to the PN532 structure
 * @param[in] config UART configuration parameters
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t pn532_uart_init(pn532_t *pn532, const pn532_uart_config_t *config)
{
    pn532->protocol = PN532_UART_PROTOCOL;
    pn532->handle.uart.port = config->port;

    // Configure the reset pin as output
    gpio_config_t reset_gpio_config = {
        .pin_bit_mask = (1ULL << config->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&reset_gpio_config);
    gpio_set_level(config->rst_pin, 1);

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};

    esp_err_t err = uart_param_config(pn532->handle.uart.port, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %d", err);
        return err;
    }

    // Set UART pins
    err = uart_set_pin(pn532->handle.uart.port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %d", err);
        return err;
    }

    // Install UART driver
    err = uart_driver_install(pn532->handle.uart.port, PN532_UART_RX_BUF_SIZE, PN532_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %d", err);
        return err;
    }

    uart_flush(pn532->handle.uart.port);

    // Perform hardware reset
    gpio_set_level(config->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(400));
    gpio_set_level(config->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create mutex for thread-safe operations
    pn532->mutex = xSemaphoreCreateMutex();
    if (!pn532->mutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        uart_driver_delete(pn532->handle.uart.port);
        return ESP_ERR_NO_MEM;
    }

    // Assign function pointers
    pn532->write_command = pn532_uart_write_command;
    pn532->read_response = pn532_uart_read_response;
    pn532->free = pn532_uart_free;

    ESP_LOGI(TAG, "PN532 UART initialized");
    return ESP_OK;
}
