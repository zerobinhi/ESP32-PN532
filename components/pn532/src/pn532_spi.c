#include "pn532_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPI_CS_PIN 5
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23
#define SPI_CLK_PIN 18

// Logger tag for debugging
static const char *TAG = "pn532_spi";

// SPI sends data to a specific address
static esp_err_t _spi_write_data(pn532_t *pn532, uint8_t addr, const uint8_t *cmd, uint8_t len)
{
    if (!cmd || len == 0)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t t = {
        .cmd = addr,
        .length = len * 8,
        .tx_buffer = cmd,
        .rx_buffer = NULL};

    esp_err_t ret = spi_device_polling_transmit(pn532->handle.spi.dev, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// SPI reads data from a specific address
static esp_err_t _spi_read_data(pn532_t *pn532, uint8_t addr, uint8_t *rx, uint8_t len)
{
    if (!rx || len == 0)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t t = {
        .cmd = addr,
        .length = len * 8,
        .tx_buffer = NULL,
        .rx_buffer = rx};

    esp_err_t ret = spi_device_polling_transmit(pn532->handle.spi.dev, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Write a command to the PN532 via SPI
esp_err_t pn532_spi_write_command(pn532_t *pn532, uint8_t *cmd, uint8_t cmd_len)
{
    if (!cmd || cmd_len == 0)
        return ESP_ERR_INVALID_ARG;

    uint8_t data_len = cmd_len + 1;
    uint8_t frame[data_len + 7];

    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;
    frame[3] = data_len;
    frame[4] = (uint8_t)(~data_len + 1);
    frame[5] = PN532_HOSTTOPN532;

    memcpy(&frame[6], cmd, cmd_len);

    uint8_t checksum = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmd_len; i++)
        checksum += cmd[i];
    frame[6 + cmd_len] = (uint8_t)(~checksum + 1);
    frame[7 + cmd_len] = PN532_POSTAMBLE;

#ifdef PN532_DEBUG1
    ESP_LOG_BUFFER_HEX(TAG, frame, sizeof(frame));
#endif

    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    esp_err_t ret = _spi_write_data(pn532, 0x01, frame, sizeof(frame));

    xSemaphoreGive(pn532->mutex);
    return ret;
}
// Read a response from the PN532 via SPI
esp_err_t pn532_spi_read_response(pn532_t *pn532, uint8_t *buf, uint8_t len)
{
    if (!buf || len == 0)
        return ESP_ERR_INVALID_ARG;

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t ready = 0;

    // 读取 READY 不需要加锁（读 0x02 不改变 PN532 状态）
    for (int i = 0; i < 20; i++)
    {
        _spi_read_data(pn532, 0x02, &ready, 1);
        if (ready == 1)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (ready == 0)
    {
        ESP_LOGE(TAG, "PN532 not ready");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    esp_err_t ret = _spi_read_data(pn532, 0x03, buf, len);
    xSemaphoreGive(pn532->mutex);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read response");
        return ret;
    }

#ifdef PN532_DEBUG
    ESP_LOG_BUFFER_HEX(TAG, buf, len);
#endif

    return ESP_OK;
}

// Free SPI resources
static esp_err_t pn532_spi_free(struct pn532_t *pn532)
{
    esp_err_t err = spi_bus_remove_device(pn532->handle.spi.dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi device removal failed: %d", err);
        return err;
    }
    vSemaphoreDelete(pn532->mutex);
    return ESP_OK;
}

// Initialize PN532 with SPI protocol
esp_err_t pn532_spi_init(pn532_t *pn532, const pn532_spi_config_t *config)
{
    pn532->protocol = PN532_SPI_PROTOCOL;

    // Configure the reset pin
    gpio_config_t reset_gpio_config = {
        .pin_bit_mask = (1ULL << config->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&reset_gpio_config);
    gpio_set_level(config->rst_pin, 1);

    // Configure the SPI bus
    spi_bus_config_t spi_bus_config = {
        .mosi_io_num = config->mosi_pin,
        .miso_io_num = config->miso_pin,
        .sclk_io_num = config->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    // Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(config->host, &spi_bus_config, SPI_DMA_DISABLED);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure the SPI device, LSB First, Mode 0
    spi_device_interface_config_t spi_device_config = {
        .command_bits = 8,
        .clock_speed_hz = config->clk_speed,
        .mode = 0,
        .spics_io_num = config->cs_pin,
        .queue_size = 1,
        .flags = SPI_DEVICE_BIT_LSBFIRST,
    };

    // Add the device to the SPI bus
    ret = spi_bus_add_device(config->host, &spi_device_config, &pn532->handle.spi.dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Perform reset
    gpio_set_level(config->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(400));
    gpio_set_level(config->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create mutex for thread safety
    pn532->mutex = xSemaphoreCreateMutex();
    if (!pn532->mutex)
    {
        ESP_LOGE(TAG, "failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Assign function pointers
    pn532->write_command = pn532_spi_write_command;
    pn532->read_response = pn532_spi_read_response;
    pn532->free = pn532_spi_free;

    ESP_LOGI(TAG, "pn532 SPI initialized");
    return ESP_OK;
}
