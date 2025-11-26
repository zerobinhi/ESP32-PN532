#include "pn532_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Logger tag for debugging
static const char *TAG = "pn532_i2c";

// Write a command to the PN532 via I2C
static esp_err_t pn532_i2c_write_command(pn532_t *pn532, uint8_t *cmd, uint8_t cmd_len)
{
    if (!cmd || cmd_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data_len = cmd_len + 1; // Include TFI in the length
    uint8_t frame[data_len + 7];    // Allocate memory for the full frame

    // Build the command frame
    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;
    frame[3] = data_len;
    frame[4] = (uint8_t)(~data_len + 1); // Checksum of the length
    frame[5] = PN532_HOSTTOPN532;

    memcpy(&frame[6], cmd, cmd_len); // Copy command data

    // Calculate checksum (DCS)
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
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    // Send the command via I2C
    esp_err_t ret = i2c_master_transmit(pn532->handle.i2c.dev, frame, sizeof(frame), PN532_DELAY_DEFAULT);

    xSemaphoreGive(pn532->mutex);

    return ret;
}

// Read a response from the PN532 via I2C
static esp_err_t pn532_i2c_read_response(pn532_t *pn532, uint8_t *buf, uint8_t len)
{
    if (!buf || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(pn532->mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    vTaskDelay(PN532_DELAY_DEFAULT);

    esp_err_t ret;
    uint8_t status = 0;

    // Read status (wait for ready)
    for (int i = 0; i < 20; i++)
    {
        ret = i2c_master_receive(pn532->handle.i2c.dev, &status, 1, PN532_DELAY_DEFAULT);

        if (ret == ESP_OK && status == 0x01)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (status != 0x01)
    {
        xSemaphoreGive(pn532->mutex);
        ESP_LOGE(TAG, "PN532 not ready");
        return ESP_FAIL;
    }

    // Read header byte + payload
    uint8_t tmp[len + 1];

    ret = i2c_master_receive(pn532->handle.i2c.dev, tmp, len + 1, PN532_DELAY_DEFAULT);

    xSemaphoreGive(pn532->mutex);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read failed");
        return ret;
    }

    if (tmp[0] != 0x01)
    {
        ESP_LOGE(TAG, "Invalid PN532 I2C header: 0x%02X", tmp[0]);
        return ESP_FAIL;
    }

    memcpy(buf, &tmp[1], len);

#ifdef PN532_DEBUG
    ESP_LOG_BUFFER_HEX(TAG, buf, len);
#endif

    return ESP_OK;
}

// Free resources associated with the PN532 I2C interface
static esp_err_t pn532_i2c_free(struct pn532_t *pn532)
{
    esp_err_t err;

    err = i2c_master_bus_rm_device(pn532->handle.i2c.dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C device removal failed: %d", err);
        return err;
    }

    err = i2c_del_master_bus(pn532->handle.i2c.bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus removal failed: %d", err);
        return err;
    }

    // vSemaphoreDelete(pn532->mutex);  // Uncomment if required
    return ESP_OK;
}

// Initialize the PN532 I2C interface
esp_err_t pn532_i2c_init(pn532_t *pn532, const pn532_i2c_config_t *config)
{
    pn532->protocol = PN532_I2C_PROTOCOL;

    // Configure the reset pin
    gpio_config_t reset_gpio_config = {
        .pin_bit_mask = (1ULL << config->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&reset_gpio_config);
    gpio_set_level(config->rst_pin, 1);

    // Configure the I2C master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = config->port,
        .scl_io_num = config->scl_pin,
        .sda_io_num = config->sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true};

    // Create the I2C master bus
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &pn532->handle.i2c.bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus");
        return ESP_FAIL;
    }

    // Configure the PN532 I2C device
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PN532_I2C_ADDRESS,
        .scl_speed_hz = config->clk_speed,
    };

    // Add the device to the I2C bus
    err = i2c_master_bus_add_device(pn532->handle.i2c.bus, &i2c_device_config, &pn532->handle.i2c.dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ESP_FAIL;
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
        ESP_LOGE(TAG, "Failed to create mutex");
        pn532_i2c_free(pn532);
        return ESP_ERR_NO_MEM;
    }

    // Assign function pointers
    pn532->write_command = pn532_i2c_write_command;
    pn532->read_response = pn532_i2c_read_response;
    pn532->free = pn532_i2c_free;

    ESP_LOGI(TAG, "PN532 I2C initialized");
    return ESP_OK;
}
