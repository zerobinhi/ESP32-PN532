#include "pn532.h"

#if CONFIG_LOG_DEFAULT_LEVEL >= 4 // 4 = LOG_LEVEL_DEBUG
#define PN532_DEBUG
#endif

#define PN532_DELAY_DEFAULT (pdMS_TO_TICKS(200))
#define PN532_DELAY(ms) (pdMS_TO_TICKS(ms))

/**
 * @brief Runtime UART handle (internal use)
 */
typedef struct
{
    uart_port_t port;
} pn532_uart_handle_t;

/**
 * @brief Runtime I2C handle (internal use)
 */
typedef struct
{
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} pn532_i2c_handle_t;

/**
 * @brief Runtime SPI handle (internal use)
 */
typedef struct
{
    spi_device_handle_t dev;
} pn532_spi_handle_t;

typedef struct pn532_t
{
    pn532_protocol_t protocol;

    union
    {
        pn532_uart_handle_t uart;
        pn532_i2c_handle_t i2c;
        pn532_spi_handle_t spi;
    } handle;

    SemaphoreHandle_t mutex;

    esp_err_t (*write_command)(struct pn532_t *pn532, uint8_t *cmd, uint8_t cmd_len);
    esp_err_t (*read_response)(struct pn532_t *pn532, uint8_t *buf, uint8_t len);
    esp_err_t (*free)(struct pn532_t *pn532);
} pn532_t;

/* 工厂函数（由 pn532.c 内部实现） */
esp_err_t pn532_protocol_init(pn532_t *dev, const void *config);