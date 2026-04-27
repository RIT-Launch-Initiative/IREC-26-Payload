#include "sx126x_hal.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <gpiod.h>

typedef struct {
    int spiFd;
    struct gpiod_line* reset;
    struct gpiod_line* busy;
} sx126x_hal_context_t;

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, 
                                     const uint16_t command_length, const uint8_t* data, 
                                     const uint16_t data_length) {
    sx126x_hal_context_t* ctx = (sx126x_hal_context_t*)context;
    struct spi_ioc_transfer buffer[2];
    memset(buffer, 0, sizeof(buffer));

    buffer[0].tx_buf = (unsigned long)command;
    buffer[0].len = command_length;

    buffer[1].tx_buf = (unsigned long)data;
    buffer[1].len = data_length;

    if (ioctl(ctx->spiFd, SPI_IOC_MESSAGE(2), buffer) < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, 
                                    const uint16_t command_length, uint8_t* data, 
                                    const uint16_t data_length) {
    sx126x_hal_context_t* ctx = (sx126x_hal_context_t*)context;
    struct spi_ioc_transfer buffer[2];
    memset(buffer, 0, sizeof(buffer));

    buffer[0].tx_buf = (unsigned long)command;
    buffer[0].len = command_length;

    buffer[1].rx_buf = (unsigned long)data;
    buffer[1].len = data_length;

    if (ioctl(ctx->spiFd, SPI_IOC_MESSAGE(2), buffer) < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    sx126x_hal_context_t* ctx = (sx126x_hal_context_t*)context;
    
    gpiod_line_set_value(ctx->reset, 0);
    usleep(2000);
    gpiod_line_set_value(ctx->reset, 1);
    usleep(5000);
    
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    sx126x_hal_context_t* ctx = (sx126x_hal_context_t*)context;
    
    // wakeup involves pulling CS low so write a dummy byte to toggle CS automatically
    uint8_t dummy = 0x00;
    struct spi_ioc_transfer buffer;
    memset(&buffer, 0, sizeof(buffer));
    buffer.tx_buf = (unsigned long)&dummy;
    buffer.len = 1;
    
    if (ioctl(ctx->spiFd, SPI_IOC_MESSAGE(1), &buffer) < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    
    // block until not busy
    if (ctx->busy) {
        while (gpiod_line_get_value(ctx->busy) == 1) {
            usleep(100);
        }
    }

    return SX126X_HAL_STATUS_OK;
}