#ifndef _ARDUINO_INTERFACE_H_
#define _ARDUINO_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"

// I2C
#define I2C_MASTER_NUM ((i2c_port_num_t)CONFIG_I2C_HOST_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO (CONFIG_IIC_SCL)                   /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO (CONFIG_IIC_SDA)                   /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_FREQ_HZ (400000)                          /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS (1000)

// SPI
#define SPI_MASTER_NUM ((spi_host_device_t)CONFIG_SPI_HOST_NUM)
#define SPI_MASTER_MOSI_IO (CONFIG_SPI_MOSI)
#define SPI_MASTER_MISO_IO (CONFIG_SPI_MISO)
#define SPI_MASTER_SCLK_IO (CONFIG_SPI_CLK)
#define SPI_MASTER_CS_IO (CONFIG_SPI_CS0)

// UART
#define UART_MASTER_NUM ((uart_port_t)CONFIG_FFB_UART_PORT_NUM)
#define UART_MASTER_TX_IO (CONFIG_FFB_UART_TXD)
#define UART_MASTER_RX_IO (CONFIG_FFB_UART_RXD)

// PWM
#define MOTOR_A (CONFIG_MOTOR_A)
#define MOTOR_B (CONFIG_MOTOR_B)
#define MOTOR_C (CONFIG_MOTOR_C)
#define MOTOR_EN (CONFIG_MOTOR_EN)

void arduino_serial_init(void);
void arduino_spi_init(void);
void arduino_tim_init(void);
void arduino_hi2c_init(void);

#ifdef __cplusplus
}
#endif

#endif
