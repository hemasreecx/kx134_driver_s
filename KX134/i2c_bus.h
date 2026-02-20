#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   I2C BUS CONFIGURATION
   ============================================================ */

/* Initialize I2C peripheral
   instance     → i2c0 or i2c1
   sda_pin      → SDA GPIO
   scl_pin      → SCL GPIO
   frequency_hz → usually 100000 or 400000
*/
void i2c_bus_init(void *instance,
                  uint8_t sda_pin,
                  uint8_t scl_pin,
                  uint32_t frequency_hz);

/* ============================================================
   BASIC REGISTER ACCESS
   ============================================================ */

/* Write single byte to register */
bool i2c_bus_write(uint8_t device_addr,
                   uint8_t reg_addr,
                   uint8_t value);

/* Read single byte from register */
bool i2c_bus_read(uint8_t device_addr,
                  uint8_t reg_addr,
                  uint8_t *value);

/* Burst read */
bool i2c_bus_read_multi(uint8_t device_addr,
                        uint8_t start_reg,
                        uint8_t *buffer,
                        uint16_t length);

#endif 