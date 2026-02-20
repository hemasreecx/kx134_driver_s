#include "kx134.hpp"
#include<inttypes.h>
#include "i2c_bus.hpp"
#include "pico/stdlib.h"

#if KX134_DEBUG
#include <stdio.h>
#define KX134_LOG(...) printf(__VA_ARGS__)
#else
#define KX134_LOG(...)
#endif

#define KX134_I2C_ADDR KX134_I2C_ADDR_LOW

static int16_t combine_bytes(uint8_t low, uint8_t high) {
    KX134_LOG("Combining bytes: high=0x%02X, low=0x%02X\n", high, low);
    return (int16_t)((high << 8) | low);
}

kx134_status_t  kx134_write_register(kx134_registers_t reg, uint8_t value) {
    
     KX134_LOG("Write Reg 0x%02X <- 0x%02X\n", reg, value);

    if (!i2c_bus_write(KX134_I2C_ADDR, (uint8_t)reg, value))
        return KX134_ERR_COMM;

    return KX134_OK;
}

kx134_status_t kx134_read_register(kx134_registers_t reg, uint8_t *value)
{
    if (!i2c_bus_read(KX134_I2C_ADDR, (uint8_t)reg, value))
        return KX134_ERR_COMM;

    KX134_LOG("Read Reg 0x%02X -> 0x%02X\n", reg, *value);
    return KX134_OK;
}

kx134_status_t kx134_read_reg_multi(kx134_registers_t reg,
                                    uint8_t *buffer,
                                    uint16_t length)
{
    if (!i2c_bus_read_multi(KX134_I2C_ADDR, (uint8_t)reg, buffer, length))
        return KX134_ERR_COMM;

    KX134_LOG("Burst Read Reg 0x%02X (%d bytes)\n", reg, length);

    return KX134_OK;
}
kx134_status_t kx134_check_id(void)
{
    uint8_t id;

    if (kx134_read_register(WHO_AM_I, &id) != KX134_OK)
        return KX134_ERR_COMM;

    KX134_LOG("WHO_AM_I = 0x%02X\n", id);

    if (id != KX134_WHO_AM_I_VALUE)
        return KX134_ERR_INVALID_ID;

    return KX134_OK;
}

kx134_status_t kx134_reset(void)
{
    KX134_LOG("Performing  kx134-1211 software reset\n");

    if (kx134_write_register(CNTL2, 0x80) != KX134_OK)
        return KX134_ERR_COMM;

    sleep_ms(2);

    return kx134_check_id();
}

kx134_status_t kx134_disable(void)
{
    uint8_t reg;

    if (kx134_read_register(CNTL1, &reg) != KX134_OK)
        return KX134_ERR_COMM;

    reg &= ~(1 << 7);   // Clear PC1

    KX134_LOG("Disable (PC1=0)\n");

    return kx134_write_register(CNTL1, reg);
}

kx134_status_t kx134_enable(void)
{
    uint8_t reg;

    if (kx134_read_register(CNTL1, &reg) != KX134_OK)
        return KX134_ERR_COMM;

    reg |= (1 << 7);    // Set PC1

    KX134_LOG("Enable (PC1=1)\n");

    return kx134_write_register(CNTL1, reg);
}

kx134_status_t kx134_set_range(kx134_range_t range)
{
    uint8_t reg;

    KX134_LOG("Setting range to %d\n", range);

    if (kx134_disable() != KX134_OK)
        return KX134_ERR_COMM;

    if (kx134_read_register(CNTL1, &reg) != KX134_OK)
        return KX134_ERR_COMM;

    reg &= ~(0x03 << 3);               // Clear GSEL bits
    reg |= ((range & 0x03) << 3);      // Set new range

    if (kx134_write_register(CNTL1, reg) != KX134_OK)
        return KX134_ERR_COMM;

    return kx134_enable();
}

kx134_status_t kx134_get_range(kx134_range_t *range)
{
    uint8_t reg;

    if (kx134_read_register(CNTL1, &reg) != KX134_OK)
        return KX134_ERR_COMM;

    *range = (kx134_range_t)((reg >> 3) & 0x03);

    KX134_LOG("Current range = %d\n", *range);

    return KX134_OK;
}
kx134_status_t kx134_read_raw(kx134_raw_t *raw_data)
{
    uint8_t buffer[6];

    if (kx134_read_reg_multi(XOUT_L, buffer, 6) != KX134_OK)
        return KX134_ERR_COMM;

    raw_data->x = combine_bytes(buffer[0], buffer[1]);
    raw_data->y = combine_bytes(buffer[2], buffer[3]);
    raw_data->z = combine_bytes(buffer[4], buffer[5]);

    KX134_LOG("RAW: X=%d Y=%d Z=%d\n",
              raw_data->x,
              raw_data->y,
              raw_data->z);

    return KX134_OK;
}

bool kx134_data_ready(void)
{
    uint8_t status;

    if (kx134_read_register(INS2, &status) != KX134_OK)
        return false;

    bool ready = (status & (1 << 4)) ? true : false;

    KX134_LOG("Data Ready = %d\n", ready);

    return ready;
}
uint16_t kx134_read_fifo(kx134_raw_t *buffer, uint16_t max_samples)
{
    uint8_t fifo_samples;

    if (kx134_read_register(BUF_STATUS_1, &fifo_samples) != KX134_OK)
        return 0;

    uint16_t samples = fifo_samples;
    if (samples > max_samples)
        samples = max_samples;

    KX134_LOG("FIFO samples available = %d\n", samples);

    for (uint16_t i = 0; i < samples; i++)
    {
        uint8_t data[6];

        if (kx134_read_reg_multi(BUF_READ, data, 6) != KX134_OK)
            return i;

        buffer[i].x = combine_bytes(data[0], data[1]);
        buffer[i].y = combine_bytes(data[2], data[3]);
        buffer[i].z = combine_bytes(data[4], data[5]);
    }

    return samples;
}
bool kx134_fifo_overflow(void)
{
    uint8_t status;

    if (kx134_read_register(BUF_STATUS_2, &status) != KX134_OK)
        return false;

    bool overflow = (status & (1 << 7)) ? true : false;

    KX134_LOG("FIFO Overflow = %d\n", overflow);

    return overflow;
}

kx134_status_t kx134_self_test(void)
{
    KX134_LOG("Starting self-test\n");

    /* Enter self-test mode (per TRM requirement) */
    if (kx134_write_register(SELF_TEST, 0xCA) != KX134_OK)
        return KX134_ERR_COMM;

    /* Allow time for self-test execution */
    sleep_ms(2);

    /* Verify COTR register */
    uint8_t cotr;
    if (kx134_read_register(COTR, &cotr) != KX134_OK)
        return KX134_ERR_COMM;

    KX134_LOG("COTR after self-test = 0x%02X\n", cotr);

    /* Exit self-test mode (VERY IMPORTANT) */
    if (kx134_write_register(SELF_TEST, 0x00) != KX134_OK)
        return KX134_ERR_COMM;

    if (cotr != 0x55)
    {
        KX134_LOG("Self-test FAILED\n");
        return KX134_ERR_CONFIG;
    }

    KX134_LOG("Self-test PASSED\n");
    return KX134_OK;
}

kx134_status_t kx134_init(const kx134_config_t *config)
{
    KX134_LOG("Initializing KX134 (deterministic mode)\n");

    if (config == NULL)
        return KX134_ERR_CONFIG;

    if (kx134_reset() != KX134_OK)
        return KX134_ERR_COMM;

    if (kx134_disable() != KX134_OK)
        return KX134_ERR_COMM;

    uint8_t cntl1 = 0;

    /* Bit 6: RES (performance mode) */
    cntl1 |= ((config->performance_mode & 0x01) << 6);

    /* Bit 5: DRDYE enable */
    if (config->enable_data_ready_interrupt)
        cntl1 |= (1 << 5);

    /* Bits 4:3 = GSEL (range) */
    cntl1 |= ((config->range & 0x03) << 3);

    /* Bits 2:1: TDTE, TPE left disabled */

    KX134_LOG("CNTL1 = 0x%02X\n", cntl1);

    if (kx134_write_register(CNTL1, cntl1) != KX134_OK)
        return KX134_ERR_CONFIG;

    /* 4️⃣ Build ODCNTL explicitly */

    uint8_t odcntl = 0;

    /* Bit 7: IIR_BYPASS = 0 (filter ON) */
    /* Bit 6: LPRO = 0 (ODR/9 rolloff) */
    /* Bit 5: FSTUP = 1 (fast startup recommended) */
    odcntl |= (1 << 5);

    /* Bits 3:0 = ODR */
    odcntl |= (config->odr & 0x0F);

    KX134_LOG("ODCNTL = 0x%02X\n", odcntl);

    if (kx134_write_register(ODCNTL, odcntl) != KX134_OK)
        return KX134_ERR_CONFIG;

    /* 5️⃣ Enable PC1 (start measurements) */
    if (kx134_enable() != KX134_OK)
        return KX134_ERR_COMM;

    KX134_LOG("Initialization complete\n");

    return KX134_OK;
}