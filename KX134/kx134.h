#ifndef KX134_H
#define KX134_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifndef KX134_DEBUG
#define KX134_DEBUG 0
#endif


#define KX134_I2C_ADDR_LOW      0x1E
#define KX134_I2C_ADDR_HIGH     0x1F

#define KX134_WHO_AM_I_VALUE    0x46

typedef enum 
    {
        MAN_ID = 0x00,
        PART_ID = 0x01,
        XADP_L = 0x02,
        XADP_H = 0x03,
        YADP_L = 0x04,
        YADP_H = 0x05,
        ZADP_L = 0x06,
        ZADP_H = 0x07,
        XOUT_L = 0x08,
        XOUT_H = 0x09,
        YOUT_L = 0x0A,
        YOUT_H = 0x0B,
        ZOUT_L = 0x0C,
        ZOUT_H = 0x0D,
        COTR = 0x12,
        WHO_AM_I = 0x13,
        TSCP = 0x14,
        TSPP = 0x15,
        INS1 = 0x16,
        INS2 = 0x17,
        INS3 = 0x18,
        STATUS_REG = 0x19,
        INT_REL = 0x1A,
        CNTL1 = 0x1B,
        CNTL2 = 0x1C,
        CNTL3 = 0x1D,
        CNTL4 = 0x1E,  
        CNTL5 = 0x1F,
        CNTL6 = 0x20,
        ODCNTL = 0x21,
        INC1 = 0x22,
        INC2 = 0x23,
        INC3 = 0x24,
        INC4 = 0x25,
        INC5 = 0x26,
        INC6 = 0x27,
        TILT_TIMER = 0x29,
        TDTRC = 0x2A,
        TDTC = 0x2B,
        TTH = 0x2C,
        TTL = 0x2D,
        FTD = 0x2E,
        STD = 0x2F,
        TLT = 0x30,
        TWS = 0x31,
        FFTH = 0x32,
        FFC = 0x33,
        FFCNTL = 0x34,
        TILT_ANGLE_LL = 0x37,
        TILT_ANGLE_HL = 0x38,
        HYST_SET = 0x39,
        LP_CNTL1 = 0x3A,
        LP_CNTL2 = 0x3B,
        WUFTH = 0x49,
        BTSWUFTH = 0x4A,
        BTSTH = 0x4B,
        BTSC = 0x4C,
        WUFC = 0x4D,
        SELF_TEST = 0x5D,
        BUF_CNTL1 = 0x5E,
        BUF_CNTL2 = 0x5F,
        BUF_STATUS_1 = 0x60,
        BUF_STATUS_2 = 0x61,
        BUF_CLEAR = 0x62,
        BUF_READ = 0x63,
        ADP_CNTL1 = 0x64,
        ADP_CNTL2 = 0x65,
        ADP_CNTL3 = 0x66,
        ADP_CNTL4 = 0x67,
        ADP_CNTL5 = 0x68,
        ADP_CNTL6 = 0x69,
        ADP_CNTL7 = 0x6A,
        ADP_CNTL8 = 0x6B,
        ADP_CNTL9 = 0x6C,
        ADP_CNTL10 = 0x6D,
        ADP_CNTL11 = 0x6E,
        ADP_CNTL12 = 0x6F,
        ADP_CNTL13 = 0x70,
        ADP_CNTL14 = 0x71,
        ADP_CNTL15 = 0x72,
        ADP_CNTL16 = 0x73,
        ADP_CNTL17 = 0x74,
        ADP_CNTL18 = 0x75,
        ADP_CNTL19 = 0x76,
        INTERNAL_0X7F = 0x7F
    } kx134_registers_t;

typedef enum{
    KX134_RANGE_8G = 0x00,
    KX134_RANGE_16G = 0x01,
    KX134_RANGE_32G = 0x02,
    KX134_RANGE_64G = 0x03
} kx134_range_t;

typedef enum {
    KX134_ODR_0_781HZ  = 0x00,
    KX134_ODR_1_563HZ  = 0x01,
    KX134_ODR_3_125HZ  = 0x02,
    KX134_ODR_6_25HZ   = 0x03,
    KX134_ODR_12_5HZ   = 0x04,
    KX134_ODR_25HZ     = 0x05,
    KX134_ODR_50HZ     = 0x06,
    KX134_ODR_100HZ    = 0x07,
    KX134_ODR_200HZ    = 0x08,
    KX134_ODR_400HZ    = 0x09,
    KX134_ODR_800HZ    = 0x0A,
    KX134_ODR_1600HZ   = 0x0B,
    KX134_ODR_3200HZ   = 0x0C,
    KX134_ODR_6400HZ   = 0x0D,
    KX134_ODR_12800HZ  = 0x0E,
    KX134_ODR_25600HZ  = 0x0F
} kx134_odr_t;


typedef enum {
    KX134_LOW_POWER = 0,
    KX134_HIGH_PERFORMANCE = 1
} kx134_perf_mode_t;


typedef enum {
    KX134_OK = 0,
    KX134_ERR_COMM,
    KX134_ERR_INVALID_ID,
    KX134_ERR_CONFIG
} kx134_status_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} kx134_raw_t;

typedef struct {
    kx134_range_t      range;
    kx134_odr_t        odr;
    kx134_perf_mode_t  performance_mode;
    bool               enable_data_ready_interrupt;
    bool               enable_fifo;
} kx134_config_t;

kx134_status_t kx134_init(const kx134_config_t *config);
kx134_status_t kx134_reset(void);
kx134_status_t kx134_check_id(void);
kx134_status_t kx134_read_raw(kx134_raw_t *raw_data);
uint16_t kx134_read_fifo(kx134_raw_t *buffer, uint16_t max_samples);
kx134_status_t kx134_get_range(kx134_range_t *range);
bool kx134_fifo_overflow(void);
bool kx134_data_ready(void);
kx134_status_t kx134_self_test(void);
kx134_status_t kx134_set_range(kx134_range_t range); 
kx134_status_t kx134_enable(void);   // set PC1 = 1
kx134_status_t kx134_disable(void);  // clear PC1 = 0
kx134_status_t kx134_write_register(kx134_registers_t reg, uint8_t value);
kx134_status_t kx134_read_register(kx134_registers_t reg, uint8_t *value);
kx134_status_t kx134_read_reg_multi(kx134_registers_t reg, uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif 