#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdio.h>

#include "drivers/kx134.hpp"
#include "modules/imu_conversion.hpp"
#include "app/data_logger.hpp"



#define SYSTEM_DEBUG            0     // 1 = Debug prints, 0 = Silent
#define LOGGER_RAW_MODE         0     // 1 = RAW, 0 = Converted

#define I2C_INSTANCE            i2c0
#define I2C_SDA_PIN             4
#define I2C_SCL_PIN             5
#define I2C_SPEED_HZ            400000

#define KX134_ADDRESS           0x1F

#define SAMPLE_RATE_HZ          50
#define MAIN_LOOP_INTERVAL_MS   (1000 / SAMPLE_RATE_HZ)  // which means  that 20ms between each sample

#define MAX_COMM_ERRORS         10  // if consecutive 10 errors happen -> then IMU reset
#define MAX_RECOVERY_ATTEMPTS   3 


#if SYSTEM_DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif


#if LOGGER_RAW_MODE
DataLogger logger(LoggerMode::RAW);
#else
DataLogger logger(LoggerMode::CONVERTED);
#endif

KX134 imu(I2C_INSTANCE, KX134_ADDRESS);
IMUConversion conversion(KX134_Range::RANGE_64G);

static volatile bool system_initialized = false;
static volatile uint32_t loop_count = 0; // tracks how many samples have been taken



static void i2c_bus_recovery()
{
    DEBUG_PRINT("I2C bus recovery\n");

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_SIO); // software input output
    /*
    generally the scl, sda pins are under the rp2040 hardware control.. it automatically pulls up and down accordingly.. now they are like another GPIO control.. 
    it is in our hands to  set sda, scl -> low / high respectively
    */
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_SIO);

    gpio_set_dir(I2C_SDA_PIN, GPIO_OUT);
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);

    for (int i = 0; i < 9; i++)
    {
        gpio_put(I2C_SCL_PIN, 0);
        sleep_us(5);
        gpio_put(I2C_SCL_PIN, 1);
        sleep_us(5);
    }
    /*
    core logic -> we first set the pins as outputs and then toggling for 9 times.. 9 is because -> if the bus got hang in between ->
    the pins are low and waiting for the next command.. so as 8 bits are for i2c..
     definetely after 9 rtimes  SDA might have sent
    */

    gpio_put(I2C_SDA_PIN, 1);
    gpio_put(I2C_SCL_PIN, 1);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
}



static bool imu_recover()
{
    DEBUG_PRINT("Attempting IMU recovery\n");

    imu.reset();
    sleep_ms(50);
    // after getting multiple errors -> gave 10 here .. IMU gets a  soft reset 

    if (imu.checkID() != KX134_Status::OK)
    {
        DEBUG_PRINT("IMU ID failed after reset\n");
        return false;
    }

    KX134_Config cfg{};
    cfg.range = KX134_Range::RANGE_64G;
    cfg.odr = KX134_ODR::KX134_ODR_50HZ;
    cfg.performance_mode = KX134_Performance::HIGH_PERFORMANCE;
    cfg.enable_fifo = false;
    cfg.enable_data_ready_interrupt = false;

    if (imu.init(cfg) != KX134_Status::OK)
    {
        DEBUG_PRINT("IMU init failed during recovery\n");
        return false;
    }

    DEBUG_PRINT("IMU recovery successful\n");
    return true;
}



bool system_init()
{
    stdio_init_all();
    sleep_ms(500);

    DEBUG_PRINT("\n=== RP2040 KX134 FAULT-PROOF LOGGER ===\n");

    i2c_init(I2C_INSTANCE, I2C_SPEED_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    sleep_ms(100);

    if (imu.checkID() != KX134_Status::OK)
    {
        DEBUG_PRINT("IMU not detected\n");
        return false;
    }

    imu.selfTest();

    if (!imu_recover())
        return false;

    printf("timestamp_us,x,y,z\n");

    system_initialized = true;
    return true;
}


void main_loop()
{
    static uint32_t last_sample_ms = 0;
    static uint32_t comm_errors = 0;
    static uint32_t recovery_attempts = 0;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    if ((now_ms - last_sample_ms) < MAIN_LOOP_INTERVAL_MS)
        return;

    last_sample_ms = now_ms;

    if (!imu.dataReady())
        return;

    KX134_Raw raw{};
    KX134_Status status = imu.readRaw(raw);

    if (status == KX134_Status::OK)
    {
        comm_errors = 0;
        recovery_attempts = 0;

        uint32_t ts_us = to_us_since_boot(get_absolute_time());
        IMU_Data conv = conversion.convert(raw);
        logger.log(ts_us, raw, conv);
    }
    else
    {
        comm_errors++;
        DEBUG_PRINT("IMU read error: %d\n", (int)status);

        if (comm_errors >= MAX_COMM_ERRORS)
        {
            DEBUG_PRINT("Comm error threshold reached\n");
            i2c_bus_recovery();

            if (!imu_recover())
            {
                recovery_attempts++;

                if (recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                {
                    DEBUG_PRINT("FATAL: IMU unrecoverable\n");
                    system_initialized = false;
                    return;
                }
            }

            comm_errors = 0;
        }
    }

    loop_count++;
}


int main()
{
    if (!system_init())
    {
        printf("FATAL: System init failed\n");
        while (true)
            sleep_ms(1000);
    }

    while (true)
    {
        if (system_initialized)
            main_loop();

        sleep_ms(1);
    }
}
