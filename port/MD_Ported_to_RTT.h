#ifndef MD_PORTED_TO_RTT_H_
#define MD_PORTED_TO_RTT_H_

#include <rtdef.h>
#include "log.h"
#include "motion_driver_example.h" /* Needed to verify the header is truely existent. */

#define i2c_write   imu_i2c_write_regs
#define i2c_read    imu_i2c_read_regs 
#define delay_ms    thread_mdelay
#define get_ms      get_tick_count
#define log_i       MPL_LOGI
#define log_e       MPL_LOGE
#define min(a,b)    ((a<b)?a:b)
#define labs(n)     (((n) < 0) ? (-(n)) : (n))

#define MPU6050
#define EMPL
// #define EMPL_NO_64BIT


// // Max size that can be read across I2C or SPI data lines
// #define INV_MAX_SERIAL_READ 256
// // Max size that can be written across I2C or SPI data lines
// #define INV_MAX_SERIAL_WRITE 256

/* rt_mpu config structure */
struct rt_mpu_config
{
    rt_uint16_t accel_range;
    rt_uint16_t gyro_range;
};

/* rt_mpu device structure */
struct rt_mpu_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct rt_mpu_config config;
};

#endif