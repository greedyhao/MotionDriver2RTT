#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "MD_Ported_to_RTT.h"
#include "inv_mpu.h"
#include "mltypes.h"

// static struct rt_device _dev_bus = {0};
// static struct rt_mpu_device _dev = {0};
extern struct rt_mpu_device *mpu_dev;

/**
 * This function writes the value of the register for rt_mpu
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for rt_mpu
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing the value of the register successfully.
 */
static rt_err_t rt_mpu_write_reg(struct rt_mpu_device *dev, rt_uint8_t reg, unsigned short length, const unsigned char *data)
{
    rt_int8_t res = 0;
    // rt_kprintf("dev i2c addr:%p, type:%d\n", dev->i2c_addr, dev->bus->type);
    // rt_kprintf("i2c starting to trans..\n");
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs;
    // rt_uint8_t buf[2] = {reg, data};
    rt_uint8_t buf[length+1];
    buf[0] = reg;
    int i;
    for ( i = 1; i <= length; i++)
    {
        buf[i] = data[i-1];
    }
    
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs.addr  = dev->i2c_addr;    /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = buf;              /* Send data pointer */
        msgs.len   = length+1;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        res = rt_spi_send_then_send((struct rt_spi_device *)dev->bus, &reg, 1, &data, 1);
#endif
    }
    return res;
}

/**
 * This function reads the value of registers for rt_mpu
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for rt_mpu
 * @param len number of register
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the value of registers successfully.
 */
static rt_err_t rt_mpu_read_regs(struct rt_mpu_device *dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
#endif
#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;              /* Read data pointer */
        msgs[1].len   = len;              /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf, len);
#endif
    }
    return res;
}


/**
 * This function initialize the rt_mpu device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct rt_mpu_device *rt_mpu_init(const char *dev_name, rt_uint8_t param, struct int_param_s *int_param)
{
    struct rt_mpu_device *dev = RT_NULL;
    inv_error_t result;
    rt_uint8_t reg = 0xFF;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct rt_mpu_device));
    if (dev == RT_NULL)
    {
        log_e("Can't allocate memory for rt_mpu device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        log_e("Can't find device:'%s'", dev_name);
        goto __exit;
    }
#if 1
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            /* find rt_mpu device at address: 0x68 */
            dev->i2c_addr = 0x68;
            if (rt_mpu_read_regs(dev, 0x75, 1, &reg) != RT_EOK)
            {
                /* find rt_mpu device at address 0x69 */
                dev->i2c_addr = 0x69;
                if (rt_mpu_read_regs(dev, 0x75, 1, &reg) != RT_EOK)
                {
                    log_e("Can't find device at '%s'!", dev_name);
                    goto __exit;
                }
            }
            log_i("Device i2c address is:'0x%x'!", dev->i2c_addr);
        }
    }
// #else
    else if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
// #ifdef RT_USING_SPI
//         struct rt_spi_configuration cfg;

//         cfg.data_width = 8;
//         cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
//         cfg.max_hz = (1000 * 1000); /* Set spi max speed */

//         rt_spi_configure((struct rt_spi_device *)dev->bus, &cfg);
// #endif
    }
    else
    {
        log_e("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    if (rt_mpu_read_regs(dev, 0x75, 1, &reg) != RT_EOK)
    {
        log_e("Failed to read device id!");
        goto __exit;
    }
// #else
    dev->id = reg;

    switch (dev->id)
    {
    case 0x68:
        log_i("Find device: mpu6050!");
        break;
    case 0x70:
        log_i("Find device: mpu6500!");
        break;
    case 0x71:
        log_i("Find device: mpu9250!");
        break;
    case 0xAF:
        log_i("Find device: icm20608!");
        break;
    case 0xFF:
        log_e("No device connection!");
        goto __exit;
    default:
        log_i("Unknown device id: 0x%x!", reg);
    }
// #else
    // rt_mpu_get_param(dev, MPU6XXX_ACCEL_RANGE, &dev->config.accel_range);
    // rt_mpu_get_param(dev, MPU6XXX_GYRO_RANGE, &dev->config.gyro_range);

    // rt_mpu_write_bits(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_CLKSEL_BIT, MPU6XXX_PWR1_CLKSEL_LENGTH, MPU6XXX_CLOCK_PLL_XGYRO);
    // rt_mpu_set_param(dev, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_250DPS);
    // rt_mpu_set_param(dev, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);
    // rt_mpu_set_param(dev, MPU6XXX_SLEEP, MPU6XXX_SLEEP_DISABLE);

    // result = mpu_init(&int_param);
    // if (result) {
    //     log_e("Could not initialize gyro.\n");
    // }
// #else
    // log_i("rt_mpu init succeed!");

    // log_i("before:%p\n", _dev);
    // _dev = dev;
    // log_i("after:%p\n", _dev);
    // rt_kprintf("point of _dev:%p\n", &_dev);
    // memcpy(&_dev, dev, sizeof(struct rt_mpu_device));
    // memcpy(&_dev_bus, dev->bus, sizeof(struct rt_device));
    // _dev.bus = &_dev_bus;
    // log_i("init i2c_addr:%p, type:%d\n", _dev.i2c_addr, _dev.bus->type);
    // log_i("i2c_add:%s\n", _dev->bus);
    // log_i("i2c_add:%s\n", mpu_dev->i2c_addr);
#endif
    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
        // _dev = RT_NULL;
        // rt_free(_dev);
    }
    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void rt_mpu_deinit(struct rt_mpu_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
    // _dev = RT_NULL;
    // rt_free(_dev);
}

rt_err_t imu_i2c_write_regs(unsigned char slave_addr, unsigned char reg_addr, unsigned short length, const unsigned char *data_ptr)
{
    int i;
    rt_err_t res;
    // log_i("befor write reg i2c_addr:%p, type:%d\n", mpu_dev->i2c_addr, mpu_dev->bus->type);

    // for(i = 0; i < (length); i++)
    // {
    //     if (!rt_mpu_write_reg(mpu_dev, reg_addr, data_ptr[i]))
    //     {
    //         res = RT_EOK;
    //     }
    //     else
    //     {
    //         log_e("i2c trans error!\n");
    //         res = -RT_ERROR;
    //     }
    // }
    if (!rt_mpu_write_reg(mpu_dev, reg_addr, length, data_ptr))
    {
        res = RT_EOK;
    }
    else
    {
        log_e("i2c trans error!\n");
        res = -RT_ERROR;
    }
    return res;
}

rt_err_t imu_i2c_read_regs(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    rt_err_t res;
    
    if (!rt_mpu_read_regs(mpu_dev, reg_addr, length, data))
    {
        res = RT_EOK;
    }
    else
    {
        log_e("i2c trans error!\n");
        res = -RT_ERROR;
    }
    return res;
}

void thread_mdelay(unsigned long ms)
{
    rt_thread_mdelay(ms);
}

rt_uint32_t get_tick_count(void)
{
    return rt_tick_get() * 1000 / RT_TICK_PER_SECOND;
}
