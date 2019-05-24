#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "MD_Ported_to_RTT.h"
#include "inv_mpu.h"
#include "mltypes.h"

// #define MD_DEBUG
#define LOG_TAG "md.example"
#include "md_log.h"

int mpu_dev_init_flag = 0;
struct rt_mpu_device *mpu_dev;

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
        LOG_E("Unsupported device bus type:'%d'!", dev->bus->type);
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
        LOG_E("Unsupported device bus type:'%d'!", dev->bus->type);
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
        LOG_E("Can't allocate memory for rt_mpu device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        goto __exit;
    }
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
                    LOG_E("Can't find device at '%s'!", dev_name);
                    goto __exit;
                }
            }
            LOG_I("Device i2c address is:'0x%x'!", dev->i2c_addr);
        }
    }

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
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    if (rt_mpu_read_regs(dev, 0x75, 1, &reg) != RT_EOK)
    {
        LOG_E("Failed to read device id!");
        goto __exit;
    }
    
    dev->id = reg;

    switch (dev->id)
    {
    case 0x68:
        LOG_I("Find device: mpu6050!");
        break;
    case 0x70:
        LOG_I("Find device: mpu6500!");
        break;
    case 0x71:
        LOG_I("Find device: mpu9250!");
        break;
    case 0xAF:
        LOG_I("Find device: icm20608!");
        break;
    case 0xFF:
        LOG_E("No device connection!");
        goto __exit;
    default:
        LOG_I("Unknown device id: 0x%x!", reg);
    }

    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
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
}

rt_err_t imu_i2c_write_regs(unsigned char slave_addr, unsigned char reg_addr, unsigned short length, const unsigned char *data_ptr)
{
    int i;
    rt_err_t res;

    if (!rt_mpu_write_reg(mpu_dev, reg_addr, length, data_ptr))
    {
        res = RT_EOK;
    }
    else
    {
        LOG_E("i2c trans error!");
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
        LOG_E("i2c trans error!");
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
