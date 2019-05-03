#include <rtthread.h>
#include <finsh.h>

#include "MD_Ported_to_RTT.h"
#include "motion_driver_example.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
/* Private typedef -----------------------------------------------------------*/
/* The bus name of the mpu. */
#define RT_MPU_DEVICE_NAME "i2c2"

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

struct rt_mpu_device *mpu_dev;

//q30，q16格式,long转float时的除数.
#define q30  1073741824.0f
#define q16  65536.0f

//陀螺仪方向设置
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
//磁力计方向设置
static signed char comp_orientation[9] = { 0, 1, 0,
                                           1, 0, 0,
                                           0, 0,-1};

rt_err_t mpu_dmp_init()
{
    rt_err_t res = 0;
    struct int_param_s int_param;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    float pitch,roll,yaw; 	        //欧拉角
	short aacx,aacy,aacz;	        //加速度传感器原始数据
	short gyrox,gyroy,gyroz;        //陀螺仪原始数据
    // unsigned short compass_fsr;

    /* Initialize mpu6xxx, The parameter is RT_NULL, means auto probing for i2c*/
    mpu_dev = rt_mpu_init(RT_MPU_DEVICE_NAME, RT_NULL);
    
    res = mpu_init(&int_param);
    rt_kprintf("mpu_init end\n");
    if (res) {
        log_e("Could not initialize gyro.\n");
    }
    else
    {
        rt_kprintf("inv_init_mpl..\n");
        res=inv_init_mpl();     //初始化MPL
        if(res)return 1;
        inv_enable_quaternion();
        inv_enable_9x_sensor_fusion();
        inv_enable_fast_nomot();
        // inv_enable_gyro_tc();
        // inv_enable_vector_compass_cal();
        // inv_enable_magnetic_disturbance();
        inv_enable_eMPL_outputs();
        rt_kprintf("inv_start_mpl..\n");
        res=inv_start_mpl();    //开启MPL
        if(res)return 1;
        rt_kprintf("mpu_set_sensors..\n");
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		if(res)return 2; 
        rt_kprintf("mpu_configure_fifo..\n");
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);   //设置FIFO
		if(res)return 3; 
        rt_kprintf("mpu_set_sample_rate..\n");
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	            //设置采样率
		if(res)return 4; 
        // rt_kprintf("mpu_set_compass_sample_rate..\n");
        // res=mpu_set_compass_sample_rate(1000/COMPASS_READ_MS);  //设置磁力计采样率
        // if(res)return 5;
        mpu_get_sample_rate(&gyro_rate);
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);
        // mpu_get_compass_fsr(&compass_fsr);
        inv_set_gyro_sample_rate(1000000L/gyro_rate);
        inv_set_accel_sample_rate(1000000L/gyro_rate);
        // inv_set_compass_sample_rate(COMPASS_READ_MS*1000L);
        inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)gyro_fsr<<15);
        inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)accel_fsr<<15);
        // inv_set_compass_orientation_and_scale(
        //     inv_orientation_matrix_to_scalar(comp_orientation),(long)compass_fsr<<15);
            
        rt_kprintf("dmp_load_motion_driver_firmware..\n");
		res=dmp_load_motion_driver_firmware();		             //加载dmp固件
		if(res)return 6; 
        rt_kprintf("dmp_set_orientation..\n");
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res)return 7; 
        rt_kprintf("dmp_enable_feature..\n");
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	            //设置dmp功能
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 8; 
        rt_kprintf("dmp_set_fifo_rate..\n");
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		if(res)return 9;   
        rt_kprintf("run_self_test..\n");
		res=run_self_test();		//自检
		if(res)return 10;    
        rt_kprintf("mpu_set_dmp_state..\n");
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 11; 
    }

    while (1)
    {
        if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
        {
            // MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			// MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
            // rt_kprintf("%d %d %d %d %d %d \n", aacx,aacy,aacz,gyrox,gyroy,gyroz);
            rt_kprintf("r:%d p:%d y:%d \n",(int)(roll*100),(int)(pitch*100),(int)(yaw*100));
        }
        rt_thread_mdelay(1000);
    }
    
    return 0;
}

//MPU9250自测试
//返回值:0,正常
//    其他,失败
uint8_t run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x7) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
        unsigned short accel_sens;
		float gyro_sens;

		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);
        //inv_set_gyro_bias(gyro, 3);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
       // inv_set_accel_bias(accel, 3);
		dmp_set_accel_bias(accel);
		return 0;
	}else return 1;
}

//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30格式转换为浮点数
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		//计算得到俯仰角/横滚角/航向角
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}else return 2;
	return 0;
}

//得到mpl处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
uint8_t mpu_mpl_get_data(float *pitch,float *roll,float *yaw)
{
	unsigned long sensor_timestamp,timestamp;
	short gyro[3], accel_short[3],compass_short[3],sensors;
	unsigned char more;
	long compass[3],accel[3],quat[4],temperature; 
    long data[9];
    int8_t accuracy;
    
    rt_kprintf("dmp_read_fifo..\n");
	if(dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,&more))return 1;	 

    if(sensors&INV_XYZ_GYRO)
    {
        rt_kprintf("inv_build_gyro..\n");
        inv_build_gyro(gyro,sensor_timestamp);          //把新数据发送给MPL
        mpu_get_temperature(&temperature,&sensor_timestamp);
        inv_build_temp(temperature,sensor_timestamp);   //把温度值发给MPL，只有陀螺仪需要温度值
    }
    
    if(sensors&INV_XYZ_ACCEL)
    {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        rt_kprintf("inv_build_accel..\n");
        inv_build_accel(accel,0,sensor_timestamp);      //把加速度值发给MPL
    }
    
    // if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) 
    // {
    //     compass[0]=(long)compass_short[0];
    //     compass[1]=(long)compass_short[1];
    //     compass[2]=(long)compass_short[2];
    //     inv_build_compass(compass,0,sensor_timestamp); //把磁力计值发给MPL
    // }
    rt_kprintf("inv_execute_on_data..\n");
    inv_execute_on_data();
    rt_kprintf("inv_get_sensor_type_euler..\n");
    inv_get_sensor_type_euler(data,&accuracy,&timestamp);
    
    *roll  = (data[0]/q16);
    *pitch = -(data[1]/q16);
    *yaw   = -data[2] / q16;
	return 0;
}

int motion_driver_example(void)
{
    rt_err_t ret = RT_EOK;
    rt_thread_t thread = RT_NULL;

    thread = rt_thread_create("motion_driver",
                            mpu_dmp_init, RT_NULL,
                            2*1024, 5, 10);
    if(thread == RT_NULL)
    {
        return RT_ERROR;
    }
    rt_thread_startup(thread);

    return RT_EOK;
}
MSH_CMD_EXPORT(motion_driver_example, motion driver test function);
