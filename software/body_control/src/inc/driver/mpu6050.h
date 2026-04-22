#ifndef __MPU6050_H
#define __MPU6050_H
#include "iic.h"
#include <math.h>

#define MPU_IIC_Init IIC_Init
#define MPU_IIC_Start IIC_Start
#define MPU_IIC_Stop IIC_Stop
#define MPU_IIC_Send_Byte IIC_Send_Byte
#define MPU_IIC_Read_Byte IIC_Read_Byte
#define MPU_IIC_Wait_Ack IIC_Wait_Ack

// #define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
// #define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到
#define MPU_SELF_TESTX_REG 0X0D   // 自检寄存器X
#define MPU_SELF_TESTY_REG 0X0E   // 自检寄存器Y
#define MPU_SELF_TESTZ_REG 0X0F   // 自检寄存器Z
#define MPU_SELF_TESTA_REG 0X10   // 自检寄存器A
#define MPU_SAMPLE_RATE_REG 0X19  // 采样频率分频器
#define MPU_CFG_REG 0X1A          // 配置寄存器
#define MPU_GYRO_CFG_REG 0X1B     // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG 0X1C    // 加速度计配置寄存器
#define MPU_MOTION_DET_REG 0X1F   // 运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG 0X23      // FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG 0X24  // IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG 0X25 // IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG 0X26      // IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG 0X27 // IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG 0X28 // IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG 0X29      // IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG 0X2A // IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG 0X2B // IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG 0X2C      // IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG 0X2D // IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG 0X2E // IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG 0X2F      // IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG 0X30 // IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG 0X31 // IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG 0X32      // IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG 0X33   // IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG 0X34 // IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG 0X35   // IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG 0X36 // IIC主机状态寄存器
#define MPU_INTBP_CFG_REG 0X37  // 中断/旁路设置寄存器
#define MPU_INT_EN_REG 0X38     // 中断使能寄存器
#define MPU_INT_STA_REG 0X3A    // 中断状态寄存器

#define MPU_ACCEL_XOUTH_REG 0X3B // 加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG 0X3C // 加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG 0X3D // 加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG 0X3E // 加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG 0X3F // 加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG 0X40 // 加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG 0X41 // 温度值高八位寄存器
#define MPU_TEMP_OUTL_REG 0X42 // 温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG 0X43 // 陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG 0X44 // 陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG 0X45 // 陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG 0X46 // 陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG 0X47 // 陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG 0X48 // 陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG 0X63 // IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG 0X64 // IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG 0X65 // IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG 0X66 // IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG 0X67 // IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG 0X68  // 信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG 0X69 // 运动检测控制寄存器
#define MPU_USER_CTRL_REG 0X6A    // 用户控制寄存器
#define MPU_PWR_MGMT1_REG 0X6B    // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG 0X6C    // 电源管理寄存器2
#define MPU_FIFO_CNTH_REG 0X72    // FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG 0X73    // FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG 0X74      // FIFO读写寄存器
#define MPU_DEVICE_ID_REG 0X75    // 器件ID寄存器
#define MPU_6050_WHO_AMI_I 0x68   // MPU6050寄存器自验值
#define MPU_6500_WHO_AMI_I 0x70   // MPU6500寄存器自验值
#define MPU_9250_WHO_AMI_I				0X71  	//MPU9250的器件ID1
#define MPU_9250_WHO_AMI_I_2     	    0X73  	//MPU9250的器件ID2

//MPU9250内部封装了一个AK8963磁力计,地址和ID如下:
#define AK8963_ADDR				0X0C	//AK8963的I2C地址
#define AK8963_ID				0X48	//AK8963的器件ID

//AK8963的内部寄存器
#define MAG_WIA					0x00	//AK8963的器件ID寄存器地址
#define MAG_CNTL1          	  	0X0A
#define MAG_CNTL2            	0X0B

#define MAG_XOUT_L				0X03
#define MAG_XOUT_H				0X04
#define MAG_YOUT_L				0X05
#define MAG_YOUT_H				0X06
#define MAG_ZOUT_L				0X07
#define MAG_ZOUT_H				0X08

// 如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
// 如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR 0X68

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define gryo_scale   	(4000.0/65536.0*3.14159/180.0)			//弧度秒
#define accel_scale 	(4.0/65536.0)												//G单位
#define mag_scale 	  (9600.0/16384.0/100.0)							//高斯单位

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

// 陀螺仪信息
struct gimbal_info
{
    short gyro_x_raw;
    short gyro_y_raw;
    short gyro_z_raw;

    double gyro_x;
    double gyro_y;
    double gyro_z;

    short accl_x_raw;
    short accl_y_raw;
    short accl_z_raw;

    double accl_x;
    double accl_y;
    double accl_z;

    short magn_x_raw;
    short magn_y_raw;
    short magn_z_raw;

    double magn_x;
    double magn_y;
    double magn_z;

    double temperature;

    double pitch;
    double roll;
    double yaw;
};

enum GYRO_RANGE
{
    BPS_250,
    BPS_500,
    BPS_1000,
    BPS_2000
};

enum ACCE_RANGE
{
    ACC_2G,
    ACC_4G,
    ACC_8G,
    ACC_16G
};

enum MPU_TYPE
{
    MPU_6050,
    MPU_6500,
    MPU_9250,
    ERROR_TYPE
};
extern enum MPU_TYPE mpu_type;
// 因为模块AD0默认接GND,所以转为读写地址后,为0XD1和0XD0(如果接VCC,则为0XD3和0XD2)
// #define MPU_READ    0XD1
// #define MPU_WRITE   0XD0

uint8_t mpu_init(void);                                                      // 初始化MPU6050
uint8_t mpu_write_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf); // IIC连续写
uint8_t mpu_read_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);  // IIC连续读
uint8_t mpu_write_byte(uint8_t addr, uint8_t reg, uint8_t data);                           // IIC写一个字节
uint8_t mpu_read_byte(uint8_t addr, uint8_t reg);                                          // IIC读一个字节

uint8_t mpu_set_gyro_fsr(uint8_t fsr);
uint8_t mpu_set_accel_fsr(uint8_t fsr);
uint8_t mpu_set_lpf(uint16_t lpf);
uint8_t mpu_set_rate(uint16_t rate);
uint8_t mpu_set_fifo(uint8_t sens);

double mpu_get_temperature(void);
uint8_t mpu_get_gyroscope(short *gx, short *gy, short *gz);
uint8_t mpu_get_accelerometer(short *ax, short *ay, short *az);
uint8_t mpu_get_magnetometer(short *mx,short *my,short *mz);

double get_accedata(short raw_data);
double get_gyrodata(short raw_data);
void mpu_compute_mag(short *imx,short *imy,short *imz,double *mx,double *my,double *mz);

void calibrate(void);

void compute_angle(struct gimbal_info *gimbal);

void ahrs_update(struct gimbal_info *gimbal);

#endif
