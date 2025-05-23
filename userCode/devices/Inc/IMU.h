//
// Created by LEGION on 2021/10/17.
//

#ifndef RM_FRAME_C_IMU_H
#define RM_FRAME_C_IMU_H

#include "BMI088driver.h"
#include "ist8310driver.h"
#include "Usermain.h"
#include "Kalman_Filter.h"
#include "MahonyAHRS.h"
#include "bsp_spi.h"
#include "PID.h"
#include <stdint.h>

// 输出数据个数
#define OUTPUT_NUM                3
 
// 是否融合磁力计数据
#define IMU_USE_MAG               1 //new

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS        0u
#define IMU_SPI_SHFITS       1u
#define IMU_UPDATE_SHFITS    2u
#define IMU_NOTIFY_SHFITS    3u


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1u
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2u

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16

//平均滤波采样值
#define SUM_WIN_SIZE 8
#define MPU6500_TEMP_PWM_MAX 1000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

extern float BMI088_ACCEL_SEN;
extern float BMI088_GYRO_SEN;

//轴偏
/*constexpr float C1 = 1.010006001930344f;
constexpr float C2 = -0.007492008326987f;
constexpr float C3 = -0.009254187692463f;
constexpr float C4 = 0.016494226767657f;
constexpr float C5 = 1.004509527569225f;
constexpr float C6 = 0.022698224839392f;
constexpr float C7 = -0.006601428647036f;
constexpr float C8 = -0.000182664761106f;
constexpr float C9 = 1.016633267989222f;
constexpr float Cx = 0.099323298159297f;
constexpr float Cy = 0.017485774998530f;
constexpr float Cz = -0.166100036392575f;*/
//椭球误差
#define SCALE_X 1.008558673981405
#define SCALE_Y 1.004707252860914
#define SCALE_Z 1.005737679878631
//零偏
constexpr float zero_ax = 0.19184339;
constexpr float zero_ay = 0.0227739215;
constexpr float zero_az = 9.63659286f;

/*枚举类型定义------------------------------------------------------------*/
/*结构体定义--------------------------------------------------------------*/


typedef struct IMU_Raw_Data{
    float accel[3], gyro[3], temp, time, mag[3], accel_offset[3], gyro_offset[3], ax, ay, az;
} IMU_Raw_Data_t;

typedef struct IMU_Pro_Data{
    float accel[3], accel_AHRS[3], gyro[3], temp, time, mag[3], ay;
} IMU_Pro_Data_t;

typedef struct IMU_Attitude{
    float yaw, pitch, rol;
    float yaw_v, pitch_v, rol_v;
    float neg_yaw_v, neg_pitch_v, neg_rol_v;
} IMU_Attitude_t;

typedef struct IMU_state{
	
    volatile uint8_t gyro_update_flag = 0;
    volatile uint8_t accel_update_flag = 0;
    volatile uint8_t accel_temp_update_flag = 0;
    volatile uint8_t mag_update_flag = 0;
    volatile uint8_t imu_start_dma_flag = 0;
	
}IMU_state_t;

typedef struct IMU_Position{
    float _accel[3] = {0};
    float velocity[3] = {0};
    float _velocity[3] = {0};
    float displace[3] = {0};
    float vy;
    float xy;
}IMU_Position_t;

typedef struct IMU_Filter{
    float current;
    float history[SUM_WIN_SIZE];//历史值，其中history[SUM_WIN_SIZE-1]为最近的记录

    int buff_init = 0; //前SUM_WIN_SIZE-1次填充后才能开始输出
    int index = 0; //环形数组可放数据的位置

    int factor[SUM_WIN_SIZE] = {1,2,3,4,5,6,7,8}; //加权系数
    int K=36; //1+2+3+4+5+6+7+8
}IMU_Filter_t;

typedef struct IMU_buffer{
	
    uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
    uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
    uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
    uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};
}IMU_buffer_t;

/*类型定义----------------------------------------------------------------*/

class IMU : public Device
{
    void ErrorHandle();

    //读取数据
    IMU_buffer buf;
    IMU_state state;
    void imu_cmd_spi_dma(void);
    void filter(float *current, IMU_Filter Filter);

    //数据处理
    void velocityVerify();

    //姿态解算
    void calibrate_offset();    //获取加速度零偏值
    void data_adjust(float accel[3],float accel_AHRS[3], float _accel[3], float gyro[3], float _gyro[3]);
    float quat_update[4];
    float quat_out[4];
    void AHRS_update(float quat[4], float time, float accel[3], float mag[3]);
    void AHRS_out(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
    void get_angle(float q[4], float *yaw, float *pitch, float *roll);
    void get_peak();
    void transmit_peak();
    void float_to_str(float data, bool flag);
    bool flag_Test;
    //温度控制
    PID tempPid;
    uint8_t first_temperate;
    uint32_t count_imu;
    void imu_temp_control(float temp);
    void IMU_temp_PWM(float pwm);
    //位移获取
    void record_accel(float _accel[3], float accel[3]);
    void get_velocity(float velocity[3],float _accel[3], float accel[3]);
    void record_velocity(float _velocity[3], float velocity[3]);
    void get_displace(float displace[3], float _velocity[3], float velocity[3]);

public:

    void Init();
    void Handle();
    void Receive();
    void ITHandle(uint16_t GPIO_Pin);
    void ITHandle(void);

	static IMU imu;
    
    IMU_Raw_Data_t rawData;
    IMU_Pro_Data_t proData;
    IMU_Attitude_t attitude;
    IMU_Position_t position;
    IMU_Filter_t axFilter;
    IMU_Filter_t ayFilter;

    float accel_peak[6];
    float velocity_peak[6];
};



//extern float variance;

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
extern void DMA2_Stream0_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_IMU_H
