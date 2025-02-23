//
// Created by LEGION on 2021/10/4.
//
//此文件为主函数和中断服务函数所在地
//每个元件的使用包含三个步骤，初始化、获取数据、执行
//初始化和执行在定时器中断函数进行
//获取串口数据在串口中断函数进行
#include "Usermain.h"
#include "IMU.h"
#include "Propeller.h"
#include "Servo.h"
#include "Sensor.h"
#include "Legacy.h"
#include "Watchdog.h"
#include "LED.h"
#include "Buzzer.h"

// 定义设备数量宏
//------TODO:修改设备数量
#define DEVICE_NUM 5

VERSION_E Robot_Version = V33;//根据潜器版本调整，V30，V31，V32，V33

// 静态实例化对象
//static IMU imu;
static Servo servo;
static Servo_I2C servo_i2c;
static Propeller_I2C propeller_i2c;
static Sonar sonar;
static Watchdog watchdog;
static LED led;
static Buzzer buzzer;

// 设备指针数组，用于统一管理和调用设备操作
//------TODO:选用需要的设备
Device *device[DEVICE_NUM]={
        &IMU::imu,                         //imu
        
        &PressureSensor::pressure_sensor,  //水压计
        &propeller_i2c,                    //推进器（扩展板pwm控制）
        //&propeller,                      //推进器（C板pwm控制）
        &servo_i2c,                        //舵机（扩展版pwm控制）
        //&servo,                          //舵机（c板pwm控制）
        //&watchdog,                       //看门狗
        &led,                              //LED灯
        //&buzzer                            //蜂鸣器
};
//------TODO:选择潜器版本


uint32_t init_Flag = 0;
uint8_t RxBuffer[SERIAL_LENGTH_MAX]={0};
bool flag_cnt = false;
extern volatile int32_t time_start,time_end,time_interval,cnt;
volatile int32_t time_start,time_end,time_interval,cnt = 0;
// 定时器中断服务函数, 用于周期性处理设备
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if(init_Flag == 0) return;
    if(htim == &htim1){//60Hz
        for(int i=0;i<DEVICE_NUM;++i){
                device[i]->Handle();
        }
    }
    /*
    if(htim == &htim7){

    }
    if(htim == &htim10){

        //aRGB_led_change(period);
    }
*/
}

volatile uint8_t key_raw_state = 1;
uint32_t key_last_stamp;

// 串口接收中断服务函数, 用于处理设备的串口接收任务
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize)//HAL_UART_RxCpltCallback
{
    if(huart->Instance==USART6) {
        for(int i=0;i<DEVICE_NUM;++i){
            device[i]->Receive();
        }
        for(int i=0;i<SERIAL_LENGTH_MAX;++i){
            RxBuffer[i] = 0;
        }
        HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
    }

}

// 按键中断，暂时无用
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(init_Flag == 0)return;
    if (GPIO_Pin == GPIO_PIN_0){
        key_raw_state = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
        uint32_t time_interval = HAL_GetTick() - key_last_stamp;
        key_last_stamp = HAL_GetTick();
        /*if (time_interval >= 50){
            if (key_raw_state == 1){
                period = 5000;
            }else{
                period = 500;
            }
        }*/
    }
    imu.ITHandle(GPIO_Pin);
}

void DMA2_Stream0_IRQHandler(void){

	imu.ITHandle();

}

// 主函数入口
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();

    MX_TIM5_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_TIM8_Init();

    MX_ADC1_Init();
    MX_ADC3_Init();

    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    //MX_IWDG_Init();
    MX_USB_DEVICE_Init();

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);


    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
    //TODO adc校准？
    //RemoteControl::init();
    bsp_flash_read(&flashData);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    // 初始化所有设备
    for(int i = 0; i < DEVICE_NUM; ++i){
        device[i] -> Init();
    }

    // 设置初始化完成标志
    init_Flag = 1;

    int cnt=0;
    while (1){ }
}