//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_DEVICE_H
#define RM_FRAME_C_DEVICE_H
#include "main.h"


#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"


#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//------TODO:修改串口长度上限、各器件数量

#define CONTROL_FREQUENCY 60
#define SERIAL_LENGTH_MAX 100//串口最大长度
#define SENSOR_NUM 4//水压计数量
#define SERVO_NUM 4//舵机数量
#define PROPELLER_NUM 8//推进器数量
#define LED_PERIOD 500//亮灯周期，没什么用

/*
1008.673 1012.036 1007.863 1010.265

#define PRESSURE_0 1022.524
#define PRESSURE_1 1010.162
#define PRESSURE_2 1011.393
#define PRESSURE_3 0958.696 
*/

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
/*枚举类型定义------------------------------------------------------------*/

extern uint8_t RxBuffer[SERIAL_LENGTH_MAX];

//extern void ChassisStart();
//extern void ChassisHandle();
//extern void CtrlHandle();
//extern void ARMHandle();

/*
 * 设备类型枚举
 */
typedef enum {
    MOTOR,
    SERVO,
} DEVICE_TYPE_E;

typedef enum{
    V30,
    V31,
    V32,
    V33
} VERSION_E;

extern VERSION_E Robot_Version;

//extern const int length;

/*extern void I2C_Extension_init();
extern void I2C_Extension();
*/
//extern void Update_level();



extern int Count_ms_dog;
extern int Flag_stop;
extern int Flag_serial;

extern uint32_t init_Flag;

/*结构体定义--------------------------------------------------------------*/



/*类型定义----------------------------------------------------------------*/



class Device {
public:
    DEVICE_TYPE_E deviceType;
    //uint32_t deviceID;

    virtual void Init() = 0;

    virtual void Handle() = 0;

    virtual void Receive() = 0;

    //virtual void ErrorHandle() = 0;

};



/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //RM_FRAME_C_DEVICE_H
