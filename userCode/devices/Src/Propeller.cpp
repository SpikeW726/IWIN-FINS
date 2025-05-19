//
// Created by admin on 2023/10/30.
//
#include "Propeller.h"
#include "Sensor.h"
#include "IMU.h"
#include "algorithm"
#define pi 3.14f

// extern IMU imu;
// IMU imu = IMU::imu;
// V30
int32_t InID_V30[4] = {1, 2, 6, 5};  // 内部的4个轮，左前-左后-右前-右后
int32_t OutID_V30[4] = {3, 0, 7, 4}; // 外部的4个轮，左前-左后-右前-右后
int32_t InitPWM_V30 = 1640;
int32_t Deadband_V30 = 120;
int32_t PWM_V30[7][4] = {
    {InitPWM_V30, InitPWM_V30, InitPWM_V30, InitPWM_V30}, // Base
	{InitPWM_V30 - 100, InitPWM_V30 - 100, InitPWM_V30 + 100, InitPWM_V30 + 100}, // Front
    {InitPWM_V30 + 100, InitPWM_V30 + 100, InitPWM_V30 - 100, InitPWM_V30 - 100}, // Back
    {InitPWM_V30 + 100, InitPWM_V30 - 100, InitPWM_V30 + 100, InitPWM_V30 - 100}, // Left
    {InitPWM_V30 - 100, InitPWM_V30 + 100, InitPWM_V30 - 100, InitPWM_V30 + 100}, // Right
    {InitPWM_V30 - 80, InitPWM_V30 - 80, InitPWM_V30 - 80, InitPWM_V30 - 80}, // ClockWise
    {InitPWM_V30 + 80, InitPWM_V30 + 80, InitPWM_V30 + 80, InitPWM_V30 + 80}  // AntiClockWise
    //{1450, 1460, 1600, 1600}, // Front
    //{1600, 1600, 1460, 1470}, // Back
    //{1590, 1455, 1600, 1455}, // Left
    //{1465, 1590, 1460, 1600}, // Right
    //{1475, 1475, 1475, 1475}, // ClockWise
    //{1585, 1585, 1585, 1585}  // AntiClockWise
};

// 直接用构造函数
PID_Regulator_t DepthPID_V30(20, 0.005, 100, 100, 100, 100, 200);
PID_Regulator_t PitchPID_V30(5, /*5*/ 0.04, 200, 100, 100, 100, 200);
PID_Regulator_t RollPID_V30(2.5, /*2.5*/ 0.02, 100, 100, 100, 100, 200);
PID_Regulator_t YawPID_V30(4, 0, 100, 30, 30, 30, 200);

Propeller_Parameter_t Parameter_V30(InID_V30, OutID_V30, InitPWM_V30, PWM_V30, DepthPID_V30, PitchPID_V30, RollPID_V30, YawPID_V30);

// V31
int32_t InID_V31[4] = {4, 5, 7, 6};  // 内部的4个轮，左前-左后-右前-右后
int32_t OutID_V31[4] = {1, 0, 2, 3}; // 外部的4个轮，左前-左后-右前-右后
int32_t InitPWM_V31 = 1550;
int32_t Deadband_V31 = 100;
int32_t PWM_V31[7][4] = {
    {1460, 1400, 1460, 1715}, // Base
    {1650, 1450, 1450, 1650}, // Front
    {1450, 1650, 1650, 1450}, // Back
    {1450, 1450, 1450, 1450}, // Left
    {1650, 1650, 1650, 1650}, // Right
    {1650, 1450, 1650, 1450}, // ClockWise
    {1450, 1650, 1450, 1650}  // AntiClockWise
};

PID_Regulator_t DepthPID_V31(20, 0.015, 33, 100, 100, 100, 200);
PID_Regulator_t PitchPID_V31(10, /*5*/ 0.03, 33, 100, 100, 100, 200);
PID_Regulator_t RollPID_V31(2.5, /*2.5*/ 0.03, 33, 100, 100, 100, 200);
// PID_Regulator_t DepthPID_V31(20, 0.005, 10, 100, 100, 100, 200);
// PID_Regulator_t PitchPID_V31(40,/*5*/ 0.02, 300, 100, 100, 100, 200);
// PID_Regulator_t RollPID_V31(20,/*2.5*/ 0.01, 150, 200, 100, 100, 200);

// PID_Regulator_t YawPID_V31(30, 0.02, 1000, 100, 100, 100, 200);
PID_Regulator_t YawPID_V31(10, 0.001, 50, 10, 100, 100, 300);
// PID_Regulator_t YawPID_V33(10, 0.0001, 50, 10, 100, 100, 300);


Propeller_Parameter_t Parameter_V31(InID_V31, OutID_V31, InitPWM_V31, PWM_V31, DepthPID_V31, PitchPID_V31, RollPID_V31, YawPID_V31);

// V32
int32_t Sign_V32[8] = {1, 1, 1, 1, -1, -1, -1, -1};
int32_t InID_V32[4] = {3, 1, 4, 5};  // 内部的4个轮，左前-左后-右前-右后
int32_t OutID_V32[4] = {2, 0, 6, 7}; // 外部的4个轮，左前-左后-右前-右后
int32_t InitPWM_V32 = 1570;
int32_t Deadband_V32 = 100;
int32_t PWM_V32[7][4] = {
    {1460, 1420, 1640, 1680},                                                                                                                                                         // Base
    {InitPWM_V32 - Sign_V32[OutID_V32[0]] * 150, InitPWM_V32 - Sign_V32[OutID_V32[1]] * 150, InitPWM_V32 - Sign_V32[OutID_V32[2]] * 150, InitPWM_V32 - Sign_V32[OutID_V32[3]] * 150}, // Front
    {InitPWM_V32 + Sign_V32[OutID_V32[0]] * 150, InitPWM_V32 + Sign_V32[OutID_V32[1]] * 150, InitPWM_V32 + Sign_V32[OutID_V32[2]] * 150, InitPWM_V32 + Sign_V32[OutID_V32[3]] * 150}, // Back
    {InitPWM_V32 + Sign_V32[OutID_V32[0]] * 150, InitPWM_V32 - Sign_V32[OutID_V32[1]] * 150, InitPWM_V32 - Sign_V32[OutID_V32[2]] * 150, InitPWM_V32 + Sign_V32[OutID_V32[3]] * 150}, // Left
    {InitPWM_V32 - Sign_V32[OutID_V32[0]] * 150, InitPWM_V32 + Sign_V32[OutID_V32[1]] * 150, InitPWM_V32 + Sign_V32[OutID_V32[2]] * 150, InitPWM_V32 - Sign_V32[OutID_V32[3]] * 150}, // Right
    {InitPWM_V32 - Sign_V32[OutID_V32[0]] * 100, InitPWM_V32 - Sign_V32[OutID_V32[1]] * 100, InitPWM_V32 + Sign_V32[OutID_V32[2]] * 100, InitPWM_V32 + Sign_V32[OutID_V32[3]] * 100}, // ClockWise
    {InitPWM_V32 + Sign_V32[OutID_V32[0]] * 100, InitPWM_V32 + Sign_V32[OutID_V32[1]] * 100, InitPWM_V32 - Sign_V32[OutID_V32[2]] * 100, InitPWM_V32 - Sign_V32[OutID_V32[3]] * 100}  // AntiClockWise
};

// PID_Regulator_t DepthPID_V32(20, 0.005, 100, 100, 100, 100, 200);
// PID_Regulator_t PitchPID_V32(10,/*5*/ 0.01, 100, 100, 100, 100, 200);
// PID_Regulator_t RollPID_V32(2.5, /*2.5*/ 0.01, 100, 100, 100, 100, 200);

PID_Regulator_t DepthPID_V32(20, 0.015, 33, 100, 100, 100, 200);
PID_Regulator_t PitchPID_V32(10, /*5*/ 0.03, 33, 100, 100, 100, 200);
PID_Regulator_t RollPID_V32(2.5, /*2.5*/ 0.03, 33, 100, 100, 100, 200);
// PID_Regulator_t DepthPID_V31(20, 0.005, 10, 100, 100, 100, 200);
// PID_Regulator_t PitchPID_V31(40,/*5*/ 0.02, 300, 100, 100, 100, 200);
// PID_Regulator_t RollPID_V31(20,/*2.5*/ 0.01, 150, 200, 100, 100, 200);

PID_Regulator_t YawPID_V32(30, 0.02, 1000, 100, 100, 100, 200);

Propeller_Parameter_t Parameter_V32(InID_V32, OutID_V32, InitPWM_V32, PWM_V32, DepthPID_V32, PitchPID_V32, RollPID_V32, YawPID_V32);

// V33
int32_t Sign_V33[8] = {1, -1, 1, 1, -1, -1, 1, -1}; // 推进器正反桨，正1反-1，序号为推进器序号

//int32_t InID_V33[4] = {1, 2, 6, 4};                 // V33-0内部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
//int32_t OutID_V33[4] = {0, 3, 7, 5};                // V33-0外部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
int32_t InID_V33[4] = {1, 2, 6, 5};                 // V33-1,2内部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
int32_t OutID_V33[4] = {0, 3, 7, 4};                // V33-1,2外部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
// V33-0,1
//int32_t InitPWM_V33 = 1610;
//int32_t Deadband_V33 = 120;
// V33-2
int32_t InitPWM_V33 = 1540;
int32_t Deadband_V33 = 120;

int32_t PWM_V33[7][4] = {
    {InitPWM_V33, InitPWM_V33 - Sign_V33[InID_V33[1]] * 100, InitPWM_V33, InitPWM_V33 - Sign_V33[InID_V33[3]] * 90},                                                                                                                             // Base
    {InitPWM_V33 - Sign_V33[OutID_V33[0]] * 90, InitPWM_V33 - Sign_V33[OutID_V33[1]] * 90, InitPWM_V33 - Sign_V33[OutID_V33[2]] * 90, InitPWM_V33 - Sign_V33[OutID_V33[3]] * 90}, // Front
    {InitPWM_V33 + Sign_V33[OutID_V33[0]] * 90, InitPWM_V33 + Sign_V33[OutID_V33[1]] * 90, InitPWM_V33 + Sign_V33[OutID_V33[2]] * 90, InitPWM_V33 + Sign_V33[OutID_V33[3]] * 90}, // Back
    {InitPWM_V33 + Sign_V33[OutID_V33[0]] * 90, InitPWM_V33 - Sign_V33[OutID_V33[1]] * 90, InitPWM_V33 - Sign_V33[OutID_V33[2]] * 90, InitPWM_V33 + Sign_V33[OutID_V33[3]] * 90}, // Left
    {InitPWM_V33 - Sign_V33[OutID_V33[0]] * 90, InitPWM_V33 + Sign_V33[OutID_V33[1]] * 90, InitPWM_V33 + Sign_V33[OutID_V33[2]] * 90, InitPWM_V33 - Sign_V33[OutID_V33[3]] * 90}, // Right
    {InitPWM_V33 - Sign_V33[OutID_V33[0]] * 70, InitPWM_V33 - Sign_V33[OutID_V33[1]] * 70, InitPWM_V33 + Sign_V33[OutID_V33[2]] * 70, InitPWM_V33 + Sign_V33[OutID_V33[3]] * 70},     // ClockWise
    {InitPWM_V33 + Sign_V33[OutID_V33[0]] * 70, InitPWM_V33 + Sign_V33[OutID_V33[1]] * 70, InitPWM_V33 - Sign_V33[OutID_V33[2]] * 70, InitPWM_V33 - Sign_V33[OutID_V33[3]] * 70}      // AntiClockWise
};

// PID_Regulator_t DepthPID_V32(20, 0.005, 100, 100, 100, 100, 200);
// PID_Regulator_t PitchPID_V32(10,/*5*/ 0.01, 100, 100, 100, 100, 200);
// PID_Regulator_t RollPID_V32(2.5, /*2.5*/ 0.01, 100, 100, 100, 100, 200);

PID_Regulator_t DepthPID_V33(20, 0.015, 33, 100, 50, 50, 200);
PID_Regulator_t PitchPID_V33(10, /*5*/ 0.03, 33, 50, 25, 25, 100);
PID_Regulator_t RollPID_V33(2.5, /*2.5*/ 0.03, 33, 50, 25, 25, 100);
// PID_Regulator_t DepthPID_V31(20, 0.005, 10, 100, 100, 100, 200);
// PID_Regulator_t PitchPID_V31(40,/*5*/ 0.02, 300, 100, 100, 100, 200);
// PID_Regulator_t RollPID_V31(20,/*2.5*/ 0.01, 150, 200, 100, 100, 200);

PID_Regulator_t YawPID_V33(12, 0.06, 300, 10, 100, 50, 300);
// PID_Regulator_t YawPID_V33(10, 0.0001, 50, 10, 100, 100, 300);

Propeller_Parameter_t Parameter_V33(InID_V33, OutID_V33, InitPWM_V33, PWM_V33, DepthPID_V33, PitchPID_V33, RollPID_V33, YawPID_V33);

void Propeller_I2C::Init()
{

    switch (Robot_Version)
    {
    case V30:
        std::memcpy(&Parameter, &Parameter_V30, sizeof(Propeller_Parameter_t));
        break;
    case V31:
        std::memcpy(&Parameter, &Parameter_V31, sizeof(Propeller_Parameter_t));
        break;
    case V32:
        std::memcpy(&Parameter, &Parameter_V32, sizeof(Propeller_Parameter_t));
        break;
    case V33:
        std::memcpy(&Parameter, &Parameter_V33, sizeof(Propeller_Parameter_t));
        break;
    }
    DepthPID.PIDInfo = Parameter.DepthPID_P;
    PitchPID.PIDInfo = Parameter.PitchPID_P;
    RollPID.PIDInfo = Parameter.RollPID_P;
    YawPID.PIDInfo = Parameter.YawPID_P;

    PitchAnglePID.PIDInfo = Parameter.PitchPID_P;
    RollAnglePID.PIDInfo = Parameter.RollPID_P;
    YawAnglePID.PIDInfo = Parameter.YawPID_P;
    
    Target_depth = 30;
    Target_angle = 0;
    Target_roll = 0;
    Target_pitch = 0;
    Target_yaw = 0;
    Target_speed[0] = 0;
    Target_speed[1] = 0;
    flag_PID = false;
    flag_roll = false;
    flag_PWM_output = false;
    TCA_SetChannel(4); // 1路 I2C 扩展为 8路
    HAL_Delay(5);
    PCA_Write(PCA9685_MODE1, 0x0);
    PCA_Setfreq(50); // Hz
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        data[i] = Parameter.InitPWM;
        //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
        PCA_Setpwm(i, 0, floor(data[i] * 4096 / 20000 + 0.5f));
    }
    /*data_receive[0] = 1500;
    data_receive[1] = 1500;
    data_receive[2] = 1500;
    data_receive[3] = 1500;
    data_receive[4] = 300;*/

    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
};

// 根据串口指令设置PWM参数
void Propeller_I2C::Receive()
{

    int32_t data_receive[8];
    unordered_map<char, int32_t *> mp;
    mp['W'] = Parameter.FrontPWM;
    mp['S'] = Parameter.BackPWM;
    mp['A'] = Parameter.LeftPWM;
    mp['D'] = Parameter.RightPWM;
    mp['E'] = Parameter.ClockwisePWM;
    mp['Q'] = Parameter.AnticlockwisePWM;

    if (flag_PID)
    {
        // 更新为前后左右的PWM
        if (mp.count(RxBuffer[0]))
        {
            for (int i = 0; i < 4; ++i)
            {
                data[Parameter.OutID[i]] = mp[RxBuffer[0]][i];
            }
        }

        // 关闭PID控制，更新为初始的PWM
        if (strncmp((char *)RxBuffer, "OFF", 3) == 0)
        {
            flag_PID = false;
            for (int i = 0; i < 8; ++i)
            {
                data[i] = Parameter.InitPWM;
            }
            Target_depth = 30;
            flag_angle = false;
            data[Parameter.OutID[0]] = data[Parameter.OutID[1]] = data[Parameter.OutID[2]] = data[Parameter.OutID[3]] = Parameter.InitPWM;
        }

        // 外圈四个电机更新为给定的PWM，更新深度
        if (strncmp((char *)RxBuffer, "PRO:", 4) == 0)
        {
            char *data_str = (char *)RxBuffer + 4;
            char *token = strtok(data_str, ",");
            int i = 0;
            while (token != NULL && i < 5)
            {
                data_receive[i] = atoi(token);
                token = strtok(NULL, ",");
                i++;
            }
            for (int i = 0; i < 4; ++i)
            {
                data[Parameter.OutID[i]] = data_receive[i];
            }
            Target_depth = data_receive[4] / 10.0;
        }

        // 更新速度与角速度
        if (strncmp((char *)RxBuffer, "VEL:", 4) == 0)
        {
            char *data_str = (char *)RxBuffer + 4;
            char *token = strtok(data_str, ",");
            int i = 0;
            while (token != NULL && i < 3)
            {
                data_receive[i] = atoi(token);
                token = strtok(NULL, ",");
                i++;
            }
            Target_speed[0] = data_receive[0];
            Target_speed[1] = data_receive[1];
            Target_angle = data_receive[2] * 3.14 / 180;
        }

        // 更新Yaw角度
        if (strncmp((char *)RxBuffer, "RPY:", 4) == 0) 
        {
            if (strncmp((char *)RxBuffer, "RPY:ON", 6) == 0){
                flag_angle = true;
            }
            else if (strncmp((char *)RxBuffer, "RPY:OFF", 7) == 0){
                flag_angle = false;
                data[Parameter.OutID[0]] = data[Parameter.OutID[1]] = data[Parameter.OutID[2]] = data[Parameter.OutID[3]] = Parameter.InitPWM;
            }
            else {
                char *data_str = (char *)RxBuffer + 4;
                char *token = strtok(data_str, ",");
                int i = 0;
                while (token != NULL && i < 3)
                {
                    data_receive[i] = atoi(token);
                    token = strtok(NULL, ",");
                    i++;
                }
                Target_roll = data_receive[0] * 3.14 / 180;
                Target_pitch = data_receive[1] * 3.14 / 180;
                Target_yaw = data_receive[2] * 3.14 / 180;
            }
        }

        // 当开启角度控制时先默认输出角度数据，可以通过串口输入改为输出PWM数据
        if(flag_angle){
            if (strncmp((char *)RxBuffer, "PWM:BEG", 7) == 0){
                flag_PWM_output = true;
            }
            else if (strncmp((char *)RxBuffer, "PWM:END", 7) == 0){
                flag_PWM_output = false;
            }
        }

        // 更新深度
        if (strncmp((char *)RxBuffer, "H:", 2) == 0)
        {
            char *data_str = (char *)RxBuffer + 2;
            Target_depth = atoi(data_str) / 10.0;
        }

        // 更新为初始PWM
        if (strncmp((char *)RxBuffer, "Z", 1) == 0)
        {
            for (int i = 0; i < 4; ++i)
            {
                data[Parameter.OutID[i]] = Parameter.InitPWM;
            }
        }

        // 启动原地横滚
        if (strncmp((char *)RxBuffer, "ROL:BEG", 7) == 0)
        {
            flag_roll = true;
        }
        // 停止原地横滚
        if (strncmp((char *)RxBuffer, "ROL:END", 7) == 0)
        {
            flag_roll = false;
        }
    }

    else
    {
        // 开启PID控制，更新为初始PWM
        if (strncmp((char *)RxBuffer, "ON", 2) == 0)
        {
            flag_PID = true;
            for (int i = 0; i < 8; ++i)
            {
                data[i] = Parameter.InitPWM;
            }
            Target_depth = (PressureSensor::pressure_sensor.data_depth - 10.0 > 30.0) ? (PressureSensor::pressure_sensor.data_depth - 10.0) : 30.0;
        }

        // 更新为给定的PWM
        if (strncmp((char *)RxBuffer, "TES:", 4) == 0)
        {
            char *data_str = (char *)RxBuffer + 4;
            char *token = strtok(data_str, ",");
            int i = 0;
            while (token != NULL && i < 8)
            {
                data[i] = atoi(token);
                token = strtok(NULL, ",");
                i++;
            }
        }
    }
}

void Propeller_I2C::Handle()
{

    TCA_SetChannel(4);
    // HAL_Delay(5);
    if (flag_PID)
    {   
        if (!flag_roll) float_ctrl(); // PID控制悬浮状态
        else roll_ctrl(); 
        // speed_ctrl();
    }
    
    if (flag_angle) {
        angle_ctrl(); // 角度控制
    }
    
    
    flag_range = true;
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        // TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号

        if ((data[i] < 1000) || (data[i] > 2000))
            flag_range = false;
    }
    if (flag_range)
    {
        for (int i = 0; i < PROPELLER_NUM; ++i)
        {
            PCA_Setpwm(i, 0, floor(data[i] * 4096 / 20000 + 0.5f));
        }
    }

    // 串口发送推进器的数据
    //  for(int i=0;i<4;++i){
    //     OutputData_single(i);
    //  }
}

void Propeller_I2C::OutputData_single(int id)
{
    uint8_t TxBuffer[5] = {0};
    int data_temp = data[Parameter.InID[id]];
    int data_digit[4];
    for (int j = 0; j < 4; ++j)
    {
        data_digit[j] = data_temp % 10;
        data_temp /= 10;
    }
    TxBuffer[0] = '0' + data_digit[3];
    TxBuffer[1] = '0' + data_digit[2];
    TxBuffer[2] = '0' + data_digit[1];
    TxBuffer[3] = '0' + data_digit[0];
    if (id == 3)
        TxBuffer[4] = '\n';
    else
        TxBuffer[4] = ',';
    HAL_UART_Transmit(&huart6, TxBuffer, sizeof(TxBuffer), 0x00ff);
}

void Propeller_I2C::float_ctrl()
{
    Component.Depth = DepthPID.PIDCalc(Target_depth, PressureSensor::pressure_sensor.data_depth);
    Component.Roll = RollPID.PIDCalc(0.0, PressureSensor::pressure_sensor.data_roll);
    Component.Pitch = PitchPID.PIDCalc(0.0, PressureSensor::pressure_sensor.data_pitch);

    switch (Robot_Version)
    {
    case V30:
        data[Parameter.InID[0]] = Parameter.BasePWM[0] - Component.Depth - Component.Roll - Component.Pitch;
        data[Parameter.InID[1]] = Parameter.BasePWM[1] - Component.Depth - Component.Roll + Component.Pitch;
        data[Parameter.InID[2]] = Parameter.BasePWM[2] - Component.Depth + Component.Roll - Component.Pitch;
        data[Parameter.InID[3]] = Parameter.BasePWM[3] - Component.Depth + Component.Roll + Component.Pitch;
        break;
    case V31:
        data[Parameter.InID[0]] = Parameter.BasePWM[0] - (-Component.Depth - Component.Roll - Component.Pitch);
        data[Parameter.InID[1]] = Parameter.BasePWM[1] - (-Component.Depth - Component.Roll + Component.Pitch);
        data[Parameter.InID[2]] = Parameter.BasePWM[2] - (-Component.Depth + Component.Roll - Component.Pitch);
        data[Parameter.InID[3]] = Parameter.BasePWM[3] - Component.Depth + Component.Roll + Component.Pitch;
        break;
    case V32:
        data[Parameter.InID[0]] = Parameter.BasePWM[0] - (-Component.Depth - Component.Roll - Component.Pitch);
        data[Parameter.InID[1]] = Parameter.BasePWM[1] - (-Component.Depth - Component.Roll + Component.Pitch);
        data[Parameter.InID[2]] = Parameter.BasePWM[2] - Component.Depth + Component.Roll - Component.Pitch;
        data[Parameter.InID[3]] = Parameter.BasePWM[3] - Component.Depth + Component.Roll + Component.Pitch;
        break;
    case V33:
        data[Parameter.InID[0]] = Parameter.BasePWM[0] - Sign_V33[Parameter.InID[0]] * (-Component.Depth - Component.Roll - Component.Pitch);  // -1  与V31相反
        data[Parameter.InID[1]] = Parameter.BasePWM[1] - Sign_V33[Parameter.InID[1]] * (-Component.Depth - Component.Roll + Component.Pitch);  // 1   与V31相同
        data[Parameter.InID[2]] = Parameter.BasePWM[2] - Sign_V33[Parameter.InID[2]] * (-Component.Depth + Component.Roll - Component.Pitch);  // 1   与V31相同
        data[Parameter.InID[3]] = Parameter.BasePWM[3] - Sign_V33[Parameter.InID[3]] * (-Component.Depth + Component.Roll + Component.Pitch);  // -1  与V31相同
        break;
    }
}

void Propeller_I2C::speed_ctrl()
{

    // Component.Vx =  DepthPID.PIDCalc(Target_speed[0], IMU::imu.position._velocity[0]);
    // Component.Vy =  RollPID.PIDCalc(Target_speed[1], IMU::imu.position._velocity[1]);
    Component.Vx = Target_speed[0];
    Component.Vy = Target_speed[1];
    angle_error = Target_angle - IMU::imu.attitude.yaw;
    if (angle_error < -3.14)
        angle_error += 6.28;
    else if (angle_error > 3.14)
        angle_error -= 6.28;

    if (angle_error < 0.02 && angle_error > -0.02)
        YawPID.PIDInfo.errSum = 0;

    Component.Yaw = YawPID.PIDCalc(0, angle_error);

    data[Parameter.OutID[0]] = 1530 + Component_Calc(-Component.Yaw - Component.Vx + Component.Vy);
    data[Parameter.OutID[1]] = 1530 + Component_Calc(-Component.Yaw - Component.Vx - Component.Vy);
    data[Parameter.OutID[2]] = 1530 + Component_Calc(-Component.Yaw + Component.Vx + Component.Vy);
    data[Parameter.OutID[3]] = 1530 + Component_Calc(-Component.Yaw + Component.Vx - Component.Vy);
}

// 控制Yaw
void Propeller_I2C::angle_ctrl()
{
    // used by filter
    float filter_rate = 0.8;
    bool useFilter = true;
    static float last_angle_diff = 0;
    static float new_angle_diff = 0;
    float angle_diff = 0;
    // used by post-process
    float factor = 2.0;
    float deadZone = 1.0; //degree
    int deadBand = 100;
    int outMax = 120;
    //debug
    int i = 0;
    uint8_t TxBuffer[6] = {0};

    // // filter imu.attitude.yaw为弧度制
    // angle_diff = IMU::imu.attitude.yaw - Target_yaw;
    // new_angle_diff = angle_diff;
    // if(fabs(angle_diff) <= 10){
    //     angle_diff = new_angle_diff*(1-filter_rate) + last_angle_diff*filter_rate;
    // }
    // last_angle_diff = new_angle_diff;
    
    if(useFilter){
        new_angle_diff = IMU::imu.attitude.yaw - Target_yaw;
        angle_diff = new_angle_diff*(1-filter_rate) + last_angle_diff*filter_rate;
        last_angle_diff = new_angle_diff;
    }
    else angle_diff = IMU::imu.attitude.yaw - Target_yaw;

    // pre-process
    if(angle_diff > pi) angle_diff = -(2*pi - angle_diff);
    if(angle_diff < -pi) angle_diff = (2*pi + angle_diff);
    // if (angle_diff < deadZone*pi/180 && angle_diff > -deadZone*pi/180) angle_diff = 0;

    // pid
    Component.Yaw_angle = YawAnglePID.PIDCalc(0, angle_diff);

    // post-process
    switch (Robot_Version) 
    {
        //可以根据不同版本调整螺旋桨方向（改变这个正负号 ↓）
    case V31: // 正负号疑似有问题
        data[Parameter.OutID[0]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[1]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[2]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[3]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        break;
    case V33: // 正负号已确定无误
        data[Parameter.OutID[0]] = Parameter.InitPWM + Sign_V33[Parameter.OutID[0]] * Component.Yaw_angle * factor;
        data[Parameter.OutID[1]] = Parameter.InitPWM + Sign_V33[Parameter.OutID[1]] * Component.Yaw_angle * factor;
        data[Parameter.OutID[2]] = Parameter.InitPWM - Sign_V33[Parameter.OutID[2]] * Component.Yaw_angle * factor;
        data[Parameter.OutID[3]] = Parameter.InitPWM - Sign_V33[Parameter.OutID[3]] * Component.Yaw_angle * factor;
        break;
    default:
        data[Parameter.OutID[0]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[1]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[2]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[3]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        break;
    }
    // output limit min and max
    for (int i = 0; i < 4; i++){
        if(data[Parameter.OutID[i]] < Parameter.InitPWM - 10){
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] < Parameter.InitPWM - outMax) ? Parameter.InitPWM - outMax : data[Parameter.OutID[i]];
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] > Parameter.InitPWM - deadBand) ? Parameter.InitPWM - deadBand : data[Parameter.OutID[i]];
        }
        if(data[Parameter.OutID[i]] > Parameter.InitPWM + 10){
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] > Parameter.InitPWM + outMax) ? Parameter.InitPWM + outMax : data[Parameter.OutID[i]];
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] < Parameter.InitPWM + deadBand) ? Parameter.InitPWM + deadBand : data[Parameter.OutID[i]];
        }
    }

    if(flag_PWM_output){
        // Output PWM value
        for (int i = 0; i < 4; i++){
            if(i==3) Output_Data(data[Parameter.OutID[i]], 1, 0);
            else Output_Data(data[Parameter.OutID[i]], 0, 0);
        }
    }
    else{
        // Output yaw data
        float tmp_data[3] = {IMU::imu.attitude.yaw, Target_yaw, angle_diff};
        for (int i = 0; i < 3; i++){
            if(i==2) Output_Data(tmp_data[i], 1, 1);
            else Output_Data(tmp_data[i], 0, 1);
        }
    }
    
    // float tmp_data[3] = {IMU::imu.attitude.yaw, Target_yaw, angle_diff};
    // for (int i = 0; i < 3; i++){
    //     if(i==2) Output_YawData(tmp_data[i], 1);
    //     else Output_YawData(tmp_data[i], 0);
    // }

}

float deg2rad(float degree){
    return degree * pi / 180;
}

float rad2deg(float rad){
    return rad * 180 / pi;
}

// 原地横滚控制 从运行方向看顺时针旋转roll角为正 180和-180为同一位置，会发生突变
void Propeller_I2C::roll_ctrl(){
    Component.Depth = DepthPID.PIDCalc(Target_depth, PressureSensor::pressure_sensor.data_depth);
    Component.Pitch = PitchPID.PIDCalc(0.0, PressureSensor::pressure_sensor.data_pitch);
    
    /*roll角控制*/
    // used by pre-process
    float switich_target_bar = deg2rad(10);
    // used by filter
    float filter_rate = 0.8;
    bool useFilter = false;
    static float last_angle_diff = 0;
    static float new_angle_diff = 0;
    float angle_diff = 0;
    // used by post-process
    float factor = 2.0;
    float deadZone = 1.0; //degree
    int deadBand = 100;
    int outMax = 120;
    //debug
    int i = 0;
    uint8_t TxBuffer[6] = {0};

    
    if(useFilter){
        new_angle_diff = IMU::imu.attitude.rol - Target_roll;
        angle_diff = new_angle_diff*(1-filter_rate) + last_angle_diff*filter_rate;
        last_angle_diff = new_angle_diff;
    }
    else angle_diff = IMU::imu.attitude.rol - Target_roll;

    // pre-process
    if(angle_diff > pi) angle_diff = -(2*pi - angle_diff);
    if(angle_diff < -pi) angle_diff = (2*pi + angle_diff);

    if(fabs(angle_diff) < switich_target_bar){
        Target_roll += deg2rad(60);
        if(Target_roll > pi){
            Target_roll = -(2*pi - Target_roll);
        }
    }


    // pid
    Component.Roll_angle = YawAnglePID.PIDCalc(0, angle_diff);

    // post-process
    switch (Robot_Version) 
    {
        //可以根据不同版本调整螺旋桨方向（改变这个正负号 ↓）
    case V31: // 正负号疑似有问题
        data[Parameter.OutID[0]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[1]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[2]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[3]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        break;
    case V33: // 正负号已确定无误
        data[Parameter.OutID[0]] = Parameter.InitPWM + Sign_V33[Parameter.OutID[0]] * Component.Yaw_angle * factor;
        data[Parameter.OutID[1]] = Parameter.InitPWM + Sign_V33[Parameter.OutID[1]] * Component.Yaw_angle * factor;
        data[Parameter.OutID[2]] = Parameter.InitPWM - Sign_V33[Parameter.OutID[2]] * Component.Yaw_angle * factor;
        data[Parameter.OutID[3]] = Parameter.InitPWM - Sign_V33[Parameter.OutID[3]] * Component.Yaw_angle * factor;
        break;
    default:
        data[Parameter.OutID[0]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[1]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[2]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        data[Parameter.OutID[3]] = Parameter.InitPWM + Component.Yaw_angle * factor;
        break;
    }
    // output limit min and max
    for (int i = 0; i < 4; i++){
        if(data[Parameter.OutID[i]] < Parameter.InitPWM - 10){
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] < Parameter.InitPWM - outMax) ? Parameter.InitPWM - outMax : data[Parameter.OutID[i]];
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] > Parameter.InitPWM - deadBand) ? Parameter.InitPWM - deadBand : data[Parameter.OutID[i]];
        }
        if(data[Parameter.OutID[i]] > Parameter.InitPWM + 10){
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] > Parameter.InitPWM + outMax) ? Parameter.InitPWM + outMax : data[Parameter.OutID[i]];
            data[Parameter.OutID[i]] = (data[Parameter.OutID[i]] < Parameter.InitPWM + deadBand) ? Parameter.InitPWM + deadBand : data[Parameter.OutID[i]];
        }
    }
}


float Propeller_I2C::Component_Calc(float data)
{
    return (data > 0) ? data + 30 : data - 30; // 老版本是+-60
}

void Propeller_I2C::Output_Data(float data, bool flag, bool is_angle){
    // 根据数据是不是角度来选择如何进行第一步处理
    int data_tmp;
    if(is_angle) data_tmp = (int)(data * 1000 * 180 / 3.14f);
    else data_tmp = (int)(data * 1000);
    
    int data_digit[6];
    bool IsPositive = true;
    uint8_t TxBuffer[9];
    TxBuffer[0] = ' ';
    for (int i = 0; i < 6; ++i)
    {
    if (data_tmp < 0) {
            TxBuffer[0] = '-';
        data_tmp = -data_tmp;
    }
        data_digit[i] = data_tmp % 10;
        data_tmp /= 10;
    }
    TxBuffer[1] = '0' + data_digit[5];
    TxBuffer[2] = '0' + data_digit[4];
    TxBuffer[3] = '0' + data_digit[3];
    TxBuffer[4] = '.';
    TxBuffer[5] = '0' + data_digit[2];
    TxBuffer[6] = '0' + data_digit[1];
    TxBuffer[7] = '0' + data_digit[0];
    if (flag) TxBuffer[8] = '\n';
    else TxBuffer[8] = ',';
    HAL_UART_Transmit(&huart6, TxBuffer, sizeof(TxBuffer), 0xffff);
}

// void Propeller_I2C::Output_YawData(float data, bool flag){
//     int data_tmp;
//     data_tmp = (int)(data * 1000 * 180 / 3.14f);
    
//     int data_digit[6];
//     bool IsPositive = true;
//     uint8_t TxBuffer[9];
//     TxBuffer[0] = ' ';
//     for (int i = 0; i < 6; ++i)
//     {
//     if (data_tmp < 0) {
//             TxBuffer[0] = '-';
//         data_tmp = -data_tmp;
//     }
//         data_digit[i] = data_tmp % 10;
//         data_tmp /= 10;
//     }
//     TxBuffer[1] = '0' + data_digit[5];
//     TxBuffer[2] = '0' + data_digit[4];
//     TxBuffer[3] = '0' + data_digit[3];
//     TxBuffer[4] = '.';
//     TxBuffer[5] = '0' + data_digit[2];
//     TxBuffer[6] = '0' + data_digit[1];
//     TxBuffer[7] = '0' + data_digit[0];
//     if (flag) TxBuffer[8] = '\n';
//     else TxBuffer[8] = ',';
//     HAL_UART_Transmit(&huart6, TxBuffer, sizeof(TxBuffer), 0xffff);
// }
// 死区为1450-1550
