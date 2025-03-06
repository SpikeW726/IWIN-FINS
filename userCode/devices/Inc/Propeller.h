//
// Created by admin on 2023/10/30.
//

#ifndef CONTROL_FRAME_MAIN_PWM_H
#define CONTROL_FRAME_MAIN_PWM_H

#include "Usermain.h"
#include "Extension.h"
#include "PID.h"
#include <vector>
#include <unordered_map>

using namespace std;

struct Propeller_Parameter_t{

    int32_t* InID;
    int32_t* OutID;
    int32_t InitPWM;
    int32_t* BasePWM;
    int32_t* FrontPWM;
    int32_t* BackPWM;
    int32_t* LeftPWM;
    int32_t* RightPWM;
    int32_t* ClockwisePWM;
    int32_t* AnticlockwisePWM;
    PID_Regulator_t DepthPID_P;
    PID_Regulator_t PitchPID_P;
    PID_Regulator_t RollPID_P;
    PID_Regulator_t YawPID_P;

    Propeller_Parameter_t(){};
    Propeller_Parameter_t(int32_t* _InID, int32_t* _OutID, int32_t _InitPWM, int32_t PWM[7][4], PID_Regulator_t _DepthPID_P, PID_Regulator_t _PitchPID_P, PID_Regulator_t _RollPID_P, PID_Regulator_t _YawPID_P):
        InID(_InID), OutID(_OutID), InitPWM(_InitPWM), BasePWM(PWM[0]), FrontPWM(PWM[1]), BackPWM(PWM[2]), LeftPWM(PWM[3]), RightPWM(PWM[4]), ClockwisePWM(PWM[5]),
        AnticlockwisePWM(PWM[6]), DepthPID_P(_DepthPID_P), PitchPID_P(_PitchPID_P), RollPID_P(_RollPID_P), YawPID_P(_YawPID_P){};
    Propeller_Parameter_t(const Propeller_Parameter_t& p){
        memcpy(this, &p, sizeof(Propeller_Parameter_t));
    }
};

typedef struct Propeller_Component{
    float Depth;
    float Roll,Pitch,Yaw;
    float Vx,Vy,Vz;
    float Roll_angle, Pitch_angle, Yaw_angle;
} Propeller_Component_t;


class Propeller_I2C: public Device
{
private:
    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data[PROPELLER_NUM]; // PWM值，1500为不转，1500-2000正转，1000-1500反转，离1500越远转速越快
    //int32_t data_receive[5];
    float Target_depth;
    float Target_angle;
    float Target_roll;
    float Target_pitch;
    float Target_yaw;
    float angle_error;
    float Target_speed[3];
    bool flag_PID;
    bool flag_angle;
    bool flag_range;
    PID DepthPID, RollPID, PitchPID;
    PID VxPID, VyPID, YawPID;
    PID RollAnglePID, PitchAnglePID, YawAnglePID;

    void float_ctrl();
    void speed_ctrl();
    void angle_ctrl();

    float Component_Calc(float data);
    void OutputData_single(int id);

    Propeller_Component_t Component;
    Propeller_Parameter_t Parameter;

public:
    void Init();
    void Handle();
    void Receive();
};


/*extern void Anglectrl_servo();

extern void Speedctrl_propeller_init();

extern void Speedctrl_propeller();

extern void Anglectrl_servo_init();
*/

#endif //CONTROL_FRAME_MAIN_PWM_H
