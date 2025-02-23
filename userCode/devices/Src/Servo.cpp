#include "Servo.h"

void Servo::Init(){
    for(int i=0;i<SERVO_NUM;++i){
        data[i]=1500;
    }
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, data[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[1]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo_I2C::Init(){
    TCA_SetChannel(4);
    PCA_Write(PCA9685_MODE1,0x0);
    PCA_Setfreq(50);//Hz
    for(int i = 0; i < SERVO_NUM; ++i){
        data[i] = 1500;
        //------TODO：i+8为舵机在扩展版上的接口编号，根据接线修改，目前为8-11号
        PCA_Setpwm(i + 8,0,floor(data[i] * 4096 / 20000 + 0.5f));
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo::Receive(){
    data_extract(RxBuffer, data, SERVO_NUM);
    //HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo_I2C::Receive(){
    data_extract(RxBuffer, data, SERVO_NUM);
    //HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo::Handle(){
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, data[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[1]);
   
}

void Servo_I2C::Handle(){
    //if(tim_id == 10){
        TCA_SetChannel(4);
	
        bool flag_range = true;
        for (int i = 0; i < PROPELLER_NUM; ++i) {
        //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
            
            if(data[i]<500 || data[i]>2500) flag_range = false;
        }
        if(flag_range){
            for (int i = 0; i < PROPELLER_NUM; ++i) {
                PCA_Setpwm(i, 0, floor(data[i] * 4096 / 20000 + 0.5f));
            }
        }
        for(int i = 0; i < SERVO_NUM; ++i) {
            //------TODO：i+8为舵机在扩展版上的接口编号，根据接线修改，目前为8-11号
            PCA_Setpwm(i+8, 0, floor(data[i] * 4096 / 20000 + 0.5f));
        }
    //}
}

void Servo::data_extract(uint8_t *rx, int32_t *data, int32_t num){
    // 示例：假设 PWM 命令格式是 "PWM:1000,2000,1500,1800,1600,1400\n"
    if (strncmp((char*)rx, "MOT:", 4) == 0) {

        char *data_str = (char*)rx + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < num) {
            data[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
    }
}

void Servo_I2C::data_extract(uint8_t *rx, int32_t *data, int32_t num){
    // 示例：假设 PWM 命令格式是 "MOT:1000,2000,1500,1800,1600,1400\n"
    if (strncmp((char*)rx, "MOT:", 4) == 0) {

        char *data_str = (char*)rx + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < num) {
            data[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
    }
}