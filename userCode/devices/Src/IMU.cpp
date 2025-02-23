//
// Created by LEGION on 2021/10/17.
//

#include "IMU.h"
#include "ist8310driver.h"

IMU IMU::imu;

void IMU::Init() {
    if (BMI088_init() != BMI088_NO_ERROR) Error_Handler();
#ifdef IMU_USE_MAG
    if(ist8310_init() != IST8310_NO_ERROR) Error_Handler();
#endif
    BMI088_read(rawData.gyro, rawData.accel, &rawData.temp);
    offset();
    count_imu=0;

    flag_Test = false;
    for(int i = 0; i < 6; ++i){
        accel_peak[i] = 0;
        velocity_peak[i] = 0;
    }

    PID_Regulator_t _tempPID(340, 0.04, 0, 2000, 900, 0, 999);

    tempPid.PIDInfo = _tempPID;

    quat_update[0] = 1.0f, quat_update[1] = quat_update[2] = quat_update[3] = 0;
    quat_out[0] = 1.0f, quat_out[1] = quat_out[2] = quat_out[3] = 0;

    if(HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
    SPI1_DMA_init((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    state.imu_start_dma_flag = 1;
}


void IMU::Handle() {

    //static float axdata[1000];
    //static int axdata_index=0;
    count_imu++;

    record_accel(position._accel, proData.accel);
    record_velocity(position._velocity, position.velocity);
    if(state.gyro_update_flag & (1u << IMU_NOTIFY_SHFITS))
    {
        state.gyro_update_flag &= ~(1u << IMU_NOTIFY_SHFITS);
        BMI088_gyro_read_over(buf.gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, rawData.gyro);
    }

    if(state.accel_update_flag & (1u << IMU_UPDATE_SHFITS))
    {
        state.accel_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
        BMI088_accel_read_over(buf.accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, rawData.accel, &rawData.time);
        rawData.ax = rawData.accel[0];
        rawData.ay = rawData.accel[1];
        rawData.az = rawData.accel[2];
    }

    if(state.accel_temp_update_flag & (1u << IMU_UPDATE_SHFITS))
    {
        state.accel_temp_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
        BMI088_temperature_read_over(buf.accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &rawData.temp);
        imu_temp_control(rawData.temp);
    }
    data_adjust(proData.accel, proData.accel_AHRS, rawData.accel, proData.gyro, rawData.gyro );

    //filter(&proData.accel[0], axFilter);
    //filter(&proData.accel[1], ayFilter);

    proData.ay = proData.accel[1];
    velocityVerify();
    get_velocity(position.velocity, position._accel, proData.accel);
    get_displace(position.displace, position._velocity, position.velocity);
    position.vy = position.velocity[1];
    position.xy = position.displace[1];

    //if(flag_Test)
        get_peak();

    if(count_imu >= 1000){
        for(int j=0; j<4; ++j){
            quat_out[j] = quat_update[j];
        }
        count_imu=0;
    }

    AHRS_update(quat_update, 0.001f, proData.accel_AHRS, rawData.mag);
    AHRS_out(quat_out, 0.001f, proData.gyro, proData.accel_AHRS, rawData.mag);

    get_angle(quat_update, &attitude.yaw, &attitude.pitch, &attitude.rol);

    attitude.yaw_v = proData.gyro[2];
    attitude.pitch_v = proData.gyro[0];
    attitude.rol_v = proData.gyro[1];

    attitude.neg_yaw_v = -attitude.yaw_v;
    attitude.neg_pitch_v = -attitude.pitch_v;
    attitude.neg_rol_v = -attitude.rol_v;
}

void IMU::Receive(){
    if (strncmp((char*)RxBuffer, "BEG", 3) == 0) {
        flag_Test = true;
    }
    if (strncmp((char*)RxBuffer, "END", 3) == 0) {
        flag_Test = false;
        transmit_peak();
        for(int i=0;i<6;++i){
            accel_peak[i] = 0;
            velocity_peak[i] = 0;
        }
    }
}

void IMU::ITHandle(uint16_t GPIO_Pin) {
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        state.accel_update_flag |= 1u << IMU_DR_SHFITS;
        state.accel_temp_update_flag |= 1u << IMU_DR_SHFITS;
        if(state.imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GRYO_Pin)
    {
        state.gyro_update_flag |= 1u << IMU_DR_SHFITS;
        if(state.imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == IST8310_DRDY_Pin)
    {
        state.mag_update_flag |= 1u << IMU_DR_SHFITS;
        if(state.mag_update_flag &= 1u << IMU_DR_SHFITS)
        {
            state.mag_update_flag &= ~(1u<< IMU_DR_SHFITS);
            state.mag_update_flag |= (1u << IMU_SPI_SHFITS);

            ist8310_read_mag(rawData.mag);
        }
    }
}

/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void IMU::imu_cmd_spi_dma(void)
{

    //开启陀螺仪的DMA传输
    if( (state.gyro_update_flag & (1u << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(state.accel_update_flag & (1u << IMU_SPI_SHFITS)) && !(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS)))
    {
        state.gyro_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.gyro_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //开启加速度计的DMA传输
    if((state.accel_update_flag & (1u << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(state.gyro_update_flag & (1u << IMU_SPI_SHFITS)) && !(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS)))
    {
        state.accel_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.accel_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_dma_tx_buf, (uint32_t)buf.accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if((state.accel_temp_update_flag & (1u << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(state.gyro_update_flag & (1u << IMU_SPI_SHFITS)) && !(state.accel_update_flag & (1u<< IMU_SPI_SHFITS)))
    {
        state.accel_temp_update_flag &= ~(1u << IMU_DR_SHFITS);
        state.accel_temp_update_flag |= (1u << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_temp_dma_tx_buf, (uint32_t)buf.accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }

}

void DMA2_Stream0_IRQHandler(void){

	IMU::imu.ITHandle();

}


void IMU::ITHandle() {

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(state.gyro_update_flag & (1u << IMU_SPI_SHFITS))
        {
           // SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_GYRO_IT);
            state.gyro_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.gyro_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

        }

        //accel read over
        //加速度计读取完毕
        if(state.accel_update_flag & (1u << IMU_SPI_SHFITS))
        {
          //  SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_ACCEL_IT);
            state.accel_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.accel_update_flag |= (1u << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(state.accel_temp_update_flag & (1u << IMU_SPI_SHFITS))
        {
           // SystemEstimator::systemEstimator.ITEstimatorTrigger(MPU_TEMP_IT);
            state.accel_temp_update_flag &= ~(1u << IMU_SPI_SHFITS);
            state.accel_temp_update_flag |= (1u << IMU_UPDATE_SHFITS);
					
            imu_temp_control(rawData.temp);
					
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(state.gyro_update_flag & (1u << IMU_UPDATE_SHFITS))
        {
            state.gyro_update_flag &= ~(1u << IMU_UPDATE_SHFITS);
            state.gyro_update_flag |= (1u << IMU_NOTIFY_SHFITS);
        }
    }

}

void IMU::ErrorHandle() {

}

/**
 * @brief 姿态解算
 * @param quat[4]:四元数
 */
void IMU::AHRS_update(float quat[4], float time, float accel[3], float mag[3])
{
    #ifdef IMU_USE_MAG
        MahonyAHRSupdate(quat, 0, 0, 0,
                accel[0], accel[1], accel[2],
                mag[0],mag[1], mag[2]);
    #else
        MahonyAHRSupdate(quat, 0, 0, 0,
                accel[0], accel[1], accel[2],
                0,0,0);
    #endif
}

void IMU::AHRS_out(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
#ifdef IMU_USE_MAG
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
            accel[0], accel[1], accel[2],
            mag[0],mag[1], mag[2]);
#else
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
                     accel[0], accel[1], accel[2],
                     0,0,0);
#endif
}

void IMU::get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}


/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void IMU::imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        tempPid.PIDCalc(60.0f,temp);
        if (tempPid.PIDInfo.output < 0.0f)
        {
            tempPid.PIDInfo.output = 0.0f;
        }
        tempPWM = (uint16_t)tempPid.PIDInfo.output;

        IMU_temp_PWM(tempPWM);

    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 60.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                tempPid.PIDInfo.componentKiMax = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

void IMU::IMU_temp_PWM(float pwm) {
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
/**
 * @brief 位置获取
 * @param _accel 上一次加速度值
 * @param accel  本次加速度值
 */
void IMU::record_accel(float _accel[3], float accel[3]){
    _accel[0] = accel[0];
    _accel[1] = accel[1];
    _accel[2] = accel[2];
}
void IMU::get_velocity(float velocity[3],float _accel[3], float accel[3]){
    velocity[0] += ((_accel[0] + accel[0]) / 2 /CONTROL_FREQUENCY);
    velocity[1] += ((_accel[1] + accel[1]) / 2 /CONTROL_FREQUENCY);
    velocity[2] += ((_accel[2] + accel[2]) / 2 /CONTROL_FREQUENCY);
}
void IMU::record_velocity(float _velocity[3], float velocity[3]){
    _velocity[0] = velocity[0];
    _velocity[1] = velocity[1];
    _velocity[2] = velocity[2];
}
void IMU::get_displace(float displace[3], float _velocity[3], float velocity[3]){
    displace[0] += ((_velocity[0] + velocity[0]) / 2 /CONTROL_FREQUENCY);
    displace[1] += ((_velocity[1] + velocity[1]) / 2 /CONTROL_FREQUENCY);
    displace[2] += ((_velocity[2] + velocity[2]) / 2 /CONTROL_FREQUENCY);
}

/**
 * @brief 数据处理
 * @param accel
 * @param _accel
 */
void IMU::offset(){
    float accel[3], gyro[3], temp;
    for (int i = 0; i<1000; i++){
        BMI088_read(gyro, accel, &temp);
        rawData.accel_offset[0] +=accel [0];
        rawData.accel_offset[1] +=accel [1];
        rawData.accel_offset[2] +=accel [2];
        rawData.gyro_offset[0] +=gyro [0];
        rawData.gyro_offset[1] +=gyro [1];
        rawData.gyro_offset[2] +=gyro [2];
    }
    rawData.accel_offset[0] /= 1000;
    rawData.accel_offset[1] /= 1000;
    rawData.accel_offset[2] /= 1000;
    rawData.gyro_offset[0] /=1000;
    rawData.gyro_offset[1] /=1000;
    rawData.gyro_offset[2] /=1000;
}
void IMU::data_adjust(float accel[3], float accel_AHRS[3], float _accel[3], float gyro[3], float _gyro[3]){
    //accel[0] = C1 * _accel[0] + C2 * _accel[1] + C3 * _accel[2] + Cx - rawData.accel_offset[0];//
    //accel[1] = C4 * _accel[0] + C5 * _accel[1] + C6 * _accel[2] + Cy - rawData.accel_offset[1];//
    accel[0] = _accel[0] - rawData.accel_offset[0];
    accel[1] = _accel[1] - rawData.accel_offset[1];
    accel[2] = _accel[2] - rawData.accel_offset[2];

    accel_AHRS[0] = _accel[0] - rawData.accel_offset[0];
    accel_AHRS[1] = _accel[1] - rawData.accel_offset[1];
    accel_AHRS[2] = _accel[2];

    gyro[0] = _gyro[0] - rawData.gyro_offset[0];
    gyro[1] = _gyro[1] - rawData.gyro_offset[1];
    gyro[2] = _gyro[2] - rawData.gyro_offset[2];

}
void IMU::velocityVerify(){
    /*static int xcount = 0;
    static int ycount = 0;
    static int zcount = 0;
    if (abs(proData.accel[0]) < 0.2) {
        xcount++;
    }
    else {xcount = 0;
    }
    if (xcount >= 50){
        position.velocity[0] = 0;
    }

    if (abs(proData.accel[1]) < 0.2) {
        ycount++;
    }
    else {ycount = 0;
    }
    if (ycount >= 50){
        position.velocity[1] = 0;
    }

    if (abs(proData.accel[2]) < 0.2) {
        zcount++;
    }
    else {zcount = 0;
    }
    if (zcount >= 50){
        position.velocity[2] = 0;
    }*/

    if (abs(proData.accel[0]) < 0.1) proData.accel[0] = 0;
    if (abs(proData.accel[1]) < 0.1) proData.accel[1] = 0;
    if (abs(proData.accel[2]) < 0.1) proData.accel[2] = 0;
}


void IMU::filter(float *current, IMU_Filter_t Filter){

    int i,j;
    float sum = 0;
    Filter.current = *current;
    if(Filter.buff_init == 0)
    {
        Filter.history[Filter.index] = Filter.current;
        Filter.index++;
        if(Filter.index >= (SUM_WIN_SIZE-1))
        {
            Filter.buff_init = 1;//index有效范围是0-5，前面放到5，下一个就可以输出
        }
        return ;//当前无法输出，做个特殊标记区分
    }
    else
    {
        Filter.history[Filter.index] = Filter.current;
        Filter.index++;
        if(Filter.index >= SUM_WIN_SIZE)
        {
            Filter.index = 0;//index有效最大5,下次再从0开始循环覆盖
        }

        j = Filter.index;
        for(i = 0;i < SUM_WIN_SIZE;i++)
        {
            //注意i=0的值并不是最早的值
            sum += Filter.history[j] * Filter.factor[i];//注意防止数据溢出
            j++;
            if(j == SUM_WIN_SIZE)
            {
                j = 0;
            }
        }
        *current = sum / Filter.K;
    }
}

void IMU::float_to_str(float data){
    int data_temp = (int)(data*1000);
    int data_digit[6];
    for(int i=0;i<6;++i){
        data_digit[i] = data_temp%10;
        data_temp/=10;
    }
    uint8_t TxBuffer[8];
    TxBuffer[0] = '0'+data_digit[5];
    TxBuffer[1] = '0'+data_digit[4];
    TxBuffer[2] = '0'+data_digit[3];
    TxBuffer[3] = '.';
    TxBuffer[4] = '0'+data_digit[2];
    TxBuffer[5] = '0'+data_digit[1];
    TxBuffer[6] = '0'+data_digit[0];
    TxBuffer[7] = ' ';
    HAL_UART_Transmit(&huart6, TxBuffer, sizeof(TxBuffer), 0xffff);
}

void IMU::get_peak(){
    if(position._accel[0]>accel_peak[0]) accel_peak[0] = position._accel[0];
    if(-position._accel[0]>accel_peak[1]) accel_peak[1] = -position._accel[0];
    if(position._accel[1]>accel_peak[2]) accel_peak[2] = position._accel[1];
    if(-position._accel[1]>accel_peak[3]) accel_peak[3] = -position._accel[1];
    if(position._accel[2]>accel_peak[4]) accel_peak[4] = position._accel[2];
    if(-position._accel[2]>accel_peak[5]) accel_peak[5] = -position._accel[2];
    if(position._velocity[0]>velocity_peak[0]) velocity_peak[0] = position._accel[0];
    if(-position._velocity[0]>velocity_peak[1]) velocity_peak[1] = -position._accel[0];
    if(position._velocity[1]>velocity_peak[2]) velocity_peak[2] = position._accel[1];
    if(-position._velocity[1]>velocity_peak[3]) velocity_peak[3] = -position._accel[1];
    if(position._velocity[2]>velocity_peak[4]) velocity_peak[4] = position._accel[2];
    if(-position._velocity[2]>velocity_peak[5]) velocity_peak[5] = -position._accel[2];
}

void IMU::transmit_peak(){
    uint8_t TxBuffer1[] ="\nMax Accel:";
    HAL_UART_Transmit(&huart6, TxBuffer1, sizeof(TxBuffer1), 0xffff);
    for(int i=0;i<6;++i) float_to_str(accel_peak[i]);
    uint8_t TxBuffer2[] ="\nMax Velocity:";
    HAL_UART_Transmit(&huart6, TxBuffer2, sizeof(TxBuffer2), 0xffff);
    for(int j=0;j<6;++j) float_to_str(velocity_peak[j]);
}