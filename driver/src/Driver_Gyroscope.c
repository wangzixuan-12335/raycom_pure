#define __DRIVER_GYROSCOPE_GLOBALS
#include "math.h"
#include "Driver_Filter.h"
#include "config.h"
#include "MadgwickAHRS.h"
#include "Driver_Gyroscope.h"
#include "handle.h"
#include "macro.h"

static float          rollAngle;
static float          pitchAngle;
static float          yawAngle;
static float          xSpeed;
static float          ySpeed;
static float          zSpeed;
static float          xAcc;
static float          yAcc;
static float          zAcc;
static float          xMag;
static float          yMag;
static float          zMag;
extern volatile float beta;

double yawoffset_add=0;

Filter_Type Filter_Yaw = {.count = 0, .driftConfficient = GYROSCOPE_YAW_FILTER_Coefficient};

/**
 * @brief IMU常规数据索引定义
 */
enum{
    IMU_X_INDEX,
    IMU_Y_INDEX,
    IMU_Z_INDEX,
    IMU_DATA_SUM_INDEX
};

/**
 * @brief IMU四元数数据索引定义
 */
enum{
    IMU_QUATERNION_W_INDEX,
    IMU_QUATERNION_X_INDEX,
    IMU_QUATERNION_Y_INDEX,
    IMU_QUATERNION_Z_INDEX,
    IMU_QUATERNION_SUM_INDEX
};


/**
 * @brief ekf filter input data structure
 */
typedef struct{
    Vector3d acc_ekf_input;     // input accelerometer data for ekf
    Vector3d gyro_ekf_input;    // input gyroscope data for ekf
    Vector3d mag_ekf_input;     // input magnetometer data for ekf
}IMU_EKF_INPUT_TPDF;

/**
 * @brief imu bias estimator data structure
 */
typedef struct{
    Vector3d gyro_bias_estimator_input; // input data for gyroscope bias estimator
    Vector3d acc_bias_estimator_input;  // input data for accelerometer bias estimator
    Vector3d gyro_bias_estimator_result; // estimated gyroscope bias result
}IMU_BIAS_ESTIMATOR_TPDF;

typedef struct{
    float acc[IMU_DATA_SUM_INDEX];
    float gyro[IMU_DATA_SUM_INDEX];
    float mag[IMU_DATA_SUM_INDEX];
}IMU_RAW_DATA_TPDF;

typedef struct {
    uint32_t us_time;                                  // IMU微秒级时间戳，单位：us
    uint32_t s_time;                                   // IMU秒级时间戳，单位：s
    uint64_t ns_time;                                  // IMU纳秒级时间戳，单位：ns 连续型数据
}IMU_TIMESTAMP_TPDF;



/**
 * @brief IMU数据结构体定义
 * @note  包含加速度、陀螺仪、磁力计、温度、气压高度、四元数、欧拉角等数据
 * @note  时间戳包括毫秒级和微秒级
 * @author ChenChen
 * @date 2025-10-15
 */
typedef struct{
    float                   rpy[IMU_DATA_SUM_INDEX];              // 欧拉角数据，单位：度，序号0-2分别对应Roll、Pitch、Yaw
    float                   quaterion[IMU_QUATERNION_SUM_INDEX];  // 四元数数据，单位：无量纲，序号0-3分别对应W X、Y、Z
    float                   temperature;                          // IMU温度，单位：℃
    IMU_RAW_DATA_TPDF       raw;                                  // 原始传感器数据结构体
    IMU_RAW_DATA_TPDF       calibrated;                           // 校准后传感器数据结构体
    IMU_EKF_INPUT_TPDF      ekf_input_data;                       // EKF滤波器输入数据结构体
    IMU_BIAS_ESTIMATOR_TPDF bias_estimator_data;                  // 零偏估计器数据结构体
    IMU_TIMESTAMP_TPDF      timestamp;                            // 时间戳结构体
    uint32_t                error_state;                          // IMU错误状态标志位
}IMU_DATA_TPDF;

// IMU数据实例，内部使用
IMU_DATA_TPDF imu_data;


void Gyroscope_Init(GyroscopeData_Type *GyroscopeData, uint16_t startupDelay) {
    GyroscopeData->startupCounter = 0;
    GyroscopeData->modification = 0;
    OrientationEKF_Init(&ekf);
    GyroscopeBiasEstimator_Init(&gyro_bias_estimator);
#ifdef STM32F427_437xx
    MPU6500_Initialize();
    MPU6500_EnableInt();
#endif
#ifdef STM32F40_41xxx
    while (BMI088_init()) {
    }
    ist8310_init();
#endif
    if (startupDelay != 0) {
#ifdef STM32F427_437xx
        beta = 5;
        while (1) {
            // BSP_IWDG_Feed();
            LED_Set_Progress(GyroscopeData->startupCounter / (startupDelay / 7) + 1);
            if (GyroscopeData->startupCounter >= startupDelay) {
                beta = 0.1;
                break;
            }
        }
#endif
#ifdef STM32F40_41xxx
        beta = 5;
        while (1) {
            LED_Set_Colour(GyroscopeData->startupCounter / startupDelay * 255, 0, 0);
            if (GyroscopeData->startupCounter >= startupDelay) {
                beta = 0.5;
                break;
            }
        }
#endif
    }
}

// MPU6500数据读取,成功返回1  失败返回0
int Gyroscope_Update(GyroscopeData_Type *GyroscopeData) {
#ifdef STM32F427_437xx
    static uint8_t mpu_buf[20];

    //尝试读取数据
    if (IIC_ReadData(MPU_IIC_ADDR, MPU6500_ACCEL_XOUT_H, mpu_buf, 14) == 0xff) return 0;

    //成功的话进行赋值
    ImuData.temp = (((int16_t) mpu_buf[6]) << 8) | mpu_buf[7];
    ImuData.az = (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
    ImuData.gz = ((((int16_t) mpu_buf[12]) << 8) | mpu_buf[13]);
    ImuData.ax = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
    ImuData.ay = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
    ImuData.gx = ((((int16_t) mpu_buf[8]) << 8) | mpu_buf[9]);
    ImuData.gy = ((((int16_t) mpu_buf[10]) << 8) | mpu_buf[11]);
#endif
#ifdef STM32F40_41xxx
    static uint8_t buf[8];

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    ImuData.ax = (int16_t)((buf[1] << 8) | buf[0]);
    ImuData.ay = (int16_t)((buf[3] << 8) | buf[2]);
    ImuData.az = (int16_t)((buf[5] << 8) | buf[4]);

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        ImuData.gx = (int16_t)((buf[3] << 8) | buf[2]);
        ImuData.gy = (int16_t)((buf[5] << 8) | buf[4]);
        ImuData.gz = (int16_t)((buf[7] << 8) | buf[6]);
    }

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    if ((int16_t)((buf[0] << 3) | (buf[1] >> 5)) > 1023) {
        ImuData.temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5)) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    } else {
        ImuData.temp = ((int16_t)((buf[0] << 3) | (buf[1] >> 5)) - 2048) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    ist8310_IIC_read_muli_reg(0x03, buf, 6);
    ImuData.mx = (int16_t)((buf[1] << 8) | buf[0]);
    ImuData.my = (int16_t)((buf[3] << 8) | buf[2]);
    ImuData.mz = (int16_t)((buf[5] << 8) | buf[4]);
#endif

    // 读取完成进行解算
    uint32_t lasttime = imu_data.timestamp.us_time;
    Gyroscope_Solve(GyroscopeData);
    VofaData->debug0 = GyroscopeData->roll;
    VofaData->debug1 = GyroscopeData->pitch;
    VofaData->debug2 = GyroscopeData->yaw;
    VofaData->debug3 = imu_data.timestamp.us_time - lasttime;
    // 返回成功值
    return 1;
}

void Gyroscope_Solve(GyroscopeData_Type *GyroscopeData) {
    float ImuData_temp[3][3];
    memset(ImuData_temp, 0, sizeof(ImuData_temp));
    ImuData_temp[0][0] = (float) ((ImuData.gx / GYROSCOPE_LSB) * PI / 180.0);
    ImuData_temp[0][1] = (float) ((ImuData.gy / GYROSCOPE_LSB) * PI / 180.0);
    ImuData_temp[0][2] = (float) ((ImuData.gz / GYROSCOPE_LSB) * PI / 180.0);
    ImuData_temp[1][0] = (float) (ImuData.ax / ACCELERATE_LSB)*9.81f;
    ImuData_temp[1][1] = (float) (ImuData.ay / ACCELERATE_LSB)*9.81f;
    ImuData_temp[1][2] = (float) (ImuData.az / ACCELERATE_LSB)*9.81f;
#ifdef STM32F40_41xxx
    ImuData_temp[2][0] = (float) (ImuData.mx / MAGNETIC_LSB);
    ImuData_temp[2][1] = (float) (ImuData.my / MAGNETIC_LSB);
    ImuData_temp[2][2] = (float) (ImuData.mz / MAGNETIC_LSB);
#endif

    //坐标系转换
    Gyroscope_axis_trans(ImuData_temp);

    imu_data.raw.gyro[IMU_X_INDEX] = xSpeed;
    imu_data.raw.gyro[IMU_Y_INDEX] = ySpeed;
    imu_data.raw.gyro[IMU_Z_INDEX] = zSpeed;
    imu_data.raw.acc[IMU_X_INDEX]  = xAcc/sqrt(xAcc*xAcc + yAcc*yAcc + zAcc*zAcc)*9.81f;
    imu_data.raw.acc[IMU_Y_INDEX]  = yAcc/sqrt(xAcc*xAcc + yAcc*yAcc + zAcc*zAcc)*9.81f;
    imu_data.raw.acc[IMU_Z_INDEX]  = zAcc/sqrt(xAcc*xAcc + yAcc*yAcc + zAcc*zAcc)*9.81f;
    imu_data.raw.mag[IMU_X_INDEX]  = xMag;
    imu_data.raw.mag[IMU_Y_INDEX]  = yMag;
    imu_data.raw.mag[IMU_Z_INDEX]  = zMag;


    
    //获取EKF时间戳
    float lastTime = imu_data.timestamp.us_time;
    imu_data.timestamp.us_time = getSysTimeUs();
    imu_data.timestamp.s_time  = getSysTimeS();
    imu_data.timestamp.ns_time = getSysTimeNs();

    //EKF
    imu_data.calibrated.acc[IMU_X_INDEX] = imu_data.raw.acc[IMU_X_INDEX];
    imu_data.calibrated.acc[IMU_Y_INDEX] = imu_data.raw.acc[IMU_Y_INDEX];
    imu_data.calibrated.acc[IMU_Z_INDEX] = imu_data.raw.acc[IMU_Z_INDEX];
    imu_data.calibrated.gyro[IMU_X_INDEX] = imu_data.raw.gyro[IMU_X_INDEX] - imu_data.bias_estimator_data.gyro_bias_estimator_result.x;
    imu_data.calibrated.gyro[IMU_Y_INDEX] = imu_data.raw.gyro[IMU_Y_INDEX] - imu_data.bias_estimator_data.gyro_bias_estimator_result.y;
    imu_data.calibrated.gyro[IMU_Z_INDEX] = imu_data.raw.gyro[IMU_Z_INDEX] - imu_data.bias_estimator_data.gyro_bias_estimator_result.z;
    imu_data.calibrated.mag[IMU_X_INDEX] = imu_data.raw.mag[IMU_X_INDEX];
    imu_data.calibrated.mag[IMU_Y_INDEX] = imu_data.raw.mag[IMU_Y_INDEX];
    imu_data.calibrated.mag[IMU_Z_INDEX] = imu_data.raw.mag[IMU_Z_INDEX];

    imu_data.ekf_input_data.acc_ekf_input.x = imu_data.calibrated.acc[IMU_X_INDEX];
    imu_data.ekf_input_data.acc_ekf_input.y = imu_data.calibrated.acc[IMU_Y_INDEX];
    imu_data.ekf_input_data.acc_ekf_input.z = imu_data.calibrated.acc[IMU_Z_INDEX];
    imu_data.ekf_input_data.gyro_ekf_input.x = imu_data.calibrated.gyro[IMU_X_INDEX];
    imu_data.ekf_input_data.gyro_ekf_input.y = imu_data.calibrated.gyro[IMU_Y_INDEX];
    imu_data.ekf_input_data.gyro_ekf_input.z = imu_data.calibrated.gyro[IMU_Z_INDEX];
    imu_data.ekf_input_data.mag_ekf_input.x = imu_data.calibrated.mag[IMU_X_INDEX];
    imu_data.ekf_input_data.mag_ekf_input.y = imu_data.calibrated.mag[IMU_Y_INDEX];
    imu_data.ekf_input_data.mag_ekf_input.z = imu_data.calibrated.mag[IMU_Z_INDEX];

    imu_data.bias_estimator_data.gyro_bias_estimator_input.x = imu_data.raw.gyro[IMU_X_INDEX];
    imu_data.bias_estimator_data.gyro_bias_estimator_input.y = imu_data.raw.gyro[IMU_Y_INDEX];
    imu_data.bias_estimator_data.gyro_bias_estimator_input.z = imu_data.raw.gyro[IMU_Z_INDEX];
    imu_data.bias_estimator_data.acc_bias_estimator_input = imu_data.ekf_input_data.acc_ekf_input;

    OrientationEKF_UpdateGyro(&ekf, &imu_data.ekf_input_data.gyro_ekf_input, imu_data.timestamp.ns_time);
    OrientationEKF_UpdateAccel(&ekf, &imu_data.ekf_input_data.acc_ekf_input, imu_data.timestamp.ns_time);
    GyroscopeBiasEstimator_ProcessGyroscope(&gyro_bias_estimator, &imu_data.bias_estimator_data.gyro_bias_estimator_input, imu_data.timestamp.ns_time);
    GyroscopeBiasEstimator_ProcessAccelerometer(&gyro_bias_estimator, &imu_data.bias_estimator_data.acc_bias_estimator_input, imu_data.timestamp.ns_time);
    GyroscopeBiasEstimator_GetGyroBias(&gyro_bias_estimator, &imu_data.bias_estimator_data.gyro_bias_estimator_result);

#ifdef STM32F40_41xxx
    OrientationEKF_UpdateMag(&ekf, &imu_data.ekf_input_data.mag_ekf_input, imu_data.timestamp.ns_time);
#endif

    //获取四元数
    OrientationEKF_GetQuaternion(&ekf, imu_data.quaterion);

    // 四元数->欧拉角
    OrientationEKF_GetRPY(imu_data.quaterion, imu_data.rpy);
    //计算角速度
    Gyroscope_Calculate_angleSpeed(GyroscopeData, imu_data.rpy[IMU_Z_INDEX], imu_data.rpy[IMU_Y_INDEX], imu_data.rpy[IMU_X_INDEX]);
    // 更新滤波器
    Filter_Update(&Filter_Yaw, imu_data.rpy[IMU_Z_INDEX] * 180.0f / PI);

    // 计算连续 Yaw 角
    if (Filter_Yaw.diff > 300) {
        Filter_Yaw.offset -= 360;
    } else if (Filter_Yaw.diff < -300) {
        Filter_Yaw.offset += 360;
    }

    // 应用YAW多圈输出
    GyroscopeData->yaw = Filter_Apply_Limit_Breadth(&Filter_Yaw);
    
//     // 开机时yaw轴转动角度补偿，用于正式启动时的yaw轴零点确定
//     #if GYROSCOPE_START_UP_DELAY_ENABLED
//     if (GyroscopeData->startupCounter == GYROSCOPE_START_UP_DELAY - 1) {
//         GyroscopeData->yawoffset = -GyroscopeData->yaw;
//     }
//     #endif

    GyroscopeData->pitch = imu_data.rpy[IMU_Y_INDEX] * 180.0f / PI;  //放弃修改顺时针为正（因为这会导致后续的云台控制要修正更多的方向，不利于模型推导
    GyroscopeData->roll  = imu_data.rpy[IMU_X_INDEX] * 180.0f / PI;

//     // 开机延迟计数
// #if GYROSCOPE_START_UP_DELAY_ENABLED
//     if (GyroscopeData->startupCounter < GYROSCOPE_START_UP_DELAY) {
//         GyroscopeData->startupCounter += 1;
//     }
// #endif
}

float Gyroscope_Get_Filter_Diff(void) {
    return Filter_Yaw.diff;
}

void Gyroscope_Set_Bias(ImuData_Type *ImuData, int16_t gx_bias, int16_t gy_bias, int16_t gz_bias) {
    ImuData->gx_bias = gx_bias;
    ImuData->gy_bias = gy_bias;
    ImuData->gz_bias = gz_bias;
}

void Gyroscope_axis_trans( float ImuData_temp[][3]) {
    float transMatrix[9] = trans_matrix;
    #ifdef STM32F427_437xx
        float ImuData_trans[2][3];
        for(int i =0; i< 2; i++) {
            for(int j =0; j<3; j++){
                ImuData_trans[i][j] = 0;
                for(int k =0; k<3; k++){
                    ImuData_trans[i][j] += (ImuData_temp[i][k]) * transMatrix[j + k*3];
                }
            }
        }
        xSpeed = ImuData_trans[0][0];
        ySpeed = ImuData_trans[0][1];
        zSpeed = ImuData_trans[0][2];
        xAcc = ImuData_trans[1][0];
        yAcc = ImuData_trans[1][1];
        zAcc = ImuData_trans[1][2];
    #endif

    #ifdef STM32F40_41xxx
        float ImuData_trans[3][3];
        for(int i =0; i< 3; i++) {
            for(int j =0; j<3; j++){
                ImuData_trans[i][j] = 0;
                for(int k =0; k<3; k++){
                    ImuData_trans[i][j] += (ImuData_temp[i][k]) * transMatrix[j + k*3];
                }
            }
        }
        xSpeed = ImuData_trans[0][0];
        ySpeed = ImuData_trans[0][1];
        zSpeed = ImuData_trans[0][2];
        xAcc = ImuData_trans[1][0];
        yAcc = ImuData_trans[1][1];
        zAcc = ImuData_trans[1][2];
        xMag = ImuData_trans[2][0];
        yMag = ImuData_trans[2][1];
        zMag = ImuData_trans[2][2];
    #endif 
}

void Gyroscope_Calculate_angleSpeed(GyroscopeData_Type *gd, float yaw, float pitch, float roll){
    yaw *= PI/180;
    pitch *= PI/180;
    roll *= PI/180;
    float angleSpeed[3] = {xSpeed, ySpeed, zSpeed};
    float angleSpeed_trans[3];
    float transMatrix[9] = {cos(roll)*cos(yaw),cos(roll)*sin(yaw),-sin(roll), \
                                cos(yaw)*sin(pitch)*sin(roll) - cos(pitch)*sin(yaw), cos(pitch)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw), cos(roll)*sin(pitch), \
                                sin(pitch)*sin(yaw) + cos(pitch)*cos(yaw)*sin(roll), cos(pitch)*sin(roll)*sin(yaw) - cos(yaw)*sin(pitch), cos(pitch)*cos(roll)};
    for(int j =0; j<3; j++){
        angleSpeed_trans[j] = 0;
        for(int k =0; k<3; k++){
            angleSpeed_trans[j] += angleSpeed[k] * transMatrix[j + k*3];
        }
    }
    float r = angleSpeed_trans[0]* 180/PI;
    float p = angleSpeed_trans[1]* 180/PI;
    float y = angleSpeed_trans[2]* 180/PI;
    gd->pitchSpeed = p;
    gd->rollSpeed = r;
    gd->yawSpeed = y;
}