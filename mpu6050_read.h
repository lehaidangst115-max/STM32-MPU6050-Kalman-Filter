#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"

// ??a ch? I2C c?a MPU6050 (AD0 n?i GND: 0x68 << 1 = 0xD0)
#define MPU6050_ADDR         0xD0

// Thanh ghi MPU6050
#define SMPLRT_DIV_REG       0x19
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define ACCEL_XOUT_H_REG     0x3B
#define TEMP_OUT_H_REG       0x41
#define GYRO_XOUT_H_REG      0x43
#define PWR_MGMT_1_REG       0x6B
#define WHO_AM_I_REG         0x75
#define CONFIG_REG           0x1A

// C?u tr?c luu tr? d? li?u
// Cau truc bo loc Kalman 1 chieu (dung rieng cho Roll va cho Pitch)
typedef struct {
    float Q_angle;    // Nhieu qua trinh cua goc (tin gyro nhieu hay it)
    float Q_bias;     // Nhieu qua trinh cua bias gyro
    float R_measure;  // Nhieu do luong (tin accel nhieu hay it)

    float angle;      // Goc uoc luong dau ra cua bo loc
    float bias;       // Bias (do troi) cua gyro, tu uoc luong
    float rate;        // Toc do goc da tru bias

    float P[2][2];     // Ma tran hiep phuong sai sai so
} Kalman_t;

typedef struct {
    // D? li?u th? (Raw)
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
	
		// Gia tri sai so tinh (Offset) cua Gyro
    float Gyro_X_Offset;
    float Gyro_Y_Offset;
    float Gyro_Z_Offset;

    // Gia tri sai so tinh (Offset) cua Accel, do khi cam bien nam ngang
    float Accel_X_Offset;
    float Accel_Y_Offset;
    float Accel_Z_Offset;

    // D? li?u d? chuy?n d?i sang don v? chu?n
    float Ax;           // Gia t?c (g)
    float Ay;
    float Az;
    float Gx;           // T?c d? g?c (deg/s)
    float Gy;
    float Gz;
	
		float Roll;
    float Pitch;

    // Bo loc Kalman rieng cho Roll va Pitch (moi truc mot bo loc)
    Kalman_t KalmanRoll;
    Kalman_t KalmanPitch;

    // Goc sau khi da qua Kalman filter (ket hop accel + gyro)
    float KalmanAngleRoll;
    float KalmanAnglePitch;

    // Yaw: CHI tich phan gyro Z (khong co accel/mag de sua) => se bi troi (drift)
    // theo thoi gian, cang lau cang lech nhieu neu khong co nguon tham chieu khac
    float Yaw;
} MPU6050_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint16_t num_samples);
void MPU6050_Calibrate_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint16_t num_samples);
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

// --- Kalman filter ---
void Kalman_Init(Kalman_t *Kalman);
float Kalman_GetAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);
void MPU6050_Read_All_Kalman(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, float dt);
void MPU6050_Yaw_ZeroRateUpdate(MPU6050_t *DataStruct);

// --- ZUPT tu dong (khong can encoder) ---
uint8_t MPU6050_IsStationary(MPU6050_t *DataStruct);
void MPU6050_AutoZUPT(MPU6050_t *DataStruct);

#endif /* INC_MPU6050_H_ */