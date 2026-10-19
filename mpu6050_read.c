#include "mpu6050_read.h"
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f

// Nguong va he so cho ZUPT TU DONG (tu phat hien dung yen qua chinh IMU,
// khong can tin hieu ngoai nhu encoder). Dieu chinh neu can nhay/chat hon:
#define ZUPT_STILL_ACC_G      0.08f    // sai lech cho phep quanh 1g (vd 0.08 = ±8%)
#define ZUPT_STILL_GYRO_DPS   1.00167f     // nguong toc do goc (deg/s) coi la dung yen
#define ZUPT_BIAS_ADAPT       0.002f   // he so hoc cham cho ca 3 truc gyro

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t data;

    // 1. Ki?m tra WHO_AM_I (m?c d?nh ph?n h?i 0x68)
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68) {
        // ??nh th?c c?m bi?n: ghi 0 v?o thanh ghi PWR_MGMT_1
        data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        // ??t Sample Rate: 1kHz / (1 + 7) = 125Hz
        data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
			
				// C?u h?nh DLPF (V? d?: 0x03 d? ch?n d?i l?c ~42-44Hz, gi?m nhi?u rung co h?c)
				data = 0x03; 
				HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 1000);

        // C?u h?nh Accelerometer: ?2g (Full Scale Range)
        data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

        // C?u h?nh Gyroscope: ?250 ?/s (Full Scale Range)
        data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

        return 0; // Kh?i t?o th?nh c?ng
    }
    return 1; // Kh?ng t?m th?y ho?c l?i k?t n?i
}

void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint16_t num_samples) {
    uint8_t buffer[6];
    int32_t sum_gx = 0;
    int32_t sum_gy = 0;
    int32_t sum_gz = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        // Doc lien tuc 6 byte tu thanh ghi GYRO_XOUT_H (0x43)
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, buffer, 6, 100);

        sum_gx += (int16_t)(buffer[0] << 8 | buffer[1]);
        sum_gy += (int16_t)(buffer[2] << 8 | buffer[3]);
        sum_gz += (int16_t)(buffer[4] << 8 | buffer[5]);

        HAL_Delay(3); // Khoang thoi gian nghi giua cac lan doc
    }

    // Tinh gia tri offset trung binh
		DataStruct->Gyro_X_Offset = roundf(((float)sum_gx / num_samples) * 1000.0f) / 1000.0f;
		DataStruct->Gyro_Y_Offset = roundf(((float)sum_gy / num_samples) * 1000.0f) / 1000.0f;
		DataStruct->Gyro_Z_Offset = roundf(((float)sum_gz / num_samples) * 1000.0f) / 1000.0f;
}

void MPU6050_Calibrate_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint16_t num_samples) {
    // QUAN TRONG: dat cam bien that NAM NGANG (dung thuoc thuy / mat ban phang)
    // truoc khi goi ham nay, neu khong offset se sai theo huong lech dat cam bien.
    uint8_t buffer[6];
    int32_t sum_ax = 0;
    int32_t sum_ay = 0;
    int32_t sum_az = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, buffer, 6, 100);

        int16_t ax_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
        int16_t ay_raw = (int16_t)(buffer[2] << 8 | buffer[3]);
        int16_t az_raw = (int16_t)(buffer[4] << 8 | buffer[5]);

        sum_ax += ax_raw;
        sum_ay += ay_raw;
        sum_az += az_raw;

        HAL_Delay(3);
    }

    float avg_ax_g = ((float)sum_ax / num_samples) / 16384.0f;
    float avg_ay_g = ((float)sum_ay / num_samples) / 16384.0f;
    float avg_az_g = ((float)sum_az / num_samples) / 16384.0f;

    // Khi nam ngang: Ax va Ay ly tuong phai = 0, Az ly tuong phai = 1 (trong luc 1g)
    DataStruct->Accel_X_Offset = avg_ax_g - 0.0f;
    DataStruct->Accel_Y_Offset = avg_ay_g - 0.0f;
    DataStruct->Accel_Z_Offset = avg_az_g - 1.0f;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];

    // ??c li?n t?c 14 byte t? thanh ghi ACCEL_XOUT_H (0x3B)
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    // Gh?p byte cao v? byte th?p c?a gia t?c k?
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Gh?p byte nhi?t d?
    int16_t raw_temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

    // Gh?p byte con quay h?i chuy?n (Gyro)
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // Chuy?n d?i sang don v? chu?n
    // H? s? chia ?2g l? 16384.0 LSB/g
		DataStruct->Ax = (roundf((DataStruct->Accel_X_RAW / 16384.0f) * 1000.0f) / 1000.0f) - DataStruct->Accel_X_Offset;
		DataStruct->Ay = (roundf((DataStruct->Accel_Y_RAW / 16384.0f) * 1000.0f) / 1000.0f) - DataStruct->Accel_Y_Offset;
		DataStruct->Az = (roundf((DataStruct->Accel_Z_RAW / 16384.0f) * 1000.0f) / 1000.0f) - DataStruct->Accel_Z_Offset;


    // H? s? chia ?250 ?/s l? 131.0 LSB/(?/s)
		DataStruct->Gx = roundf(((DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_Offset) / 131.0f) * 1000.0f) / 1000.0f;
		DataStruct->Gy = roundf(((DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_Offset) / 131.0f) * 1000.0f) / 1000.0f;
		DataStruct->Gz = roundf(((DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_Offset) / 131.0f) * 1000.0f) / 1000.0f;
		
		float den_pitch = sqrtf(DataStruct->Ay * DataStruct->Ay + DataStruct->Az * DataStruct->Az);

    // Roll: cong thuc CHINH XAC chi can 2 truc (Ay, Az) - so hang cos(Pitch)
    // tu triet tieu trong ty so Ay/Az nen KHONG can chuan hoa bang sqrt(Ax^2+Az^2)
    // (cong thuc do se chi la xap xi, sai lech tang theo do lon cua Pitch)
    DataStruct->Roll = atan2f(DataStruct->Ay, DataStruct->Az) * RAD_TO_DEG;

    // Tinh Pitch: atan2f(-Ax, mau_so)
    DataStruct->Pitch = atan2f(-DataStruct->Ax, den_pitch) * RAD_TO_DEG;

    // Lam tron 3 chu so thap phan de xem tren Watch Window
    DataStruct->Roll = roundf(DataStruct->Roll * 1000.0f) / 1000.0f;
    DataStruct->Pitch = roundf(DataStruct->Pitch * 1000.0f) / 1000.0f;
}

/* ============================================================
 *  KALMAN FILTER - uoc luong goc Roll / Pitch
 *  Ket hop:
 *   - Accelerometer: chinh xac ve lau dai, nhung nhieu (rung dong)
 *      lam gia tri nhay/xoc trong ngan han
 *   - Gyroscope: min, it nhieu trong ngan han, nhung tich luy sai so
 *      (troi - drift) neu chi lay tich phan don thuan theo thoi gian
 *  Kalman filter ket hop ca hai de ra goc muot va khong bi troi.
 * ============================================================ */

void Kalman_Init(Kalman_t *Kalman) {
    // Cac tham so nay co the tinh chinh (tune) de toi uu dap ung:
    //  - Q_angle tang  -> tin accel nhieu hon, dap ung nhanh hon nhung ren hon
    //  - Q_angle giam  -> tin gyro nhieu hon, muot hon nhung phan hoi cham hon
    //  - R_measure tang -> bot tin accel (dung khi rung dong nhieu)
    //  - R_measure giam -> tin accel nhieu hon
    Kalman->Q_angle   = 0.001f;
    Kalman->Q_bias    = 0.003f;
    Kalman->R_measure = 0.03f;

    Kalman->angle = 0.0f;
    Kalman->bias  = 0.0f;

    Kalman->P[0][0] = 0.0f;
    Kalman->P[0][1] = 0.0f;
    Kalman->P[1][0] = 0.0f;
    Kalman->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt) {
    // ----- Buoc du doan (Predict) -----
    // Uoc luong goc moi bang tich phan toc do goc (da tru bias) theo dt
    Kalman->rate = newRate - Kalman->bias;
    Kalman->angle += dt * Kalman->rate;

    // Cap nhat ma tran hiep phuong sai sai so du doan
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // ----- Buoc cap nhat (Update / Correction) -----
    // Dung goc tinh tu accel (newAngle) de hieu chinh lai du doan tu gyro
    float S = Kalman->P[0][0] + Kalman->R_measure; // Sai so tong hop
    float K[2]; // He so Kalman gain
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle; // Do lech giua do luong va du doan (innovation)
    Kalman->angle += K[0] * y;
    Kalman->bias  += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

void MPU6050_AutoZUPT(MPU6050_t *DataStruct) {
    if (MPU6050_IsStationary(DataStruct)) {
        // Dieu chinh dan ca 3 truc offset (dang raw LSB, cung don vi voi
        // Gyro_X/Y/Z_Offset da luu) ve gia tri raw hien tai - giong ham
        // MPU6050_Yaw_ZeroRateUpdate nhung tu dong kich hoat + ca 3 truc.
        DataStruct->Gyro_X_Offset += ZUPT_BIAS_ADAPT * ((float)DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_Offset);
        DataStruct->Gyro_Y_Offset += ZUPT_BIAS_ADAPT * ((float)DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_Offset);
        DataStruct->Gyro_Z_Offset += ZUPT_BIAS_ADAPT * ((float)DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_Offset);
    }
}

void MPU6050_Read_All_Kalman(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, float dt) {
    // Buoc 1: doc raw data + tinh Ax,Ay,Az,Gx,Gy,Gz + Roll/Pitch tu accel (ham cu, khong doi)
    MPU6050_Read_All(I2Cx, DataStruct);

    // Buoc 1.5: ZUPT tu dong - tu phat hien dung yen va chinh dan bias gyro
    // 3 truc (giam troi cho lan doc TIEP THEO, khong sua duoc mau vua doc)
    MPU6050_AutoZUPT(DataStruct);

    // Buoc 2: dua qua Kalman filter
    //  - DataStruct->Roll / Pitch (tu accel)  = "newAngle" (do luong)
    //  - DataStruct->Gx / Gy (tu gyro)        = "newRate"  (toc do goc)
    // LUU Y VE TRUC: neu chieu quay cua Kalman-angle nguoc voi accel-angle
    // khi ban nghieng cam bien, doi dau Gx/Gy (vi du dung -DataStruct->Gx)
    // cho phu hop huong lap dat IMU cua ban.
    DataStruct->KalmanAngleRoll  = Kalman_GetAngle(&DataStruct->KalmanRoll,  DataStruct->Roll,  DataStruct->Gx, dt);
    DataStruct->KalmanAnglePitch = Kalman_GetAngle(&DataStruct->KalmanPitch, DataStruct->Pitch, DataStruct->Gy, dt);

    // Buoc 3: Yaw - CHI tich phan gyro Z, khong co gi de "sua" nhu Roll/Pitch
    // (accelerometer khong nhay voi xoay quanh truc thang dung)
    // => gia tri nay se troi dan theo thoi gian, cang chay lau cang lech nhieu.
    DataStruct->Yaw += DataStruct->Gz * dt;

    // Gioi han Yaw trong khoang -180..180 do cho de theo doi (tuy chon)
    if (DataStruct->Yaw > 180.0f)  DataStruct->Yaw -= 360.0f;
    if (DataStruct->Yaw < -180.0f) DataStruct->Yaw += 360.0f;
}

/* ============================================================
 *  ZERO-RATE UPDATE (ZUPT) - giam toc do troi cua Yaw
 *  CHI goi ham nay khi BAN CHAC CHAN robot dang dung yen that su
 *  (vi du: 2 banh encoder deu bao van toc = 0 lien tuc trong ~200-500ms,
 *   hoac robot dang o trang thai dung cho lenh, chua chay dong co)
 *
 *  Neu goi luc robot dang di chuyen/rung dong that, se lam SAI bias
 *  chu khong sua duoc gi - vi luc do Gz != 0 la dung, khong phai loi.
 * ============================================================ */
void MPU6050_Yaw_ZeroRateUpdate(MPU6050_t *DataStruct) {
    // He so cap nhat CHAM (0.001 - 0.02): tranh giat/loi neu bi goi nham
    // luc chua that su dung yen (rung nhe do dong co/rung san). Cang nho
    // cang an toan nhung sua bias cang cham; cang lon sua nhanh nhung
    // de nhay cam voi nhieu tuc thoi.
    const float alpha = 0.01f;

    // Neu dung yen that, Gyro_Z_RAW nen bang dung Gyro_Z_Offset hien tai
    // (tuc Gz sau khi tru offset se = 0). Phan du con lai duoc "hoc" dan
    // vao offset de khu bias troi theo thoi gian/nhiet do.
    DataStruct->Gyro_Z_Offset += alpha * (float)DataStruct->Gyro_Z_RAW - alpha * DataStruct->Gyro_Z_Offset;
}

/* ============================================================
 *  ZUPT TU DONG (Zero-velocity/rate UPdaTe) - khong can encoder
 *  Tu phat hien "dung yen" bang chinh IMU (giong logic 'still' trong
 *  cac code Arduino/ESP32 pho bien): dieu kien dung yen la:
 *   1) Tong do lon vector accel gan dung 1g (khong co gia toc chuyen
 *      dong thuc su, chi co trong luc)
 *   2) Ca 3 truc gyro deu gan 0 (khong dang xoay)
 *  Neu ca 2 dieu kien dung, tu dong dieu chinh dan Gyro_X/Y/Z_Offset -
 *  giam toc do troi cho ca Roll/Pitch (qua Kalman) lan Yaw.
 *
 *  LUU Y: kem tin cay hon dung tin hieu encoder that (banh xe = 0),
 *  vi rung dong co hoac dung nghieng van co the bi nham la "dung yen".
 *  Neu sau nay co encoder, uu tien dung MPU6050_Yaw_ZeroRateUpdate()
 *  voi dieu kien encoder that se chinh xac hon ham nay.
 * ============================================================ */

uint8_t MPU6050_IsStationary(MPU6050_t *DataStruct) {
    float accMag = sqrtf(DataStruct->Ax * DataStruct->Ax +
                          DataStruct->Ay * DataStruct->Ay +
                          DataStruct->Az * DataStruct->Az);

    uint8_t still = (fabsf(accMag - 1.0f) < ZUPT_STILL_ACC_G) &&
                    (fabsf(DataStruct->Gx) < ZUPT_STILL_GYRO_DPS) &&
                    (fabsf(DataStruct->Gy) < ZUPT_STILL_GYRO_DPS) &&
                    (fabsf(DataStruct->Gz) < ZUPT_STILL_GYRO_DPS);

    return still;
}

