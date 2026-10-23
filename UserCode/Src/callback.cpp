#include "main.h"
#include "bmi088.h"
#include <math.h>

extern "C" {
volatile uint32_t tim6_irq_count = 0;
volatile int16_t accel_raw_x = 0, accel_raw_y = 0, accel_raw_z = 0;
volatile int16_t gyro_raw_x = 0, gyro_raw_y = 0, gyro_raw_z = 0;
volatile float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
volatile float accel_bias_x = 0.0f, accel_bias_y = 0.0f, accel_bias_z = 0.0f;
volatile float gyro_x_dps = 0.0f, gyro_y_dps = 0.0f, gyro_z_dps = 0.0f;
volatile float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
volatile float gyro_corrected_x = 0.0f, gyro_corrected_y = 0.0f, gyro_corrected_z = 0.0f;

volatile float gyro_lpf_tau_ms = 30.0f;
volatile float gyro_filtered_x = 0.0f, gyro_filtered_y = 0.0f, gyro_filtered_z = 0.0f;
volatile uint16_t gyro_calib_count = 0;
volatile uint8_t gyro_calib_done = 0;
volatile uint8_t imu_attitude_ready = 0;

volatile float imu_dt_s = 0.001f;
volatile float q_w = 1.0f, q_x = 0.0f, q_y = 0.0f, q_z = 0.0f;
volatile float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;

volatile float mahony_kp = 2.0f;
volatile float mahony_ki = 0.05f;

volatile float mahony_bias_x = 0.0f, mahony_bias_y = 0.0f, mahony_bias_z = 0.0f;
volatile uint8_t accel_correction_active = 0;
}

static const float kGravity = 9.80665f;
static const float kRadPerDeg = 0.01745329252f;
static const float kDegPerRad = 57.29577951f;
static const uint16_t kCalibSamples = 2000U;

static int16_t combine_bytes(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static void init_quaternion(float ax, float ay, float az)
{
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    const float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
    const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);

    q_w = cr * cp;
    q_x = sr * cp;
    q_y = cr * sp;
    q_z = -sr * sp;
    roll_deg = roll * kDegPerRad;
    pitch_deg = pitch * kDegPerRad;
    yaw_deg = 0.0f;
}

static void filter_gyro(void)
{
    float tau = gyro_lpf_tau_ms;
    if (tau < 0.0f || tau > 5000.0f) tau = 30.0f;
    float alpha = imu_dt_s / (tau * 0.001f + imu_dt_s);
    gyro_filtered_x += alpha * (gyro_corrected_x - gyro_filtered_x);
    gyro_filtered_y += alpha * (gyro_corrected_y - gyro_filtered_y);
    gyro_filtered_z += alpha * (gyro_corrected_z - gyro_filtered_z);
}

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void mahony_update(float ax, float ay, float az,
                          float gx, float gy, float gz)
{
    float wx = gx * kRadPerDeg;
    float wy = gy * kRadPerDeg;
    float wz = gz * kRadPerDeg;
    float kp = mahony_kp;
    float ki = mahony_ki;
    float a2 = ax * ax + ay * ay + az * az;

    accel_correction_active = 0;
    if (kp >= 0.0f && kp <= 20.0f && ki >= 0.0f && ki <= 5.0f &&
        a2 >= (0.85f * kGravity) * (0.85f * kGravity) &&
        a2 <= (1.15f * kGravity) * (1.15f * kGravity))
    {

        float a = sqrtf(a2);
        ax /= a;
        ay /= a;
        az /= a;

        float px = 2.0f * (q_x * q_z - q_w * q_y);
        float py = 2.0f * (q_y * q_z + q_w * q_x);
        float pz = 1.0f - 2.0f * (q_x * q_x + q_y * q_y);

        float ex = ay * pz - az * py;
        float ey = az * px - ax * pz;
        float ez = ax * py - ay * px;
        float max_bias = 5.0f * kRadPerDeg;
        mahony_bias_x = clampf(mahony_bias_x - ki * ex * imu_dt_s, max_bias);
        mahony_bias_y = clampf(mahony_bias_y - ki * ey * imu_dt_s, max_bias);
        mahony_bias_z = clampf(mahony_bias_z - ki * ez * imu_dt_s, max_bias);
        wx += kp * ex;
        wy += kp * ey;
        wz += kp * ez;
        accel_correction_active = 1;
    }

    wx -= mahony_bias_x;
    wy -= mahony_bias_y;
    wz -= mahony_bias_z;

    float w = q_w, x = q_x, y = q_y, z = q_z;
    float half_dt = 0.5f * imu_dt_s;
    float new_w = w - half_dt * (x * wx + y * wy + z * wz);
    float new_x = x + half_dt * (w * wx + y * wz - z * wy);
    float new_y = y + half_dt * (w * wy + z * wx - x * wz);
    float new_z = z + half_dt * (w * wz + x * wy - y * wx);
    float norm = sqrtf(new_w * new_w + new_x * new_x +
                       new_y * new_y + new_z * new_z);
    if (norm < 0.000001f) return;
    q_w = new_w / norm;
    q_x = new_x / norm;
    q_y = new_y / norm;
    q_z = new_z / norm;

    roll_deg = atan2f(2.0f * (q_w * q_x + q_y * q_z),
                      1.0f - 2.0f * (q_x * q_x + q_y * q_y)) * kDegPerRad;
    float pitch_sin = 2.0f * (q_w * q_y - q_z * q_x);
    if (pitch_sin > 1.0f) pitch_sin = 1.0f;
    if (pitch_sin < -1.0f) pitch_sin = -1.0f;
    pitch_deg = asinf(pitch_sin) * kDegPerRad;
    yaw_deg = atan2f(2.0f * (q_w * q_z + q_x * q_y),
                     1.0f - 2.0f * (q_y * q_y + q_z * q_z)) * kDegPerRad;
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static float acc_sum_x = 0.0f, acc_sum_y = 0.0f, acc_sum_z = 0.0f;
    static float gyro_sum_x = 0.0f, gyro_sum_y = 0.0f, gyro_sum_z = 0.0f;
    uint8_t accel_data[6], gyro_data[6];
    float ax, ay, az, gx, gy, gz;

    if (htim->Instance != TIM6) return;
    tim6_irq_count++;
    if (!bmi088_init_ok) return;

    if (!bmi088_accel_read_reg(0x12, accel_data, 6)) return;
    if (!bmi088_gyro_read_reg(0x02, gyro_data, 6)) return;

    accel_raw_x = combine_bytes(accel_data[0], accel_data[1]);
    accel_raw_y = combine_bytes(accel_data[2], accel_data[3]);
    accel_raw_z = combine_bytes(accel_data[4], accel_data[5]);
    gyro_raw_x = combine_bytes(gyro_data[0], gyro_data[1]);
    gyro_raw_y = combine_bytes(gyro_data[2], gyro_data[3]);
    gyro_raw_z = combine_bytes(gyro_data[4], gyro_data[5]);

    const float accel_scale = 6.0f * kGravity / 32768.0f;
    const float gyro_scale = 2000.0f / 32768.0f;
    gyro_x_dps = gyro_raw_x * gyro_scale;
    gyro_y_dps = gyro_raw_y * gyro_scale;
    gyro_z_dps = gyro_raw_z * gyro_scale;

    ax = -accel_raw_x * accel_scale;
    ay = -accel_raw_y * accel_scale;
    az =  accel_raw_z * accel_scale;
    gx = -gyro_x_dps;
    gy = -gyro_y_dps;
    gz =  gyro_z_dps;

    if (!gyro_calib_done)
    {

        acc_sum_x += ax; acc_sum_y += ay; acc_sum_z += az;
        gyro_sum_x += gx; gyro_sum_y += gy; gyro_sum_z += gz;
        if (++gyro_calib_count == kCalibSamples)
        {
            accel_bias_x = acc_sum_x / kCalibSamples;
            accel_bias_y = acc_sum_y / kCalibSamples;
            accel_bias_z = acc_sum_z / kCalibSamples - kGravity;
            gyro_bias_x = gyro_sum_x / kCalibSamples;
            gyro_bias_y = gyro_sum_y / kCalibSamples;
            gyro_bias_z = gyro_sum_z / kCalibSamples;
            gyro_calib_done = 1;
        }
        return;
    }

    accel_x = ax - accel_bias_x;
    accel_y = ay - accel_bias_y;
    accel_z = az - accel_bias_z;
    gyro_corrected_x = gx - gyro_bias_x;
    gyro_corrected_y = gy - gyro_bias_y;
    gyro_corrected_z = gz - gyro_bias_z;

    if (!imu_attitude_ready)
    {
        if (accel_x * accel_x + accel_y * accel_y + accel_z * accel_z < 1.0f) return;
        init_quaternion(accel_x, accel_y, accel_z);
        gyro_filtered_x = gyro_corrected_x;
        gyro_filtered_y = gyro_corrected_y;
        gyro_filtered_z = gyro_corrected_z;
        imu_attitude_ready = 1;
        return;
    }

    mahony_update(accel_x, accel_y, accel_z,
                  gyro_corrected_x, gyro_corrected_y, gyro_corrected_z);
    filter_gyro();
}
