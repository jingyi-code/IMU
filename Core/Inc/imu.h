#ifndef IMU_H
#define IMU_H

#include <stdint.h>

class IMU
{
public:
    void update(const int16_t accel[3], const int16_t gyro[3]);

private:
    float accel_sum_[3];
    float gyro_sum_[3];

    void init_quaternion(float ax, float ay, float az);
    void filter_gyro();
    void mahony_update(float ax, float ay, float az,
                       float gx, float gy, float gz);
};

#endif
