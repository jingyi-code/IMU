#include "main.h"
#include "bmi088.h"
#include "imu.h"

extern "C" {
volatile uint32_t tim6_irq_count = 0;
}
static IMU imu;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int16_t accel[3], gyro[3];

    if (htim->Instance != TIM6) return;
    tim6_irq_count++;
    if (!bmi088_init_ok) return;
    if (!bmi088_read_raw(accel, gyro)) return;
    imu.update(accel, gyro);
}
