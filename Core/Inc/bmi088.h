#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t bmi088_init(void);
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
uint8_t bmi088_accel_read_reg(uint8_t reg, uint8_t *data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data);
uint8_t bmi088_gyro_read_reg(uint8_t reg, uint8_t *data, uint8_t length);
uint8_t bmi088_read_raw(int16_t accel[3], int16_t gyro[3]);

extern volatile uint8_t bmi088_accel_chip_id;
extern volatile uint8_t bmi088_gyro_chip_id;
extern volatile uint8_t bmi088_init_ok;
extern volatile uint8_t bmi088_init_error;

#ifdef __cplusplus
}
#endif

#endif
