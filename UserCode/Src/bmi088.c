#include "main.h"
#include "spi.h"
#include "bmi088.h"

volatile uint8_t bmi088_accel_chip_id = 0;
volatile uint8_t bmi088_gyro_chip_id = 0;
volatile uint8_t bmi088_init_ok = 0;
volatile uint8_t bmi088_init_error = 0;

static void accel_select(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

static void accel_deselect(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

static void gyro_select(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

static void gyro_deselect(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = data;
    accel_select();
    HAL_SPI_Transmit(&hspi1, tx, 2, 1000);
    accel_deselect();
}

void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = data;
    gyro_select();
    HAL_SPI_Transmit(&hspi1, tx, 2, 1000);
    gyro_deselect();
}

uint8_t bmi088_accel_read_reg(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t addr = reg | 0x80;
    uint8_t dummy;
    uint8_t ok = 0;
    if (data == NULL || length == 0) return 0;

    accel_select();
    if (HAL_SPI_Transmit(&hspi1, &addr, 1, 100) == HAL_OK &&
        HAL_SPI_Receive(&hspi1, &dummy, 1, 100) == HAL_OK)
    {
        ok = (HAL_SPI_Receive(&hspi1, data, length, 100) == HAL_OK);
    }
    accel_deselect();
    return ok;
}

uint8_t bmi088_gyro_read_reg(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t addr = reg | 0x80;
    uint8_t ok = 0;
    if (data == NULL || length == 0) return 0;

    gyro_select();
    if (HAL_SPI_Transmit(&hspi1, &addr, 1, 100) == HAL_OK)
    {
        ok = (HAL_SPI_Receive(&hspi1, data, length, 100) == HAL_OK);
    }
    gyro_deselect();
    return ok;
}

uint8_t bmi088_init(void)
{
    uint8_t dummy = 0;
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    uint8_t acc_power[2] = {0};
    uint8_t acc_config[2] = {0};
    uint8_t gyro_config[3] = {0};

    bmi088_init_error = 0;
    bmi088_accel_chip_id = 0;
    bmi088_gyro_chip_id = 0;
    accel_deselect();
    gyro_deselect();
    HAL_Delay(10);

    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);
    bmi088_accel_write_single_reg(0x7E, 0xB6);
    HAL_Delay(50);
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    bmi088_gyro_write_single_reg(0x14, 0xB6);
    HAL_Delay(50);

    bmi088_accel_read_reg(0x00, &acc_id, 1);
    bmi088_accel_chip_id = acc_id;
    if (bmi088_accel_chip_id != 0x1E)
    {
        bmi088_init_error = 1;
        return 0;
    }
    bmi088_gyro_read_reg(0x00, &gyro_id, 1);
    bmi088_gyro_chip_id = gyro_id;
    if (bmi088_gyro_chip_id != 0x0F)
    {
        bmi088_init_error = 2;
        return 0;
    }

    bmi088_accel_write_single_reg(0x7C, 0x00);
    HAL_Delay(5);
    bmi088_accel_write_single_reg(0x7D, 0x04);
    HAL_Delay(5);
    bmi088_accel_write_single_reg(0x41, 0x01);
    HAL_Delay(1);
    bmi088_accel_write_single_reg(0x40, 0xAC);
    HAL_Delay(10);

    bmi088_gyro_write_single_reg(0x11, 0x00);
    HAL_Delay(30);
    bmi088_gyro_write_single_reg(0x0F, 0x00);
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x10, 0x02);
    HAL_Delay(10);

    bmi088_accel_read_reg(0x7C, acc_power, 2);
    if (acc_power[0] != 0x00 || acc_power[1] != 0x04)
    {
        bmi088_init_error = 3;
        return 0;
    }
    bmi088_accel_read_reg(0x40, acc_config, 2);
    if (acc_config[0] != 0xAC || (acc_config[1] & 0x03) != 0x01)
    {
        bmi088_init_error = 4;
        return 0;
    }
    bmi088_gyro_read_reg(0x0F, gyro_config, 3);
    if ((gyro_config[0] & 0x07) != 0x00 ||
        (gyro_config[1] & 0x07) != 0x02 ||
        gyro_config[2] != 0x00)
    {
        bmi088_init_error = 5;
        return 0;
    }
    return 1;
}
