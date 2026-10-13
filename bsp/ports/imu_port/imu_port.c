#include "imu_port.h"
#include "spi.h"
#include "main.h"
#include <string.h>

#define IMU_SPI_TIMEOUT_MS 1U
#define IMU_SPI_MAX_FRAME 10U

bool ImuPortInit(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
    return hspi2.State == HAL_SPI_STATE_READY;
}

bool ImuPortTransfer(void *context, Bmi088Device_e device,
                     const uint8_t *tx, uint8_t *rx, size_t length)
{
    (void)context;
    if (tx == NULL || rx == NULL || length == 0U || length > IMU_SPI_MAX_FRAME ||
        (device != BMI088_SENSOR_ACCEL && device != BMI088_SENSOR_GYRO))
    {
        return false;
    }
    GPIO_TypeDef *port = device == BMI088_SENSOR_ACCEL ? ACC_CS_GPIO_Port : GYRO_CS_GPIO_Port;
    uint16_t pin = device == BMI088_SENSOR_ACCEL ? ACC_CS_Pin : GYRO_CS_Pin;
    /* HAL accepts a mutable TX pointer. Preserve the caller's const contract. */
    uint8_t transmit[IMU_SPI_MAX_FRAME];
    memcpy(transmit, tx, length);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi2, transmit, rx,
                                                      (uint16_t)length, IMU_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return result == HAL_OK;
}
