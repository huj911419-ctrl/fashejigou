#ifndef TEST_IMU_MAIN_H
#define TEST_IMU_MAIN_H

#include <stdint.h>
typedef struct { unsigned unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { unsigned State; } SPI_HandleTypeDef;
extern GPIO_TypeDef test_gpio;
extern SPI_HandleTypeDef hspi2;
#define HAL_SPI_STATE_READY 1U
#define ACC_CS_GPIO_Port (&test_gpio)
#define GYRO_CS_GPIO_Port (&test_gpio)
#define ACC_CS_Pin 1U
#define GYRO_CS_Pin 8U
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle,
                                         uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t timeout);
#endif
