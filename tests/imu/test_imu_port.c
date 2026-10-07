#include "imu_port.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef test_gpio;
SPI_HandleTypeDef hspi2;
static uint16_t selected;
static unsigned transfers;
static HAL_StatusTypeDef transfer_result = HAL_OK;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == &test_gpio && (pin == ACC_CS_Pin || pin == GYRO_CS_Pin));
    if (state == GPIO_PIN_RESET)
    {
        assert(selected == 0U);
        selected = pin;
    }
    else if (selected == pin)
    {
        selected = 0U;
    }
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle,
                                         uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t timeout)
{
    assert(handle == &hspi2 && selected != 0U && size <= 10U && timeout == 1U);
    ++transfers;
    memcpy(rx, tx, size);
    tx[0] = 0U; /* Caller-owned const TX must not be writable through HAL. */
    return transfer_result;
}

int main(void)
{
    assert(!ImuPortInit());
    hspi2.State = HAL_SPI_STATE_READY;
    assert(ImuPortInit());
    const uint8_t tx[3] = {0x80U, 0U, 0U};
    uint8_t rx[3];
    for (unsigned device = 0U; device < 2U; ++device)
    {
        for (unsigned result = HAL_OK; result <= HAL_TIMEOUT; ++result)
        {
            transfer_result = (HAL_StatusTypeDef)result;
            assert(ImuPortTransfer(NULL, (Bmi088Device_e)device, tx, rx, 3U) == (result == HAL_OK));
            assert(selected == 0U && tx[0] == 0x80U && rx[0] == 0x80U);
        }
    }
    assert(transfers == 8U);
    assert(!ImuPortTransfer(NULL, (Bmi088Device_e)2, tx, rx, 3U));
    assert(!ImuPortTransfer(NULL, BMI088_SENSOR_ACCEL, tx, rx, 11U));
    assert(!ImuPortTransfer(NULL, BMI088_SENSOR_ACCEL, NULL, rx, 3U));
    assert(!ImuPortTransfer(NULL, BMI088_SENSOR_ACCEL, tx, NULL, 3U));
    assert(transfers == 8U && selected == 0U);
    puts("PASS: real SPI port CS ownership/release, bounded HAL timeout, const buffers and error returns");
    return 0;
}
