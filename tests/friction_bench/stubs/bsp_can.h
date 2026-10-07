#ifndef TEST_BSP_CAN_H
#define TEST_BSP_CAN_H
#include <stdint.h>
typedef struct { int bus; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier, IdType, TxFrameType, DataLength; } FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t BusOff; } FDCAN_ProtocolStatusTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct CANInstance
{
    FDCAN_HandleTypeDef *can_handle;
    FDCAN_TxHeaderTypeDef txconf;
    uint32_t tx_id, rx_id;
    uint8_t tx_buff[8], rx_buff[8], rx_len;
    void (*can_module_callback)(struct CANInstance *);
    void *id;
} CANInstance;
typedef struct
{
    FDCAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_DLC_BYTES_8 8
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
CANInstance *CANRegister(CAN_Init_Config_s *config);
HAL_StatusTypeDef CANTransmit(CANInstance *instance, float timeout);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *bus);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *bus, FDCAN_ProtocolStatusTypeDef *status);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *bus, FDCAN_TxHeaderTypeDef *header, uint8_t *data);
#endif
