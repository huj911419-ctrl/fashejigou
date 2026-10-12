#ifndef TEST_USB_CDC_H
#define TEST_USB_CDC_H
#include <stdint.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
#define USBD_SPEED_FULL 1U
#define CDC_DATA_FS_MAX_PACKET_SIZE 64U
#define CDC_SET_LINE_CODING 0x20U
#define CDC_GET_LINE_CODING 0x21U
#define CDC_SET_CONTROL_LINE_STATE 0x22U
typedef struct
{
    uint32_t TxState;
    uint8_t *TxBuffer;
    uint8_t *RxBuffer;
    uint32_t TxLength;
} USBD_CDC_HandleTypeDef;
typedef struct
{
    void *pClassData;
    uint8_t dev_state;
    uint8_t dev_speed;
} USBD_HandleTypeDef;
typedef struct
{
    uint8_t bmRequest;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USBD_SetupReqTypedef;
typedef struct
{
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *, uint32_t);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *, uint8_t *);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *);
#endif
