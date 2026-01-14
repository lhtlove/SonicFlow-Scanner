#include "stm32f1xx_hal.h"

#define MASK_SINGLE 0x7FF
#define MASK_PARTICULAR 0x700
#define MASK_ALL 0x000

#define DEVICE_ID 0x304
#define DEVICE_MASTER 0x700

#ifndef CAN_H_
#define CAN_H_

typedef void (*can_handler_t)(uint8_t* data, uint8_t len);

typedef struct _CANHandler {
    uint32_t id;
    can_handler_t handler;
} CANHandler;

typedef union {
    float f;
    uint8_t bytes[sizeof(float)];
} FloatBytes;

void send_int32(int32_t value, uint8_t *data);
void send_int16(int16_t value, uint8_t *data);
void packFloat(float value, uint8_t *buffer);

int32_t receive_int32(uint8_t* data);
int16_t receive_int16(uint8_t* data);
float unpackFloat(const uint8_t *buffer);

void CAN_INIT(CAN_HandleTypeDef *hcan, uint16_t device_id, uint16_t mask_id);
void CAN_Tx(uint8_t *data, uint8_t len, uint16_t device_id);

#endif
