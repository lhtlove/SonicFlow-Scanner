#include "stm32f1xx_hal.h"
#include "motors.hpp"
#include "encoders.hpp"

#define MASK_SINGLE 0x7FF
#define MASK_PARTICULAR 0x700
#define MASK_ALL 0x000

#define DEVICE_ID 0x001

#ifndef CAN_H_
#define CAN_H_

typedef void (*can_handler_t)(uint8_t* data, uint8_t len);

typedef struct _CANHandler {
    uint32_t id;
    can_handler_t handler;
} CANHandler;

void CAN_INIT(CAN_HandleTypeDef *hcan, uint16_t device_id, uint16_t mask_id);
void CAN_Tx(uint8_t *data, uint8_t len, uint16_t device_id);

#endif
