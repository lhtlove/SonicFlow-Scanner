#include "stm32f1xx_hal.h"
#include "math.h"

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#define MT6701_ADDR (0x06 << 1)  // Default slave address 0x06 shifted for HAL
#define REG_ANGLE_HIGH 0x03
#define REG_ANGLE_LOW 0x04

#define DEG 0
#define RAD 1
#define RAW 2

extern I2C_HandleTypeDef hi2c1;
extern uint16_t raw_angle;
extern uint16_t increment;

float stepper_encoder_angle(uint8_t id);

#endif /* INC_ENCODERS_H_ */
