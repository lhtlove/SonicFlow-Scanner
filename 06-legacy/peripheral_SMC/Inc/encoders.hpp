#include "stm32f1xx_hal.h"
#include "math.h"

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#define MT6701_ADDR (0x06 << 1)  // Default slave address 0x06 shifted for HAL
#define REG_ANGLE_HIGH 0x03
#define REG_ANGLE_LOW 0x04

#define DEG 0
#define RAD 1

extern uint16_t E1_speed;
extern uint16_t E2_speed;

void stepper_encoder_init(I2C_HandleTypeDef *hi2c);
float stepper_encoder_angle(uint8_t id);
void dc_encoder_reset();
void dc_encoder_update(uint8_t id, uint8_t a, uint8_t b);
int16_t dc_encoder_count(uint8_t id);
void dc_encoder_speed(uint8_t id);


#endif /* INC_ENCODERS_H_ */
