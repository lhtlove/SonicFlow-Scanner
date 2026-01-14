#include <encoders.hpp>

uint8_t angle_high, angle_low;
uint16_t raw_angle;

float angle_degrees;
float angle_radians;

int16_t E1_count;
int16_t E2_count;

uint8_t last_state;

HAL_StatusTypeDef status;

TIM_HandleTypeDef *htim_speed;

I2C_HandleTypeDef *hi2c;

void stepper_encoder_init(I2C_HandleTypeDef *hi2c1) {
	hi2c = hi2c1;
}

float stepper_encoder_angle(uint8_t id) {
    // Read high byte (register 0x03) first as specified
    status = HAL_I2C_Mem_Read(hi2c, MT6701_ADDR, REG_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, &angle_high, 1, HAL_MAX_DELAY);
    if(status != HAL_OK) return -1;

    // Then read low byte (register 0x04)
    status = HAL_I2C_Mem_Read(hi2c, MT6701_ADDR, REG_ANGLE_LOW, I2C_MEMADD_SIZE_8BIT, &angle_low, 1, HAL_MAX_DELAY);
    if(status != HAL_OK) return -1;

    // angle_high contains bits 13:6
    // angle_low contains bits 5:0 (top 6 bits)
    raw_angle = ((uint16_t)angle_high << 6) | (angle_low >> 2);

    // Convert to degrees using the formula:
    // θ = (Σ(angle<i> * 2^i)) / 16384 * (unit)
    angle_degrees = (float)raw_angle * 360.0f / 16384.0f;
    angle_radians = (float)raw_angle * M_PI / 16384.0f;

    return id ? angle_radians : angle_degrees;
}

void dc_encoder_reset() {
	E1_count = 0;
	E2_count = 0;
}

//void dc_encoder_update(uint8_t id, uint8_t a, uint8_t b) {
//    uint8_t current_state = (a << 1) | b;
//    uint8_t last = last_state;
//
//    switch(last << 2 | current_state) {
//        // Valid transitions CW: 00->10->11->01->00
//        case 0b0010: // 00->10
//        case 0b1011: // 10->11
//        case 0b1101: // 11->01
//        case 0b0100: // 01->00
//            id ? E2_count-- : E1_count++;
//            break;
//
//        // Valid transitions CCW: 00->01->11->10->00
//        case 0b0001: // 00->01
//        case 0b0111: // 01->11
//        case 0b1110: // 11->10
//        case 0b1000: // 10->00
//        	id ? E2_count++ : E1_count--;
//            break;
//
//        // Invalid transitions (jitter) are ignored
//    }
//
//    last_state = current_state;
//}

void dc_encoder_update(uint8_t id, uint8_t a, uint8_t b) {
    static uint8_t old_AB = 0;  // Remember previous state

    // Read current state
    uint8_t current_AB = (a << 1) | b;

    // Lookup table for state transitions
    const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    // Get transition state
    old_AB <<= 2;                   // Remember previous state
    old_AB |= current_AB;           // Add current state
    E1_count += enc_states[old_AB & 0x0F];  // Update position based on transition
}

int16_t dc_encoder_count(uint8_t id) {
	return id ? E2_count : E1_count;
}

void dc_encoder_speed(uint8_t id) {
	id ? HAL_TIM_Base_Start_IT(htim_speed) : HAL_TIM_Base_Stop_IT(htim_speed);
}



