#include <encoders.hpp>

uint8_t angle_high, angle_low;
uint16_t raw_angle;
uint16_t increment = 0;

float angle_degrees;
float angle_radians;

HAL_StatusTypeDef stat;

float stepper_encoder_angle(uint8_t id) {
    // Read high byte (register 0x03) first as specified
    stat = HAL_I2C_Mem_Read(&hi2c1, MT6701_ADDR, REG_ANGLE_HIGH,
            I2C_MEMADD_SIZE_8BIT, &angle_high, 1, HAL_MAX_DELAY);
    if(stat != HAL_OK) return -1;

    // Then read low byte (register 0x04)
    stat = HAL_I2C_Mem_Read(&hi2c1, MT6701_ADDR, REG_ANGLE_LOW,
            I2C_MEMADD_SIZE_8BIT, &angle_low, 1, HAL_MAX_DELAY);
    if(stat != HAL_OK) return -1;

    // angle_high contains bits 13:6
    // angle_low contains bits 5:0 (top 6 bits)
    raw_angle = ((uint16_t)angle_high << 6) | (angle_low >> 2);

    // Convert to degrees using the formula:
    // θ = (Σ(angle<i> * 2^i)) / 16384 * (unit)
    angle_degrees = (float)raw_angle * 360.0f / 16384.0f;
    angle_radians = (float)raw_angle * M_PI / 16384.0f;

    switch (id) {
    case DEG:
        return angle_degrees;
        break;
    case RAD:
        return angle_radians;
        break;
    default:
        return raw_angle;
    }
}
