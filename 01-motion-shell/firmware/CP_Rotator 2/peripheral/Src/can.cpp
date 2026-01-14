#include "string.h"
#include "stdio.h"

#include "can.h"
#include "motors.hpp"
#include "stepper_setting.hpp"
#include "rotator.h"

void prompt(uint8_t* data, uint8_t len) {
//	uint8_t uartData[8];
//	for (uint8_t i = 0; i < 8; i++) uartData[i] = data[i];
//	HAL_UART_Transmit(&huart2, uartData, 8, 10);

	printf("mEncoder: %d\n", receive_int16(data + 2));
}

void motor(uint8_t* data, uint8_t len) {
	static uint8_t cmd;
	static uint8_t send[8];

	cmd = data[0] & 0xF;

	if (cmd == 0) Stepper_Move(receive_int32(data + 1) * (data[5] ? 1 : -1));
	else if (cmd == 0x1) Stepper_Stop();
	else if (cmd == 0x2) Stepper_Enable(data[1]);
	else if (cmd == 0x3) Stepper_Reset();
	else if (cmd == 0x4) Stepper_SetSpeed(receive_int16(data + 1));
	else if (cmd == 0x5) {
		if (data[1]) current = data[1];
		if (data[2]) stall_current = data[2];
		if (data[3]) hold_current = data[3];
		currentSetup(driver_current_, hold_current, current);
	}

	else if (cmd == 0xE) {
		send[0] = cmd;
		send[1] = stepper.is_running;
		CAN_Tx(send, 8, send[7]);
	} else if (cmd == 0xF) {
		send[0] = cmd;
		send[1] = stepper.location_steps < 0 ? 0 : 1;
		send_int32(stepper.location_steps, send + 2);
		CAN_Tx(send, 8, send[7]);
	}
}

void rotator(uint8_t* data, uint8_t len) {
	static uint8_t cmd;
	static uint8_t send[8];

	cmd = data[0] & 0xF;

	if (cmd == 0x0) home(0);
	else if (cmd == 0x1) home(1);
	else if (cmd == 0x2) full_home();
	else if (cmd == 0x3) {
		static float actual_pos;
		actual_pos = rotator_move(unpackFloat(data + 1));

		send[0] = cmd;
		packFloat(actual_pos, send + 1);
		CAN_Tx(send, 8, send[7]);
	} else if (cmd == 0x4) {
		speed_default = receive_int16(data + 1);
		Stepper_SetSpeed(receive_int16(data + 1));
	}
}

CANHandler handlers[10];
const uint8_t handlers_length = sizeof(handlers) / 16;

void CAN_INIT(uint16_t device_id, uint16_t mask_id) {
    CAN_FilterTypeDef filter;

    filter.FilterIdHigh = device_id << 5;
    filter.FilterMaskIdHigh = mask_id << 5;        // 0x7FF -> exact match filter
//    filter.FilterIdLow = device_id << 5;
//    filter.FilterMaskIdLow= mask_id << 5;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank = 0;
    filter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan, &filter);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    handlers[0].id = 0x1; handlers[0].handler = motor;
    handlers[1].id = 0x2; handlers[1].handler = rotator;
    handlers[2].id = 0x3; handlers[2].handler = prompt;
}

// Send data
void CAN_Tx(uint8_t *data, uint8_t len, uint16_t device_id) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = device_id;
    TxHeader.RTR = CAN_RTR_DATA;         // Normal data frame
    TxHeader.IDE = CAN_ID_STD;           // Standard ID
    TxHeader.DLC = len;                  // Data length
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
}

// Receive callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    if(HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
//        if(RxHeader.StdId == CAN_DEVICE_ID_2) Process(data, RxHeader.DLC);

//    	Set_PWM_Duty(100);
    	for (int h = 0; h < handlers_length; h++) {
    		if (handlers[h].id == data[0] >> 4) handlers[h].handler(data, RxHeader.DLC);
    	}
    }
}

void send_int32(int32_t value, uint8_t *data) {
    data[0] = (value >> 24) & 0xFF;  // MSB
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;          // LSB
}

void send_int16(int16_t value, uint8_t *data) {
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;          // LSB
}

void packFloat(float value, uint8_t *buffer) {
    FloatBytes fb;
    fb.f = value;
    memcpy(buffer, fb.bytes, sizeof(float));
}

int32_t receive_int32(uint8_t* data) {
    return ((int32_t)data[0] << 24) |
           ((int32_t)data[1] << 16) |
           ((int32_t)data[2] << 8) |
            (int32_t)data[3];
}

int16_t receive_int16(uint8_t* data) {
    return ((int16_t)data[0] << 8) |
            (int16_t)data[1];
}

float unpackFloat(const uint8_t *buffer) {
    FloatBytes fb;
    memcpy(fb.bytes, buffer, sizeof(float));
    return fb.f;
}
