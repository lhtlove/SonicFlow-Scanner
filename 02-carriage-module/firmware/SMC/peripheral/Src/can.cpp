#include "can.hpp"
#include "string.h"

#include "motors.hpp"
#include "encoders.hpp"
#include "stepper_setting.hpp"
#include "smc.hpp"

uint8_t replyData[8];
void stp(uint8_t* data, uint8_t len) {
	static uint8_t cmd;
	static uint8_t send[8];

	cmd = data[0] & 0xF;

	if (cmd == 0x1) stepper.location_steps = 0;
	else if (cmd == 0x2) Stepper_Move(receive_int32(data + 2) * (data[1] ? 1 : -1));
	else if (cmd == 0x3) Stepper_Stop();
	else if (cmd == 0x4) Stepper_SetSpeed(receive_int32(data + 1));
	else if (cmd == 0x5) {
//		CAN_Tx(replyData, 8, DEVICE_MASTER);
	} else if (cmd == 0x6) {
		current = data[1];
		hold_current = data[2];

		currentSetup(driver_current_, hold_current, current);
	} else if (cmd == 0x7) cconfSetup(chopper_config_, data[1]);
}

void dc(uint8_t* data, uint8_t len) {
	static uint8_t cmd;
	static uint8_t send[8];

	cmd = data[0] & 0xF;

	if (cmd == 0x1) DC_Move(data[1], receive_int16(data + 3), data[2]);
	else if (cmd == 0x2) DC_Reset(data[1]);
}

void smc(uint8_t* data, uint8_t len) {
	static uint8_t cmd;
	static uint8_t send[8];

	cmd = data[0] & 0xF;

	if (cmd == 0x1) setup(data[1]);
	else if (cmd == 0x2) module_move(unpackFloat(data + 1));
	else if (cmd == 0x3) dc_home();
	else if (cmd == 0x4) dc_orient(unpackFloat(data + 1));
}



void enc(uint8_t* data, uint8_t len) {
	if (data[1] == 0x01) {
		data[2] ? DCR.countE = 0 : DCL.countE = 0;
	} else if (data[1] == 0x02) {
		replyData[0] = data[0];
		replyData[1] = data[1];
		replyData[2] = data[2];
		replyData[3] = data[2] ? DCR.countE : DCL.countE;
		replyData[4] = data[2] ? DCR.speedE : DCL.speedE;

		CAN_Tx(replyData, 8, DEVICE_MASTER);
	}

	else if (data[1] == 0x10) {
		replyData[0] = data[0];
		replyData[1] = data[1];
		replyData[2] = data[2];

//		packFloat(stepper_encoder_angle(data[1]), data + 1);
		CAN_Tx(replyData, 8, DEVICE_MASTER);
	}
}

CANHandler handlers[10];

const uint8_t handlers_length = sizeof(handlers) / 16;

CAN_HandleTypeDef *hCan;

void CAN_INIT(CAN_HandleTypeDef *hcan, uint16_t device_id, uint16_t mask_id) {
    CAN_FilterTypeDef filter;

    filter.FilterIdHigh 		= device_id << 5;
    filter.FilterMaskIdHigh 	= mask_id << 5;        // 0x7FF -> exact match filter
//    filter.FilterIdLow = device_id << 5;
//    filter.FilterMaskIdLow= mask_id << 5;
    filter.FilterScale 		= CAN_FILTERSCALE_32BIT;
    filter.FilterMode 		= CAN_FILTERMODE_IDMASK;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank 		= 0;
    filter.FilterActivation 	= ENABLE;

    HAL_CAN_ConfigFilter(hcan, &filter);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    hCan = hcan;

    handlers[0].id = 0x1; handlers[0].handler = stp;
    handlers[1].id = 0x2; handlers[1].handler = dc;
    handlers[2].id = 0x3; handlers[2].handler = smc;

//    handlers[2].id = 0x3; handlers[2].handler = enc;
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

    HAL_CAN_AddTxMessage(hCan, &TxHeader, data, &TxMailbox);
}

// Receive callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    if(HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
//    	Stepper_Move(1000, 0);
    	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
//        if(RxHeader.StdId == CAN_DEVICE_ID_2) Process(data, RxHeader.DLC);
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
