#include "can.h"
#include "sF.h"

void sF(uint8_t* data, uint8_t len) {
	static uint8_t cmd;
	static uint8_t send[8];

	cmd = data[0] & 0xF;

	if (cmd == 0x1) {
		Ultrasonic_SendBurst(receive_int32(data + 1));
	}
	else if (cmd == 0x2) Ultrasonic_StartMeasurement();

	else if (cmd == 0x3) {
        uint32_t tof = __HAL_TIM_GET_COUNTER(&htim2);
        HAL_TIM_Base_Stop(&htim2);

    	send[0] = 0x40; send_int32(tof, send + 1);
    	CAN_Tx(send, 8, DEVICE_MASTER);
	}
}

CANHandler handlers[10];
const uint8_t handlers_length = sizeof(handlers) / 16;

void CAN_INIT(uint16_t device_id, uint16_t mask_id) {
    FDCAN_FilterTypeDef filter;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = device_id;           // No shift needed for FDCAN
    filter.FilterID2 = mask_id;             // 0x7FF for exact match

    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    handlers[0].id = 0x1; handlers[0].handler = sF;
}

// Send data
void CAN_Tx(uint8_t *data, uint8_t len, uint16_t device_id) {
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = device_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;        // Standard ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;    // Normal data frame
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;     // No bit rate switching
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;      // Classic CAN format
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
    	Error_Handler();
    }
}

// Receive callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
//        if(RxHeader.StdId == CAN_DEVICE_ID_2) Process(data, RxHeader.DLC);
    	for (int h = 0; h < handlers_length; h++) {
    		if (handlers[h].id == data[0] >> 4) handlers[h].handler(data, RxHeader.DataLength >> 16);
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
