#include "stm32f1xx_hal.h"

#ifndef STEPPER_SETTING_
#define STEPPER_SETTING_

#define TMC_WRITE_READ_DELAY 100

#define USTEP 0b0100
#define USTEP_RATE 16

extern UART_HandleTypeDef huart3;

const static uint8_t WRITE_REPLY_DATAGRAM_SIZE = 8;
const static uint8_t READ_DATAGRAM_SIZE = 4;
const static uint8_t DATA_SIZE = 4;

union WriteReplyDatagram
{
    struct
    {
        uint64_t sync : 4;
        uint64_t reserved : 4;
        uint64_t serial_address : 8;
        uint64_t register_address : 7;
        uint64_t rw : 1;
        uint64_t data : 32;
        uint64_t crc : 8;
    };
    uint64_t bytes;
};

union ReadDatagram
{
    struct
    {
        uint32_t sync : 4;
        uint32_t reserved : 4;
        uint32_t serial_address : 8;
        uint32_t register_address : 7;
        uint32_t rw : 1;
        uint32_t crc : 8;
    };
    uint32_t bytes;
};

const static uint8_t SYNC = 0b101;
const static uint8_t RW_READ = 0;
const static uint8_t RW_WRITE = 1;
const static uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;

const static uint8_t ADDRESS_GCONF = 0x00;
union GlobalConfig
{
    struct
    {
        uint32_t i_scale_analog : 1;
        uint32_t internal_rsense : 1;
        uint32_t enable_spread_cycle : 1;
        uint32_t shaft : 1;
        uint32_t index_otpw : 1;
        uint32_t index_step : 1;
        uint32_t pdn_disable : 1;
        uint32_t mstep_reg_select : 1;
        uint32_t multistep_filt : 1;
        uint32_t test_mode : 1;
        uint32_t reserved : 22;
    };
    uint32_t bytes;
};

const static uint8_t ADDRESS_CHOPCONF = 0x6C;
union ChopperConfig
{
  struct
  {
    uint32_t toff : 4;
    uint32_t hstart : 3;
    uint32_t hend : 4;
    uint32_t reserved_0 : 4;
    uint32_t tbl : 2;
    uint32_t vsense : 1;
    uint32_t reserved_1 : 6;
    uint32_t mres : 4;
    uint32_t interpolation : 1;
    uint32_t double_edge : 1;
    uint32_t diss2g : 1;
    uint32_t diss2vs : 1;
  };
  uint32_t bytes;
};

const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
union DriverCurrent
{
  struct
  {
    uint32_t ihold : 5;
    uint32_t reserved_0 : 3;
    uint32_t irun : 5;
    uint32_t reserved_1 : 3;
    uint32_t iholddelay : 4;
    uint32_t reserved_2 : 12;
  };
  uint32_t bytes;
};

const static uint8_t ADDRESS_COOLCONF = 0x42;
union CoolConfig
{
  struct
  {
    uint16_t semin : 4;
    uint16_t reserved_0 : 1;
    uint16_t seup : 2;
    uint16_t reserved_1 : 1;
    uint16_t semax : 4;
    uint16_t reserved_2 : 1;
    uint16_t sedn : 2;
    uint16_t seimin : 1;
  };
  uint16_t bytes;
};

extern GlobalConfig global_config_;
extern ChopperConfig chopper_config_;
extern DriverCurrent driver_current_;
extern CoolConfig cool_config_;

extern WriteReplyDatagram reply_buffer;
extern uint8_t uart_rx_buffer[8];
extern uint16_t sg_result;

extern uint8_t current, stall_current, hold_current;

extern void delay_us(uint16_t time);

void gconfSetup(GlobalConfig & gc);
void cconfSetup(ChopperConfig & cc, uint8_t ustep);
void currentSetup(DriverCurrent & dc, uint8_t hold, uint8_t run);
void coolSetup(CoolConfig & cs);

template<typename Datagram>
void sendDatagramUnidirectional(Datagram & datagram, uint8_t datagram_size);

template<typename Datagram>
uint8_t calculateCrc(Datagram & datagram, uint8_t datagram_size);

uint32_t reverseData(uint32_t data);
void write(uint8_t register_address, uint32_t data);
void read(uint8_t register_address);
void received();

#endif
