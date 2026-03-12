#ifndef PCAP01_HPP
#define PCAP01_HPP

#include "main.h"

// --- Opcodes ---
const uint8_t PCAP01_OPCODE_RESET              = 0x88;
const uint8_t PCAP01_OPCODE_PARTIAL_RESET      = 0x8A;
const uint8_t PCAP01_OPCODE_START_CDC_MEAS     = 0x8C;
const uint8_t PCAP01_OPCODE_START_RDC_MEAS     = 0x8E;

// --- Write regs ---
const uint8_t PCAP01_CMD_WRITE_REG             = 0xC0;
const uint8_t PCAP01_RUNBIT_REG_ADDR           = 0x14;
const uint32_t PCAP01_RUNBIT_REG_DATA          = 0x000001;

// --- Read regs ---
const uint8_t PCAP01_CMD_READ_REG              = 0x40;
const uint8_t PCAP01_C1_DIV_C0_REG_ADDR        = 0x01;
const uint8_t PCAP01_C2_DIV_C0_REG_ADDR        = 0x02;
const uint8_t PCAP01_C3_DIV_C0_REG_ADDR        = 0x03;
const uint8_t PCAP01_STATUS_REG_ADDR           = 0x08;
const uint8_t PCAP01_FRACTIONAL_BITS           = 21;

// --- Reference Cap in pF ---
const float PCAP01_REF_CAP                     = 100.0f;

enum class PCAP01_ERROR : int8_t {
    NO_ERROR             = 0,
    INIT_ERROR           = -1,
    COMM_TEST_FAILED     = -2,
    FIRMWARE_LOAD_FAILED = -3,
    SPI_BUS_ERROR        = -4
};

class PCAP01 {
public:
    PCAP01();
    ~PCAP01() = default;

    PCAP01_ERROR begin(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);

    void sendOpcode(uint8_t opcode);
    void writeRegister(uint8_t reg_addr, uint32_t data);
    float readRegister(uint8_t reg_addr, uint8_t fractional_bits = PCAP01_FRACTIONAL_BITS);
    uint32_t readRawRegister(uint8_t reg_addr);

private:
    SPI_HandleTypeDef* _hspi = nullptr;
    GPIO_TypeDef* _cs_port   = nullptr;
    uint16_t _cs_pin         = 0;

    inline void csLow();
    inline void csHigh();
    uint8_t spiReadWriteByte(uint8_t txdata);
    
    uint8_t sramComm(uint8_t comm_type, uint16_t addr, uint8_t data);
    PCAP01_ERROR commTest();
    PCAP01_ERROR writeStdFirmware();
    void writeStdConfig();
};

#endif