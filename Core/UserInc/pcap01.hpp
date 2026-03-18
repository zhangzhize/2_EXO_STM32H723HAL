#ifndef PCAP01_HPP
#define PCAP01_HPP

#include "main.h"

enum class PCAP01_ERROR : int8_t {
    OK                   = 0,
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
    float readRegister(uint8_t reg_addr, uint8_t fractional_bits = KFractionalBits);
    uint32_t readRawRegister(uint8_t reg_addr);

    void initLowPassFilter(float cutoff_freq, float sampling_freq);
    float applyLowPassFilter(uint8_t node_index, float raw_capacitance);
    static constexpr uint8_t MAX_MATRIX_NODES = 4;
    float filter_alpha_ = 1.0f;
    float filtered_data_[MAX_MATRIX_NODES] = {0.0f};
    bool  is_filter_first_run_[MAX_MATRIX_NODES];

    // --- Opcodes ---
    static constexpr uint8_t KOpcodeReset            = 0x88;
    static constexpr uint8_t KOpcodePartialReset     = 0x8A;
    static constexpr uint8_t KOpcodeStartCDCMeas     = 0x8C;
    static constexpr uint8_t KOpcodeStartRDCMeas     = 0x8E;
    // --- Write regs ---
    static constexpr uint8_t KCmdWriteReg            = 0xC0;
    static constexpr uint8_t KRunbitRegAddr          = 0x14;
    static constexpr uint32_t KRunbitRegData         = 0x000001;
    // --- Read regs ---
    static constexpr uint8_t KCmdReadReg            = 0x40;
    static constexpr uint8_t KC1DivC0RegAddr        = 0x01;
    static constexpr uint8_t KC2DivC0RegAddr        = 0x02;
    static constexpr uint8_t KC3DivC0RegAddr        = 0x03;
    static constexpr uint8_t KStatusRegAddr         = 0x08;
    static constexpr uint8_t KFractionalBits        = 21;
    // --- Reference Cap in pF ---
    static constexpr float KRefCap                  = 15.0f;
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
