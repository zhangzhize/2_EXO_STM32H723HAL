#ifndef AS7341_HPP
#define AS7341_HPP

#include "main.h"

#define AS7341_DEFAULT_I2C_ADDR 0x39

enum class AS7341_ERROR : int8_t {
    OK                  = 0,
    I2C_COMM_ERROR      = -1,
    WRONG_CHIP_ID       = -2,
    MEASUREMENT_TIMEOUT = -3,
    INVALID_DEVICE      = -4,
    CUSTOM_ERROR        = -5
};


// Gain enumeration
enum class AS7341_GAIN
{
	GAIN_HALF,
	GAIN_X1,
	GAIN_X2,
	GAIN_X4,
	GAIN_X8,
	GAIN_X16,
	GAIN_X32,
	GAIN_X64,
	GAIN_X128,
	GAIN_X256,
	GAIN_X512,
	GAIN_INVALID
};

enum class AS7341_CH : uint8_t {
    F1, F2, F3, F4, F5, F6, F7, F8, CLEAR, NIR, NONE
};

class AS7341
{
public:
    AS7341();
    ~AS7341() = default;

	// Initialize AS7341
	AS7341_ERROR begin(I2C_HandleTypeDef* hi2c, uint8_t addr = AS7341_DEFAULT_I2C_ADDR);

	// Check if board's devices are connected and responding properly
	bool isConnected();

	// Read all channels raw values
	AS7341_ERROR readAllChannels(unsigned int* channelData);

	// Read all channels basic counts. Further information can be found in AN000633, page 7
	AS7341_ERROR readAllChannelsBasicCounts(float* channelDataBasicCounts);

	// Enable AS7341
	void enable_AS7341();

    // Power down AS7341
	void disable_AS7341();

	// Set LED driver forward current value
	void setLedDrive(unsigned int current);
	// Return LED driver forward current value
	unsigned int getLedDrive();

	// Set ADC integration time (defaults to 29)
	void setATIME(uint8_t aTime = 29);
	// Set ADC integration steps (defaults to 599)
	void setASTEP(unsigned int aStep = 599);
	// Return ADC integration time
	uint8_t getATIME();
	// Get ADC integration steps
	unsigned int getASTEP();

	// Set ADC gain
	void setGain(AS7341_GAIN gain = AS7341_GAIN::GAIN_X256);
	// Return ADC gain
	AS7341_GAIN getGain();

	// Enables interrupt pin functionality
	void enablePinInterupt();
	// Disables interrupt pin functionality
	void disablePinInterrupt();
	// Clears the interrupt flag
	void clearPinInterrupt();

	// Reads register
	uint8_t readRegister(uint8_t reg);
	// Writes register
	void writeRegister(uint8_t reg, uint8_t value);

	// Sets GPIO as input
	void setGpioPinInput();
	// Sets GPIO as output
	void setGpioPinOutput();
	// Reads GPIO pin value
	bool digitalRead();
	// Inverts GPIO pin output
	void invertGpioOutput(bool isInverted);
	// Writes GPIO pin
	void digitalWrite(uint8_t value);

	// Read raw value of 415 nm channel
	unsigned int read415nm();
	// Read 445 nm channel raw value
	unsigned int read445nm();
	// Read raw value of 480 nm channel
	unsigned int read480nm();
	// Read raw value of 515 nm channel
	unsigned int read515nm();
	// Read raw value of 555 nm channel
	unsigned int read555nm();
	// Read raw value of 590 nm channel
	unsigned int read590nm();
	// Read raw value of 630 nm channel
	unsigned int read630nm();
	// Read raw value of 680 nm channel
	unsigned int read680nm();
	// Read raw value of clear channel
	unsigned int readClear();
	// Read raw value of NIR channel
	unsigned int readNIR();
	
	// Read basic count value of 445 nm channel
	float readBasicCount415nm();
	// Read basic count value of 445 nm channel
	float readBasicCount445nm();
	// Read basic count value of 480 nm channel
	float readBasicCount480nm();
	// Read basic count value of 515 nm channel
	float readBasicCount515nm();
	// Read basic count value of 555 nm channel
	float readBasicCount555nm();
	// Read basic count value of 590 nm channel
	float readBasicCount590nm();
	// Read basic count value of 630 nm channel
	float readBasicCount630nm();
	// Read basic count value of 680 nm channel
	float readBasicCount680nm();
	// Read basic count value of clear channel
	float readBasicCountClear();
	// Read basic count value of NIR channel
	float readBasicCountNIR();

	// Sets the low threshold value
	void setLowThreshold(unsigned int threshold);
	// Sets the high threshold value
	void setHighThreshold(unsigned int threshold);
	// Reads low threshold value
	unsigned int getLowThreshold();
	// Reads high threshold value;
	unsigned int getHighThreshold();

	// Sets APERS register which will dictate how many consecutive samples must be above/below thresholds to trigger an interrupt. Values allowed: 0 to 15.
	void setAPERS(uint8_t value);
	// Gets APERS register value
	uint8_t getAPERS();

	// Enable measurements by setting SP_EN bit
	void enableMeasurements();
	// Disables measurements by clearing SP_EN bit
	void disableMeasurements();
	// Returns true if AS7341 has SP_EN set
	bool isMeasurementEnabled();
		
	// Enable threshold interrupt generation
	void enableThresholdInterrupt();
	// Disable threshold interrupt generation
	void disableThresholdInterrupt();
	// Clear threshold interrupt for allowing new interrupts to be triggered
	void clearThresholdInterrupts();
	// Returns true if the channel value is lower than the low threshold value
	bool lowThresholdInterruptSet();
	// Returns true if the channel value is higher than the high threshold value
	bool highThresholdInterruptSet();
	
	// AS7341 specific function - returns 100 for 100 Hz, 120 for 120 Hz, 0 for unknown and -1 for invalid device
	int getFlickerFrequency();

    //For RGB readings
    void setMuxCustom(AS7341_CH adc0,  AS7341_CH adc1 = AS7341_CH::NONE, AS7341_CH adc2 = AS7341_CH::NONE, AS7341_CH adc3 = AS7341_CH::NONE, AS7341_CH adc4 = AS7341_CH::NONE, AS7341_CH adc5 = AS7341_CH::NONE);
    AS7341_ERROR readCustomChannels(uint16_t* results);

private:
    uint8_t _addr = AS7341_DEFAULT_I2C_ADDR;
    I2C_HandleTypeDef* _hi2c = nullptr;

    bool _currentBankIsHigh = false;

	bool whiteLedPowered = false;
	bool IRLedPowered = false;
	// Sets F1 to F4 + Clear + NIR to ADCs inputs
	void setMuxLo();
	// Sets F5 to F8 + Clear + NIR to ADCs inputs
	void setMuxHi();
	// Reads single channel value after mux setup
	uint16_t readSingleChannelValue();
	// Converts raw value to basic count value
	float readSingleBasicCountChannelValue(uint16_t raw);

    void setBankConfiguration(uint8_t regAddress);
    uint8_t readSingleByte(uint8_t registerAddress);
    void writeSingleByte(uint8_t registerAddress, uint8_t const value);
    void readMultipleBytes(uint8_t registerAddress, uint8_t* buffer, uint8_t const packetLength);
    void writeMultipleBytes(uint8_t registerAddress, const uint8_t* buffer, uint8_t const packetLength);
    void setRegisterBit(uint8_t registerAddress, uint8_t const bitPosition);
    void clearRegisterBit(uint8_t registerAddress, uint8_t const bitPosition);
    bool isBitSet(uint8_t registerAddress, uint8_t const bitPosition);

    void writeSMUX(const uint8_t* smux_data);

    // Registers definitions
    static constexpr uint8_t REGISTER_CH0_DATA_L	= 0x95;
    static constexpr uint8_t REGISTER_CH0_DATA_H	= 0x96;
    static constexpr uint8_t REGISTER_ITIME_L		= 0x63;
    static constexpr uint8_t REGISTER_ITIME_M		= 0x64;
    static constexpr uint8_t REGISTER_ITIME_H		= 0x65;
    static constexpr uint8_t REGISTER_CH1_DATA_L	= 0x97;
    static constexpr uint8_t REGISTER_CH1_DATA_H	= 0x98;
    static constexpr uint8_t REGISTER_CH2_DATA_L	= 0x99;
    static constexpr uint8_t REGISTER_CH2_DATA_H	= 0x9a;
    static constexpr uint8_t REGISTER_CH3_DATA_L	= 0x9b;
    static constexpr uint8_t REGISTER_CH3_DATA_H	= 0x9c;
    static constexpr uint8_t REGISTER_CH4_DATA_L	= 0x9d;
    static constexpr uint8_t REGISTER_CH4_DATA_H	= 0x9e;
    static constexpr uint8_t REGISTER_CH5_DATA_L	= 0x9f;
    static constexpr uint8_t REGISTER_CH5_DATA_H	= 0xa0;
    static constexpr uint8_t REGISTER_CONFIG		= 0x70;
    static constexpr uint8_t REGISTER_STAT			= 0x71;
    static constexpr uint8_t REGISTER_EDGE			= 0x72;
    static constexpr uint8_t REGISTER_GPIO			= 0x73;
    static constexpr uint8_t REGISTER_LED			= 0x74;
    static constexpr uint8_t REGISTER_ENABLE		= 0x80;
    static constexpr uint8_t REGISTER_ATIME			= 0x81;
    static constexpr uint8_t REGISTER_WTIME			= 0x82;
    static constexpr uint8_t REGISTER_SP_TH_L_LSB	= 0x84;
    static constexpr uint8_t REGISTER_SP_TH_L_MSB	= 0x85;
    static constexpr uint8_t REGISTER_SP_TH_H_LSB	= 0x86;
    static constexpr uint8_t REGISTER_SP_TH_H_MSB	= 0x87;
    static constexpr uint8_t REGISTER_AUXID			= 0x90;
    static constexpr uint8_t REGISTER_REVID			= 0x91;
    static constexpr uint8_t REGISTER_ID			= 0x92;
    static constexpr uint8_t REGISTER_STATUS		= 0x93;
    static constexpr uint8_t REGISTER_ASTATUS		= 0x94;
    static constexpr uint8_t REGISTER_STATUS_2		= 0xa3;
    static constexpr uint8_t REGISTER_STATUS_3		= 0xa4;
    static constexpr uint8_t REGISTER_STATUS_5		= 0xa6;
    static constexpr uint8_t REGISTER_STATUS_6		= 0xa7;
    static constexpr uint8_t REGISTER_CFG_0			= 0xa9;
    static constexpr uint8_t REGISTER_CFG_1			= 0xaa;
    static constexpr uint8_t REGISTER_CFG_3			= 0xac;
    static constexpr uint8_t REGISTER_CFG_6			= 0xaf;
    static constexpr uint8_t REGISTER_CFG_8			= 0xb1;
    static constexpr uint8_t REGISTER_CFG_9			= 0xb2;
    static constexpr uint8_t REGISTER_CFG_10		= 0xb3;
    static constexpr uint8_t REGISTER_CFG_12		= 0xb5;
    static constexpr uint8_t REGISTER_PERS			= 0xbd;
    static constexpr uint8_t REGISTER_GPIO_2		= 0xbe;
    static constexpr uint8_t REGISTER_ASTEP_L		= 0xca;
    static constexpr uint8_t REGISTER_ASTEP_H		= 0xcb;
    static constexpr uint8_t REGISTER_AGC_GAIN_MAX	= 0xcf;
    static constexpr uint8_t REGISTER_AZ_CONFIG		= 0xd6;
    static constexpr uint8_t REGISTER_FD_TIME_1		= 0xd8;
    static constexpr uint8_t REGISTER_FD_TIME_2		= 0xda;
    static constexpr uint8_t REGISTER_FD_STATUS		= 0xdb;
    static constexpr uint8_t REGISTER_INTENAB		= 0xf9;
    static constexpr uint8_t REGISTER_CONTROL		= 0xfa;
    static constexpr uint8_t REGISTER_FIFO_MAP		= 0xfc;
    static constexpr uint8_t REGISTER_FIFO_LVL		= 0xfd;
    static constexpr uint8_t REGISTER_FDATA_L		= 0xfe;
    static constexpr uint8_t REGISTER_FDATA_H		= 0xff;
};



#endif