
#include "M451Series.h"

/**
 * AD5933 Register Map
 *  Datasheet p23
 */
 
#define M_PI 3.14159265358979323846
 
// Device address and address pointer
#define AD5933_ADDR     (0x0D)

// Address pointer command 
#define AD5933_ADDR_PTR_CMD        (0xB0)

// Control Register
#define AD5933_CTRL_REG1       (0x80)
#define AD5933_CTRL_REG2       (0x81)
// Start Frequency Register
#define AD5933_START_FREQ_1    (0x82)
#define AD5933_START_FREQ_2    (0x83)
#define AD5933_START_FREQ_3    (0x84)
// Frequency increment register
#define AD5933_INC_FREQ_1      (0x85)
#define AD5933_INC_FREQ_2      (0x86)
#define AD5933_INC_FREQ_3      (0x87)
// Number of increments register
#define AD5933_NUM_INC_1       (0x88)
#define AD5933_NUM_INC_2       (0x89)
// Number of settling time cycles register
#define AD5933_NUM_SCYCLES_1   (0x8A)
#define AD5933_NUM_SCYCLES_2   (0x8B)
// Status register
#define AD5933_STATUS_REG      (0x8F)
// Temperature data register
#define AD5933_TEMP_DATA_1     (0x92)
#define AD5933_TEMP_DATA_2     (0x93)
// Real data register
#define AD5933_REAL_DATA_1     (0x94)
#define AD5933_REAL_DATA_2     (0x95)
// Imaginary data register
#define AD5933_IMAG_DATA_1     (0x96)
#define AD5933_IMAG_DATA_2     (0x97)

/**
 * Constants
 *  Constants for use with the AD5933 library class.
 */
// Temperature measuring
#define AD5933_TEMP_MEASURE    (AD5933_CTRL_TEMP_MEASURE)
#define AD5933_TEMP_NO_MEASURE (AD5933_CTRL_NO_OPERATION)
// Clock sources
#define AD5933_CLOCK_INTERNAL  (AD5933_CTRL_CLOCK_INTERNAL)
#define AD5933_CLOCK_EXTERNAL  (AD5933_CTRL_CLOCK_EXTERNAL)
// PGA gain options
#define AD5933_PGA_GAIN_X1     (AD5933_CTRL_PGA_GAIN_X1)
#define AD5933_PGA_GAIN_X5     (AD5933_CTRL_PGA_GAIN_X5)
// Power modes
#define AD5933_POWER_STANDBY   (AD5933_CTRL_STANDBY_MODE)
#define AD5933_POWER_DOWN      (AD5933_CTRL_POWER_DOWN_MODE)
#define AD5933_POWER_ON        (AD5933_CTRL_NO_OPERATION)
// I2C result success/fail
#define AD5933_I2C_RESULT_SUCCESS       (0)
#define AD5933_I2C_RESULT_DATA_TOO_LONG (1)
#define AD5933_I2C_RESULT_ADDR_NAK      (2)
#define AD5933_I2C_RESULT_DATA_NAK      (3)
#define AD5933_I2C_RESULT_OTHER_FAIL    (4)
// Control register options
#define AD5933_CTRL_NO_OPERATION       (0b00000000)
#define AD5933_CTRL_INIT_START_FREQ    (0b00010000)
#define AD5933_CTRL_START_FREQ_SWEEP   (0b00100000)
#define AD5933_CTRL_INCREMENT_FREQ     (0b00110000)
#define AD5933_CTRL_REPEAT_FREQ        (0b01000000)
#define AD5933_CTRL_TEMP_MEASURE       (0b10010000)
#define AD5933_CTRL_POWER_DOWN_MODE    (0b10100000)
#define AD5933_CTRL_STANDBY_MODE       (0b10110000)
#define AD5933_CTRL_RESET              (0b00010000)
#define AD5933_CTRL_CLOCK_EXTERNAL     (0b00001000)
#define AD5933_CTRL_CLOCK_INTERNAL     (0b00000000)
#define AD5933_CTRL_PGA_GAIN_X1        (0b00000001)
#define AD5933_CTRL_PGA_GAIN_X5        (0b00000000)
#define AD5933_CTRL_OUTPUT_RANGE_2V    (0b00000000)
#define AD5933_CTRL_OUTPUT_RANGE_200mV (0b00000010)
#define AD5933_CTRL_OUTPUT_RANGE_400mV (0b00000100)
#define AD5933_CTRL_OUTPUT_RANGE_1V    (0b00000110)


// Status register options
#define AD5933_STATUS_TEMP_VALID       (0x01)
#define AD5933_STATUS_DATA_VALID       (0x02)
#define AD5933_STATUS_SWEEP_DONE       (0x04)
#define AD5933_STATUS_ERROR            (0xFF)
// Frequency sweep parameters
#define AD5933_SWEEP_DELAY             (1

static const unsigned int AD5933_clockSpeed = 16776000;

void AD5933_init();
void AD5933_deinit();

uint8_t AD5933_getByte(uint8_t address, uint8_t *value);
uint8_t AD5933_sendByte(uint8_t address, uint8_t value);
uint8_t AD5933_setControlMode(uint8_t mode);
uint8_t AD5933_reset();
uint8_t AD5933_enableTemperature(uint8_t enable);
double AD5933_getTemperature();
uint8_t AD5933_setClockSource(uint8_t source);
uint8_t AD5933_setInternalClock(uint8_t internal);
uint8_t AD5933_setStartFrequency(unsigned long start);
uint8_t AD5933_setIncrementFrequency(unsigned long increment);
uint8_t AD5933_setNumberIncrements(unsigned int);

// Gain configuration
uint8_t AD5933_setPGAGain(uint8_t);

// Excitation range configuration
//bool setRange(byte, int); // not implemented - not used yet

// Read registers
uint8_t AD5933_readRegister(uint8_t);
uint8_t AD5933_readStatusRegister(void);
int AD5933_readControlRegister(void);

// Impedance data
uint8_t AD5933_getComplexData(int16_t*, int16_t*);

// Set control mode register (CTRL_REG1)
uint8_t AD5933_setControlMode(uint8_t);

// Power mode
uint8_t AD5933_setPowerMode(uint8_t);

// Perform frequency sweeps
uint8_t AD5933_frequencySweep(int16_t real[], int16_t imag[], int);
uint8_t AD5933_calibrate(double gain[], double phase[],
                       int ref, int16_t* real, int16_t* imag, unsigned int n);
double AD5933_calculate_phase(int16_t real, int16_t imag);
double AD5933_calculate_resistance(int16_t real, int16_t imag, double gain, double phase);
double AD5933_calculate_reactance(int16_t real, int16_t imag, double gain, double phase);
uint8_t AD5933_readStatusRegister();
