#include "AD5933.h"
#include "arm_math.h"

#define AD5933_MaxTxLen 1
#define AD5933_Read_Flag 1
#define AD5933_Write_Flag 0


volatile uint16_t AD5933_TxData = 0;
volatile uint8_t AD5933_RxData = 0;
//volatile uint8_t AD5933_TxDataLen = 0;
//volatile uint8_t AD5933_RxDataLen = 0;
//volatile uint8_t AD5933_TxRWFlag = 0;   // 0 -> write, 1 -> read
volatile uint8_t AD5933_RWFlag = 0; // 0 -> write, 1 -> read
volatile uint8_t AD5933_TXEndFlag = 0;
volatile uint8_t AD5933_EndFlag = 0;
volatile uint32_t AD5933_32Status; 


void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);

}

void I2C1_Start()
{
	AD5933_EndFlag = 0;
	AD5933_TXEndFlag = 0;
	I2C_START(I2C1);

}

void I2C1_Stop()
{
	I2C_STOP(I2C1);
	AD5933_EndFlag = 1;
	AD5933_TXEndFlag = 1;
}

void AD5933_init()
{
	    /* Open I2C module and set bus clock */
    I2C_Open(I2C1, 250000);

    /* Get I2C1 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C Slave Addresses */
    //I2C_SetSlaveAddr(I2C1, 0, AD5933_ADDR, 0);  
		//I2C_DisableWakeup(I2C1);
		//printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C1));
	
    /* Enable I2C interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
	
}

void AD5933_deinit(){

	I2C1_Close();
}




/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    AD5933_32Status = I2C_GET_STATUS(I2C1);

    if(I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
			  AD5933_32Status = 0xFF; 
				AD5933_TXEndFlag = 1;
				AD5933_EndFlag = 1;
    }
    else if(AD5933_32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
		{
				I2C_SET_DATA(I2C1, (AD5933_ADDR << 1));    /* Write SLA+W to Register I2CDAT */
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
		}
		else if(AD5933_32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
		{
				I2C_SET_DATA(I2C1, AD5933_TxData >> 8);
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
		}
		else if(AD5933_32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
		{
				I2C1_Stop();
				I2C1_Start();
		}
		else if(AD5933_32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
		{
				
				if (!AD5933_TXEndFlag)
				{
						I2C_SET_DATA(I2C1, AD5933_TxData & 0xFF);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
						AD5933_TXEndFlag = 1;
				}
				else
				{
					
					if (AD5933_RWFlag == AD5933_Read_Flag){  /* Prepare repeating start */
						I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA_SI);
					}
					else {
						I2C1_Stop();
					}
				}
		}
		else if(AD5933_32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
		{
				I2C_SET_DATA(I2C1, ((AD5933_ADDR << 1)|0x01));   /* Write SLA+R to Register I2CDAT */
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
		}
		else if(AD5933_32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
		{
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
		}
		else if(AD5933_32Status == 0x58)                  /* DATA has been received and NACK has been returned */
		{
				AD5933_RxData = I2C_GET_DATA(I2C1);
				// printf("I2C RX Data: %d\n", AD5933_RxData);
				I2C1_Stop();
		}
		else
		{
				/* TO DO */
				printf("Status 0x%x is NOT processed\n", AD5933_32Status);
				AD5933_32Status = 0xFF; 
				AD5933_EndFlag = 1;
				AD5933_TXEndFlag = 1;
		}
}



/**
 * Request to read a uint8_t from the AD5933.
 *
 * @param address Address of register requesting data from
 * @param value Pointer to a uint8_t where the return value should be stored, or
 *        where the error code will be stored if fail.
 * @return Success or failure
 */
uint8_t AD5933_getByte(uint8_t address, uint8_t *value) {
    // Request to read a uint8_t using the address pointer register
		AD5933_TxData = ((uint16_t)AD5933_ADDR_PTR_CMD << 8) | address;
	  AD5933_RWFlag = AD5933_Read_Flag;
	
		/* I2C as master sends START signal */
		I2C1_Start();
	
		while(AD5933_EndFlag == 0);

	
		if (AD5933_32Status == 0xFF){
			return 0;
		}
		else{
			*value = AD5933_RxData;
			return 1;
		}
}

/**
 * Write a uint8_t to a register on the AD5933.
 *
 * @param address The register address to write to
 * @param value The uint8_t to write to the address
 * @return Success or failure of transmission
 */
uint8_t AD5933_sendByte(uint8_t address, uint8_t value) {
	
	// Request to read a uint8_t using the address pointer register
		AD5933_TxData = (((uint16_t)address) << 8) | value;
	  AD5933_RWFlag = AD5933_Write_Flag;
	
		/* I2C as master sends START signal */
		I2C1_Start();
	
	  while(AD5933_EndFlag == 0){};
		

    // Check that transmission completed successfully
    if (AD5933_32Status == 0xFF) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * Set the control mode register, CTRL_REG1. This is the register where the
 * current command needs to be written to so this is used a lot.
 *
 * @param mode The control mode to set
 * @return Success or failure
 */
uint8_t AD5933_setControlMode(uint8_t mode) {
    // Get the current value of the control register
    uint8_t val;
    if (!AD5933_getByte(AD5933_CTRL_REG1, &val))
        return 0;

    // Wipe out the top 4 bits...mode bits are bits 5 through 8.
    val &= 0x0F;

    // Set the top 4 bits appropriately
    val |= mode;

    // Write back to the register
    return AD5933_sendByte(AD5933_CTRL_REG1, val);
}


/**
 * Reset the AD5933. This interrupts a sweep if one is running, but the start
 * frequency, number of increments, and frequency increment register contents
 * are not overwritten, but an initialize start frequency command is required
 * to restart a frequency sweep.
 *
 * @return Success or failure
 */
uint8_t AD5933_reset() {
    // Get the current value of the control register
    uint8_t val;
    if (!AD5933_getByte(AD5933_CTRL_REG2, &val))
        return 0;

    // Set bit D4 for restart
    val |= AD5933_CTRL_RESET;

    // Send uint8_t back
    return AD5933_sendByte(AD5933_CTRL_REG2, val);
}

/**
 * Set enable temperature measurement. This interferes with frequency sweep
 * operation, of course.
 *
 * @param enable Option to enable to disable temperature measurement.
 * @return Success or failure
 */
uint8_t AD5933_enableTemperature(uint8_t enable) {
    // If enable, set temp measure bits. If disable, reset to no operation.
    if (enable == AD5933_TEMP_MEASURE) {
        return AD5933_setControlMode(AD5933_CTRL_TEMP_MEASURE);
    } else {
        return AD5933_setControlMode(AD5933_CTRL_NO_OPERATION);
    }
}

/**
 * Get the temperature reading from the AD5933. Waits until a temperature is
 * ready. Also ensures temperature measurement mode is active.
 *
 * @return The temperature in celcius, or -1 if fail.
 */
double AD5933_getTemperature() {
    // Set temperature mode
    if (AD5933_enableTemperature(AD5933_TEMP_MEASURE)) {
        // Wait for a valid temperature to be ready
        while((AD5933_readStatusRegister() & AD5933_STATUS_TEMP_VALID) != AD5933_STATUS_TEMP_VALID) ;

        // Read raw temperature from temperature registers
        uint8_t rawTemp[2];
        if (AD5933_getByte(AD5933_TEMP_DATA_1, &rawTemp[0]) &&
            AD5933_getByte(AD5933_TEMP_DATA_2, &rawTemp[1]))
        {
            // Combine raw temperature uint8_ts into an interger. The ADC
            // returns a 14-bit 2's C value where the 14th bit is a sign
            // bit. As such, we only need to keep the bottom 13 bits.
            int rawTempVal = (rawTemp[0] << 8 | rawTemp[1]) & 0x1FFF;

            // Convert into celcius using the formula given in the
            // datasheet. There is a different formula depending on the sign
            // bit, which is the 5th bit of the uint8_t in TEMP_DATA_1.
            if ((rawTemp[0] & (1<<5)) == 0) {
                return rawTempVal / 32.0;
            } else {
                return (rawTempVal - 16384) / 32.0;
            }
        }
    }
    return -1;
}


/**
 * Set the color source. Choices are between internal and external.
 *
 * @param source Internal or External clock
 * @return Success or failure
 */
uint8_t AD5933_setClockSource(uint8_t source) {
    // Determine what source was selected and set it appropriately
    switch (source) {
        case AD5933_CLOCK_EXTERNAL:
            return AD5933_sendByte(AD5933_CTRL_REG2, AD5933_CTRL_CLOCK_EXTERNAL);
        case AD5933_CLOCK_INTERNAL:
            return AD5933_sendByte(AD5933_CTRL_REG2, AD5933_CTRL_CLOCK_INTERNAL);
        default:
            return 0;
    }
}

/**
 * Set the color source to internal or not.
 *
 * @param internal Whether or not to set the clock source as internal.
 * @return Success or failure
 */
uint8_t AD5933_setInternalClock(uint8_t internal) {
    // This function is mainly a wrapper for setClockSource()
    if (internal)
        return AD5933_setClockSource(AD5933_CLOCK_INTERNAL);
    else
        return AD5933_setClockSource(AD5933_CLOCK_EXTERNAL);
}

/**
 * Set the start frequency for a frequency sweep.
 *
 * @param start The initial frequency.
 * @return Success or failure
 */
uint8_t AD5933_setStartFrequency(unsigned long start) {
    // Page 24 of the Datasheet gives the following formula to represent the
    // start frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
    long freqHex = (start / (AD5933_clockSpeed / 4.0))*pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return 0;   // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 uint8_ts.
    uint8_t highByte = (freqHex >> 16) & 0xFF;
    uint8_t midByte = (freqHex >> 8) & 0xFF;
    uint8_t lowByte = freqHex & 0xFF;

    // Attempt sending all three uint8_ts
    return AD5933_sendByte(AD5933_START_FREQ_1, highByte) &&
           AD5933_sendByte(AD5933_START_FREQ_2, midByte) &&
           AD5933_sendByte(AD5933_START_FREQ_3, lowByte);
}

/**
 * Set the increment frequency for a frequency sweep.
 *
 * @param start The frequency to increment by. Max of 0xFFFFFF.
 * @return Success or failure
 */
uint8_t AD5933_setIncrementFrequency(unsigned long increment) {
    // Page 25 of the Datasheet gives the following formula to represent the
    // increment frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
    long freqHex = (increment / (AD5933_clockSpeed / 4.0))*pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return 0;   // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 uint8_ts.
    uint8_t highByte = (freqHex >> 16) & 0xFF;
    uint8_t midByte = (freqHex >> 8) & 0xFF;
    uint8_t lowByte = freqHex & 0xFF;

    // Attempt sending all three uint8_ts
    return AD5933_sendByte(AD5933_INC_FREQ_1, highByte) &&
           AD5933_sendByte(AD5933_INC_FREQ_2, midByte) &&
           AD5933_sendByte(AD5933_INC_FREQ_3, lowByte);
}

/**
 * Set the number of frequency increments for a frequency sweep.
 *
 * @param start The number of increments to use. Max 511.
 * @return Success or failure
 */
uint8_t AD5933_setNumberIncrements(unsigned int num) {
    // Check that the number sent in is valid.
    if (num > 511) {
        return 0;
    }

    // Divide the 9-bit integer into 2 uint8_ts.
    uint8_t highByte = (num >> 8) & 0xFF;
    uint8_t lowByte = num & 0xFF;

    // Write to register.
    return AD5933_sendByte(AD5933_NUM_INC_1, highByte) &&
           AD5933_sendByte(AD5933_NUM_INC_2, lowByte);
}

/**
 * Set the PGA gain factor.
 *
 * @param gain The gain factor to select. Use constants or 1/5.
 * @return Success or failure
 */
uint8_t AD5933_setPGAGain(uint8_t gain) {
    // Get the current value of the control register
    uint8_t val;
    if (!AD5933_getByte(AD5933_CTRL_REG1, &val))
        return 0;

    // Clear out the bottom bit, D8, which is the PGA gain set bit
    val &= 0xFE;

    // Determine what gain factor was selected
    if (gain == AD5933_PGA_GAIN_X1 || gain == 1) {
        // Set PGA gain to x1 in CTRL_REG1
        val |= AD5933_PGA_GAIN_X1;
        return AD5933_sendByte(AD5933_CTRL_REG1, val);
    } else if (gain == AD5933_PGA_GAIN_X5 || gain == 5) {
        // Set PGA gain to x5 in CTRL_REG1
        val |= AD5933_PGA_GAIN_X5;
        return AD5933_sendByte(AD5933_CTRL_REG1, val);
    } else {
        return 0;
    }
}

/**
 * Read the value of a register.
 *
 * @param reg The address of the register to read.
 * @return The value of the register. Returns 0xFF if can't read it.
 */
uint8_t AD5933_readRegister(uint8_t reg) {
    // Read status register and return it's value. If fail, return 0xFF.
    uint8_t val;
    if (AD5933_getByte(reg, &val)) {
        return val;
    } else {
        return AD5933_STATUS_ERROR;
    }
}

/**
 * Read the value of the status register.
 *
 * @return The value of the status register. Returns 0xFF if can't read it.
 */
uint8_t AD5933_readStatusRegister() {
    return AD5933_readRegister(AD5933_STATUS_REG);
}

/**
 * Read the value of the control register.
 *
 * @return The value of the control register. Returns 0xFFFF if can't read it.
 */
int AD5933_readControlRegister() {
    return ((AD5933_readRegister(AD5933_CTRL_REG1) << 8) | AD5933_readRegister(AD5933_CTRL_REG2)) & 0xFFFF;
}

/**
 * Get a raw complex number for a specific frequency measurement.
 *
 * @param real Pointer to an int that will contain the real component.
 * @param imag Pointer to an int that will contain the imaginary component.
 * @return Success or failure
 */
uint8_t AD5933_getComplexData(int16_t *real, int16_t *imag) {
    // Wait for a measurement to be available
    while ((AD5933_readStatusRegister() & AD5933_STATUS_DATA_VALID) != AD5933_STATUS_DATA_VALID);

    // Read the four data registers.
    // TODO: Do this faster with a block read
    uint8_t realComp[2];
    uint8_t imagComp[2];
    if (AD5933_getByte(AD5933_REAL_DATA_1, &realComp[0]) &&
        AD5933_getByte(AD5933_REAL_DATA_2, &realComp[1]) &&
        AD5933_getByte(AD5933_IMAG_DATA_1, &imagComp[0]) &&
        AD5933_getByte(AD5933_IMAG_DATA_2, &imagComp[1]))
    {
        // Combine the two separate uint8_ts into a single 16-bit value and store
        // them at the locations specified.
        *real = (int16_t)(((int16_t)(realComp[0] << 8) | realComp[1]) & 0xFFFF);
        *imag = (int16_t)(((int16_t)(imagComp[0] << 8) | imagComp[1]) & 0xFFFF);

        return 1;
    } else {
        *real = -1;
        *imag = -1;
        return 0;
    }
}

/**
 * Set the power level of the AD5933.
 *
 * @param level The power level to choose. Can be on, standby, or down.
 * @return Success or failure
 */
uint8_t AD5933_setPowerMode(uint8_t level) {
    // Make the appropriate switch. TODO: Does no operation even do anything?
    switch (level) {
        case AD5933_POWER_ON:
            return AD5933_setControlMode(AD5933_CTRL_NO_OPERATION);
        case AD5933_POWER_STANDBY:
            return AD5933_setControlMode(AD5933_CTRL_STANDBY_MODE);
        case AD5933_POWER_DOWN:
            return AD5933_setControlMode(AD5933_CTRL_POWER_DOWN_MODE);
        default:
            return 0;
    }
}

/**
 * Perform a complete frequency sweep.
 *
 * @param real An array of appropriate size to hold the real data.
 * @param imag An array of appropriate size to hold the imaginary data.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
uint8_t AD5933_frequencySweep(int16_t real[], int16_t imag[], int n) {
    // Begin by issuing a sequence of commands
    // If the commands aren't taking hold, add a brief delay
    if (!(AD5933_setPowerMode(AD5933_POWER_STANDBY) &&         // place in standby
         AD5933_setControlMode(AD5933_CTRL_INIT_START_FREQ) && // init start freq
         AD5933_setControlMode(AD5933_CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             return 0;
         }

    // Perform the sweep. Make sure we don't exceed n.
    int i = 0;
    while ((AD5933_readStatusRegister() & AD5933_STATUS_SWEEP_DONE) != AD5933_STATUS_SWEEP_DONE) {
        // Make sure we aren't exceeding the bounds of our buffer
        if (i >= n) {
            return 0;
        }

        // Get the data for this frequency point and store it in the array
        if (!AD5933_getComplexData(&real[i], &imag[i])) {
            return 0;
        }

        // Increment the frequency and our index.
        i++;
        AD5933_setControlMode(AD5933_CTRL_INCREMENT_FREQ);
    }

    // Put into standby
    return AD5933_setPowerMode(AD5933_POWER_STANDBY);
}

/**
 * Computes the gain factor and phase for each point in a frequency sweep.
 * Also provides the caller with the real and imaginary data.
 *
 * @param gain An array of appropriate size to hold the gain factors
 * @param phase An array of appropriate size to hold the phase data
 * @param ref The known reference resistance.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
uint8_t AD5933_calibrate(double gain[], double phase[],
                       int ref, int16_t* real, int16_t* imag, unsigned int n) {											 
    // Perform the frequency sweep
    if (!AD5933_frequencySweep(real, imag, n)) {
        return 0;
    }
		int i = 0;
    // For each point in the sweep, calculate the gain factor and phase
    for (i = 0; i < n; i++) {
			  double magnitude = sqrt(real[i]*real[i] + imag[i]*imag[i]);
        gain[i] = (double)(1.0/(double)ref)/magnitude;
				phase[i] = AD5933_calculate_phase(real[i], imag[i]);
    }

    return 1;
}

double AD5933_calculate_phase(int16_t real, int16_t imag){
	double degree = atan((double)imag/(double)real)*180/M_PI;
	if(real > 0 && imag > 0){
		return degree;
	}
	if(real < 0 && imag > 0){
		return 180 + degree;
	}
	if(real < 0 && imag < 0){
		return 180 + degree;
	}
	if(real > 0 && imag < 0){
		return 360 + degree;
	}
	
	return 0;
}


double AD5933_calculate_resistance(int16_t real, int16_t imag, double gain, double phase){
	
		double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
		double impedance = 1/(magnitude*gain);
		return arm_cos_f32(phase * M_PI / 180) * impedance;
}

double AD5933_calculate_reactance(int16_t real, int16_t imag, double gain, double phase){
		double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
		double impedance = 1/(magnitude*gain);
		return arm_sin_f32(phase * M_PI / 180) * impedance;
}


