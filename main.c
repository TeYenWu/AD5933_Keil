/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 10 $
 * $Date: 15/09/02 10:04a $
 * @brief
 *           Show a Master how to access Slave.
 *           This sample code needs to work with I2C_Slave.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "AD5933.h"
#include "myUart.h"
#include "arm_math.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C1 module clock */
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD, TXD and */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set I2C PA multi-function pins */
		SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE0MFP_Msk);
		SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk);
		SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE0MFP_I2C1_SDA);
		SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC4MFP_I2C1_SCL);
    //SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    //SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_I2C1_SDA | SYS_GPA_MFPL_PA3MFP_I2C1_SCL);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
	
		myUartOpen();
}

void doUART(){


}

#define DEFAULT_START_FREQ  (80000)
#define DEFAULT_FREQ_INCR   (1000)
#define DEFAULT_NUM_INCR    (40)
#define DEFAULT_REF_RESIST  (150000)

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init I2C1 */
    AD5933_init();
		
		unsigned long start_freq = DEFAULT_START_FREQ;
		unsigned long freq_incr = DEFAULT_FREQ_INCR;
		unsigned int num_incr = DEFAULT_NUM_INCR;
	
		
	// Perform initial configuration. Fail if any one of these fail.
	if (!(AD5933_reset() &&
			AD5933_setInternalClock(1) &&
			AD5933_setStartFrequency(start_freq) &&
			AD5933_setIncrementFrequency(freq_incr) &&
			AD5933_setNumberIncrements(num_incr) &&
			AD5933_setPGAGain( AD5933_PGA_GAIN_X1)))
			{
					printf("FAILED in initialization!");
					 while (1) ;
			} 
        
	unsigned int ref_resist = DEFAULT_REF_RESIST;
	int n = num_incr+1;
  int16_t* real_array = (int16_t*)calloc(n, sizeof(int16_t));
	int16_t* imag_array = (int16_t*)calloc(n, sizeof(int16_t));
  uint8_t isStarting = 1;
	double* gain = (double*)calloc(n, sizeof(double));
	double* sysphase = (double*)calloc(n, sizeof(double));
				
  // Perform calibration sweep
  if (AD5933_calibrate(gain, sysphase, ref_resist,real_array, imag_array, n))
    printf("Calibrated!");
  else {
    printf("Calibration failed...");
		while(1){}
	}
	 
	 while(AD5933_frequencySweep(real_array, imag_array, n) && 	isStarting){
			
			int cfreq = start_freq/1000;
      for (int i = 0; i < n; i++, cfreq += freq_incr/1000) {
        // Print raw frequency data
				printf("Frequency: %d \n", cfreq);
				// Compute impedance

				double magnitude = sqrt(real_array[i]* real_array[i] + imag_array[i]* imag_array[i]);
				double phase = AD5933_calculate_phase(real_array[i], imag_array[i]) - sysphase[i];
				double impedance = 1/(magnitude*gain[i]);

				printf("Real: %d \n", real_array[i]);
				printf("Imag: %d \n", imag_array[i]);
 				printf("Phase: %lf \n", phase);
        printf("Resistance: %lf \n", arm_cos_f32(phase*M_PI / 180.0) * impedance);
				printf("Reactance: %lf \n",arm_sin_f32(phase*M_PI / 180.0) * impedance);

      }

	 }	
	
		free(real_array);
		free(imag_array);
	
		doUART();
    /* Close I2C1 */
    AD5933_deinit();

    while(1);
}

