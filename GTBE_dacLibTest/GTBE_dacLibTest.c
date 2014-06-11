/********************************************************************************
 * GTBE_dacLibTest.c
 *  Tests out the functionality of the ADI ad5754 driver library
 *  Utilizes the Georgia Tech Backend (GTBE) Hardware
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  February 2014
 *  
 *  GT BE Peripherals:
 *   DAC: AD5754
 *    INPUTS
 *     ~LDAC_Forcer -> GPIO PE1
 *     ~LDAC_Quad   -> GPIO PE2
 *     ~CLR         -> GPIO PE3
 *	   SERIAL
 *     SCLK         -> SPI0_CLK: PA2
 *     ~SYNC		-> SPI0_FSS: PA3 (CS)
 *     SDO		    -> SPI0_RX:  PA4 (MISO)
 *     SDIN         -> SPI0_TX:  PA5 (MOSI)
 *    OUTPUTS
 *     DAC A 		-> DACout_D
 *     DAC B		-> DACout_C
 *     DAC C		-> QUADRATURE_CONTROL_2_L
 *     DAC D		-> QUADRATURE_CONTROL_2_H
 *     DAC E 		-> QUADRATURE_CONTROL_1_L
 *     DAC F 		-> QUADRATURE_CONTROL_1_H
 *     DAC G 		-> ANTINODE_FORCER_2_H
 *     DAC H 		-> NODE_FORCER_2_H
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ********************************************************************************/

#define TARGET_IS_BLIZZARD_RB1
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
//#include "examples/boards/ek-tm4c123gxl/drivers/rgb.c"
#include "driverlib/rom.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"
//#include "driverlib/udma.h"
#include "dac_ad5754.h"

#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif

/**************
 * Prototypes *
 **************/
void Timer2IntHandlerDACout(void);

/*********************
 * Hardware Settings *
 *********************/
#define BIN OFFSET_BINARY // DAC data format in bipolar modes

#define NUM_SAMPLES 6250 // 8Hz at 50ksps
float gSineSamples[NUM_SAMPLES];
uint32_t gSampleNumOut = 0;
/*****************
 * FPU Functions *
 *****************/
// The FPU is used to calculate digital output from the voltage

void setupFPU(void) {
	FPULazyStackingEnable();
	FPUEnable();
}

/*******************
 * Timer Functions *
 *******************/

/**
 * Initialize DAC update timer
 **/
void initTimer(void) {
	uint32_t timerCount = 0;
	//uint32_t clk = 0;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Full-width periodic timer

	// Set the timer period to 250ms

	//clk = SysCtlClockGet();
	//timerCount = (SysCtlClockGet()) /12500000;
	timerCount = SysCtlClockGet()/50000;
	TimerLoadSet(TIMER2_BASE, TIMER_A, timerCount -1);

	//Start Timer Interrupt
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	TimerEnable(TIMER2_BASE, TIMER_A);
}

// Timer Interrupt handler
// Be sure that the interrupt is added to the NVIC by adding it to the "Timer 2 subtimer A” location in tm4c123gh6pm_startup_CCS.c
void Timer2IntHandlerDACout(void) {
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	DACd_updateDataDig(DAC_ADDR_A | DAC_ADDR_NONE_EH, (uint32_t) (32768*(gSineSamples[gSampleNumOut]+1)),0x00000000);

	ROM_SysCtlDelay(1000);
	DAC_loadDACsPin();

	gSampleNumOut++;
	if(gSampleNumOut == NUM_SAMPLES) {
		gSampleNumOut = 0;
	}
}

void main() {
	// Set system clock to 50 MHz (400MHz main PLL (predivided by 2 and divide by 4 sysdiv)  [16MHz external xtal drives PLL]
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	setupFPU();

   /*******************************
	* Stand Alone Operation Tests *
	*******************************/

	DAC_initDAC(DAC_RANGE_PM5V, DAC_PWR_PUA | DAC_PWR_PUB);

   /**
	* Test 1: Write a digital value to DAC A and load it using the serial
	* command.
	*
	* Result: Should see +2.5v on DAC A (GTBE DACout_D)
	**/
	DAC_updateDataDig(DAC_ADDR_A, 49152);
	ROM_SysCtlDelay(4000);
	DAC_loadDACs();

   /**
	* Test 2: Write a voltage value of 1.2v to DAC B and load it using the serial
	* command.
	*
	* Result: Should see +1.2v on DAC B (GTBE DACout_C).
	**/
	DAC_updateDataVolt(DAC_ADDR_B, DAC_RANGE_PM5V, BIN, 1.2);
	SysCtlDelay(1500);
	DAC_loadDACsPin();

   /*******************************
	* Daisy Chain Operation Tests *
	*******************************/
	DACd_initDAC(DAC_RANGE_PM5V  | DAC_RANGE_PM5V_EH,
				 DAC_PWR_SKIP_AD | DAC_PWR_PUALL_EH);

   /**
	* Test 3: Test Daisy chain operation by writing to DAC E
	*
	* Result: Should see +2.5v on DAC E (QUADRATURE_CONTROL_1_L).
	**/
	ROM_SysCtlDelay(4000);
	DACd_updateDataDig(DAC_ADDR_NONE_AD | DAC_ADDR_E, 0, 49152);
	ROM_SysCtlDelay(4000);
	DACd_loadDACs_EH();

	/**
	* Test 4: Test Daisy chain operation by writing to DAC F
	*
	* Result: Should see -1.2v on DAC F (QUADRATURE_CONTROL_1_H).
	**/
	ROM_SysCtlDelay(4000);
	DACd_updateDataVolt(DAC_ADDR_NONE_AD  | DAC_ADDR_F,
						DAC_RANGE_SKIP_AD | DAC_RANGE_PM5V_EH,
						BIN, BIN,
			 	 	 	0, -1.2);
	DACd_loadDACsPin_EH();

	/**
	* Test 5: Test Daisy chain operation by clearing DAC G
	*
	* Result: Should see 0v on DAC G (ANTINODE_FORCER_2_H).
	* 		  Clears all DACs E-H
	**/
	ROM_SysCtlDelay(4000);
	DACd_updateDataVolt(DAC_ADDR_NONE_AD  | DAC_ADDR_G,
						DAC_RANGE_SKIP_AD | DAC_RANGE_PM5V_EH,
						BIN, BIN,
						0, 2);
	ROM_SysCtlDelay(3000);
	DACd_loadDACsPin_EH();
	ROM_SysCtlDelay(3000);
	DACd_clearDACs_EH();

	/**
	* Test 5: Test Daisy chain operation by clearing all DACs
	*
	* Result: Should see 0v on DAC H (NODE_FORCER_2_H).
	* 		  Clears all DACs A-H
	**/
	ROM_SysCtlDelay(3000);
	DACd_updateDataVolt(DAC_ADDR_NONE_AD  | DAC_ADDR_H,
						DAC_RANGE_SKIP_AD | DAC_RANGE_PM5V_EH,
						BIN, BIN,
						0, 2);
	DAC_clearDACsPin();

	// Generate a sine wave to output

	// amplitude A = 2.5v
	// frequency f = 5kHz
	float fRadians = ((2 * M_PI) / NUM_SAMPLES);
	uint32_t dataCount = 0;
    while(dataCount < NUM_SAMPLES)
    {
    	gSineSamples[dataCount] = sinf(fRadians * dataCount);

    	dataCount++;
    }
	initTimer();
	// Wait at end of program
	while(1) {

	}
}
