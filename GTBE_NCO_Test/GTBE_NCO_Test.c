/******************************************************************************
 *
 *  GTBE_mainTestDRDY.c - Full PLL test Program with hardware interface.  Uses the
 *   Georgia Tech Back-end interface board along with a Tiva C Launchpad
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *  rev1 March 2014
 *
 *  Originally written for the MRIG gyroscope project
 *
 *  GT BE Peripherals:
 *   ADC
 *    GPIO PB5
 *    SSI2
 *    M1PWM5
 *   DAC
 *    GPIO PE1 - PE3
 *    SSI0
 *    uDMA Software Channel
 *   Data Storage
 *    Flash Memory (see memory map below)
 *   Processing
 *    FPU
 *   Communications
 *    UART0
 *    uDMA
 *   RGB
 *    Timer0B
 *    Timer1A
 *    Timer1B
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_udma.h"
#include "inc/hw_gpio.h"
#include "inc/hw_flash.h"
#include "inc/hw_uart.h"

// Tivaware
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/flash.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"

// Launchpad Drivers
#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL
#include "examples/boards/ek-tm4c123gxl/drivers/rgb.h"
#endif

// CMSIS
#include "arm_math.h"
#include "math_helper.h"

// GTBE Lib
#include "dac_ad5754.h"
#include "adc_ads1278.h"
#include "tw_extension.h"
#include "swpll.h"

 // Library error routine
 #ifdef DEBUG
 void
 __error__(char *pcFilename, uint32_t ui32Line)
 {
 	while(1) {
 			//
 			// Hang on runtime error.
 			//
 	}
 }
 #endif


/*********
 * Modes *
 *********/
//#define UART_OUT_MODE
//#define QSSI_OUT_MODE
//#define DEBUG_THROUGH
//#define DEBUG_NCO

/**************
 * Parameters *
 **************/
#define DAC_ADDRESS_FORCER 		DAC_ADDR_A

// Flash Memory Map
#define FLASH_ADDR_CODE   0x0000 // 0x0000 - 0x2800 10KB Code (Protected - Execute Only)
#define FLASH_ADDR_SIG_I  0x2800 // 0x2800 - 0x4800  8KB gpADC_reading (Data Storage)
#define FLASH_ADDR_MSIG_I 0x4800 // 0x8800 - 0x6800  8KB gp_msig_I (Data Storage)
#define FLASH_ADDR_MSIG_Q 0x6800 // 0x6800 - 0x8800  8KB gp_msig_Q (Data Storage)
#define FLASH_ADDR_ERR    0x8800 // 0x8800 - 0xA800  8KB gp_err (Data Storage)
#define FLASH_ADDR_PHASE  0xA800 // 0xA800 - 0xB000  8KB gp_currPhase (Data Storage)
#define FLASH_ADDR_NCO_I  0xB000 // 0xB000 - 0xB800  8KB gp_nco_I (Data Storage)
#define FLASH_ADDR_NCO_Q  0xB800 // 0xB800 - 0xC000  8KB gp_nco_Q (Data Storage)

#define FLASH_SIZE_CODE   0x2800 // Size of each buffer
#define FLASH_SIZE_DATA   0x2000

#define FLASH_START_DATA  FLASH_ADDR_SIG_I // Start address of data in flash memory
#define FLASH_LENGTH_DATA 0x9800 // Total length of data
								 // = 0xC000 - 0x2800


/*********************
 * Hardware Settings *
 *********************/
#define BIN TWOS_COMPLEMENT // DAC data format in bipolar modes

 /*************
  * Constants *
  *************/

 #ifndef M_PI
 #define M_PI                    3.14159265358979323846
 #endif

/********************
 * Global Variables *
 ********************/
// The control table used by the uDMA controller.  This table must be aligned to a 1024 byte boundary.
#pragma DATA_ALIGN(uDMAcontrolTable, 1024)
uint8_t uDMAcontrolTable[1024];

//volatile bool g_dataProcessing = false;
volatile bool DAC_g_dataReady = false;

// ADC Buffers
volatile char ADC_g_dataBufferBytes[3];
volatile int32_t gADC_reading;

// DAC output value
int32_t g_output;
volatile uint8_t DAC_g_bufferPRI[3];
volatile uint8_t DAC_g_bufferALT[3];
volatile uint8_t DAC_g_bufferSel = DAC_BUFFER_SEL_ALT;
// Signal Gain
uint32_t A = 2;

/* DSP variables */

//PFD
floatFlash g_sig_I; //uses aliasing to store a float that can be accessed as an unsigned int
floatFlash g_msig_I;
floatFlash g_msig_Q;

floatFlash g_fsig_I;

// FIR Filter
uint32_t blockSize = BLOCK_SIZE;
/* 121 Taps - too Long
const float32_t firCoeffs32[NUM_TAPS] = {
0.0006830547454, 0.0007069055674, 0.0007422978673, 0.000790029838, 0.0008508535077, 0.0009254696142, 0.001014522694,  0.001118596418, 0.001238209203,  0.001373810118,  0.001525775132,
0.001694403697,  0.001879915716,  0.002082448903,  0.002302056559, 0.002538705765,  0.002792276037,  0.003062558419,  0.003349255047, 0.00365197918,   0.003970255708,  0.00430352213,
0.00465113001,   0.005012346894,  0.0053863587,    0.005772272552, 0.006169120058,  0.006575861016,  0.006991387523,  0.00741452848,  0.007844054463,  0.008278682938,  0.008717083793,
0.009157885164,  0.00959967952,   0.01004102998,   0.01048047682,  0.01091654416,   0.01134774676,   0.01177259695,   0.01218961154,  0.01259731883,   0.01299426559,   0.01337902392,
0.01375019813,   0.01410643139,   0.01444641227,   0.0147688811,   0.01507263604,   0.01535653896,   0.01561952094,   0.01586058754,  0.01607882362,   0.01627339785,   0.01644356679,
0.01658867855,   0.01670817598,   0.01680159946,   0.0168685891,   0.01690888657,   0.01692233637,   0.01690888657,   0.0168685891,   0.01680159946,   0.01670817598,   0.01658867855,
0.01644356679,   0.01627339785,   0.01607882362,   0.01586058754,  0.01561952094,   0.01535653896,   0.01507263604,   0.0147688811,   0.01444641227,   0.01410643139,   0.01375019813,
0.01337902392,   0.01299426559,   0.01259731883,   0.01218961154,  0.01177259695,   0.01134774676,   0.01091654416,   0.01048047682,  0.01004102998,   0.00959967952,   0.009157885164,
0.008717083793,  0.008278682938,  0.007844054463,  0.00741452848,  0.006991387523,  0.006575861016,  0.006169120058,  0.005772272552, 0.0053863587,    0.005012346894,  0.00465113001,
0.00430352213,   0.003970255708,  0.00365197918,   0.003349255047, 0.003062558419,  0.002792276037,  0.002538705765,  0.002302056559, 0.002082448903,  0.001879915716,  0.001694403697,
0.001525775132,  0.001373810118,  0.001238209203,  0.001118596418, 0.001014522694,  0.0009254696142, 0.0008508535077, 0.000790029838, 0.0007422978673, 0.0007069055674, 0.0006830547454
};


float32_t firCoeffs32[NUM_TAPS] = {
0.0006830547454, 0.0007069055674, 0.0007422978673, 0.000790029838, 0.0008508535077, 0.0009254696142, 0.001014522694,  0.001118596418, 0.001238209203,  0.001373810118,  0.001525775132,
0.001694403697,  0.001879915716,  0.002082448903,  0.002302056559
};

float32_t firCoeffs32[NUM_TAPS] = {
	0.01459655625, 0.03062761155, 0.07259853985, 0.1244798118, 0.1664539958, 0.1824869696, 0.1664539958, 0.1244798118, 0.07259853985, 0.03062761155, 0.01459655625
};
*/
/*
float32_t firCoeffs32[NUM_TAPS] = {
0.009776944552,0.0146385454,0.02838242752,0.04863375381,0.07189150947,0.09413405763,0.1115148508,0.1210279108,
0.1210279108,0.1115148508,0.09413405763,0.07189150947,0.04863375381,0.02838242752,0.0146385454,0.009776944552
};
*/

// 10 taps
//float32_t firCoeffs32[NUM_TAPS] = {
//	0.01619255261,0.03797749327,0.09314041809,0.1558717791,0.196817757,0.196817757,0.1558717791,0.09314041809,0.03797749327,0.01619255261
//};

// 8 taps
float32_t firCoeffs32[NUM_TAPS] = {
		0.02072401815,0.0655927317,0.1664143573,0.2472688929,0.2472688929,0.1664143573,0.0655927317,0.02072401815
};
//static float32_t firStateF32_I[BLOCK_SIZE + NUM_TAPS - 1]; //Declare State buffer of size (numTaps + blockSize - 1)
//static float32_t firStateF32_Q[BLOCK_SIZE + NUM_TAPS - 1];
static float32_t I_err[1]; //output of I FIR filter
static float32_t Q_err[1]; //output of Q FIR filter

// Custom FIR implementation
//float g_FIRoutput;


// Loop Gain and phase offsets
floatFlash g_err;
//uint32_t  gp_err = FLASH_ADDR_ERR; // Address to save data to in flash
//float32_t g_errAdj;
//float32_t g_K = K_GAIN;
//float32_t g_phaseOffset = PHASE_OFFSET;
// NCO
floatFlash g_currPhase;
//g_currPhase.address  = FLASH_ADDR_PHASE;
floatFlash g_nco_I;
floatFlash g_nco_Iprev;
floatFlash g_nco_Q;
//g_nco_I.address = FLASH_ADDR_NCO_I;
//g_nco_Q.address = FLASH_ADDR_NCO_Q;


//UART Communication
volatile uint32_t twe_g_UARTtxBufferIdx;
volatile unsigned char twe_g_UARTtxBufferPRI[TWE_UART_TX_BUFFER_LENGTH];
volatile unsigned char twe_g_UARTtxBufferALT[TWE_UART_TX_BUFFER_LENGTH];

volatile bool twe_g_UARTtxBufferSelectFlag = TWE_UART_TX_BUFFER_LENGTH;
//volatile int32_t twe_g_UARTCommand; // UART Command Handling

static float32_t phaseInc = (2 * M_PI * F_C) / FS;

/******************
 * GPIO Functions *
 ******************/

/**
 * An Interrupt handler which executes each time ~DRDY goes low.  The
 **/
void intHandlerDRDY(void) {
	uint32_t intStatus;

	//HWREG(TWE_PROCESSING_GPIO_BASE + GPIO_O_DATA + (TWE_PROCESSING_PIN <<2)) = TWE_PROCESSING_PIN; // Write high to GPIO PC7

	//intStatus = GPIOIntStatus(ADC_NDRDY_GPIO_BASE, true);
	intStatus = HWREG(ADC_NDRDY_GPIO_BASE + GPIO_O_MIS);
	//GPIOIntClear(GPIO_PORTB_BASE, intStatus);
	HWREG(ADC_NDRDY_GPIO_BASE + GPIO_O_ICR) = intStatus;
	if(intStatus == ADC_NDRDY_INT_PIN) {
		// Receive Each byte over SSI
		//MAP_SSIDataPut(ADC_SSI_BASE,0x00000000); // Byte 2
		//MAP_SSIDataPut(ADC_SSI_BASE,0x00000000); // Byte 1
		//MAP_SSIDataPut(ADC_SSI_BASE,0x00000000); // Byte 0
		HWREG(ADC_SSI_BASE + SSI_O_DR) = 0x00000000; // Byte 2
		HWREG(ADC_SSI_BASE + SSI_O_DR) = 0x00000000; // Byte 1
		HWREG(ADC_SSI_BASE + SSI_O_DR) = 0x00000000; // Byte 0

		// Transfer each byte to the buffers
		// Now using uDMA to transfer to the data buffer
		//MAP_SSIDataGet(ADC_SSI_BASE,&gADC_dataBufferByte2[0]);
		//MAP_SSIDataGet(ADC_SSI_BASE,&gADC_dataBufferByte1[0]);
		//MAP_SSIDataGet(ADC_SSI_BASE,&gADC_dataBufferByte0[0]);

		//MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX );
		HWREG(UDMA_ENASET) = 1 << (DAC_SSI_TX_UDMA_CHANNEL);

		// Update DAC output
		//DACd_updateDataDig(DAC_ADDR_A | DAC_ADDR_NONE_EH, gDAC_outputBuffer[0], 0x00000000);
		//uDMAChannelRequest(DAC_SSI_TX_UDMA_CHANNEL );
		HWREG(UDMA_SWREQ) = 1 << (DAC_SSI_TX_UDMA_CHANNEL);
	}

	//HWREG(GPIO_PORTC_BASE + GPIO_O_DATA + (GPIO_PIN_7 <<2)) = 0x00; // Write high to GPIO PC7
}


/******************
 * NVIC Functions *
 ******************/

/**
 * Sets the priority of the interrupts
 *  The ~DRDY sample ISR is given highest priority
 *  The UART RX ISR is given lower priority
 *  Smaller numbers correspond to higher interrupt priorities; priority 0 is the highest
 *  interrupt priority.
 **/
 void initIntPriority(void) {
	 // Highest Priority
	 MAP_IntPrioritySet(ADC_NDRDY_INT, 0x00);
	 MAP_IntPrioritySet(ADC_SSI_INT, 0x20);
	 MAP_IntPrioritySet(DAC_SSI_INT, 0x30);
	 //MAP_IntPrioritySet(UART_INT_RX, 0xE0);
	 //MAP_IntPrioritySet(UART_INT_TX, 0xE0);
	 // Lowest Priority
 }

 firInst g_firFilt_I;
 firInst*gp_firFilt_I = &g_firFilt_I;
 firInst g_firFilt_Q;
 firInst*gp_firFilt_Q = &g_firFilt_Q;

 int32_t g_test_inI[1024];
 float 	  g_test_F[1024];
 uint32_t g_test_i;

 uint32_t SysClkFreq;


/********
 * Main *
 ********/
int main(void) {
	/* System initialization */
	#ifdef PART_TM4C123GH6PM
		// Set system clock to 80 MHz (400MHz main PLL (divided by 5 - uses DIV400 bit)  [16MHz external xtal drives PLL]
		SysClkFreq = MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	#endif
	#ifdef PART_TM4C1294NCPDT
		// Set system clock to 120 MHz
		SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
	#endif

	twe_initFPU();
	IntMasterEnable();
	initIntPriority();
	twe_initUDMAcontroller();
	MAP_uDMAControlBaseSet(uDMAcontrolTable);
	twe_initProcessingIndicator();

	/* RGB: System Running Indicator - Green, Error - Red */
	#ifdef PART_TM4C123GH6PM // EK-TM4C123GXL
		twe_RGBInitSetGreen();
	#endif
	/* DAC initialization */
	DAC_initDACuDMA(DAC_RANGE_PM5V, DAC_PWR_PUA | DAC_PWR_PUB, SysClkFreq);
	//DAC_initTimersLDAC(true, false, 4);
	DAC_initSSIint();
	DAC_inituDMAautoSSI();


	/* ADC initialization */
	ADC_initADC(SysClkFreq);
	ADC_initDRDYint();
	ADC_initUDMAssiRX();

	/* Flash initialization */
	/*
	twe_initFlash(FLASH_ADDR_CODE, 0x0,
				  FLASH_START_DATA,FLASH_LENGTH_DATA); // Doesn't set protection but does erase the data
	g_sig_I.address = FLASH_ADDR_SIG_I;
	*/

	/* UART initialization */
	#ifdef UART_OUT_MODE
	twe_initUART(SysClkFreq, 4608000);
	twe_initUARTtxUDMA();
	#else
		#ifdef QSSI_OUT_MODE

		#endif
	#endif
	/* Control Loop init */
	// Initialize FIR Filter
	// Call FIR init function to initialize the instance structure
	//arm_fir_instance_f32 S_I;
	//arm_fir_instance_f32 S_Q;
	//arm_fir_init_f32(&S_I, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32_I[0], blockSize);
	//arm_fir_init_f32(&S_Q, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32_Q[0], blockSize);
	//float32_t  *inputF32_I, *outputF32_I;
	//float32_t  *inputF32_Q, *outputF32_Q;
	//Initialize input and output buffer pointers
	//inputF32_I  = &g_sig_I.data.fn[0];
	//outputF32_I = &I_err[0];
	//inputF32_Q  = &g_msig_Q.data.fn[0];
	//outputF32_Q = &Q_err[0];
	initFIR(gp_firFilt_I,firCoeffs32);
	initFIR(gp_firFilt_Q,firCoeffs32);
	volatile float acc;
	// Initialize data
	g_nco_Iprev.data.fn[0] = 1;

	SysCtlDelay(1000);

	uint32_t trashBin[1];
	while(SSIDataGetNonBlocking(DAC_SSI_BASE, &trashBin[0])) {
	}

	//bool uDMAstatusStart;
	//bool uDMAstatusEnd;
	//int32_t flashStatus;
	//uint32_t flashError = 0;
	// Process data between samples
	while(1) {

		// Collect Data
		if(DAC_g_dataReady == true) {

			DAC_g_dataReady = false;

			// Processing Indicator - Turn on
			//GPIOPinWrite(TWE_PROCESSING_GPIO_BASE, TWE_PROCESSING_PIN, TWE_PROCESSING_PIN);
			HWREG(TWE_PROCESSING_GPIO_BASE + GPIO_O_DATA + (TWE_PROCESSING_PIN <<2)) = TWE_PROCESSING_PIN; // Write high to GPIO PC7
			//g_dataProcessing = true;

			// Clear out DAC RX FIFO
			//while(SSIDataGetNonBlocking(DAC_SSI_BASE, &trashBin[0])) {
			//}
			/*
			trashBin[0] = HWREG(DAC_SSI_BASE + SSI_O_DR);
			trashBin[0] = HWREG(DAC_SSI_BASE + SSI_O_DR);
			trashBin[0] = HWREG(DAC_SSI_BASE + SSI_O_DR);
			*/


			// ADC output: MSB first (big endian input), MCU: Little Endian
			gADC_reading = ((((ADC_g_dataBufferBytes[UDMA_DATA_BUFFER_BYTE2]<<16) +
								 (ADC_g_dataBufferBytes[UDMA_DATA_BUFFER_BYTE1]<<8)  +
								  ADC_g_dataBufferBytes[UDMA_DATA_BUFFER_BYTE0])));

			// Sign Extension: Convert ADC reading from signed 24-bit to signed 32-bit  format
//			if((ADC_g_dataBuffer.uintn[0] &  0x00800000) == 0x00800000) {
//				ADC_g_dataBuffer.uintn[0] |= 0xFF000000; // Convert Negative values to 32-bit representation
//			}
			if((gADC_reading &  0x00800000) == 0x00800000) {
				gADC_reading |= 0xFF000000; // Convert Negative values to 32-bit representation
			}

//			g_sig_I.data.fn[0] = (float) ADC_g_dataBuffer.intn[0];
			g_sig_I.data.fn[0] = (float) gADC_reading;

			/*
			if(g_sig_I.address < 0x4800) {
				flashStatus = FlashProgram(g_sig_I.data.uintn,g_sig_I.address,4);
				g_sig_I.address += 4;
				if(flashStatus != 0) {
					flashError++;
				}

				HWREG(FLASH_FWBN + (g_sig_I.address & 0x7c)) = g_sig_I.data.uintn; // Write this word into the write buffer.
				// HWREG(FLASH_FMC2) = FLASH_FMC2_WRKEY | FLASH_FMC2_WRBUF; // Program the contents of the write buffer into flash.
				g_sig_I.address += 4;
			}
			*/

			/* Phase Detector */
			/*
			// PD Multipliers
			g_msig_I.data.fn[0] = g_sig_I.data.fn[0] * g_nco_I.data.fn[0];
			g_msig_Q.data.fn[0] = g_sig_I.data.fn[0] * g_nco_Q.data.fn[0];

			// PD FIR Filter
			//arm_fir_f32(&S_I, inputF32_I, outputF32_I, blockSize);
			//arm_fir_f32(&S_Q, inputF32_Q, outputF32_Q, blockSize);
			//g_FIRoutput = FIRfilter(gp_firFilt_I, g_sig_I.data.fn[0]);

			// Filter I
			acc = 0;
			// Update filter data buffer with a new data point
			*g_firFilt_I.dataHead = g_msig_I.data.fn[0];
			g_firFilt_I.dataHead++;
			if(g_firFilt_I.dataHead > g_firFilt_I.dataBufferEnd) {
				g_firFilt_I.dataHead = g_firFilt_I.dataBufferStart;
			}
			// Apply I Filter
			uint32_t i;
			for(i=0; i < NUM_TAPS; i++) {
				acc += (*(g_firFilt_I.pData[i])) * (g_firFilt_I.coeff[i]);
				g_firFilt_I.pData[i]++;
				if(g_firFilt_I.pData[i] > g_firFilt_I.dataBufferEnd) {
					g_firFilt_I.pData[i] = g_firFilt_I.dataBufferStart;
				}
			}

			I_err[0] = acc;
			g_fsig_I.data.fn[0] = acc;

			// Filter Q
			acc = 0;
			// Update filter data buffer with new data point
			*g_firFilt_Q.dataHead = g_msig_Q.data.fn[0];
			g_firFilt_Q.dataHead++;
			if(g_firFilt_Q.dataHead > g_firFilt_Q.dataBufferEnd) {
				g_firFilt_Q.dataHead = g_firFilt_Q.dataBufferStart;
			}
			// Apply Q Filter
			for(i=0; i < NUM_TAPS; i++) {
				acc += (*(g_firFilt_Q.pData[i])) * (g_firFilt_Q.coeff[i]);
				g_firFilt_Q.pData[i]++;
				if(g_firFilt_Q.pData[i] > g_firFilt_Q.dataBufferEnd) {
					g_firFilt_Q.pData[i] = g_firFilt_Q.dataBufferStart;
				}
			}

			Q_err[0] = acc;

			// Error Signal Calculations
			//g_err.data.fn[0] = atan2(Q_err[0],I_err[0]); // Take the 4 quadrant arctan, atan2(y,x) = atan(y/x)
			//g_err.data.fn[0] = atan2(4,1); // Takes too long

			//g_err.data.fn[0] = M_PI_DIV4 * Q_err[0]; // /I_err[0];
			//g_err.data.fn[0] = M_PI_DIV4 * (Q_err[0]/I_err[0]);

			if(I_err[0] > 0) {
				g_err.data.fn[0] = M_PI_DIV4 * (Q_err[0]/I_err[0]);
			}
			else if((Q_err[0] >= 0) && (I_err[0] < 0)) {
				g_err.data.fn[0] =  M_PI + M_PI_DIV4 * (Q_err[0]/I_err[0]);
			}
			else if((Q_err[0] < 0) && (I_err[0] < 0)) {
				g_err.data.fn[0] =  M_N_PI + M_PI_DIV4 * (Q_err[0]/I_err[0]);
			}
			else if((Q_err[0] > 0) && (I_err[0] == 0)) {
				g_err.data.fn[0] =  M_PI_DIV2;
			}
			else if((Q_err[0] < 0) && (I_err[0] == 0)) {
				g_err.data.fn[0] =  M_N_PI_DIV2;
			}

			//g_err.data.fn[0] = (I_err[0]/Q_err[0]);
			*/

			/* NCO */

			// Phase Accumulator
			#ifdef DEBUG_NCO
			g_currPhase.data.fn[0] = g_currPhase.data.fn[0] + phaseInc;
			#else
			//g_currPhase.data.fn[0] = g_currPhase.fn[0] + phaseInc + g_errAdj;
			g_currPhase.data.fn[0] = g_currPhase.data.fn[0] + phaseInc + g_err.data.fn[0];
			#endif
			// DDS
			g_nco_I.data.fn[0] = arm_cos_f32(g_currPhase.data.fn[0]);
			//g_nco_Q.data.fn[0] = arm_sin_f32(g_currPhase.data.fn[0]);
			g_nco_Q.data.fn[0] = (g_nco_I.data.fn[0] - g_nco_Iprev.data.fn[0]) * FREQ_RATIO_FS_FC;
			g_nco_Iprev.data.fn[0] = g_nco_I.data.fn[0];

			// Scale output
			//gDAC_outputBuffer[0] = Aout * ((uint32_t) g_nco_I.fn[0]);
//			g_output = A * ADC_g_dataBuffer.intn[0];

			#ifdef DEBUG_THROUGH
				g_output = A * gADC_reading;
			#else
				g_output = (int32_t) (A * 2 * 524288 * g_nco_I.data.fn[0]);
			#endif




			/* Update Output Buffers */
			if(DAC_g_bufferSel == DAC_BUFFER_SEL_PRI) {
				DAC_g_bufferSel = DAC_BUFFER_SEL_ALT; // flag to indicate that ALT output buffer
													  // should be sent to DAC

				// Set up the transfer parameters for the SW uDMA channel.  This will
				// configure the transfer buffers and the transfer size.
				//while(uDMAChannelModeGet(UDMA_CHANNEL_SSI0TX  | UDMA_PRI_SELECT) != UDMA_MODE_STOP) {

				//}
				//uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX  | UDMA_PRI_SELECT,
				//					   UDMA_MODE_AUTO,
				//					   (void *)DAC_g_bufferALT, (void *)(SSI0_BASE + SSI_O_DR),
				//					   3);
				HWREG(HWREG(UDMA_CTLBASE) + (DAC_SSI_TX_UDMA_CHANNEL << 4) + UDMA_O_SRCENDP) = (uint32_t)DAC_g_bufferALT + 3 - 1;
				HWREG(HWREG(UDMA_CTLBASE) + (DAC_SSI_TX_UDMA_CHANNEL << 4) + UDMA_O_CHCTL) |= (UDMA_CHCTL_XFERMODE_AUTO | (((3-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));

				//MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX );
				DAC_g_bufferALT[0] = DAC_ADDRESS_FORCER; // Input reg Command
				DAC_g_bufferALT[1] = (unsigned char)(g_output >> 16); // First data byte
				DAC_g_bufferALT[2] = (unsigned char)(g_output >> 8);  // Second data byte
			}
			else if(DAC_g_bufferSel == DAC_BUFFER_SEL_ALT) {
				DAC_g_bufferSel = DAC_BUFFER_SEL_PRI; // flag to indicate that ALT output buffer
													  // should be sent to DAC
				// Set up the transfer parameters for the SW uDMA channel.  This will
				// configure the transfer buffers and the transfer size.
				//while(uDMAChannelModeGet(UDMA_CHANNEL_SSI0TX  | UDMA_PRI_SELECT) != UDMA_MODE_STOP) {

				//}

				//uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX  | UDMA_PRI_SELECT,
				//					   UDMA_MODE_AUTO,
				//					   (void *)DAC_g_bufferPRI, (void *)(SSI0_BASE + SSI_O_DR),
				//					   3);
				HWREG(HWREG(UDMA_CTLBASE) + (DAC_SSI_TX_UDMA_CHANNEL << 4) + UDMA_O_SRCENDP) = (uint32_t)DAC_g_bufferPRI + 3 - 1;
				HWREG(HWREG(UDMA_CTLBASE) + (DAC_SSI_TX_UDMA_CHANNEL << 4) + UDMA_O_CHCTL) |= (UDMA_CHCTL_XFERMODE_AUTO | (((3-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));


				//MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX );
				DAC_g_bufferPRI[0] = DAC_ADDRESS_FORCER; // Input reg Command
				DAC_g_bufferPRI[1] = (unsigned char)(g_output >> 16); 		 // First data byte
				DAC_g_bufferPRI[2] = (unsigned char)(g_output >> 8);				 // Second data byte
			}

			// Processing Indicator - Turn off
			//GPIOPinWrite(TWE_PROCESSING_GPIO_BASE, TWE_PROCESSING_PIN, 0x00);
			HWREG(TWE_PROCESSING_GPIO_BASE + GPIO_O_DATA + (TWE_PROCESSING_PIN <<2)) = 0x00;
			//g_dataProcessing = false;

			//uDMAEnable();
			//SSIDMAEnable(SSI0_BASE, SSI_DMA_TX); // Enable SSI Tx uDMA
			//if(g_sig_I.address > 0x4800) {
				//while(1) {} // Program is complete when the flash is full.
			//}
			#ifdef UART_OUT_MODE
				if(twe_g_UARTtxBufferSelectFlag == TWE_UART_TX_BUFFER_PRI) {
					twe_g_UARTtxBufferPRI[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0] >> 24);
					twe_g_UARTtxBufferPRI[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0] >> 16);
					twe_g_UARTtxBufferPRI[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0] >> 8);
					twe_g_UARTtxBufferPRI[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0]);
				}
				else {
					twe_g_UARTtxBufferALT[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0] >> 24);
					twe_g_UARTtxBufferALT[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0] >> 16);
					twe_g_UARTtxBufferALT[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0] >> 8);
					twe_g_UARTtxBufferALT[twe_g_UARTtxBufferIdx++] = (unsigned char)(g_fsig_I.data.uintn[0]);
				}

				//Check to see if the UART data needs to be sent and send if necessary
				if(twe_g_UARTtxBufferIdx >= (TWE_UART_TX_BUFFER_PRI)) {

					twe_g_UARTtxBufferIdx = 0; // Update Index

					// Transmit output data
					//MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX );
					HWREG(UDMA_ENASET) = 1 << (TWE_UART_COMM_UDMA_CHANNEL);
					//SysCtlDelay(1);
					//uDMAChannelRequest(UDMA_CHANNEL_UART0TX );
					HWREG(UDMA_SWREQ) = 1 << (TWE_UART_COMM_UDMA_CHANNEL);
					SysCtlDelay(1);
					if(twe_g_UARTtxBufferSelectFlag) {
						twe_g_UARTtxBufferSelectFlag = TWE_UART_TX_BUFFER_ALT;
						//uDMAChannelTransferSet(UDMA_CHANNEL_UART0TX  | UDMA_PRI_SELECT,
						//					   UDMA_MODE_AUTO,
						//					   (void *)twe_g_UARTtxBufferALT, (void *)(UART0_BASE + UART_O_DR),
						//					   TWE_UART_TX_BUFFER_LENGTH);
						HWREG(HWREG(UDMA_CTLBASE) + (TWE_UART_COMM_UDMA_CHANNEL << 4) + UDMA_O_SRCENDP) = (uint32_t)twe_g_UARTtxBufferALT + TWE_UART_TX_BUFFER_LENGTH - 1;
						HWREG(HWREG(UDMA_CTLBASE) + (TWE_UART_COMM_UDMA_CHANNEL << 4) + UDMA_O_CHCTL) |= (UDMA_CHCTL_XFERMODE_AUTO | (((TWE_UART_TX_BUFFER_LENGTH-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));
					}
					else {
						twe_g_UARTtxBufferSelectFlag = TWE_UART_TX_BUFFER_PRI;
						//uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX  | UDMA_PRI_SELECT,
						//					   UDMA_MODE_AUTO,
						//					   (void *)twe_g_UARTtxBufferPRI, (void *)(SSI0_BASE + SSI_O_DR),
						//					   3);
						HWREG(HWREG(UDMA_CTLBASE) + (TWE_UART_COMM_UDMA_CHANNEL << 4) + UDMA_O_SRCENDP) = (uint32_t)twe_g_UARTtxBufferPRI + TWE_UART_TX_BUFFER_LENGTH - 1;
						HWREG(HWREG(UDMA_CTLBASE) + (TWE_UART_COMM_UDMA_CHANNEL << 4) + UDMA_O_CHCTL) |= (UDMA_CHCTL_XFERMODE_AUTO | (((TWE_UART_TX_BUFFER_LENGTH-1) << UDMA_CHCTL_XFERSIZE_S) & UDMA_CHCTL_XFERSIZE_M));
					}
	//				ADC_g_dataBuffer.uintn[0] = 0x00000000;
					gADC_reading = 0x00000000;
				}
			#endif
		}
	}
}
