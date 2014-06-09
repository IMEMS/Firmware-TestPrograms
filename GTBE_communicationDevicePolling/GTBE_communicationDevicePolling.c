/*
 * GTBE_communicationDevicePolling.c
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"

// Tivaware
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"

// GTBE Lib
#include "tw_extension.h"


/********************
 * Global Variables *
 ********************/
// The control table used by the uDMA controller.  This table must be aligned to a 1024 byte boundary.
#pragma DATA_ALIGN(uDMAcontrolTable, 1024)
uint8_t uDMAcontrolTable[1024];

volatile unsigned char twe_g_QSSItxBuffer[TWE_QSSI_TX_BUFFER_LENGTH];
volatile unsigned char twe_g_UARTtxBufferPRI[TWE_UART_TX_BUFFER_LENGTH];
volatile unsigned char twe_g_UARTtxBufferALT[TWE_UART_TX_BUFFER_LENGTH];

uint32_t SysClkFreq;
uint32_t dataBuffer[1];
/*
typedef struct {
	union {
		float fn[1];
		uint32_t uint[1];
	} data;
	uint32_t address;
} floatFlash;
*/

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

	IntMasterEnable();
	//twe_initUDMAcontroller();
	//MAP_uDMAControlBaseSet(uDMAcontrolTable);

	// QSSI Communication
	co1
	(SysClkFreq, true);
	//twe_initQSSIuDMArx();

	// UART Communication
	twe_initUART(SysClkFreq, 4608000);

	while(1) {
		SSIDataPut(twe_QSSI_COMM_BASE,0x00);
		SSIDataGet(twe_QSSI_COMM_BASE, &dataBuffer[0]);
		UARTCharPut(TWE_UART_COMM_BASE,(unsigned char)dataBuffer[0]);
	}
}
