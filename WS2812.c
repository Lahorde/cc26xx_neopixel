/******************************************************************************
 * @file    WS2812.c
 * @author  Rémi Pincent - INRIA
 * @date    10/04/2016
 *
 * @brief WS2812 driver implementation on Texas Instruments CC26xx.
 * UDMA peripheral used to control WS2812.
 *
 * Project : WS2812_driver
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * refer https://github.com/Lahorde/cc26xx_neopixel.git
 *
 * LICENSE :
 * cc26xx_neopixel (c) by Rémi Pincent
 * cc26xx_neopixel is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/

/**************************************************************************
 * Include Files
 **************************************************************************/
#include "WS2812.h"

#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/BIOS.h>

#include <driverlib/ioc.h>
#include <driverlib/timer.h>

#include <ti/drivers/dma/UDMACC26XX.h>

#include <xdc/runtime/Types.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/
#ifndef NB_PIXELS
#define NB_PIXELS 3U
#endif

// 800kHz freq
#define NEOPIXEL_FREQ 800000U

//1 bit low level duration
#define NEOPIXEL_ZERO_L_NS 800U

//0 bit low level duration
#define NEOPIXEL_ZERO_H_NS 600U

#define NB_COlORS 3U

#define TIMER_0_A_PWM_MATCH_ADD (GPT0_BASE + GPT_O_TAMATCHR )

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Variables
 **************************************************************************/
static uint8_t _pixels[NB_PIXELS][NB_COlORS] = {0};
//42 x Bit Duration = T reset > 50µs between two transfer
//TODO : array members on 8 bits - DMA transfer from 8 bit
// type to 32 bit type
static uint32_t _pixelsDMA[NB_PIXELS*NB_COlORS*8 + 42] = {0};
static Hwi_Struct _hwi;
static UDMACC26XX_Handle      _udmaHandle;

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Local Functions Declarations
 **************************************************************************/
static void WS2812_gpioInit(uint8_t arg_u8_pin);
static void WS2812_timerInit(void);
static void WS2812_timerDeinit(void);
static void WS2812_udmaInit(void);
static void WS2812_udmaDeinit(void);
static void WS2812_fillDMAPixels(void);
static void WS2812_hwTimerAIT(UArg arg);
/**************************************************************************
 * Global Functions Defintions
 **************************************************************************/

/**************************************************************************
 * Local Functions Definitions
 **************************************************************************/

void WS2812_begin(uint8_t arg_u8_pin)
{
	WS2812_gpioInit(arg_u8_pin);

	//Register hw IT
	//TODO register it in timer init and destroy it after transfer
	Hwi_Params hwiParams;
	Hwi_Params_init(&hwiParams);
	hwiParams.enableInt = true;
	hwiParams.priority  = 0;
	//construct an HWI
	Hwi_construct(&_hwi, INT_TIMER0A, WS2812_hwTimerAIT, &hwiParams, NULL);
}
void WS2812_show(void)
{
	// Disable switching to standby mode as it switch off timer
	Power_setConstraint(Power_SB_DISALLOW);

	WS2812_udmaInit();
	WS2812_timerInit();
	WS2812_fillDMAPixels();

	// GOOOOOOOO!!!!!!!!
	TimerEnable(GPT0_BASE, TIMER_A);
}

void WS2812_setPin(uint8_t p)
{

}

void WS2812_setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{
	_pixels[n][0] = g;
	_pixels[n][1] = r;
	_pixels[n][2] = b;
}

/**
 * Initialize WS2812 data output pin
 */
void WS2812_gpioInit(uint8_t arg_u8_pin)
{
	// Configure pin as PWM output (PIn <-> IOC_PORT_MCU_PORT_EVENT0 corresponding to GPT0)
	IOCPortConfigureSet(arg_u8_pin, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_OUTPUT);
}

/**
 * Initialize PWM timer
 */
void WS2812_timerInit(void)
{
	Types_FreqHz loc_cpuFreq;
	BIOS_getCpuFreq(&loc_cpuFreq);

	// GPT not enabled by default Turn on PERIPH power domain and clock for GPT0
	Power_setDependency(PERIPH_GPT0);

	// be sure timer disabled before making any changes
	TimerDisable(GPT0_BASE, TIMER_A);

	// clear all timer ITs
	TimerIntClear(GPT0_BASE, TIMER_TIMA_DMA);
	TimerIntClear(GPT0_BASE, TIMER_TIMA_MATCH);
	TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
	TimerIntClear(GPT0_BASE, TIMER_CAPA_MATCH);
	TimerIntClear(GPT0_BASE, TIMER_TIMA_TIMEOUT);

	// disable all unneeded IT
	TimerIntDisable(GPT0_BASE, TIMER_TIMA_MATCH);
	TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
	TimerIntDisable(GPT0_BASE, TIMER_CAPA_MATCH);
	TimerIntDisable(GPT0_BASE, TIMER_TIMA_TIMEOUT);

	// Periodic timer
	//TimerConfigure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_PERIODIC);
	//PWM timer
	TimerConfigure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);

	// set the load value of timer-A
	TimerLoadSet(GPT0_BASE, TIMER_A, loc_cpuFreq.lo/NEOPIXEL_FREQ);
	//TimerMatchSet(GPT0_BASE, TIMER_A, test);

	//Enable IT on rising edge
	TimerEventControl(GPT0_BASE, TIMER_A, GPT_CTL_TAEVENT_POS);

	// enable DMA IT
	TimerIntEnable(GPT0_BASE, TIMER_TIMA_DMA);
	// Enable capture event - if not enabled some DMA transfer are done and
	// then IT not called whereas Capture Event IT set
	TimerIntEnable(GPT0_BASE, TIMER_CAPA_EVENT);

	// Enable DMA Trigger Enable
	HWREG(GPT0_BASE + GPT_O_DMAEV) |= GPT_DMAEV_CAEDMAEN;
}

void WS2812_timerDeinit(void)
{
	// be sure timer disabled before making any changes
	TimerDisable(GPT0_BASE, TIMER_A);

	// clear all timer ITs
	TimerIntClear(GPT0_BASE, TIMER_TIMA_DMA);
	TimerIntClear(GPT0_BASE, TIMER_TIMA_MATCH);
	TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
	TimerIntClear(GPT0_BASE, TIMER_CAPA_MATCH);
	TimerIntClear(GPT0_BASE, TIMER_TIMA_TIMEOUT);

	// disable all unneeded IT
	TimerIntDisable(GPT0_BASE, TIMER_TIMA_DMA);
	TimerIntDisable(GPT0_BASE, TIMER_TIMA_MATCH);
	TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
	TimerIntDisable(GPT0_BASE, TIMER_CAPA_MATCH);
	TimerIntDisable(GPT0_BASE, TIMER_TIMA_TIMEOUT);

	// destruct HWI
	//Hwi_destruct(&_hwi);

	// Disable DMA Trigger Enable
	HWREG(GPT0_BASE + GPT_O_DMAEV) &= ~GPT_DMAEV_CAEDMAEN;

	//Power_releaseDependency(PERIPH_GPT0);

	TimerDisable(GPT0_BASE, TIMER_A);
}

/**
 * Initialize DMA transfer
 */
void WS2812_udmaInit(void)
{
	// UDMACC26XX should only be used by SPI driver. Use it anyway as it configures power domain and control table
	_udmaHandle = UDMACC26XX_open();

	// Event fabric must be configured to route TIMER0_A events to UDMA channel 9 single request trigger
	// pas besoin de la fabrique???
	//EventRegister(EVENT_UDMA_PROG0, EVENT_TIMER0_A );

	//Be sure these attributes are disabled
	uDMAChannelAttributeDisable(UDMA0_BASE,
			UDMA_CHAN_TIMER0_A,
			UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
			UDMA_ATTR_HIGH_PRIORITY |
			UDMA_ATTR_REQMASK);

	//32 bit transfer - no incrementation on source and destination
	uDMAChannelControlSet(UDMA0_BASE,
			UDMA_CHAN_TIMER0_A | UDMA_PRI_SELECT,
			UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_NONE |
			UDMA_ARB_1);

	// give data to transfer
	uDMAChannelTransferSet(UDMA0_BASE,
			UDMA_CHAN_TIMER0_A | UDMA_PRI_SELECT,
			UDMA_MODE_BASIC,
			(void*)_pixelsDMA, (void*) TIMER_0_A_PWM_MATCH_ADD,
			sizeof(_pixelsDMA)/sizeof(_pixelsDMA[0]));

	// Clear DMA done flag
	// !! mask and not channel must be given as parameter
	UDMACC26XX_clearInterrupt(_udmaHandle, 1 << UDMA_CHAN_TIMER0_A);

	//Enable the DMA chanel
	// !! mask and not channel must be given as parameter
	UDMACC26XX_channelEnable(_udmaHandle, 1 << UDMA_CHAN_TIMER0_A);
}

void WS2812_udmaDeinit(void)
{
	// Clear DMA done flag
	// !! mask and not channel must be given as parameter
	UDMACC26XX_clearInterrupt(_udmaHandle, 1 << UDMA_CHAN_TIMER0_A);

	//Disable the DMA chanel
	// !! mask and not channel must be given as parameter
	UDMACC26XX_channelDisable(_udmaHandle, 1 << UDMA_CHAN_TIMER0_A);

	UDMACC26XX_close(_udmaHandle);
}

void WS2812_fillDMAPixels(void){
	uint8_t loc_u8_pixelIndex = 0;
	uint32_t loc_u32_bitIndex = 0;
	uint32_t loc_u32_tabIndex = 0;
	uint32_t loc_u8_colorIndex = 0;
	Types_FreqHz loc_cpuFreq;
	BIOS_getCpuFreq(&loc_cpuFreq);

	// TODO - could be defined in a preprocessor directive - check if some CPU FREQ define exist
	const uint32_t ZERO_LOW_NB_CYCLES = ((loc_cpuFreq.lo/1000000)*NEOPIXEL_ZERO_L_NS)/1000;
	const uint32_t ZERO_HIGH_NB_CYCLES = ((loc_cpuFreq.lo/1000000)*NEOPIXEL_ZERO_H_NS)/1000;
	// TODO -1 needed?
	const uint32_t RESET_NB_CYCLES = HWREG(GPT0_BASE + GPT_O_TAILR) - 1;

	// Fill PWM match value for each period corresponding to a 1/0 bit
	for(loc_u8_pixelIndex = 0; loc_u8_pixelIndex < NB_PIXELS; loc_u8_pixelIndex++)
	{
		for(loc_u8_colorIndex = 0; loc_u8_colorIndex < NB_COlORS; loc_u8_colorIndex++)
		{
			for(loc_u32_bitIndex = 0; loc_u32_bitIndex < 8; loc_u32_bitIndex++)
			{
				if((1 << loc_u32_bitIndex) &  _pixels[loc_u8_pixelIndex][loc_u8_colorIndex])
				{
					_pixelsDMA[loc_u32_tabIndex++] = ZERO_HIGH_NB_CYCLES;
				}
				else
				{
					_pixelsDMA[loc_u32_tabIndex++] = ZERO_LOW_NB_CYCLES;
				}
			}
		}
	}

	//Reset signal on all other periods
	while(loc_u32_tabIndex < sizeof(_pixelsDMA) / sizeof(_pixelsDMA[0]))
	{
		_pixelsDMA[loc_u32_tabIndex++] = RESET_NB_CYCLES;
	}
}

void WS2812_hwTimerAIT(UArg arg)
{
	// Clear Timer a event (rising edge)
	if(TimerIntStatus(GPT0_BASE, true) & TIMER_CAPA_EVENT)
		TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);

	// DMA transfer done
	if (UDMACC26XX_channelDone(_udmaHandle, 1 << UDMA_CHAN_TIMER0_A)) {

		//clear DMA req done flag
		HWREG(UDMA0_BASE+UDMA_O_REQDONE) |= 0x00000200; //clear DMA req done flag

		TimerIntClear(GPT0_BASE, TIMER_TIMA_DMA);

		//At the end of a complete μDMA transfer, the controller automatically disables the channel.

		WS2812_timerDeinit();
		WS2812_udmaDeinit();

		// Enable switching to standby mode as it switch off timer
		//Power_releaseConstraint(Power_SB_DISALLOW);
	}
	else
	{
		ASSERT(false);
	}
}
