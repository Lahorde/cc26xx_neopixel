/******************************************************************************
 * @file    WS2812.h
 * @author  Rémi Pincent - INRIA
 * @date    10/04/2018
 *
 * @brief Neopixel WS2812 driver - http://cdn.sparkfun.com/datasheets/Components/LED/WS2812.pdf
 *
 * Project : cc26xx_neopixel
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

#ifndef WS2812__include
#define WS2812__include

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Global Functions Declarations
 **************************************************************************/
extern void WS2812_begin(uint8_t arg_u8_pin);
extern void WS2812_show(void);
extern void WS2812_setPin(uint8_t p);
extern void WS2812_setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* WS2812__include */
