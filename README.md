# Description
WS2812 driver for CC26XX platforms using SPI UDMA driver.

# Prerequisities

* Add in Board.h Pin corresponding to used SPI MISO i.e. Neopixel data pin
* set Number of neo_pixels set using NB_PIXEL preoprocessor directive

#Example code

Number of neo_pixel set using NB_PIXEL preoprocessor directive.

    WS2812_begin(Board_DP0);
    WS2812_setPixelColor(0, 0xFF, 0, 0);
    WS2812_setPixelColor(1, 0, 0xFF, 0);
    WS2812_setPixelColor(2, 0, 0, 0xFF);
    WS2812_show();
    
# References

Issues on first implementation using GPT configured as PWM and UDMA :

[ UDMA SPEED - 1 ] (https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/541648)
[ UDMA SPEED - 2 ] (https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/542169)