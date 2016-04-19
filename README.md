# Description
WS2812 driver for CC26XX platforms

#Example code

Number of neo_pixel set using NB_PIXEL preoprocessor directive.

    WS2812_begin(Board_DP0);
    WS2812_setPixelColor(0, 0xFF, 0, 0);
    WS2812_setPixelColor(1, 0, 0xFF, 0);
    WS2812_setPixelColor(2, 0, 0, 0xFF);
    WS2812_show();