#ifndef PINMAP_H
#define PINMAP_H

#define BOOSTER_R_U 22
#define BOOSTER_R_R 21
#define BOOSTER_R_D 20
#define BOOSTER_R_L 19
#define BOOSTER_R_PRESS 18
#define BOOSTER_R_LED 17

#define BOOSTER_L_U 4
#define BOOSTER_L_R 5
#define BOOSTER_L_D 6
#define BOOSTER_L_L 7
#define BOOSTER_L_LED 8
#define BOOSTER_L_PRESS 9

#define BTN_START 10
#define BTN_START_LED 11

#define BOOSTER_L_WS2812_DIN 15
#define BOOSTER_L_NUM_WS2812 10
#define BOOSTER_R_WS2812_DIN 16
#define BOOSTER_R_NUM_WS2812 10

#define NUM_BTN 7
#define NUM_DPAD 4
#define NUM_WS2812 16

/*
 * Useful if we need to see where we are in code, but are in a time critical
 * situation where using a UART is too slow.
 */
#define PIN_DBG_0 12
#define PIN_DBG_1 13
#define PIN_DBG_2 14

#endif
