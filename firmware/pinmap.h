#ifndef PINMAP_H
#define PINMAP_H

#define BOOSTER_R_U 21
#define BOOSTER_R_R 20
#define BOOSTER_R_D 19
#define BOOSTER_R_L 18
#define BOOSTER_R_PRESS 17
#define BOOSTER_R_LED_1 22
#define BOOSTER_R_LED_2 26

#define BOOSTER_L_U 6
#define BOOSTER_L_R 7
#define BOOSTER_L_D 8
#define BOOSTER_L_L 9
#define BOOSTER_L_PRESS 10
#define BOOSTER_L_LED_1 27
#define BOOSTER_L_LED_2 28

#define BTN_START 11
#define BTN_START_LED_1 5
#define BTN_START_LED_2 4

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
