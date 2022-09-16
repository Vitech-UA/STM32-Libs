// vim: set ai et ts=4 sw=4:

#ifndef _SI5351_H_
#define _SI5351_H_

#include "stdint.h"
typedef enum {
    SI5351_PLL_A = 0,
    SI5351_PLL_B,
} si5351PLL_t;

typedef enum {
    SI5351_R_DIV_1   = 0,
    SI5351_R_DIV_2   = 1,
    SI5351_R_DIV_4   = 2,
    SI5351_R_DIV_8   = 3,
    SI5351_R_DIV_16  = 4,
    SI5351_R_DIV_32  = 5,
    SI5351_R_DIV_64  = 6,
    SI5351_R_DIV_128 = 7,
} si5351RDiv_t;

typedef enum {
    SI5351_DRIVE_STRENGTH_2MA = 0x00, //  ~ 2.2 dBm
    SI5351_DRIVE_STRENGTH_4MA = 0x01, //  ~ 7.5 dBm
    SI5351_DRIVE_STRENGTH_6MA = 0x02, //  ~ 9.5 dBm
    SI5351_DRIVE_STRENGTH_8MA = 0x03, // ~ 10.7 dBm
} si5351DriveStrength_t;

typedef struct {
    int32_t mult;
    int32_t num;
    int32_t denom;
} si5351PLLConfig_t;

typedef struct {
    uint8_t allowIntegerMode;
    int32_t div;
    int32_t num;
    int32_t denom;
    si5351RDiv_t rdiv;
} si5351OutputConfig_t;


void si5351_Init(int32_t correction);
void si5351_SetupCLK0(int32_t Fclk, si5351DriveStrength_t driveStrength);
void si5351_SetupCLK2(int32_t Fclk, si5351DriveStrength_t driveStrength);
void si5351_EnableOutputs(uint8_t enabled);

/*
 * Advanced interface. Use it if you need:
 *
 * a. CLK0, CLK1 and CLK2 simultaneously;
 * b. A phase shift 90° between two channels;
 *
 * si5351_Calc() always uses 900 MHz PLL for frequencies below 81 MHz.
 * This PLL can safely be shared between all CLKx that work @ <= 81 MHz.
 * You can also modify si5351.c to share one PLL for any frequencies <= 112.5 MHz,
 * however this will increase the worse case calculation error to 13 Hz.
 */
void si5351_Calc(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf);

/*
 * si5351_CalcIQ() finds PLL and MS parameters that give phase shift 90° between two channels,
 * if 0 and (uint8_t)out_conf.div are passed as phaseOffset for these channels. Channels should
 * use the same PLL to make it work.
 */
void si5351_CalcIQ(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf);

void si5351_SetupPLL(si5351PLL_t pll, si5351PLLConfig_t* conf);
int si5351_SetupOutput(uint8_t output, si5351PLL_t pllSource, si5351DriveStrength_t driveStength, si5351OutputConfig_t* conf, uint8_t phaseOffset);

#endif
