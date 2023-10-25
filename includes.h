#ifndef __INCLUDES_H
#define __INCLUDES_H

// Define CPU Frequency
// This must be defined, if __delay_ms()
// or __delay_us() functions are used in the code
#define _XTAL_FREQ   4000000      // Hz

// Include files
#include <xc.h>
#include <stdint.h>

#define T_MS 1
#define DATA_PIN GP5
#define TX_EN_PIN GP4
#define BTN1_PIN GP0
#define BTN2_PIN GP1
#define BTN3_PIN GP2
#define BTN4_PIN GP3

#define TOGGLE_DATA if(DATA_PIN == 1) DATA_PIN = 0; else DATA_PIN = 1
#define BIT_HIGH DATA_PIN = 1
#define BIT_LOW DATA_PIN = 0
#define ADDRESS 170


#endif