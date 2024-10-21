/*
 * File:   main.c
 * Author: johni
 *
 * Created on 17 октября 2023 г., 19:38
 */

// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-Up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

#include "includes.h"

/* Pin configuration
 * 
 * GP0 = Button 1
 * GP1 = Button 2
 * GP2 = DATA
 */

__EEPROM_DATA(
        0x00, 0x03, // version
        0xFF, 0xFF,
        0xFF, 0xFF,
        0xFF, 0xFF
        );

void interrupt globalInterrupt() {
    if (GPIF) {
        GPIO = 0;
        GPIF = 0;
    }
}

uint8_t crcByte(uint8_t crc, uint8_t data) {
    uint8_t i = 8;
    while (i--) {
        crc = ((crc ^ data) & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
        data >>= 1;
    }
    return crc;
}

uint8_t crc8(uint8_t buffer[], uint8_t size) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < size; i++) crc = crcByte(crc, buffer[i]);
    return crc;
}

uint8_t crcXOR(uint8_t *buffer, uint8_t size) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < size; i++) crc ^= buffer[i];
    return crc;
}

void send_bit(uint8_t v) {
    if (v > 0) {
        BIT_HIGH;
        __delay_ms(T_MS);
        BIT_LOW;
        __delay_ms(2 * T_MS);
    } else {
        BIT_HIGH;
        __delay_ms(2 * T_MS);
        BIT_LOW;
        __delay_ms(T_MS);
    }
}

void send_stop_bit() {
    BIT_HIGH;
    __delay_ms(3 * T_MS);
    BIT_LOW;
}

void send_byte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        send_bit((data << i) & 0x80);
    }
}

void send(uint8_t *buffer, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        send_byte(buffer[i]);
    }
}

void repeat(uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        send_byte(0xAA);
    }
}

void main(void) {
#ifdef _12F675
    ANSEL = 0x00; // Set ports as digital I/O, not analog input
    ADCON0 = 0x00; // Shut off the A/D Converter
#endif     
    CMCON = 0x07; // Shut off the Comparator
    VRCON = 0x00; // Shut off the Voltage Reference

    PEIE = 0; // Peref interrupt disable
    INTE = 0;

    GPIO = 0;
    TRISIO = 0b00111001;
    nGPPU = 0; // enable WPU
    WPU = 0b00111001; // setup WPU
    IOCB = 0b00110000; // setup GPIO interrupts
    GPIE = 1; // enable GPIO interrupt

    while (1) {
        GIE = 1; // Global interrupt enable

        SLEEP();

        GIE = 0;

        uint8_t cmd = 0;

        __delay_ms(50);
        if (!BTN1_PIN)cmd = 1;
        if (!BTN2_PIN)cmd = 2;
        if (!BTN3_PIN)cmd = 3;
        if (!BTN4_PIN)cmd = 4;

        while (!BTN1_PIN || !BTN2_PIN || !BTN3_PIN || !BTN4_PIN) __delay_ms(100);

        if (cmd > 0) {
            uint8_t data[3];
            data[0] = ADDRESS;
            data[1] = cmd;
            uint8_t crc = crc8(data, 2);
            data[2] = crc;

            uint8_t cnt = 3; // about 1.5 sec
            TX_EN_PIN = 1; // enable TX
            __delay_ms(500);

            while (cnt > 0) {

                // preambule
                repeat(10);

                // start byte
                send_byte(0x0F);

                // data
                send(data, 3);

                // stop bit
                send_stop_bit();

                __delay_ms(100);
                cnt--;
            }

            TX_EN_PIN = 0; // disable TX
        }

        GPIO = 0;
        GPIF = 0;

    }

}

