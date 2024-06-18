# ch32v003 test

This MounRiver project example shows SysTick interrupt, GPIO, A2D&DMA, DMA interrupt, button reading,
Option area flash, persistent (non initialize) variables, Watchdog, UART debug printf and
PD1/SWIO as GPIO usage (commented in void GPIO_INIT(void)).

The following files are modified:
ch32v00x_flash.c & ch32v00x_flash.h (void FlashOptionData(uint8_t data0, uint8_t data1); and
void FlashOptionUser(uint16_t user); added)

Link.ld (section ".no_init" added)

 Pins usage (CH32V003F4P6_MINI EVB):
 PD7/nRST,
 PD6/Rx,
 PD5/Tx,
 PD4/LD3,
 PD0/LD1,
 PD1/SWIO/LD2,
 PC1/BTN,
 PA1/XTAL1,
 PA2/XTAL2,
 PD3/AN4,
 PD2/AN3,
 PC4/AN2.

The program constantly prints the value of three analog channels and the Vref channel and the DMA 
transfer counter. When you short press the button (up to 500ms), three bits of the button release 
counter are displayed on LD1..LD3. When the program starts, the reset counter saved in Data0 of the 
Option bytesis printed. Reset button if PD7 enabled supress whatchdog feeding and cause MCU reset.