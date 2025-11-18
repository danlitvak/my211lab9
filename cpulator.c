#include <unistd.h>

#include <stdio.h>

// if using CPUlator, you should copy+paste contents of the file below instead of using #include
// #include "address_map_niosv.h"

/*******************************************************************************
 * This file provides address values that exist in the DE10-Lite Computer
 * This file also works for DE1-SoC, except change #define DE10LITE to 0
 ******************************************************************************/

#ifndef __SYSTEM_INFO__
#define __SYSTEM_INFO__

#define DE10LITE 0 // change to 0 for CPUlator or DE1-SoC, 1 for DE10-Lite

/* do not change anything after this line */

#if DE10LITE
#define BOARD "DE10-Lite"
#define MAX_X 160
#define MAX_Y 120
#define YSHIFT 8
#else
#define MAX_X 320
#define MAX_Y 240
#define YSHIFT 9
#endif

/* Memory */
#define SDRAM_BASE 0x00000000
#define SDRAM_END 0x03FFFFFF
#define FPGA_PIXEL_BUF_BASE 0x08000000
#define FPGA_PIXEL_BUF_END 0x0800FFFF
#define FPGA_CHAR_BASE 0x09000000
#define FPGA_CHAR_END 0x09001FFF

/* Devices */
#define LED_BASE 0xFF200000
#define LEDR_BASE 0xFF200000
#define HEX3_HEX0_BASE 0xFF200020
#define HEX5_HEX4_BASE 0xFF200030
#define SW_BASE 0xFF200040
#define KEY_BASE 0xFF200050
#define JP1_BASE 0xFF200060
#define ARDUINO_GPIO 0xFF200100
#define ARDUINO_RESET_N 0xFF200110
#define JTAG_UART_BASE 0xFF201000
#define TIMER_BASE 0xFF202000
#define TIMER_2_BASE 0xFF202020
#define MTIMER_BASE 0xFF202100
#define RGB_RESAMPLER_BASE 0xFF203010
#define PIXEL_BUF_CTRL_BASE 0xFF203020
#define CHAR_BUF_CTRL_BASE 0xFF203030
#define ADC_BASE 0xFF204000
#define ACCELEROMETER_BASE 0xFF204020

/* Nios V memory-mapped registers */
#define MTIME_BASE 0xFF202100

#endif

typedef uint16_t pixel_t;

volatile pixel_t * pVGA = (pixel_t * ) FPGA_PIXEL_BUF_BASE;

const pixel_t blk = 0x0000;
const pixel_t wht = 0xffff;
const pixel_t red = 0xf800;
const pixel_t grn = 0x07e0;
const pixel_t blu = 0x001f;

void delay(int N) {
  for (int i = 0; i < N; i++)
    *
    pVGA; // read volatile memory location to waste time
}

/* STARTER CODE BELOW. FEEL FREE TO DELETE IT AND START OVER */

void drawPixel(int y, int x, pixel_t colour) {
  *(pVGA + (y << YSHIFT) + x) = colour;
}

pixel_t makePixel(uint8_t r8, uint8_t g8, uint8_t b8) {
  // inputs: 8b of each: red, green, blue
  const uint16_t r5 = (r8 & 0xf8) >> 3; // keep 5b red
  const uint16_t g6 = (g8 & 0xfc) >> 2; // keep 6b green
  const uint16_t b5 = (b8 & 0xf8) >> 3; // keep 5b blue
  return (pixel_t)((r5 << 11) | (g6 << 5) | b5);
}

void rect(int y1, int y2, int x1, int x2, pixel_t c) {
  for (int y = y1; y < y2; y++)
    for (int x = x1; x < x2; x++)
      drawPixel(y, x, c);
}

int main() {

  pixel_t colour;

  const int half_y = MAX_Y / 2;
  const int half_x = MAX_X / 2;

  printf("start\n");
  rect(0, MAX_Y, 0, MAX_X, blk);

  for (int y = 0; y < half_y; y++) {
    for (int x = 0; x < half_x; x++) {
      // red, green, blue and white colour bars
      const uint32_t scale = 256 * x / half_x; // scale = 0..255
      colour = makePixel(0, 0, scale);
      drawPixel(y, x, colour);

      colour = makePixel(0, scale, 0);
      drawPixel(y, x + half_x, colour);

      colour = makePixel(scale, 0, 0);
      drawPixel(y + half_y, x, colour);

      colour = makePixel(scale, scale, scale);
      drawPixel(y + half_y, x + half_x, colour);

      delay(1000);
    }

    printf("drew row: %i\n", y);
  }
  printf("done\n");
}