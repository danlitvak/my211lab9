#include <stdint.h>
#include <stdbool.h>
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
#define BOARD "DE1-SoC"
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

volatile pixel_t *pVGA = (pixel_t *)FPGA_PIXEL_BUF_BASE;

const pixel_t blk = 0x0000;
const pixel_t wht = 0xffff;
const pixel_t red = 0xf800;
const pixel_t grn = 0x07e0;
const pixel_t blu = 0x001f;

void delay(int N) {
  for (int i = 0; i < N; i++) {
    *pVGA; // read volatile memory location to waste time
  }
}

void drawPixel(int y, int x, pixel_t colour) { *(pVGA + (y << YSHIFT) + x) = colour; }

pixel_t readPixel(int y, int x) { return *(pVGA + (y << YSHIFT) + x); }

pixel_t makePixel(uint8_t r8, uint8_t g8, uint8_t b8) {
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

/* 7-seg encoding for 0-9 (active high). */
static const uint8_t HEX_DIGITS[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

void set_hex_digit(volatile int *base, int digit_idx, int value) {
  if (value < 0 || value > 9) return;
  const int shift = 8 * digit_idx;
  int current = *base;
  current &= ~(0xFF << shift);
  current |= (HEX_DIGITS[value] << shift);
  *base = current;
}

typedef enum { DIR_UP, DIR_RIGHT, DIR_DOWN, DIR_LEFT } Direction;

typedef struct {
  int x;
  int y;
  Direction dir;
  pixel_t colour;
  bool is_human;
} Player;

static const int BORDER = 1;
static const int TICK_DELAY = 1200;

Direction turn_left(Direction d) {
  switch (d) {
  case DIR_UP:
    return DIR_LEFT;
  case DIR_LEFT:
    return DIR_DOWN;
  case DIR_DOWN:
    return DIR_RIGHT;
  default:
    return DIR_UP;
  }
}

Direction turn_right(Direction d) {
  switch (d) {
  case DIR_UP:
    return DIR_RIGHT;
  case DIR_RIGHT:
    return DIR_DOWN;
  case DIR_DOWN:
    return DIR_LEFT;
  default:
    return DIR_UP;
  }
}

void draw_border(void) {
  rect(0, MAX_Y, 0, BORDER, wht);
  rect(0, MAX_Y, MAX_X - BORDER, MAX_X, wht);
  rect(0, BORDER, 0, MAX_X, wht);
  rect(MAX_Y - BORDER, MAX_Y, 0, MAX_X, wht);
}

void draw_obstacles(void) {
  const int cx = MAX_X / 2;
  const int cy = MAX_Y / 2;
  rect(cy - 10, cy + 10, cx - 2, cx + 2, wht);
  rect(cy - 2, cy + 2, cx - 20, cx + 20, wht);
}

bool out_of_bounds(int x, int y) { return x < 0 || x >= MAX_X || y < 0 || y >= MAX_Y; }

bool is_collision(int x, int y) {
  if (out_of_bounds(x, y)) return true;
  return readPixel(y, x) != blk;
}

void apply_human_input(Player *p, int keys) {
  if (!p->is_human) return;
  // Keys are active-low; invert bits to detect presses
  const int pressed = (~keys) & 0xF;
  if (pressed & 0x1) p->dir = DIR_UP;
  if (pressed & 0x2) p->dir = DIR_DOWN;
  if (pressed & 0x4) p->dir = DIR_LEFT;
  if (pressed & 0x8) p->dir = DIR_RIGHT;
}

Direction choose_robot_direction(Player *p) {
  const int dx[4] = {0, 1, 0, -1};
  const int dy[4] = {-1, 0, 1, 0};
  int nx1 = p->x + dx[p->dir];
  int ny1 = p->y + dy[p->dir];
  int nx2 = nx1 + dx[p->dir];
  int ny2 = ny1 + dy[p->dir];
  bool ahead_blocked = is_collision(nx1, ny1) || is_collision(nx2, ny2);
  if (!ahead_blocked) return p->dir;

  Direction left = turn_left(p->dir);
  int lx = p->x + dx[left];
  int ly = p->y + dy[left];
  if (!is_collision(lx, ly)) return left;

  Direction right = turn_right(p->dir);
  int rx = p->x + dx[right];
  int ry = p->y + dy[right];
  if (!is_collision(rx, ry)) return right;

  return p->dir;
}

bool step_player(Player *p) {
  const int dx[4] = {0, 1, 0, -1};
  const int dy[4] = {-1, 0, 1, 0};
  int nx = p->x + dx[p->dir];
  int ny = p->y + dy[p->dir];
  if (is_collision(nx, ny)) return false;
  drawPixel(ny, nx, p->colour);
  p->x = nx;
  p->y = ny;
  return true;
}

void reset_round(Player *human, Player *robot) {
  rect(0, MAX_Y, 0, MAX_X, blk);
  draw_border();
  draw_obstacles();

  human->x = MAX_X / 3;
  human->y = MAX_Y / 2;
  human->dir = DIR_RIGHT;

  robot->x = (2 * MAX_X) / 3;
  robot->y = MAX_Y / 2;
  robot->dir = DIR_LEFT;

  drawPixel(human->y, human->x, human->colour);
  drawPixel(robot->y, robot->x, robot->colour);
}

void fill_winner_screen(pixel_t colour) { rect(0, MAX_Y, 0, MAX_X, colour); }

int main(void) {
  volatile int *key_ptr = (int *)KEY_BASE;
  volatile int *hex0_ptr = (int *)HEX3_HEX0_BASE; // human score on HEX0
  volatile int *hex4_ptr = (int *)HEX5_HEX4_BASE; // robot score on HEX4

  Player human = {0};
  human.colour = blu;
  human.is_human = true;

  Player robot = {0};
  robot.colour = red;
  robot.is_human = false;

  int score_human = 0;
  int score_robot = 0;

  printf("Starting Tron on %s\n", BOARD);
  set_hex_digit(hex0_ptr, 0, score_human);
  set_hex_digit(hex4_ptr, 0, score_robot);

  while (score_human < 9 && score_robot < 9) {
    reset_round(&human, &robot);

    bool human_alive = true;
    bool robot_alive = true;

    while (human_alive && robot_alive) {
      int keys = *key_ptr;
      apply_human_input(&human, keys);
      robot.dir = choose_robot_direction(&robot);

      human_alive = step_player(&human);
      robot_alive = step_player(&robot);

      delay(TICK_DELAY);
    }

    if (human_alive != robot_alive) {
      if (human_alive) {
        score_human++;
      } else {
        score_robot++;
      }
      set_hex_digit(hex0_ptr, 0, score_human);
      set_hex_digit(hex4_ptr, 0, score_robot);
    }
  }

  if (score_human > score_robot) {
    fill_winner_screen(human.colour);
  } else {
    fill_winner_screen(robot.colour);
  }

  printf("Game over: H=%d R=%d\n", score_human, score_robot);
  while (1) {
    delay(100000);
  }
}