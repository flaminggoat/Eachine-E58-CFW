#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t temp;
} Gyro;

typedef struct {
  uint8_t throttle;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
} Controls;

typedef struct {
  uint8_t a;
  uint8_t b;
  uint8_t c;
  uint8_t d;
} Motors;

void control_loop(const Gyro *gyro, const Controls *controls, Motors *motors);
