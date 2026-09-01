#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

namespace vanjee {
namespace lidar {
namespace lidar_718 {
float Gs_ax[100], Gs_ax_sum;
uint8_t GS_ROW_ONE = 5;
void Get_GS_param(float varr) {
  uint8_t i = 0, l_mid = GS_ROW_ONE / 2;
  float l_temp = 0, l_varr = 0;
  Gs_ax_sum = 0;

  for (i = 0; i < GS_ROW_ONE; i++) {
    l_temp = (i - l_mid);
    l_temp = l_temp * l_temp;
    l_varr = varr * varr;
    l_temp = l_temp / l_varr;
    l_temp = 0 - (l_temp / 2);
    Gs_ax[i] = exp(l_temp);
    Gs_ax_sum += Gs_ax[i];
  }
}
uint8_t adnormal_point(uint16_t *lddata) {
  uint32_t sum = 0;
  uint16_t arvg = 0;
  for (int i = 0; i < GS_ROW_ONE; i++) {
    sum += lddata[i];
  }
  arvg = sum / GS_ROW_ONE;
  for (int i = 0; i < GS_ROW_ONE; i++) {
    if (abs(lddata[i] - arvg) > 100)
      return 1;
  }
  return 0;
}

static uint8_t GSparam_ensure = 0;
void GSFilter(uint16_t *lddata) {
  uint16_t x = 0, y = 0, mid = GS_ROW_ONE / 2, valid = 0;
  float l_sum = 0, temp;
  if (GSparam_ensure == 0) {
    GSparam_ensure = 1;
    Get_GS_param(30);
  }
  for (x = mid; x < 1440 - mid; x++) {
    l_sum = 0;
    valid = 1;
    for (y = x - mid; y < x + GS_ROW_ONE - mid; y++) {
      temp = lddata[y];
      if (temp < 20 || temp > 25000) {
        valid = 0;
        break;
      }
      temp = temp * Gs_ax[y + mid - x];
      l_sum += temp;
    }
    if (adnormal_point(&lddata[x - mid])) {
      valid = 0;
    }
    if (valid) {
      l_sum = l_sum / Gs_ax_sum;
      lddata[x] = l_sum;
    } else {
      lddata[x] = lddata[x];
    }
  }
  for (x = 0; x < mid; x++) {
    lddata[x] = lddata[x];
  }
  for (x = 1440 - mid; x < 1440; x++) {
    lddata[x] = lddata[x];
  }
}
}  // namespace lidar_718
}  // namespace lidar
}  // namespace vanjee
