/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _NOZZLE_IDENTIFY_H_
#define _NOZZLE_IDENTIFY_H_

#include "device_base.h"
#include "../HAL/hal_adc.h"
#include "src/core/thermistor_table.h"

#define PT100_NOZZLE_TYPE_BASE_COUNT                (0)
#define NTC3950_NOZZLE_TYPE_BASE_COUNT              (20)
#define NTC3950_PULLUP_4K7_NOZZLE_TYPE_BASE_COUNT   (40)
#define INVALID_NOZZLE_TYPE_BASE_COUNT              (245)

#define NOZZLE_SUB_TYPE_0                           (0)
#define NOZZLE_SUB_TYPE_1                           (1)
#define NOZZLE_SUB_TYPE_2                           (2)
#define NOZZLE_SUB_TYPE_3                           (3)
#define NOZZLE_SUB_TYPE_4                           (4)
#define NOZZLE_SUB_TYPE_5                           (5)
#define NOZZLE_SUB_TYPE_6                           (6)
#define NOZZLE_SUB_TYPE_7                           (7)
#define NOZZLE_SUB_TYPE_8                           (8)
#define NOZZLE_SUB_TYPE_9                           (9)
#define NOZZLE_SUB_TYPE_MAX                         (10)

typedef struct {
  uint16_t min;
  uint16_t max;
}nozzle_adc_domain_t;

const nozzle_adc_domain_t pt100_nozzle_type_array[NOZZLE_SUB_TYPE_MAX] = {{.min = 143,  .max = 392},  \
                                                                       {.min = 483,  .max = 732},  \
                                                                       {.min = 866,  .max = 1114}, \
                                                                       {.min = 1259, .max = 1508}, \
                                                                       {.min = 1564, .max = 1813}, \
                                                                       {.min = 1923, .max = 2172}, \
                                                                       {.min = 2296, .max = 2546}, \
                                                                       {.min = 2660, .max = 2909}, \
                                                                       {.min = 2993, .max = 3242}, \
                                                                       {.min = 3598, .max = 3847}};

const nozzle_adc_domain_t ntc3950_nozzle_type_array[NOZZLE_SUB_TYPE_MAX] = {{.min = 235,    .max = 360},    \
                                                                         {.min = 96,     .max = 159},    \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}};

const nozzle_adc_domain_t pullup_4k7_ntc3950_nozzle_type_array[NOZZLE_SUB_TYPE_MAX] = {{.min = 505, .max = 754}, \
                                                                          {.min = 1108,   .max = 1357},    \
                                                                          {.min = 1745,   .max = 1995},    \
                                                                          {.min = 2277,   .max = 2526}, \
                                                                          {.min = 2991,   .max = 3241}, \
                                                                          {.min = 0xffff, .max = 0xffff}, \
                                                                          {.min = 0xffff, .max = 0xffff}, \
                                                                          {.min = 0xffff, .max = 0xffff}, \
                                                                          {.min = 0xffff, .max = 0xffff}, \
                                                                          {.min = 0xffff, .max = 0xffff}};

class NozzleIdentify {
 public:
  NozzleIdentify() {
    nozzle_type_base_count_ = INVALID_NOZZLE_TYPE_BASE_COUNT;
    nozzle_sub_type_ = NOZZLE_SUB_TYPE_MAX;
    adc_filter_count_ = 0;
    raw_adc_value_ = 0xffff;
  }
  uint8_t Init(uint8_t adc_pin, ADC_TIM_E adc_tim);
  void SetAdcIndex(uint8_t index) { adc_index_ = index; }
  void SetNozzleTypeCheckArray(thermistor_type_e type);
  uint8_t CheckNozzleSubType(uint16_t adc);
  uint8_t GetNozzleSubType();
  uint8_t GetNozzleTypeBase() {return nozzle_type_base_count_;}
  // void ReportNozzle(uint8_t nozzle);
  bool CheckLoop();
  static bool DirectlyConfirmTypeArrayBelong(const nozzle_adc_domain_t *array, uint16_t adc_val);

 private:
  uint8_t adc_index_;
  uint16_t raw_adc_value_;
  uint8_t nozzle_sub_type_;
  uint8_t adc_filter_count_;
  uint8_t nozzle_type_base_count_;
  const nozzle_adc_domain_t *nozzle_type_array_;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
