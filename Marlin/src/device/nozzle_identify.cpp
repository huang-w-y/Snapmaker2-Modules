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

#include <stdint.h>
#include <src/core/can_bus.h>
#include <wirish.h>
#include "nozzle_identify.h"
#include "src/HAL/hal_adc.h"
#include "src/HAL/hal_flash.h"
#include "src/configuration.h"
#include "src/registry/registry.h"

uint8_t NozzleIdentify::Init(uint8_t adc_pin, ADC_TIM_E adc_tim) {
  adc_index_ = HAL_adc_init(adc_pin, adc_tim, ADC_PERIOD_DEFAULT);
  return adc_index_;
}

void NozzleIdentify::SetNozzleTypeCheckArray(thermistor_type_e type) {
  switch (type) {
    case THERMISTOR_NTC3950:
      nozzle_type_array_ = ntc3950_nozzle_type_array;
      nozzle_type_base_count_ = NTC3950_NOZZLE_TYPE_BASE_COUNT;
      break;
    case THERMISTOR_NTC3950_PULLUP_4K7:
      nozzle_type_array_ = pullup_4k7_ntc3950_nozzle_type_array;
      nozzle_type_base_count_ = NTC3950_PULLUP_4K7_NOZZLE_TYPE_BASE_COUNT;
      break;
    case THERMISTOR_PT100:
      nozzle_type_array_ = pt100_nozzle_type_array;
      nozzle_type_base_count_ = PT100_NOZZLE_TYPE_BASE_COUNT;
      break;
    default:
      break;
  }
}

uint8_t NozzleIdentify::CheckNozzleSubType(uint16_t adc) {
  uint8_t i;

  for (i = 0; i < NOZZLE_SUB_TYPE_MAX; i++) {
    if (adc >= nozzle_type_array_[i].min && adc <= nozzle_type_array_[i].max && 0xffff != nozzle_type_array_[i].min) {
      return i;
    }
  }

  return NOZZLE_SUB_TYPE_MAX;
}

uint8_t NozzleIdentify::GetNozzleSubType() {
  return nozzle_sub_type_;
}

// void NozzleIdentify::ReportNozzle(uint8_t nozzle) {
//   uint8_t buf[8];
//   uint8_t index = 0;

//   uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_NOZZLE_TYPE);
//   if (msgid != INVALID_VALUE) {
//     buf[index++] = nozzle;
//     // TODO
//     buf[index++] = (uint8_t)nozzle_sub_type_;
//     canbus_g.PushSendStandardData(msgid, buf, index);
//   }
// }

bool NozzleIdentify::CheckLoop() {
  uint16_t raw_adc_tmp;
  uint16_t raw_adc_diff = 0;

  if (nozzle_sub_type_ != NOZZLE_SUB_TYPE_MAX && INVALID_NOZZLE_TYPE_BASE_COUNT != nozzle_type_base_count_) {
    return true;
  }

  raw_adc_tmp = ADC_Get(adc_index_);

  if (raw_adc_tmp >= raw_adc_value_) {
    raw_adc_diff = raw_adc_tmp - raw_adc_value_;
  } else {
    raw_adc_diff = raw_adc_value_ - raw_adc_tmp;
  }

  if (raw_adc_diff > 20) {
    adc_filter_count_++;
  } else {
    adc_filter_count_ = 0;
    return false;
  }

  if (adc_filter_count_ == 10) {
    adc_filter_count_ = 0;
    raw_adc_value_ = raw_adc_tmp;
  } else {
    return false;
  }

  uint8_t sub_type = CheckNozzleSubType(raw_adc_value_);
  if (nozzle_sub_type_ != sub_type) {
    nozzle_sub_type_ = sub_type;
    return true;
  }

  return false;
}


bool NozzleIdentify::DirectlyConfirmTypeArrayBelong(const nozzle_adc_domain_t *array, uint16_t adc_val) {
  uint8_t i = 0;

  if (NULL == array) {
    return false;
  }

  for (i = 0; i < NOZZLE_SUB_TYPE_MAX; i++) {
    if (adc_val >= array[i].min && adc_val <= array[i].max && 0xffff != array[i].min ) {
      return true;
    }
  }

  return false;
}

