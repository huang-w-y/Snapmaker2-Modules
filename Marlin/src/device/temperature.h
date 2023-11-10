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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_

#include <src/core/pid.h>
#include "device_base.h"
#include "../HAL/hal_adc.h"
#include "src/HAL/hal_pwm.h"
#include "src/core/thermistor_table.h"

class Temperature {
 public:
  Temperature() {
    thermistor_type_ = THERMISTOR_NTC3950;
    is_temp_ready_ = false;
  }
  static uint8_t TempertuerStatus();
  void SetAdcIndex(uint8_t index) { adc_index_ = index; }
  void SetThermistorType(thermistor_type_e type = THERMISTOR_NTC3950) { thermistor_type_ = type; }
  uint8_t InitCapture(uint8_t adc_pin, ADC_TIM_E adc_tim);
  void InitOutCtrl(uint8_t tim_num, uint8_t tim_chn, uint8_t tim_pin, uint32_t pre_scaler=1000000);
  void ReportTemprature();
  void ReportPid();
  void SetPID(uint8_t pid_index, float val);
  void TemperatureOut();
  void PrfetchTempMaintain();
  void GetTemperature(float &celsius);
  uint16_t GetCurTemprature() {return detect_celsius_ * 10;}
  uint16_t GetTargetTemprature() {return pid_.getTarget();}
  void SetPwmDutyLimitAndThreshold(uint8_t count, int32_t threshold);
  void ShutDown();
  float GetTemp();
  void Maintain();
  void TempMaintain(float celsius);
  void ChangeTarget(uint32_t target);

  bool isEnabled();

  // 当前探测到的温度
  float detect_celsius_;
  bool detect_ready_;
 private:
  // 传感器类型
  thermistor_type_e thermistor_type_;
  int last_time_;
  uint8_t adc_index_;
  uint8_t pwm_tim_num_;
  uint8_t pwm_tim_chn_;
  Pid  pid_;
  uint8_t pid_set_flag_ = 0;
  int count_;
  bool enabled_;
  
  bool is_temp_ready_;
  void InitPID();
  void SavePID();
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
