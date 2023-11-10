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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_

#include <stdint.h>
#include "device_base.h"
#include "../HAL/hal_pwm.h"

#define FAN_MAX_THRESHOLD 255

// 风扇- 硬件 PWM 相关配置信息
typedef struct {
  // 定时器通道
  PWM_TIM_CHN_E  tim_chn;
  // 风扇引脚
  uint8_t  fan_pin;
  // 极性
  uint16_t ocpolarity;
  // 周期
  uint16_t period;
  // 频率
  uint32_t freq;
} fan_hardware_pwm_info;

class Fan {
 public:
  //  初始化
  void Init(uint8_t fan_pin);
  // 初始化
  void Init(uint8_t fan_pin, uint32_t threshold);
  // PWM相关初始化
  void InitUseHardwarePwm(PWM_TIM_CHN_E tim_chn, uint8_t pin, uint32_t freq, uint16_t period, uint16_t ocpolarity=0xFFFF);
  // 例行程序
  void Loop();
  // 更改风扇 PWM 脉冲宽度
  void ChangePwm(uint8_t threshold, uint16_t delay_close_time_s);


 private:
  // 风扇索引
  // 使用软件 PWM 控制风扇时的 PWM 通道索引值
  uint8_t fan_index_;
  // 延时关闭时间
  uint32_t delay_close_time_;
  // 延时开始时间
  uint32_t delay_start_time_;
  // 使能延时关闭
  bool  delay_close_enadle_;
  // 标记是否使用硬件 PWM 控制风扇
  bool  is_hardware_pwm = false;
  // 硬件 PWM 信息
  fan_hardware_pwm_info pwm_info;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_
