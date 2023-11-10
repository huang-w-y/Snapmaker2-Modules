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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_

#include <stdint.h>
#include "device_base.h"
#include <io.h>

// 输入引脚类
class SwitchInput {
 public:
  SwitchInput () {
    input_io_need_reverse_ = false;
  }
  void Init(uint8_t pin, WiringPinMode mode=INPUT_PULLUP);
  void Init(uint8_t pin, bool input_io_reverse, WiringPinMode mode);
  // 获取 IO 口状态
  uint8_t Read();
  // 检测 IO 口状态
  bool CheckStatusLoop();
  // 上报 IO 状态
  void ReportStatus(uint16_t funcid);
 private:
  // 引脚号
  uint8_t pin_;
  // 状态值，状态滤波器，可以起到类似消抖作用
  uint8_t status_;
  // 当前 IO 状态
  uint8_t cur_statu;
  // 上一次 IO 状态，可以配合 cur_statu 判断 IO 状态是否发生变化
  uint8_t last_statu_;
  // 用于执行例行程序
  uint32_t time_;
  // IO 状态是否需要反转
  // 也就是 1 代表开 还是 0 代表 开，也就是有效信号是 1 还是 0
  bool input_io_need_reverse_;
};

// 输出引脚类
class SwitchOutput {
 public:
  // 初始化
  void Init(uint8_t pin, uint8_t out_val, WiringPinMode mode=OUTPUT_OPEN_DRAIN);
  // 输出 IO 状态
  void Out(uint8_t out);
  // 延时输出 IO 状态
  void DelayOut(uint8_t out, uint32_t delay_time_ms);
  // 应该是重置 IO 状态
  void ReastOut(uint32_t reset_time_ms);
  // 例行程序，控制延迟输出功能
  void OutCtrlLoop();
 private:
  // 输出引脚
  uint8_t pin_;
  // 可以理解位最近一次设置延迟输出的时间
  uint32_t time_;
  // 延迟输出的时间
  uint32_t delay_time_;
  // bit0 是输出状态，bit1 是是否延迟输出标志位
  uint8_t out_val_;  // bit0 out val, bit1 out flag
};
#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_
