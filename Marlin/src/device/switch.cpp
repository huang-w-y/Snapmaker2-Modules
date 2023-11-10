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

#include <board/board.h>
#include <src/core/can_bus.h>
#include <wirish_time.h>
#include "switch.h"
#include "src/configuration.h"
#include "src/registry/registry.h"

// 输入引脚初始化
void SwitchInput::Init(uint8_t pin, WiringPinMode mode) {
  this->pin_ = pin;
  pinMode(pin, mode);
  this->time_ = millis();
  this->status_ = 0;
  digitalWrite(pin, 1);
  this->cur_statu = digitalRead(this->pin_);
}

// 输入引脚初始化
void SwitchInput::Init(uint8_t pin, bool input_io_reverse, WiringPinMode mode) {
  input_io_need_reverse_ = input_io_reverse;
  Init(pin, mode);
}

// 检测状态例程
// 若 IO 状态更新，则返回 true
bool SwitchInput::CheckStatusLoop() {
  bool ret = false;

  // 2ms 执行一次
  if (PENDING(millis(), this->time_ + 2)) {
    return ret;
  }

  // Disappears Shakes
  this->time_ = millis();
  // 获取当前 Io 状态
  uint8_t cur_statu = digitalRead(this->pin_);
  this->status_ = (this->status_ << 1) | (cur_statu != 0);

  // 待 IO 状态稳定后，更新 IO 状态
  if (((this->status_ & 0x0f) == 0x0f) || ((this->status_ & 0x0f) == 0x0)) {
    this->cur_statu = this->status_ & 0x1;
  }

  // IO 状态变化后，则更新上一次的 IO 状态
  if (this->cur_statu != this->last_statu_) {
    ret = true;
    this->last_statu_ = this->cur_statu;
  }
  return ret;
}

// 获取输入引脚状态
uint8_t SwitchInput::Read() {
  return input_io_need_reverse_ ? !this->cur_statu : this->cur_statu;
}

// 上报输入引脚状态
void SwitchInput::ReportStatus(uint16_t funcid) {
  uint16_t msgid = registryInstance.FuncId2MsgId(funcid);
  if (msgid != INVALID_VALUE) {
    uint8_t statu = this->Read();
    canbus_g.PushSendStandardData(msgid, &statu, 1);
  }
}


// 输出引脚初始化
void SwitchOutput::Init(uint8_t pin, uint8_t default_out, WiringPinMode mode) {
  this->pin_ = pin;
  pinMode(pin, mode);
  this->time_ = millis();
  this->out_val_ = 0;
  digitalWrite(pin, default_out);
}

// 输出 IO 状态
void SwitchOutput::Out(uint8_t out) {
  digitalWrite(this->pin_, out);
}

// 设置延迟输出
void SwitchOutput::DelayOut(uint8_t out, uint32_t delay_time_ms) {
  this->delay_time_ = delay_time_ms;
  this->time_ = millis();
  this->out_val_ = 0 | 0x2 | (out != 0);
}

// 应该是重置 IO 状态
void SwitchOutput::ReastOut(uint32_t reset_time_ms) {
  // 未配置延迟输出
  if (!(this->out_val_ & 0x2)) {
    uint8_t cur_out = digitalRead(this->pin_);
    this->Out(!cur_out);
    this->DelayOut(cur_out, reset_time_ms);
  } else {
    // 已配置延迟输出
    this->DelayOut(this->out_val_ & 1, reset_time_ms);
  }
}

// 例行程序，控制延迟输出功能
void SwitchOutput::OutCtrlLoop() {
  // 若当前时间尚未到达延迟输出的时间，
  if (PENDING(millis(), (this->time_ + this->delay_time_)) || !(this->out_val_ & 0x2)) {
      return ;
  }
  digitalWrite(this->pin_, this->out_val_ & 0x1);
  this->out_val_ = 0;
}