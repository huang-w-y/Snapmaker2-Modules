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

#include <stdexcept>
#include <board/board.h>
#include <io.h>
#include <src/HAL/hal_tim.h>
#include "src/device/soft_pwm.h"


SoftPwm soft_pwm_g;

// 中断处理函数
void PwmTimIsrCallBack() {
  soft_pwm_g.Isr();
}

// 相关定时器初始化
void SoftPwm::HalTimInit() {
  if (this->tim_init_falg_ == true) {
    return ;
  }

  // 周期为 10us 
  HAL_timer_init(SOFT_PWM_TIM, 72, 10);
  // 中断优先级设置
  HAL_timer_nvic_init(SOFT_PWM_TIM, 3, 3);
  // 注册中断处理函数
  HAL_timer_cb_init(SOFT_PWM_TIM, PwmTimIsrCallBack);
  // 启用中断
  HAL_timer_enable(SOFT_PWM_TIM);
  this->tim_init_falg_ = true;
}

// 启用定时器
void SoftPwm::TimStart() {
  HAL_timer_init(SOFT_PWM_TIM, 72, 10);
  HAL_timer_nvic_init(SOFT_PWM_TIM, 3, 3);
  HAL_timer_cb_init(SOFT_PWM_TIM, PwmTimIsrCallBack);
  HAL_timer_enable(SOFT_PWM_TIM);
}

// 中断处理
void SoftPwm::Isr() {
  for (int i = 0; i < this->used_count_; i++) {

    if (this->cnt_[i] < (this->threshold_[i])) {
      // 输出高电平
      digitalWrite(this->pin_list_[i], HIGH);
    } else {
      // 输出低电平
      digitalWrite(this->pin_list_[i], LOW);
    }
    // 计数值累积
    this->cnt_[i] = (this->cnt_[i] + 1) % this->period_[i];
  }
}

// 新增PWM输出通道
int SoftPwm::AddPwm(uint8_t pwm_pin, uint32_t period) {
  uint8_t cur_index = this->used_count_;
  if (cur_index >= PWM_MAX_COUNT) {
    return -1;
  }
  HalTimInit();
  this->pin_list_[cur_index] = pwm_pin;
  this->threshold_[cur_index] = 0;
  this->period_[cur_index] = period;
  this->cnt_[cur_index] = 0;
  pinMode(this->pin_list_[cur_index], OUTPUT);
  this->used_count_++;
  return cur_index;
}

// 设置 PWM 占空比
void SoftPwm::ChangeSoftPWM(uint8_t pwm_index, uint32_t threshold) {
  if (pwm_index >= PWM_MAX_COUNT) {
    return ;
  }
  this->threshold_[pwm_index] = (threshold > this->period_[pwm_index]) ? this->period_[pwm_index] : threshold;
}
