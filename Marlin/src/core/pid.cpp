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

#include "pid.h"
#include "src/registry/context.h"


// 初始化
void Pid::Init(float p, float i, float d) {
  k1_ = 0.95;
  k2_ = 1 - k1_;

  bang_threshold_ = 20;
  bang_max_ = 255;
  pid_max_ = 255;

  i_sum_min_ = 0;
  i_sum_max_ = 0;
  d_term_ = 0;

  target_ = 0;
  this->k_p(p);
  this->k_i(i);
  this->k_d(d);

  // 获取模组类型
  MODULE_TYPE module_type = registryInstance.module();
  switch (module_type) {
    // 单头单喷嘴
    case MODULE_PRINT:
      max_target_temperature_ = SINGLE_EXTRUDER_MAX_TARGET_TEMPERATURE;
      min_target_temperature_ = SINGLE_EXTRUDER_MIN_TARGET_TEMPERATURE;
      max_temperature_        = SINGLE_EXTRUDER_MAX_TEMPERATURE;
      min_temperature_        = SINGLE_EXTRUDER_MIN_TEMPERATURE;
    break;
    // 单头双喷嘴
    case MODULE_DUAL_EXTRUDER:
      max_target_temperature_ = DUAL_EXTRUDER_MAX_TARGET_TEMPERATURE;
      min_target_temperature_ = DUAL_EXTRUDER_MIN_TARGET_TEMPERATURE;
      max_temperature_        = DUAL_EXTRUDER_MAX_TEMPERATURE;
      min_temperature_        = DUAL_EXTRUDER_MIN_TEMPERATURE;
      break;
    default:
      break;
  }
}

// 设置控温的 PWM 占空比的最大值 和 误差最大阈值 
void Pid::SetPwmDutyLimitAndThreshold(uint8_t count, int32_t threshold) {
  bang_max_ = count;
  pid_max_  = count;
  bang_threshold_ = threshold;
}

// 刷新最大最小积分值
void Pid::Refresh() {
  if (k_i_ != 0) {
    i_sum_max_ = pid_max_ / k_i_;
  } else {
    i_sum_max_ = 0;
  }

  i_sum_ = 0;
}

// 控制输出
//  actual 当前实际值
uint32_t Pid::output(float actual) {
  float ret_val = 0;
  // 计算出与目标值的误差值
  float  err = target_ - actual;

  float p_term = 0;
  float i_term = 0;

  // 加权滤波方式来计算微分值
  d_term_ = k2_ * k_d_ * (actual - pre_err_) + k1_ * d_term_;
  // 更新上一次的误差值
  pre_err_ = actual;

  // 如果温度异常，超过最大或者最小值，则输出 0，即不加热了
  if ((actual > max_temperature_) || (actual < min_temperature_)) {
    ret_val = 0;
    i_sum_ = 0;
  } else if (err > bang_threshold_) {
    // 如果要加热的增幅太大，则直接按最大速度加热
    ret_val = bang_max_;
    i_sum_ = 0;
  } else if ((err < (-bang_threshold_)) || (target_ == 0)) {
    // 如果要降温的降幅太大，则直接按最大速度降温，即不加热
    ret_val = 0;
    i_sum_ = 0;
  } else {
    // 计算比例系数值
    p_term = k_p_ * err;
    // 更新积分值
    i_sum_ += err;

    // 约束积分值
    if (i_sum_ < i_sum_min_) {
      i_sum_ = i_sum_min_;
    } else if (i_sum_ > i_sum_max_) {
      i_sum_ = i_sum_max_;
    }

    i_term = k_i_ * i_sum_;
    // 计算输出值
    ret_val = p_term + i_term - d_term_;

    // if exceed limit, then undo integral calculation
    // 如果超过极限，就撤销积分
    if (ret_val > pid_max_) {
      if (err > 0) {
        i_sum_ -= err;
      }
      ret_val = pid_max_;
    } else if (ret_val < 0) {
      if (err < 0) {
        i_sum_ -= err;
      }
      ret_val = 0;
    }
  }

  return ((uint32_t ) ret_val);
}

// 设置目标温度
void Pid::target(int32_t target) {
  if (target > max_target_temperature_) {
    target = max_target_temperature_;
  } else if (target < min_target_temperature_) {
    target = min_target_temperature_;
  }
  this->target_ = target;
}

// 设置比例系数
void Pid::k_p(float k_p) {
  this->k_p_ = k_p;
}

// 设置积分系数
void Pid::k_i(float k_i) {
  this->k_i_ = k_i;
  Refresh();
}

// 设置微分系数
void Pid::k_d(float k_d) {
  this->k_d_ = k_d;
}

// 获取目标温度
uint32_t Pid::getTarget() {
  return target_;
}




