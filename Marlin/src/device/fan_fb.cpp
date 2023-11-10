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

#include <wirish_time.h>
#include "fan_fb.h"

feed_back_t FanFeedBack::fb = {0, 0, 0, 0};


// 相关硬件定时器初始化
// 配置相关定时器，可见定时时间为 1s
void FanFeedBack::Init(uint8_t fan_pin, uint8_t ic_tim, uint8_t ic_ch, uint8_t it_type, uint16_t threshold) {
    // 内部 风扇实际控制使用 软件模拟 PWM 输出方式
    fan_base_->Init(fan_pin);

    // 1s 定时
    ic_timer = ic_tim;
    HAL_timer_init(ic_tim, 7200, 10000);
    HAL_timer_nvic_ic_init(ic_tim, 2, 2, it_type | TIM_IT_UPDATE);
    // 捕获的是风扇转速信号的引脚
    HAL_timer_ic_init(ic_tim, ic_ch, &FanFeedBack::ic_isr_cb, &FanFeedBack::update_isr_cb);
    set_fb_threshold(threshold);
}


// 更新 PWM
void FanFeedBack::ChangePwm(uint8_t threshold, uint16_t delay_close_time_s) {
    // 修改 风扇的 PWM 输出的比较阈值
    fan_base_->ChangePwm(threshold, delay_close_time_s);

    // 若无需风扇反馈功能，则可直接返回
    if (!fb_enable_)
        return;

    // 若启用风扇反馈功能
    // PWM 阈值配置不为 0
    if (threshold) {
        fb_check_ = true;
        fb_result = true;
        if (ic_timer < TIM_MAX) {
            fb.record_pulse_ = 0;
            fb.current_pulse_ = 0xffff;
            // 开始时间
            fb.start_time_ = millis();
            fb.last_time_ = fb.start_time_;
            // 启用定时器
            HAL_timer_enable(ic_timer);
        }
    } else {
        // 关闭检测
        fb_check_ = false;
        fb_result = true;
        if (ic_timer < TIM_MAX)
            HAL_timer_disable(ic_timer);
    }
}


// 定时器捕获触发中断服务函数
void FanFeedBack::ic_isr_cb(void) {
    // 累积已记录脉冲数
    ++fb.record_pulse_;
}


// 定时器更新触发回调
// 1s 一次
void FanFeedBack::update_isr_cb(void) {

    // 更新 3 秒内检测到的脉冲个数
    if ((uint32_t)(millis() - fb.start_time_) > 3000)
        fb.current_pulse_ = fb.record_pulse_;

    // 更新最近的时间
    fb.last_time_ = millis();
    fb.record_pulse_ = 0;
}


// 例行程序
void FanFeedBack::Loop(void) {
    // 风扇例行程序，监测是否需要执行延时关闭功能
    fan_base_->Loop();

    if (!fb_enable_)
        return;

    // 若处于风扇反馈检测中
    if (fb_check_) {
        // 这个应该是不会触发的
        if ((uint32_t)(millis() - fb.last_time_) > 3000) {
            fb_result = false;
            return;
        }

        if (fb.current_pulse_ != 0xffff && fb.current_pulse_ < threshold_) {
            fb_result = false;
        }
    }
}