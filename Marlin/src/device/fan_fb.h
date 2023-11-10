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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_FAN_FEEDBACK_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_FAN_FEEDBACK_H_

#include <stdint.h>
#include <src/HAL/hal_tim_ic.h>
#include "fan.h"

// 风扇反馈一般用于反馈转速信息，可以用来监测风扇是否正常

// 风扇反馈信息
typedef struct {
    // 开始时间
    uint32_t start_time_;
    // 上一次时间
    uint32_t last_time_;

    // 已记录脉冲数
    volatile uint32_t record_pulse_;
    // 当前 脉冲数
    volatile uint32_t current_pulse_;
} feed_back_t;


// 风扇反馈类
class FanFeedBack : public Fan {
    public:
        // 构造函数中创建对象
        FanFeedBack() { fan_base_ = new Fan(); }
        // 析构函数中删除对象
        virtual ~FanFeedBack() { delete fan_base_; }

        void Init(uint8_t fan_pin, uint8_t ic_tim, uint8_t ic_ch, uint8_t it_type, uint16_t threshold);
        void ChangePwm(uint8_t threshold, uint16_t delay_close_time_s);
        void Loop();

        // 设置启用禁用风扇反馈功能
        void set_feed_back_enable(bool state) { fb_enable_ = state; }
        // 设置阈值
        void set_fb_threshold(uint16_t threshold) { threshold_ = threshold; }
        bool get_feed_back_state() { return fb_result; }

    private:
        // 指向风扇基类
        Fan* fan_base_   = nullptr;
        // 标记是否在检测中
        // 风扇关闭了就无需检测了
        bool fb_check_   = false;
        // 反馈结果
        bool fb_result   = true;
        // 启用禁用风扇反馈功能
        bool fb_enable_  = false;
        // 定时器 TIM1 、TIM2、TIM3...
        uint8_t ic_timer = 0xFF;
        // 阈值
        // 是脉冲个数阈值
        // 也就是在检测时间（3s）内检测到的个数要超过这个才行
        uint16_t threshold_ = 0;

        static feed_back_t fb;
        static void ic_isr_cb();
        static void update_isr_cb();
};

#endif