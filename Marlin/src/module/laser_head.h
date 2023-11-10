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

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_LASER_HEAD_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_LASER_HEAD_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "module_base.h"
#include "laser_hw_version.h"

// 风扇引脚
#define LASER_FAN_PIN PA4
// 摄像头电源引脚
#define LASER_CAMERA_POWER_PIN PA8

// 激光头类定义
class LaserHead : public ModuleBase {
 public:
  // 初始化
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  // 例行程序
  void Loop();
  // 急停接口
  void EmergencyStop();
  // 保存焦点
  void LaserSaveFocus(uint8_t type, uint16_t foch);
  // 上报焦点
  void LaserReportFocus(uint8_t type);
  // 风扇对象
  Fan fan_;
  // 摄像头电源对象
  SwitchOutput camera_power_;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_LINEAR_MODULE_H_
