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

#include <Arduino.h>
#include <src/bootstrap/engine.h>
#include <src/bootstrap/startup.h>
#include "src/HAL/hal_reset.h"
/*
 * Below line mush define to override default weak definition in `boards_setup.cpp`, or CAN can't work as expected.
 * Because by default, maple enable USB port. PA12 should be saved for CAN usage.
 */
namespace wirish {
  namespace priv {
    void board_setup_usb(void) {
    }
  }
}

void setup() {
  /*Initialization process
    1 init module id
    2 init Class Route module variable
    3 init module
    4 init func id list
  */
  
  // 自检
  // 模组信息初始化、APP参数初始化等
  startupInstance.SelfDetct();
  // 基本外设初始化
  // 初始化 CAN 总线
  startupInstance.BasePeriphInit();
  // 外设初始化
  // 根据模组ID，创建相应对象实例并初始化，初始化相应 FuncID 列表，设置基础版本号
  startupInstance.PeriphInit();
// FuncID 列表初始化
  startupInstance.FuncIdListInit();
}

void loop() {
  // won't return
  //HAL_reset();
  engineInstance.Run();
}