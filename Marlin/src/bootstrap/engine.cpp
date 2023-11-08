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

#include <src/registry/registry.h>
#include <src/core/can_bus.h>
#include <src/device/temperature.h>
#include <src/registry/route.h>
#include "engine.h"

void Engine::Run() {
  bool loop_flag = true;

  while (loop_flag) {
    // 检测并发送CAN总线数据
    canbus_g.Handler();
    // 检测并处理CAN总线接收到的模组配置消息
    registryInstance.ConfigHandler();
    // 接收处理标准数据帧，执行功能
    registryInstance.ServerHandler();
    // 接收处理系统配置命令
    registryInstance.SystemHandler();
    // 执行模组例行程序
    routeInstance.ModuleLoop();
  }
}

Engine engineInstance;