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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_CONTEXT_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_CONTEXT_H_

#include <include/libmaple/libmaple_types.h>
#include <src/configuration.h>
#include "registry.h"

// Data for invocation

// 功能调用时的上下文
class Context {
 public:
  // Func ID
  uint16_t funcid_;
  // MsgID
  uint16_t msgid_;
  // 具体数据
  uint8_t * data_;
  // 数据长度
  uint32 len_;

  // 获取模组 ID
  MODULE_TYPE module() {
    return registryInstance.module();
  }
};

extern Context contextInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_CONTEXT_H_
