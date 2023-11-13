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

#include <src/core/can_bus.h>
#include <src/core/utils.h>

#include "Longpack.h"

// 使用的是 V0 协议

// 协议帧头部
#define MAGIC_PART_1 0xAA
#define MAGIC_PART_2 0x55

// 解析命令
ERR_E Longpack::parseCmd() {
  uint8_t data;
  uint16_t dataFieldLen;
  // 检测扩展数据帧是否为空
  while (!canbus_g.extended_recv_buffer_.isEmpty()) {
    data = canbus_g.extended_recv_buffer_.remove();

    // 包头1检测
    if (recv_index_ == 0 && data == MAGIC_PART_1) {
      // parse started
      packData_[recv_index_++] = data;
      continue;
    } else if (recv_index_ == 1 && data != MAGIC_PART_2) {
      // 包头2检测

      // wrong data, skip
      recv_index_ = 0;
      continue;
    } else if (recv_index_ > 0) {
      // 包数据接收
      packData_[recv_index_++] = data;
      if (recv_index_ == 6) {
        // 验证校验值
        // len_high(bit 2) concat len_low(bit 3)  = len_check(bit 5)
        if ((packData_[2] ^ packData_[3]) != packData_[5]) {
          // wrong data, skip
          recv_index_ = 0;
          continue;
        }
      } else if (recv_index_ > 6) {
        // 包长度
        dataFieldLen = packData_[2] << 8 | packData_[3];
        if (dataFieldLen + sizeof(PackHead) == recv_index_) {
          len_ = recv_index_ - sizeof(PackHead);
          packData_[recv_index_] = 0;
          recv_index_ = 0;
          // reach the end of the pack
          uint8_t * dataFiled = packData_ + sizeof(PackHead); // skip the packhead
          uint16_t checksum = CalcChecksum(dataFiled, dataFieldLen);

          // len_check_high(bit 6) concat len_check_low(bit 7) were calculated by caller.
          // This check will avoid most data corruption.
          // 检验校验值
          if (checksum == ((packData_[6] << 8) | packData_[7])) {
            return E_TRUE;
          } else {
            return E_FALSE;
          }
        }
      }
    }
    // if all above three criteria are not matched, then that is wrong data, skip.

  }

  // 接收处理中
  return E_DOING;
}

/**
 * @brief 发送长包
 * 
 * @param data 
 * @param len 
 */
void Longpack::sendLongpack(uint8_t *data, uint16_t len) {
  uint16_t dataLen = 0;
  dataLen = (data == NULL) ? 0 : len;

  PackHead headInfo;

  headInfo.magic1 = MAGIC_PART_1;
  headInfo.magic2 = MAGIC_PART_2;

  headInfo.lenHigh = dataLen >> 8 & 0xff;
  headInfo.lenLow = dataLen & 0xff;

  headInfo.version = 0x00;

  headInfo.lenCheck = headInfo.lenHigh ^ headInfo.lenLow;

  uint16_t checksum = CalcChecksum(data, len);
  headInfo.dataCheckHigh = checksum >> 8 & 0xff;
  headInfo.dataCheckLow = checksum & 0xff;

  // send head info
  uint8_t * iter = (uint8_t *) &headInfo;
  for (int i = 0; i < 8; ++i) {
    canbus_g.extended_send_buffer_.insert(*iter);
    ++iter;
  }

  // send data field
  for (int i = 0; i < len; ++i) {
    canbus_g.extended_send_buffer_.insert(data[i]);
  }

}
void Longpack::sendLongpack(uint16_t *data, uint16_t len) {
  sendLongpack((uint8_t*) data, len * 2);
}

/**
 * @brief 清除命令
 * 
 */
void Longpack::cmd_clean() {
  memset(packData_, 0, sizeof(packData_));
}


Longpack longpackInstance;
