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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_CANBUS_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_CANBUS_H_

#include <src/utils/RingBuffer.h>

#define STANDARD_SEND_BUFFER_SIZE 16
#define STANDARD_RECV_BUFFER_SIZE 16
#define EXTENDED_SEND_BUFFER_SIZE 128
#define EXTENDED_RECV_BUFFER_SIZE 256
#define REMOTE_SEND_BUFFER_SIZE 3
#define REMOTE_RECV_BUFFER_SIZE 3

#define REMOTE_EXT_REPORT_MAC 0x01
#define REMOTE_STD_HEARTBEAT  0x01
#define REMOTE_STD_EM_STOP    0x02

// CAN 总线发送数据结构
struct CanTxStruct {
  uint32_t std_id;
  uint8_t data[8];
  uint8_t len;
};

// CAN 总线接收数据结构
struct CanRxStruct {
  uint32_t std_id;
  uint8_t data[8];
  uint32_t len;
};

// CAN 总线类
class CanBus {
 public:
  CanBus();
  ~CanBus();

  void Init(uint32_t module_id);
  void Handler();
  void PushSendRemoteData(uint32_t std_id);
  void PushSendStandardData(uint32_t std, uint8_t *data, uint8_t len);
  void PushSendExtendedData(uint8_t *data, uint8_t len);
  void PushRecvRemoteData(uint32_t id, uint8_t ide);
  void PushRecvExtendedData(uint8_t *data, uint8_t len);
  void PushRecvStandardData(uint32_t std_id, uint8_t *data, uint8_t len);
  void RenewExternedID();
  void SetNewExternedID(uint32_t id);
  void SetRemoteCtrlCmd();
  void SetRecvSysCfgCmd(uint32_t module_id);
  void SetRecvMsgID(uint16_t msg_id);
  uint32_t GetSendTime();
  // 扩展帧的发送ID
  uint32_t extend_send_id_ = 0;
  // （扩展）远程帧发送缓冲区
  RingBuffer<uint32_t> remote_send_buffer_{REMOTE_SEND_BUFFER_SIZE};
  // 扩展远程帧接收缓冲区
  // 通常用于接收主控的广播通知
  RingBuffer<uint32_t> remote_extended_recv_buffer_{REMOTE_RECV_BUFFER_SIZE};
  // 标准远程帧接收缓冲区
  RingBuffer<uint16_t> remote_standard_recv_buffer_{REMOTE_RECV_BUFFER_SIZE};
  // 扩展数据帧发送缓冲区
  RingBuffer<uint8_t> extended_send_buffer_{EXTENDED_SEND_BUFFER_SIZE};
  // 扩展数据帧接收缓冲区
  RingBuffer<uint8_t> extended_recv_buffer_{EXTENDED_RECV_BUFFER_SIZE};
  // 标准数据帧发送缓冲区
  RingBuffer<CanTxStruct> standard_send_buffer_{STANDARD_SEND_BUFFER_SIZE};
  // 标准数据帧接收缓冲区
  RingBuffer<CanRxStruct> standard_recv_buffer_{STANDARD_RECV_BUFFER_SIZE};

 private:
  void IncomingHandler();
  void OutgoingHandler();
  // 更新 扩展帧 ID
  uint32_t new_extended_id_;
};

extern CanBus canbus_g;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CORE_CANBUS_H_
