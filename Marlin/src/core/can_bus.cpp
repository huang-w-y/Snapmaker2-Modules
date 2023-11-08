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

#include <src/HAL/hal_can.h>
#include "can_bus.h"
#include "src/HAL/hal_can.h"
#include <wirish_time.h>

// 构造函数
CanBus::CanBus() {

//  rb_init(&standardSendBuffer, 1024, rawStandardSendBuffer);
//  rb_init(&standard_recv_buffer_, 512, rawStandardRecvBuffer);
}

// 析构函数
CanBus::~CanBus() {

}

// 设置远程控制命令
// 这里是设置远程帧的 ID 滤波器
void CanBus::SetRemoteCtrlCmd() {
  // 添加扩展远程帧 ID 滤波器
  Can_AddRemoteExtIdFilter(REMOTE_EXT_REPORT_MAC);
  // 添加标准远程帧 ID 滤波器
  Can_AddRemoteStdIdFilter(REMOTE_STD_HEARTBEAT);
  // 添加标准远程帧 ID 滤波器
  Can_AddRemoteStdIdFilter(REMOTE_STD_EM_STOP);
}

// 设置接收系统配置命令
// 这里是设置扩展数据帧的 ID 滤波器
void CanBus::SetRecvSysCfgCmd(uint32_t module_id) {
  Can_AddDataExtIdFilter(module_id & (~0x01));
}

// 设置接收消息
// 这里是设置标准数据帧的 ID 滤波器
void CanBus::SetRecvMsgID(uint16_t msg_id) {
  Can_AddDataStdIdFilter(msg_id);
}

// 初始化
void CanBus::Init(uint32_t module_id) {
  CAN_ConfigInit();
  extend_send_id_ = module_id;
  new_extended_id_ = extend_send_id_;
  // 设置扩展远程帧滤波器
  this->SetRemoteCtrlCmd();
  // 设置扩展数据帧滤波器
  this->SetRecvSysCfgCmd(extend_send_id_);
}

// 更新扩展帧ID
void CanBus::RenewExternedID() {
  if (this->extend_send_id_ != this->new_extended_id_) {
    if (this->extended_send_buffer_.isEmpty()) {
      this->extend_send_id_ = this->new_extended_id_;
      this->SetRecvSysCfgCmd(extend_send_id_);
    }
  }
}

// 设置新的扩展ID
void  CanBus::SetNewExternedID(uint32_t id) {
  this->new_extended_id_ = id;
}

// 处理总线数据--发送总线数据
// handle incoming data
// send outgoing data
void CanBus::Handler() {
//  IncomingHandler();
  OutgoingHandler();
}

// put data into specific buffer
void CanBus::IncomingHandler() {

}

// 发送总线数据
void CanBus::OutgoingHandler() {
  HAL_CAN_try_send();
}

// 添加接收到的远程帧数据到缓冲区
void CanBus::PushRecvRemoteData(uint32_t id, uint8_t ide) {
  if (ide) {
    // 扩展远程帧
    remote_extended_recv_buffer_.insert(id);
  } else {
    // 标准远程帧
    remote_standard_recv_buffer_.insert((uint16_t)id);
  }
}

// 添加接收到的扩展数据帧数据到缓冲区
void CanBus::PushRecvExtendedData(uint8_t *data, uint8_t len) {
  for (int i = 0; i < len; ++i) {
    extended_recv_buffer_.insert(data[i]);
  }
}

// 添加接收到的标准数据帧数据到缓冲区
void CanBus::PushRecvStandardData(uint32_t stdId, uint8_t *data, uint8_t len) {
  CanRxStruct rx_struct;
  rx_struct.std_id = stdId;
  for (int i = 0; i < len; ++i) {
    rx_struct.data[i] = data[i];
  }
  rx_struct.len = len;

  // 标准数据帧
  standard_recv_buffer_.insert(rx_struct);
}

// 添加要发送的远程帧数据到缓冲区
void CanBus::PushSendRemoteData(uint32_t std_id) {
  remote_send_buffer_.insert(std_id);
}

// 添加要发送的标准数据帧数据到缓冲区
void CanBus::PushSendStandardData(uint32_t std_id, uint8_t *data, uint8_t len) {
  CanTxStruct tx_struct;
  tx_struct.std_id = std_id;
  tx_struct.len = len;
  for (int i = 0; i < len; ++i) {
    tx_struct.data[i] = data[i];
  }
  standard_send_buffer_.insert(tx_struct);
}

// 添加要发送的扩展数据帧数据到缓冲区
void CanBus::PushSendExtendedData(uint8_t *data, uint8_t len) {
  for (int i = 0; i < len; ++i) {
    extended_send_buffer_.insert(data[i]);
  }
}

// 获取发送时间
uint32_t CanBus::GetSendTime() {
  return millis();
}


CanBus canbus_g;
