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
#include <src/core/protocal/Longpack.h>
#include "registry.h"
#include "context.h"
#include "route.h"
#include "src/core/utils.h"
#include "src/device/module_index.h"
#include "src/HAL/hal_reset.h"
#include "src/HAL/hal_flash.h"
#include "src/module/laser_head.h"
#include "src/utils/str.h"
#include <wirish_time.h>

// 心跳包
// 更新心跳时间
void Registry::Heartbeat() {
  last_recv_time_ = millis();
}

// 处理配置消息
void Registry::ConfigHandler() {
  // 如果扩展远程帧接收缓冲区不为空
  while (!canbus_g.remote_extended_recv_buffer_.isEmpty()) {
    uint32_t id = canbus_g.remote_extended_recv_buffer_.remove();
    switch (id) {
      case 0x1:
        // report module Id
        // 上报模组 ID
        canbus_g.remote_send_buffer_.insert(canbus_g.extend_send_id_);
    }
  }
  // 如果标准远程帧接收缓冲区不为空
  while (!canbus_g.remote_standard_recv_buffer_.isEmpty()) {
    uint32_t id = canbus_g.remote_standard_recv_buffer_.remove();
    switch (id) {
      // 心跳包
      case REMOTE_STD_HEARTBEAT:
        if (!is_configured) {
          // 暂未支持动态拔插
          // TODO: The current master control is not supported
          // canbus_g.remote_send_buffer_.insert(canbus_g.extend_send_id_);
        }
        break;
      // 急停操作
      case REMOTE_STD_EM_STOP:
        routeInstance.module_->EmergencyStop();
        break;
    }
    // 更新心跳时间
    Heartbeat();
  }
}

// 设置连接超时时间
void Registry::SetConnectTimeout(uint32_t timeout) {
  timeout_ms_ = timeout;
}

// 检测设备是否正常连接
bool Registry::IsConnect() {
  return is_configured && ((last_recv_time_ + timeout_ms_) > millis());
}

// 接收处理标准数据帧，执行功能
// Receive and parse standard frame, execute function invocation
void Registry::ServerHandler() {
  while (!canbus_g.standard_recv_buffer_.isEmpty()) {
    CanRxStruct txItem = canbus_g.standard_recv_buffer_.remove();
    txItem.std_id &= 0x1ff;  // msgID is 9 bit
    // Convert msgId to funcId
    // 根据MsgID 找到功能ID
    uint32_t funcId = MsgId2FuncId(txItem.std_id);
    // 执行功能
    contextInstance.funcid_ = funcId;
    contextInstance.msgid_ = txItem.std_id;
    contextInstance.data_ = txItem.data;
    contextInstance.len_ = txItem.len;
    routeInstance.Invoke();
  }
}

// 设置随机ID
void Registry::set_random_id(uint32_t randomId) {

}

// 获取模组 ID
MODULE_TYPE Registry::module() {
  return (MODULE_TYPE) module_id_;
}

// 接收处理系统配置命令
void Registry::SystemHandler() {

  // Receive and parse long data, update msgId mapping
  if (longpackInstance.parseCmd() != E_TRUE) {
    return;
  }
  uint8_t cmd = longpackInstance.cmd[0];
  uint8_t * cmdData = longpackInstance.cmd + 1;

  // 解析系统配置命令，执行具体配置功能
  switch (cmd) {
    case CMD_M_CONFIG :
      ReportModuleIndex(cmdData);
      break;
    case CMD_M_REQUEST_FUNCID :
      ReportFunctionIds();
      break;
    case CMD_M_CONFIG_FUNCID :
      RegisterMsgId(cmdData);
      break;
    case CMD_M_UPDATE_REQUEST :
      IsUpdate(cmdData);
      break;
    case CMD_M_VERSIONS_REQUEST:
      ReportVersions(cmdData);
      break;
    case CMD_M_SET_RANDOM:
      SetMacRandom(cmdData);
      ReportMacInfo(cmd, cmdData[0]);
      this->RenewMoudleID();
      break;
    case CMD_M_SET_LINEAR_LEN:
      SetLinearModuleLen(cmdData);
      ReportMacInfo(cmd, cmdData[0]);
      break;
    case CMD_M_SET_LINEAR_LEAD:
      SetLinearModuleLead(cmdData);
      ReportMacInfo(cmd, cmdData[0]);
      break;
    case CMD_M_SET_LINEAR_LIMIT:
      SetLinearModuleLimit(cmdData);
      ReportMacInfo(cmd, cmdData[0]);
      break;
    case CMD_M_DEBUG_INFO:
      ReportFuncidAndMsgid();
      break;
  }
  longpackInstance.cmd_clean();
}

// 设置模组ID
void Registry::set_module_id(uint16_t moduleId) {
  this->module_id_ = moduleId;
  InitlizeFuncIds();
}

// 上报 FuncID 列表
void Registry::ReportFunctionIds() {
  uint16_t * funcIds = this->func_ids_;
  uint16_t count = this->len_;
  uint16_t index = 0;
  uint8_t cache[60];
  cache[index++] = CMD_S_REPORT_FUNCID;
  cache[index++] = count;
  for (int i = 0; i < count; i++) {
    cache[index++] = funcIds[i] >> 8;
    cache[index++] = funcIds[i];
  }
  longpackInstance.sendLongpack(cache, index);
}

// 发送升级请求应答ACK
void Registry::SendUpdateRequest() {
  uint8_t  data[2];
  data[0] = CMD_S_UPDATE_REQUEST_REACK;
  data[1] = 1;

  longpackInstance.sendLongpack(data, 2);
}

// 执行模组系统升级
void Registry::SysUpdate(uint8_t *versions) {
    uint8_t data_len = 0;
    AppParmInfo * app_parm = &registryInstance.cfg_;
    HAL_flash_erase_page(FLASH_PUBLIC_PARA, 1);

    memset(app_parm->versions, 0, sizeof(app_parm->versions));
    data_len = (strlen((char *)versions) > APP_VARSIONS_SIZE) ?
                 (APP_VARSIONS_SIZE - 1) : strlen((char *)versions);
    // 更新升级固件版本号
    memcpy(app_parm->versions, versions, data_len);
    // 保存APP配置参数
    registryInstance.SaveCfg();
    // 发送应答ACK
    SendUpdateRequest();
    // 重启后，bootloader 会接收升级包
    HAL_reset();
}

// 解析版本号
bool VerisionToNumber(uint8_t *version, uint8_t *ver_num, uint8_t ver_num_count) {
  uint8_t *p = version;
  if (!version)
    return false;

  ToLowers(version, APP_VARSIONS_SIZE);
  if (!IsBeginWith(version, (uint8_t*)"snapmaker_v")) {
    return false;
  }
  while (!IS_NUMBER(*p) && ((*p) != '\0')) {
    p++;
  }
  int32_t num = 0;
  for (uint8_t i = 0; i < ver_num_count; i++) {
    if (*p == '\0')
      return false;

    if (!StringToInt(p, num)) {
      return false;
    }
    ver_num[i] = num;
    while ((*p != '.') && ((*p) != '\0')) {
      p++;
    }
    if (*p == '.') {
      p++;
    }
  }
  return true;
}

// 接收到主控的升级请求后，检测是否可以升级
void Registry::IsUpdate(uint8_t * data) {
  uint8_t  send_data[2] = {CMD_S_UPDATE_REQUEST_REACK, 0};
  uint8_t  cmd = data[0];
  uint8_t  *versions = data + 1;
  AppParmInfo * app_parm = (AppParmInfo *)FLASH_APP_PARA;
  versions[APP_VARSIONS_SIZE-1] = 0;
  if (cmd == 1) { //强制升级
      SysUpdate(versions);
      return;
  } else if (strcmp((char *)versions, (char *)app_parm->versions) != 0) {
      // 迭代版本大于当前版本，才允许升级
      uint8_t ver_num[3];
      if (VerisionToNumber(versions, ver_num, 3)) {
        if (routeInstance.VersionComparison(ver_num[0], ver_num[1], ver_num[2])) {
          SysUpdate(versions);
          return;
        }
      }
  }
  longpackInstance.sendLongpack(send_data, 2);
}

// used debug
// 上报 FuncID & MsgID 映射关系
void Registry::ReportFuncidAndMsgid() {
    uint16_t index = 0, i;
    uint8_t cache[50];

    cache[index++] = CMD_S_DEBUG_INFO;
    cache[index++] = 0;
    cache[index++] = len_;
    for (i = 0; i < len_; i++) {
        cache[index++] = func_ids_[i] >> 8;
        cache[index++] = func_ids_[i];
        cache[index++] = msg_ids_[i] >> 8;
        cache[index++] = msg_ids_[i];
    }
    longpackInstance.sendLongpack(cache, index);
}

// 上报版本号
void Registry::ReportVersions(uint8_t * data) {
  uint8_t versions[32];
  uint8_t index = 0;
  AppParmInfo * app_parm = (AppParmInfo *)FLASH_APP_PARA;

  versions[index++] = CMD_S_VERSIONS_REACK;
  const char *version;
  if (data[0] == 0) {
    versions[index++] = 0;
    version = APP_VERSIONS;
  } else {
    versions[index++] = 1;
    version = reinterpret_cast<const char*>(app_parm->versions);
  }
  size_t length = strlen(version);
  memcpy(versions + index, version, length);
  index += length;
  longpackInstance.sendLongpack(versions, index);
}

// 注册(绑定) MsgID
void Registry::RegisterMsgId(uint8_t * data) {
  uint8_t count = data[0];
  uint16_t msgid, funcid, index = 1;
  for (int i = 0; i < count; i++) {
    msgid = data[index++] << 8;
    msgid |= (data[index++] & 0xff);

    funcid = data[index++] << 8;
    funcid |= (data[index++] & 0xff);
    for (int j = 0; j < len_; j++) {
      if (funcid == func_ids_[j]) {
        msg_ids_[j] = msgid;
        canbus_g.SetRecvMsgID(msgid);
      }
    }
  }
  is_configured = true;
}

static uint16_t GetAxisModuleLen() {
    ModuleMacInfo * pLaserPram = MODULE_MAC_INFO_ADDR;
    return pLaserPram->other_parm[0];
}

static uint8_t GetAxisLimitSite() {
    ModuleMacInfo * pLaserPram = MODULE_MAC_INFO_ADDR;
    return pLaserPram->other_parm[1];
}

void Registry::ReportModuleIndex(uint8_t * data) {
  uint8_t   cache[8];
  uint8_t   index  = 0;
  cache[index++] = CMD_S_CONFIG_REACK;
  cache[index++] = moduleIndex.SetModuleIndex(data[0]);
  if ((module_id_ == MODULE_LINEAR) || (module_id_ == MODULE_LINEAR_TMC)) {
    uint16_t u16AxisLen = GetAxisModuleLen();
    uint8_t  u8AxisLimitSite = GetAxisLimitSite();
    cache[index++] = u16AxisLen >> 8;
    cache[index++] = (u16AxisLen & 0xff);
    cache[index++] = u8AxisLimitSite;
    // u8Cache[u8Index++] = Switch_TemporaryGetStatu(dMsgIdAndFuncITable.stMsg2Func[0].u8ParmIndex);
  }
  longpackInstance.sendLongpack(cache, index);
}

// 初始化 FuncID
void Registry::InitlizeFuncIds() {
  len_ = routeInstance.func_count_;
  for (int i = 0; i < len_; ++i) {
    func_ids_[i] = routeInstance.func_list_[i];
  }
}

// 根据 MsgID 查找 FuncID
uint32_t Registry::MsgId2FuncId(uint32_t msgId) {
  for (int i = 0; i < len_; ++i) {
    if (msg_ids_[i] == msgId) {
      return func_ids_[i];
    }
  }
  return INVALID_VALUE;
}

// 根据 FuncID 查找 MsgID
uint16_t Registry::FuncId2MsgId(uint16_t funcid) {
  for (int i = 0; i < this->len_; ++i) {
    if (this->func_ids_[i] == funcid) {
      return this->msg_ids_[i];
    }
  }
  return INVALID_VALUE;
}

// 模组信息初始化
void Registry::ModuleInfoInit() {
  ModuleMacInfo * mac = (ModuleMacInfo *)FLASH_MODULE_PARA;
  module_id_ = Number36To10(mac->moduleId, sizeof(mac->moduleId));
  randome_id_ = mac->u32random;
}

// 模组CANID
uint32_t Registry::ModuleCanId() {
  uint32_t module_id = ((module_id_ & 0x1ff) << 19) | (randome_id_ & 0x7ffff);
  // 最后一位是主从位：1代表从机
  return (1) | (module_id << 1);
}

// 使能 APP
void Registry::EnableAPP() {
  uint8_t  enter_app_flag[2];
  HAL_flash_read(FLASH_PUBLIC_PARA, enter_app_flag, sizeof(enter_app_flag));
  if ((enter_app_flag[0] != 0xff) || (enter_app_flag[1] != 0xaa)) {
    enter_app_flag[0] = 0xff;
    enter_app_flag[1] = 0xaa;
    HAL_flash_erase_page(FLASH_PUBLIC_PARA, 1);
    HAL_flash_write(FLASH_PUBLIC_PARA, enter_app_flag, sizeof(enter_app_flag));
  }
}

// 获取 APP 参数
void Registry::LoadCfg() {
  HAL_flash_read(FLASH_APP_PARA, (uint8_t*)&cfg_, sizeof(cfg_));
}

// 保存 APP 参数
void Registry::SaveCfg() {
  cfg_.parm_mark[0] = 0xaa;
  cfg_.parm_mark[1] = 0x55;
  HAL_flash_erase_page(FLASH_APP_PARA, 1);
  HAL_flash_write(FLASH_APP_PARA, (uint8_t *)&cfg_, sizeof(cfg_));
}

// 初始化
void Registry::Init() {
  this->EnableAPP();
  this->ModuleInfoInit();
  moduleIndex.Init();
  LoadCfg();
  for (int i = 0; i < FUNC_MAX_LEN; i++) {
    func_ids_[i] = INVALID_VALUE;
    msg_ids_[i]  = INVALID_VALUE;
  }
}

// 更新模组ID
// 其实是个调试接口
void Registry::RenewMoudleID() {
  this->ModuleInfoInit();
  canbus_g.SetNewExternedID(this->ModuleCanId());
}

// 获取 MAC 信息
void Registry::ReadMacInfo(ModuleMacInfo * mac) {
  HAL_flash_read(FLASH_MODULE_PARA, (uint8_t *)mac ,sizeof(ModuleMacInfo));
}

// 写入 MAC 信息
void Registry::WriteMacInfo(ModuleMacInfo  *mac) {
  HAL_flash_erase_page(FLASH_MODULE_PARA, 1);
  HAL_flash_write(FLASH_MODULE_PARA, (uint8_t *)mac, sizeof(ModuleMacInfo));
}

// 上报 MAC 信息
void Registry::ReportMacInfo(uint8_t type, uint8_t cmd) {
    ModuleMacInfo * mac = MODULE_MAC_INFO_ADDR;
    uint8_t   data[8];
    uint8_t  index = 0;
    data[index++] = type + 1;
    data[index++] = cmd;
    if (CMD_M_SET_RANDOM == type) {
        this->ModuleInfoInit();
        data[index++] = this->randome_id_ >> 24;
        data[index++] = this->randome_id_ >> 16;
        data[index++] = this->randome_id_ >> 8;
        data[index++] = this->randome_id_;
        longpackInstance.sendLongpack(data, index);
    } else if (CMD_M_SET_LINEAR_LIMIT == type) {
        data[2] = (uint8_t)mac->other_parm[1];
        index++;
        longpackInstance.sendLongpack(data, index);
    } else if (CMD_M_SET_LINEAR_LEAD == type) {
        data[index++] = mac->other_parm[2] >> 24;
        data[index++] = mac->other_parm[2] >> 16;
        data[index++] = mac->other_parm[2] >> 8;
        data[index++] = mac->other_parm[2];
        longpackInstance.sendLongpack(data, index);
    } else if (CMD_M_SET_LINEAR_LEN == type) {
        data[index++] = mac->other_parm[0] >> 24;
        data[index++] = mac->other_parm[0] >> 16;
        data[index++] = mac->other_parm[0] >> 8;
        data[index++] = mac->other_parm[0];
        longpackInstance.sendLongpack(data, index);
    }

}

// 设置 MAC 里的随机ID
void Registry::SetMacRandom(uint8_t *data) {
    ModuleMacInfo  mac;
    if (data[0] == 0) {
        return ;
    }
    data += 1;
    uint32_t random = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    random &= 0x7ffff;
    this->ReadMacInfo(&mac);
    mac.u32random = random;
    this->WriteMacInfo(&mac);
}

void Registry::SetLinearModuleLen(uint8_t *data) {
    ModuleMacInfo  mac;
    if (data[0] == 0) {
        return ;
    }
    data += 1;
    this->ReadMacInfo(&mac);
    mac.other_parm[0] = (data[0] << 24) | (data[1] << 16 | (data[2] << 8) | data[3]);
    this->WriteMacInfo(&mac);
}

void Registry::SetLinearModuleLead(uint8_t *data) {
    ModuleMacInfo  mac;
    if (data[0] == 0) {
        return ;
    }
    data += 1;
    this->ReadMacInfo(&mac);
    mac.other_parm[2] = (data[0] << 24) | (data[1] << 16 | (data[2] << 8) | data[3]);
    this->WriteMacInfo(&mac);
}

void Registry::SetLinearModuleLimit(uint8_t *data) {
    ModuleMacInfo  mac;
    if (data[0] == 0) {
        return ;
    }
    data += 1;
    this->ReadMacInfo(&mac);
    mac.other_parm[1] = data[0];
    this->WriteMacInfo(&mac);
}
Registry registryInstance;

