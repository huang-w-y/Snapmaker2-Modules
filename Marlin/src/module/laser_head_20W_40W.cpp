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

#include <board/board.h>
#include "src/HAL/hal_flash.h"
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
#include "src/device/icm4xxxx/icm4xxxx_driver.h"
#include <wirish_time.h>
// #include "../registry/route.h"
#include <src/HAL/hal_tim.h>
#include <math.h>
#include "laser_head_20w_40W.h"

// 初始化
void LaserHead20W40W::Init()
{
  // 调试引脚配置
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  // 激光电源控制引脚初始化
  // 这个是总开关，主控控制的是激光功率的 PWM
  laser_power_ctrl_.Init(LASER_20W_40W_ENBLE_PIN, 0, OUTPUT);

  // 初始化风扇反馈
  fan_.Init(LASER_20W_40W_FAN_PIN, LSAER_FAN_FB_IC_TIM, LSAER_FAN_FB_CH, LSAER_FAN_FB_IT_CH, FAN_FEEDBACK_THRESHOLD);
  fan_.set_feed_back_enable(true);

  // 初始化打印头温度采样相关
  temperature_.InitCapture(LASER_20W_40W_TEMP_PIN, ADC_TIM_4);

  // 硬件版本相关初始化
  // 这里硬件版本并未发挥作用，最终获取到的都是固定的
  hw_version_.index = HAL_adc_init(LASER_20W_40W_HW_VERSION_PIN, ADC_TIM_4, ADC_PERIOD_DEFAULT);

  // 初始化PWM 检测引脚
  pwm_detect_.Init(LASER_20W_40W_PWM_DETECT, INPUT_PULLUP);

  // 十字光初始化
  cross_light_.Init(LASER_20W_40W_CROSS_LIGHT, 0, OUTPUT);

  // 激光2 的开关初始化
  // 可以用于半功率输出
  laser2_off_ctrl_.Init(LASER_40W_LASER2_OFF_CTRL_PIN, 0, OUTPUT);     // the laser control pin is enabled by default
  
  // 火焰传感器初始化
  fire_sensor_adc_index_ = HAL_adc_init(LASER_20W_40W_FIRE_SENSOR_PIN, LASER_20W_40W_FIRE_SENSOR_ADC_TIMER, LASER_20W_40W_FIRE_SENSOR_ADC_PERIOD_US);

  // 配置 APP 参数
  AppParmInfo *param = &registryInstance.cfg_;
  uint16_t cal_checksum = LaserParmChecksumCal(param);
  if (param->parm_mark[0] != 0xaa || param->parm_mark[1] != 0x55 || cal_checksum != param->laser_parm_checksum)
  {
    param->laser_protect_temp = LASER_20W_40W_TEMP_LIMIT;
    param->laser_recovery_temp = LASER_20W_40W_TEMP_RECOVERY;
    param->fire_sensor_trigger_value = FIRE_DETECT_TRIGGER_ADC_VALUE;
    // 设置 XY 十字光与激光工作点的偏移量
    if (MODULE_LASER_20W == registryInstance.module())
    {
      param->laser_crosslight_offset_x = LASER_20W_CL_OFFSET_X;
      param->laser_crosslight_offset_y = LASER_20W_CL_OFFSET_Y;
    }
    else
    {
      param->laser_crosslight_offset_x = LASER_40W_CL_OFFSET_X;
      param->laser_crosslight_offset_y = LASER_40W_CL_OFFSET_Y;
    }
    param->laser_parm_checksum = LaserParmChecksumCal(param);
    registryInstance.SaveCfg();
  }
  pre_check_cnt_ = 0;
  sync_id_ = param->module_sync_id;
  protect_temp_ = param->laser_protect_temp;
  recovery_temp_ = param->laser_recovery_temp;
  fire_sensor_trigger_value_ = param->fire_sensor_trigger_value;
  fire_sensor_trigger_ = false;
  fire_sensor_raw_data_report_tick_ms_ = millis();
  fire_sensor_raw_data_report_interval_ms_ = 0;
  crosslight_offset_x_ = param->laser_crosslight_offset_x;
  crosslight_offset_y_ = param->laser_crosslight_offset_y;

  // 初始化打印头保护温度
  if ((uint8_t)param->laser_protect_temp == 0xff) {
      protect_temp_ = LASER_20W_40W_TEMP_LIMIT;
      recovery_temp_ = LASER_20W_40W_TEMP_RECOVERY;
  }

  // 初始化打印头恢复温度
  if ((uint8_t)param->laser_recovery_temp == 0xff) {
      protect_temp_ = LASER_20W_40W_TEMP_LIMIT;
      recovery_temp_ = LASER_20W_40W_TEMP_RECOVERY;
  }

  security_status_ |= FAULT_LASER_PWM_PIN;
  // 初始化 IMU
  if (icm42670.ChipInit() == false)
  {
    security_status_ |= FAULT_IMU_CONNECTION;
  }
}

// 获取硬件版本
void LaserHead20W40W::GetHwVersion()
{
  hw_version_.adc_value = ADC_Get(hw_version_.index);

  if (hw_version_.adc_value == 0)
    return;

  if (hw_version_.number != 0xAA)
    return;
}

// calculate the checksum of the parameter
// 计算 APP 参数中激光模组相关的参数校验值
uint16_t LaserHead20W40W::LaserParmChecksumCal(AppParmInfo *param) {
  uint16_t check_sum = 0;
  uint16_t tmp_c;
  float   tmp_f;

  if (!param)
   return 0xFFFF;

  tmp_c = param->fire_sensor_trigger_value;
  check_sum += (*(((uint8_t*)(&tmp_c))) + *(((uint8_t*)(&tmp_c))+1));

  tmp_f = param->laser_crosslight_offset_x;
  for (int i =0; i < 4; i++) {
    check_sum += *(((uint8_t*)(&tmp_f))+i);
  }

  tmp_f = param->laser_crosslight_offset_y;
  for (int i =0; i < 4; i++) {
    check_sum += *(((uint8_t*)(&tmp_f))+i);
  }

  check_sum ^= 0x21;
  return check_sum;
}

// 模组例程
void LaserHead20W40W::Loop()
{
  // GetHwVersion();
  // 风扇反馈例程，检测并执行风扇延时关闭功能，检测并反馈转速异常
  fan_.Loop();
  // 激光电源控制例程，检测并执行延时输出功能
  laser_power_ctrl_.OutCtrlLoop();
  // 十字光开关例程，检测并执行延时输出功能
  cross_light_.OutCtrlLoop();
  // 激光2开关例程，检测并执行延时开关功能
  laser2_off_ctrl_.OutCtrlLoop();
  // 激光安全状态检测，上报对应激光安全状态
  SecurityStatusCheck();
  // 采样火焰传感器数据，检测是否触发，检测是否需要清除触发标志
  LaserFireSensorLoop();
  // 检测并上报火焰传感器数据
  LaserFireSensorReportLoop();
}

// 模块处理句柄
// 根据命令，执行功能
void LaserHead20W40W::HandModule(uint16_t func_id, uint8_t *data, uint8_t data_len)
{
  uint8_t focus_type;
  uint16_t rp_itv;
  float x_offset, y_offset;

  switch (func_id)
  {

  // 控制风扇1，设置转速、延时关闭时间等参数
  case FUNC_SET_FAN:
    fan_.ChangePwm(data[1], data[0]);
    break;
  // 设置摄像头电源
  case FUNC_SET_CAMERA_POWER:
    //
    break;
  // 设置激光焦距
  case FUNC_SET_LASER_FOCUS:
    focus_type = data_len > 2 ? data[2] : 0;
    LaserSaveFocus(focus_type, data[0] << 8 | data[1]);
    break;
  // 上报激光焦距
  case FUNC_REPORT_LASER_FOCUS:
    focus_type = data_len ? data[0] : 0;
    LaserReportFocus(focus_type);
    break;
  // 自动对焦辅助灯控制
  case FUNC_SET_AUTOFOCUS_LIGHT:
    //
    break;
  // 上报激光安全状态
  case FUNC_REPORT_SECURITY_STATUS:
    ReportSecurityStatus();
    break;
  // 高功率激光在线ID配置
  case FUNC_MODULE_ONLINE_SYNC:
    LaserOnlineStateSync(data);
    break;
  // 设置保护温度
  case FUNC_MODULE_SET_TEMP:
    LaserSetProtectTemp(data);
    break;
  // 高功率激光引脚使能控制
  // 也就是控制激光电源总开关
  case FUNC_MODULE_LASER_CTRL:
    LaserCtrl(data);
    break;
  // 控制 40W 激光分支电源开关
  case FUNC_MODULE_LASER_BRANCH_CTRL:
    LaserBranchCtrl(!!data[0]);
    break;
  case FUNC_MODULE_GET_HW_VERSION:
    LaserReportHWVersion();
    break;
  // 上报 PWM 检测引脚状态
  case FUNC_REPORT_PIN_STATUS:
    LaserReportPinState();
    break;
  // PWM 引脚检测确认
  case FUNC_CONFIRM_PIN_STATUS:
    LaserConfirmPinState();
    break;
  // 设置十字光开关
  case FUNC_SET_CROSSLIGHT:
    LaserSetCrossLight(!!data[0]);
    break;
  // 获取十字光状态
  case FUNC_GET_CROSSLIGHT_STATE:
    LaserGetCrossLightState();
    break;
  // 设置火焰传感器灵敏度
  case FUNC_SET_FIRE_SENSOR_SENSITIVITY:
    if (data_len > 0) {
      if (data_len == 1) {
        LaserSetFireSensorSensitivity(data[0]);
      }
      else {
        LaserSetFireSensorSensitivity((uint16_t)((data[1] << 8) | data[0]), data_len > 2 ? !!data[2] : true);
      }
    }
    break;
  // 获取火焰传感器灵敏度
  case FUNC_GET_FIRE_SENSOR_SENSITIVITY:
    LaserGetFireSensorSensitivity();
    break;
  // 设置火焰传感器原始数据上报间隔
  case FUNC_SET_FIRE_SENSOR_REPORT_TIME:
    rp_itv = (data[1] << 8) | data[0];
    LaserSetFireSensorRawDataReportTime(rp_itv);
    break;
  // 上报火焰传感器原始数据
  case FUNC_REPORT_FIRE_SENSOR_RAW_DATA:
    LaserReportFireSensorRawData();
    break;
  // 设置十字光与激光工作点的偏移量
  case FUNC_SET_CROSSLIGHT_OFFSET:
    x_offset = *((float *)(&data[0]));
    y_offset = *((float *)(&data[4]));
    LaserSetCrosslightOffset(x_offset, y_offset);
    break;
  // 获取十字光与激光工作点的偏移量
  case FUNC_GET_CROSSLIGHT_OFFSET:
    LaserGetCrosslightOffset();
    break;
  default:
    break;
  }
}

// 紧急停止接口
void LaserHead20W40W::EmergencyStop()
{
  // 关闭激光电源
  laser_power_ctrl_.Out(0);
  // 关闭风扇
  fan_.ChangePwm(0, 0);
  // 关闭十字光
  LaserSetCrossLight(false);
}

// 激光安全状态检测
void LaserHead20W40W::SecurityStatusCheck()
{
  // 获取打印头温度
  temperature_.GetTemperature(laser_celsius_);

  // 检测 IMU 是否连接正常
  if ((security_status_ & FAULT_IMU_CONNECTION) == 0)
  {
    if (icm42670.AttitudeSolving() == true)
    {
      icm42670.GetGesture(yaw_, pitch_, roll_);
    }
  }

  // 检测打印头温度
  if (laser_celsius_ > protect_temp_)
  {
    security_status_ |= FAULT_LASER_TEMP;
  }
  else if (laser_celsius_ < recovery_temp_)
  {
    security_status_ &= ~FAULT_LASER_TEMP;
  }

  // 检测 IMU 角度是否正常
  if ((roll_ <= roll_min_) || (roll_ >= roll_max_) || (pitch_ <= pitch_min_) || (pitch_ >= pitch_max_))
  {
    security_status_ |= FAULT_LASER_GESTURE;
  }
  else
  {
    security_status_ &= ~FAULT_LASER_GESTURE;
  }

  // 检测风扇转速是否异常
  if (fan_.get_feed_back_state() == false)
  {
    security_status_ |= FAULT_LASER_FAN_RUN;
  }
  else
  {
    security_status_ &= ~FAULT_LASER_FAN_RUN;
  }

  // 检测火焰传感器是否触发
  if (fire_sensor_trigger_)
  {
    security_status_ |= FAULT_FIRE_DECT;
  }
  else
  {
    security_status_ &= ~FAULT_FIRE_DECT;
  }

  // 若存在安全状态异常问题，则关闭激光电源
  if (security_status_ != 0)
  {
    laser_power_ctrl_.Out(0);
  }

  // 更新上一次检测的激光安全状态，同时上报激光安全状态
  if (security_status_ != security_status_pre_)
  {
    security_status_pre_ = security_status_;
    // if (registryInstance.FuncId2MsgId(FUNC_REPORT_SECURITY_STATUS) != INVALID_VALUE)
    {
      ReportSecurityStatus();
    }
  }
}

// 上报激光安全状态
void LaserHead20W40W::ReportSecurityStatus()
{
  uint8_t buf[8];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_SECURITY_STATUS);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    icm42670.GetGesture(yaw_, pitch_, roll_);
    imu_celsius_ = (int8_t)icm42670.GetTemperature();
    int16_t pitch_int16, roll_int16;
    int8_t celsius_int8;
    pitch_int16 = (int16_t)pitch_;
    roll_int16 = (int16_t)roll_;
    celsius_int8 = (signed char)laser_celsius_;

    buf[index++] = security_status_;
    buf[index++] = (pitch_int16 >> 8) & 0xff;
    buf[index++] = pitch_int16 & 0xff;
    buf[index++] = (roll_int16 >> 8) & 0xff;
    ;
    buf[index++] = roll_int16 & 0xff;
    buf[index++] = celsius_int8;
    buf[index++] = (uint8_t)imu_celsius_;
    buf[index++] = fire_sensor_trigger_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 保存焦距
void LaserHead20W40W::LaserSaveFocus(uint8_t type, uint16_t foch)
{
  AppParmInfo *param = &registryInstance.cfg_;
  if (type)
  {
    param->laser_high_4_axis = foch;
  }
  else
  {
    param->laser_high = foch;
  }
  registryInstance.SaveCfg();
}

// 上报激光焦距
void LaserHead20W40W::LaserReportFocus(uint8_t type)
{
  AppParmInfo *param = &registryInstance.cfg_;
  uint8_t u8DataBuf[8], u8Index = 0;
  uint16_t u16Focu = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_LASER_FOCUS);
  if (msgid != INVALID_VALUE)
  {
    if (type)
    {
      u16Focu = param->laser_high_4_axis;
    }
    else
    {
      u16Focu = param->laser_high;
    }
    if (!(param->parm_mark[0] == 0xaa && param->parm_mark[1] == 0x55) || (u16Focu == 0xffff))
    {
      u16Focu = (uint16_t)LASER_DEFAULT_HIGH;
    }
    u8DataBuf[u8Index++] = u16Focu >> 8;
    u8DataBuf[u8Index++] = u16Focu;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

// 高功率激光在线ID配置
// 也就是个序列号
// 实际是为屏幕的开机引导服务，也就是屏幕如果检测到该模组是第一次接入，那么就会引导用户走开机引导流程。
void LaserHead20W40W::LaserOnlineStateSync(uint8_t *data)
{
  AppParmInfo *param = &registryInstance.cfg_;
  if (data[0] == 1)
  {
    // set module sync id
    sync_id_ = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    param->module_sync_id = sync_id_;
    registryInstance.SaveCfg();
  }
  else if (data[0] == 0)
  {
    // report module sync id
    uint8_t buf[8];
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_ONLINE_SYNC);
    uint8_t index = 0;
    if (msgid != INVALID_VALUE)
    {
      buf[index++] = sync_id_ & 0xff;
      buf[index++] = (sync_id_ >> 8) & 0xff;
      buf[index++] = (sync_id_ >> 16) & 0xff;
      buf[index++] = (sync_id_ >> 24) & 0xff;
      canbus_g.PushSendStandardData(msgid, buf, index);
    }
  }
}

// 设置保护温度
void LaserHead20W40W::LaserSetProtectTemp(uint8_t *data)
{
  AppParmInfo *param = &registryInstance.cfg_;
  protect_temp_ = data[0];
  recovery_temp_ = data[1];

  param->laser_protect_temp = protect_temp_;
  param->laser_recovery_temp = recovery_temp_;
  registryInstance.SaveCfg();
}

// 激光电源总开关控制

void LaserHead20W40W::LaserCtrl(uint8_t *data)
{
  switch (!!data[0])
  {
    case 0:
      laser_power_ctrl_.Out(0);
      break;
    case 1:
      laser_power_ctrl_.Out(1);
      break;
  }

  uint8_t buf[1];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_LASER_CTRL);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE)
  {
    buf[index++] = data[0];
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 激光分支电源控制
// 与半功率模式相关
void LaserHead20W40W::LaserBranchCtrl(bool onoff)
{
  laser2_off_ctrl_.Out(!onoff);

  uint8_t buf[1];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_LASER_BRANCH_CTRL);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE)
  {
    buf[index++] = !!onoff;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 上报模组硬件版本号
void LaserHead20W40W::LaserReportHWVersion()
{
  ModuleMacInfo *mac = (ModuleMacInfo *)FLASH_MODULE_PARA;

  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_GET_HW_VERSION);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE)
  {
    if (hw_version_.number == 0xAA)
      buf[index++] = mac->hw_version;
    else
      buf[index++] = hw_version_.number;
    // to have a simple checksum
    buf[index++] = ~buf[0];
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 上报 PWM 检测引脚状态
void LaserHead20W40W::LaserReportPinState()
{
  uint8_t buf[1];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PIN_STATUS);
  if (msgid != INVALID_VALUE)
  {
    buf[index++] = digitalRead(LASER_20W_40W_PWM_DETECT);
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// PWM 引脚检测确认
// 也就是确认 PWM 引脚检测正确了，需要清除相应标志位
void LaserHead20W40W::LaserConfirmPinState()
{
  security_status_ &= ~FAULT_LASER_PWM_PIN;
}

// 设置十字光开关状态
void LaserHead20W40W::LaserSetCrossLight(bool onoff)
{
  cross_light_.Out(onoff);
}

// 获取十字光状态
void LaserHead20W40W::LaserGetCrossLightState(void)
{
  uint8_t buf[1];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_GET_CROSSLIGHT_STATE);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    buf[index++] = digitalRead(LASER_20W_40W_CROSS_LIGHT);
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 设置火焰传感器灵敏度等级
void LaserHead20W40W::LaserSetFireSensorSensitivity(uint8_t fds, bool need_save)
{
  if (fds > FIRE_DETECT_SENSITIVITY_HIGHT)
    fds = FIRE_DETECT_SENSITIVITY_HIGHT;

  switch (fds) {
    case FIRE_DETECT_SENSITIVITY_HIGHT:
      fire_sensor_trigger_value_ = FIRE_DETECT_SENSITIVITY_HIGHT_ADC_VALUE;
    break;

    case FIRE_DETECT_SENSITIVITY_MID:
      fire_sensor_trigger_value_ = FIRE_DETECT_SENSITIVITY_MID_ADC_VALUE;
    break;

    case FIRE_DETECT_SENSITIVITY_LOW:
      fire_sensor_trigger_value_ = FIRE_DETECT_SENSITIVITY_LOW_ADC_VALUE;
    break;

    default:
      fire_sensor_trigger_value_ = FIRE_DETECT_TRIGGER_DISABLE_ADC_VALUE;
    break;

  }

  if (need_save) {
    AppParmInfo *param = &registryInstance.cfg_;
    param->fire_sensor_trigger_value = fire_sensor_trigger_value_;
    param->laser_parm_checksum = LaserParmChecksumCal(param);
    registryInstance.SaveCfg();
  }
}

// 设置火焰监测灵敏度
void LaserHead20W40W::LaserSetFireSensorSensitivity(uint16_t fdv, bool need_save) {

  if (fdv > FIRE_DETECT_TRIGGER_LIMIT_ADC_VALUE)
    fdv = FIRE_DETECT_TRIGGER_DISABLE_ADC_VALUE;

  fire_sensor_trigger_value_ = fdv;

  if (need_save) {
    AppParmInfo *param = &registryInstance.cfg_;
    param->fire_sensor_trigger_value = fire_sensor_trigger_value_;
    param->laser_parm_checksum = LaserParmChecksumCal(param);
    registryInstance.SaveCfg();
  }

}

// 获取火焰监测灵敏度
void LaserHead20W40W::LaserGetFireSensorSensitivity(void)
{
  uint8_t buf[8];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_GET_FIRE_SENSOR_SENSITIVITY);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    buf[index++] = fire_sensor_trigger_value_ & 0xff;
    buf[index++] = (fire_sensor_trigger_value_ >> 8) & 0xff;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 设置火焰传感器原始数据上报间隔
void LaserHead20W40W::LaserSetFireSensorRawDataReportTime(uint16_t rp_itv_ms)
{
  fire_sensor_raw_data_report_interval_ms_ = rp_itv_ms;
  // Turn on report, reset time start tick
  if (0 != fire_sensor_raw_data_report_interval_ms_)
    fire_sensor_raw_data_report_tick_ms_ = millis();
}

// 设置十字光偏移量
void LaserHead20W40W::LaserSetCrosslightOffset(float x, float y)
{
  crosslight_offset_x_ = x;
  crosslight_offset_y_ = y;
  AppParmInfo *param = &registryInstance.cfg_;
  param->laser_crosslight_offset_x = crosslight_offset_x_;
  param->laser_crosslight_offset_y = crosslight_offset_y_;
  param->laser_parm_checksum = LaserParmChecksumCal(param);
  registryInstance.SaveCfg();
  LaserGetCrosslightOffset();
}

// 上报火焰传感器原始数据
void LaserHead20W40W::LaserReportFireSensorRawData(void)
{
  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_FIRE_SENSOR_RAW_DATA);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    buf[index++] = fire_sensor_raw_adc_ & 0xff;
    buf[index++] = (fire_sensor_raw_adc_ >> 8) & 0xff;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 获取十字光偏移量
void LaserHead20W40W::LaserGetCrosslightOffset(void)
{
  uint8_t buf[8];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_GET_CROSSLIGHT_OFFSET);

  if (msgid != INVALID_VALUE)
  {
    float *t = (float *)(&buf[0]);
    *t = crosslight_offset_x_;
    t = (float *)(&buf[4]);
    *t = crosslight_offset_y_;
    canbus_g.PushSendStandardData(msgid, buf, 8);
  }
}

// 检测并上报火焰传感器数据
void LaserHead20W40W::LaserFireSensorReportLoop(void)
{
  if (0 == fire_sensor_raw_data_report_interval_ms_)
    return;

  if (ELAPSED(millis(), fire_sensor_raw_data_report_tick_ms_ + fire_sensor_raw_data_report_interval_ms_))
  {
    fire_sensor_raw_data_report_tick_ms_ = millis();
    LaserReportFireSensorRawData();
  }
}

// 火焰传感器例程
// 采样火焰传感器数据，检测是否触发，检测是否需要清除触发标志
void LaserHead20W40W::LaserFireSensorLoop(void)
{
  bool trigger = false;
  if (PENDING(millis(), fire_sensor_maf_last_ms_))
    return;

  // 10ms 检测一次
  fire_sensor_maf_last_ms_ = millis() + 1000 / LASER_FIRE_SENSOR_SAMPLE_FREQ;
  fire_sensor_raw_adc_ = ADC_Get(fire_sensor_adc_index_);
  fire_sensor_maf_.addValue(fire_sensor_raw_adc_);

  // 连续采样，等待采样足够多的数据
  // 需要3.8秒左右
  if (pre_check_cnt_ < LASER_FIRE_SENSOR_PRE_CHECK_CNT) {
    pre_check_cnt_++;
    if (pre_check_cnt_ < LASER_FIRE_SENSOR_PRE_CHECK_CNT)
      return;
  }

  // 检测火焰传感器是否触发
  if (fire_sensor_trigger_value_ <= FIRE_DETECT_TRIGGER_LIMIT_ADC_VALUE) {
    if (fire_sensor_maf_.getMovingAverage() <= fire_sensor_trigger_value_) {
      trigger = true;
    }
  }
  else {
    fire_sensor_trigger_reset_delay_ = 0;
  }

  // 设置清除触发标志的延时时间
  if (trigger) {
    fire_sensor_trigger_ = 1;
    fire_sensor_trigger_reset_delay_ = 5 * LASER_FIRE_SENSOR_SAMPLE_FREQ;   // 5 second delay
  }
  else {
    if (fire_sensor_trigger_reset_delay_)
      fire_sensor_trigger_reset_delay_--;
    else
      fire_sensor_trigger_ = 0;
  }
}
