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
#include <wirish_time.h>
#include "../registry/route.h"
#include <src/HAL/hal_tim.h>
#include <math.h>
#include "dual_extruder.h"
#include "../device/soft_pwm.h"

#define NTC3950_ADC_MIN 168
#define NTC3950_ADC_MAX 417

#define Z_MAX_POS                      5.0
#define STEPPER_TIMER                  3
// #define Z_AXIS_STEPS_PER_UNIT          1600         // 2mm/r
#define Z_AXIS_STEPS_PER_UNIT          3200         // 1mm/r
// 电机运动的加速过程的加速度
#define ACCELERATION                   40

#define TEMP_REPORT_INTERVAL    (500)
#define OVER_TEMP_DEBOUNCE      (1000)

#define DEFAULT_PID_P (150)
#define DEFAULT_PID_I (1)
#define DEFAULT_PID_D (30000)

#define DEFAULT_HOTEND_OFFSET_X   (26)
#define DEFAULT_HOTEND_OFFSET_Y   (0)
#define DEFAULT_HOTEND_OFFSET_Z   (-1.5)

#define DEFAULT_SENSOR_COMPENSATION_L (0.8)
#define DEFAULT_SENSOR_COMPENSATION_R (0.8)

#define FAN_SPEED_MIN ((uint8_t)(255 * 0.6))

static DualExtruder * dual_extruder_p;

// 步进定时器回调函数
static void StepperTimerCallback() {
  dual_extruder_p->Stepper();
}


// 初始化
void DualExtruder::Init() {
  AppParmInfo *param = &registryInstance.cfg_;;

  dual_extruder_p = this;

  // 初始化 接近开关传感器
  probe_proximity_switch_.Init(PROBE_PROXIMITY_SWITCH_PIN);
  // 初始化左右挤出机光耦
  probe_left_extruder_optocoupler_.Init(PROBE_LEFT_EXTRUDER_OPTOCOUPLER_PIN);
  probe_right_extruder_optocoupler_.Init(PROBE_RIGHT_EXTRUDER_OPTOCOUPLER_PIN);
  // probe_left_extruder_conductive_.Init(PROBE_LEFT_EXTRUDER_CONDUCTIVE_PIN);
  // probe_right_extruder_conductive_.Init(PROBE_RIGHT_EXTRUDER_CONDUCTIVE_PIN);
  // 初始化断料探测传感器
  out_of_material_detect_0_.Init(OUT_OF_MATERIAL_DETECT_0_PIN, true, INPUT_PULLUP);
  out_of_material_detect_1_.Init(OUT_OF_MATERIAL_DETECT_1_PIN, true, INPUT_PULLUP);
  // 挤出机位选脚
  extruder_cs_0_.Init(EXTRUDER_0_CS_PIN, 1, OUTPUT);
  extruder_cs_1_.Init(EXTRUDER_1_CS_PIN, 0, OUTPUT);

  // 左挤出机风扇
  left_model_fan_.Init(LEFT_MODEL_FAN_PIN, 100);
  // 右挤出机风扇
  right_model_fan_.Init(RIGHT_MODEL_FAN_PIN, 100);
  // 喷嘴风扇
  nozzle_fan_.Init(NOZZLE_FAN_PIN, 100);
  proximity_power_.Init(PROXIMITY_SWITCH_PIN, 0, OUTPUT);

  // 初始化电机控制相关引脚
  // 此电机用于切换喷嘴
  z_motor_cur_ctrl_.Init(LIFT_MOTOR_CUR_CTRL_PIN, 1, OUTPUT);

  // 初始化电机控制相关引脚
  // 此电机用于切换喷嘴
  z_motor_dir_.Init(LIFT_MOTOR_DIR_PIN, 0, OUTPUT);
  z_motor_step_.Init(LIFT_MOTOR_STEP_PIN, 0, OUTPUT);
  z_motor_en_.Init(LIFT_MOTOR_ENABLE_PIN, 0, OUTPUT);

  uint8_t adc_index0_temp, adc_index0_identify, adc_index1_temp, adc_index1_identify;

  if (param->parm_mark[0] != 0xaa || param->parm_mark[1] != 0x55) {
    // 初始化 PID
    param->temp_P = DEFAULT_PID_P;
    param->temp_I = DEFAULT_PID_I;
    param->temp_D = DEFAULT_PID_D;

    // 初始化 热端偏移量
    param->x_hotend_offset = DEFAULT_HOTEND_OFFSET_X;
    param->y_hotend_offset = DEFAULT_HOTEND_OFFSET_Y;
    param->z_hotend_offset = DEFAULT_HOTEND_OFFSET_Z;

    // 探针？补偿
    param->probe_sensor_compensation_0 = DEFAULT_SENSOR_COMPENSATION_L;
    param->probe_sensor_compensation_1 = DEFAULT_SENSOR_COMPENSATION_R;

    registryInstance.SaveCfg();
  }

  adc_index0_temp = temperature_0_.InitCapture(TEMP_0_PIN, ADC_TIM_4);
  temperature_0_.SetThermistorType(THERMISTOR_PT100);
  temperature_0_.InitOutCtrl(PWM_TIM1, PWM_CH2, HEATER_0_PIN);
  adc_index1_temp  = temperature_1_.InitCapture(TEMP_1_PIN, ADC_TIM_4);
  temperature_1_.SetThermistorType(THERMISTOR_PT100);
  temperature_1_.InitOutCtrl(PWM_TIM2, PWM_CH1, HEATER_1_PIN);

  // 喷嘴类型识别引脚初始化
  adc_index0_identify = nozzle_identify_0_.Init(NOZZLE_ID_0_PIN, ADC_TIM_4);
  adc_index1_identify = nozzle_identify_1_.Init(NOZZLE_ID_1_PIN, ADC_TIM_4);

  // 硬件版本号识别引脚
  hw_ver_.Init(HW_VERSION_ADC_PIN, ADC_TIM_4);

  hal_start_adc();

  temperature_0_.SetAdcIndex(adc_index0_temp);
  temperature_0_.SetThermistorType(THERMISTOR_PT100);
  nozzle_identify_0_.SetAdcIndex(adc_index0_identify);
  nozzle_identify_0_.SetNozzleTypeCheckArray(THERMISTOR_PT100);

  temperature_1_.SetAdcIndex(adc_index1_temp);
  temperature_1_.SetThermistorType(THERMISTOR_PT100);
  nozzle_identify_1_.SetAdcIndex(adc_index1_identify);
  nozzle_identify_1_.SetNozzleTypeCheckArray(THERMISTOR_PT100);

  temp_report_time_ = millis() + TEMP_REPORT_INTERVAL;
  overtemp_debounce_[0] = millis() + OVER_TEMP_DEBOUNCE;
  overtemp_debounce_[1] = millis() + OVER_TEMP_DEBOUNCE;
}

// 模块处理句柄
// 根据命令，执行功能
void DualExtruder::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  float val = 0.0;
  switch ((uint32_t)func_id) {
    case FUNC_REPORT_CUT:
      ReportOutOfMaterial();
      break;

    // 主控查询，模组回报接近开关状态
    case FUNC_REPORT_PROBE:
      ReportProbe();
      break;

    // 设置风扇
    case FUNC_SET_FAN:
    case FUNC_SET_FAN2:
      FanCtrl(LEFT_MODEL_FAN, data[1], data[0]);
      FanCtrl(RIGHT_MODEL_FAN, data[1], data[0]);
      break;

    // 设置喷嘴散热风扇
    case FUNC_SET_FAN_NOZZLE:
      FanCtrl(NOZZLE_FAN, data[1], data[0]);
      break;

    // 设置打印头温度
    case FUNC_SET_TEMPEARTURE:
      SetTemperature(data);
      break;

    // 上报打印头温度
    case FUNC_REPORT_TEMPEARTURE:
      ReportTemprature();
      break;

    // 获取温度 PID
    case FUNC_REPORT_TEMP_PID:
      temperature_0_.ReportPid();
      break;

    // 设置温度 PID
    case FUNC_SET_PID:
      val = (float)(((data[1]) << 24) | ((data[2]) << 16) | ((data[3]) << 8 | (data[4]))) / 1000;
      temperature_0_.SetPID(data[0], val);
      break;

    // 喷嘴切换
    case FUNC_SWITCH_EXTRUDER:
      ExtruderSwitcingWithMotor(data);
      break;

    // 喷嘴类型上报
    case FUNC_REPORT_NOZZLE_TYPE:
      ReportNozzleType();
      break;

    // 挤出机状态上报
    case FUNC_REPORT_EXTRUDER_INFO:
      ReportExtruderInfo();
      break;

    // 设置挤出机状态检测控制
    case FUNC_SET_EXTRUDER_CHECK:
      ExtruderStatusCheckCtrl((extruder_status_e)data[0]);
      break;

    // 设置热端偏移量
    case FUNC_SET_HOTEND_OFFSET:
      SetHotendOffset(data);
      break;

    // 上报热端偏移量
    case FUNC_REPORT_HOTEND_OFFSET:
      ReportHotendOffset();
      break;

    // 设置 探针 补偿
    case FUNC_SET_PROBE_SENSOR_COMPENSATION:
      SetProbeSensorCompensation(data);
      break;

    // 获取探针补偿
    case FUNC_REPORT_PROBE_SENSOR_COMPENSATION:
      ReportProbeSensorCompensation();
      break;

    // 单头双喷移动到目标位置
    case FUNC_MOVE_TO_DEST:
      MoveToDestination(data);
      break;

    // 设置单头双喷右喷嘴位置
    case FUNC_SET_RIGHT_EXTRUDER_POS:
      SetRightExtruderPos(data);
      break;

    // 上报单头双喷右喷嘴位置
    case FUNC_REPORT_RIGHT_EXTRUDER_POS:
      ReportRightExtruderPos();
      break;

    // 设置单头双喷接近开关电源
    case FUNC_PROXIMITY_SWITCH_POWER_CTRL:
      ProximitySwitchPowerCtrl(data[0]);
      break;

    // 获取硬件版本
    case FUNC_MODULE_GET_HW_VERSION:
      ReportHWVersion();
      break;

    default:
      break;
  }
}

// 步进运动函数
void DualExtruder::Stepper() {
  if (end_stop_enable_ == true) {
    // 光耦器触发-说明到限位位置了，就无需再运动了
    if (probe_right_extruder_optocoupler_.Read()) {
      hit_state_ = 1;
      stepps_count_ = 0;
      stepps_sum_   = 0;
      motor_state_  = 0;
      z_motor_step_.Out(0);
      z_motor_en_.Out(0);
      z_motor_cur_ctrl_.Out(1);
      StepperTimerStop();
      return;
    }
  }

  while (stepps_count_ == speed_ctrl_buffer_[speed_ctrl_index_].pulse_count) {
    // 整个运动是否结束？
    // 尚未结束，则切换到下一个速度
    if (speed_ctrl_index_ != 19) {
      speed_ctrl_index_++;
      StepperTimerStop();
      StepperTimerStart(speed_ctrl_buffer_[speed_ctrl_index_].timer_time);
    } else {
      // 若已结束，则停止运动
      stepps_count_ = 0;
      stepps_sum_   = 0;
      motor_state_  = 0;
      z_motor_step_.Out(0);
      z_motor_en_.Out(0);
      z_motor_cur_ctrl_.Out(1);
      StepperTimerStop();
      return;
    }
  }

  // 输出脉冲信号
  if (step_pin_state_ == 0) {
    // 输出高电平
    step_pin_state_ = 1;
    z_motor_step_.Out(1);
    stepps_count_++;
  } else {
    // 输出低电平
    step_pin_state_ = 0;
    z_motor_step_.Out(0);

    // 运动完成
    if (stepps_count_ == stepps_sum_) {
      stepps_count_ = 0;
      stepps_sum_   = 0;
      motor_state_  = 0;
      z_motor_en_.Out(0);
      z_motor_cur_ctrl_.Out(1);
      StepperTimerStop();
    }
  }
}

// 开启步进定时器
void DualExtruder::StepperTimerStart(uint16_t time) {
  HAL_timer_disable(STEPPER_TIMER);

  HAL_timer_init(STEPPER_TIMER, 72, time);
  HAL_timer_nvic_init(STEPPER_TIMER, 3, 3);
  HAL_timer_cb_init(STEPPER_TIMER, StepperTimerCallback);
  HAL_timer_enable(STEPPER_TIMER);
}

// 停止步进定时器
void DualExtruder::StepperTimerStop() {
  HAL_timer_disable(STEPPER_TIMER);
  soft_pwm_g.TimStart();
}

// 同步运动情况
void DualExtruder::MoveSync() {
  while(motor_state_) {
    canbus_g.Handler();
    registryInstance.ConfigHandler();
    registryInstance.SystemHandler();
    routeInstance.ModuleLoop();
  }
}

// 回 home
move_state_e DualExtruder::GoHome() {
  extruder_check_status_ = EXTRUDER_STATUS_IDLE;
  move_state_e move_state = MOVE_STATE_SUCCESS;

  // if endstop triggered, leave current position
  // 若到达 endstop，则离开当前位置
  uint32_t i = 0;
  for (i = 0; i < 4; i++) {
    if (digitalRead(PROBE_RIGHT_EXTRUDER_OPTOCOUPLER_PIN)) {
      DoBlockingMoveToZ(-2, 9);
      MoveSync();
    } else {
      DoBlockingMoveToZ(-0.2, 9);
      MoveSync();
      break;
    }
  }

  if (i >= 4) {
    move_state = MOVE_STATE_FAIL;
    goto EXIT;
  }

  end_stop_enable_ = true;
  DoBlockingMoveToZ(9, 9);
  MoveSync();
  if (hit_state_ == 1) {
    hit_state_ = 0;
  } else {
    move_state = MOVE_STATE_FAIL;
    goto EXIT;
  }
  end_stop_enable_ = false;

  // bump
  DoBlockingMoveToZ(-1, 9);
  MoveSync();

  end_stop_enable_ = true;
  DoBlockingMoveToZ(1.5, 1);
  MoveSync();
  if (hit_state_ == 1) {
    hit_state_ = 0;
  } else {
    move_state = MOVE_STATE_FAIL;
    goto EXIT;
  }
  end_stop_enable_ = false;

  // go to the home position
  DoBlockingMoveToZ(-raise_for_home_pos_, 6);
  MoveSync();

  homed_state_ = 1;
  current_position_ = 0;
  extruder_check_status_ = EXTRUDER_STATUS_CHECK;

  active_extruder_ = TOOLHEAD_3DP_EXTRUDER0;
  target_extruder_ = TOOLHEAD_3DP_EXTRUDER0;
  ActiveExtruder(TOOLHEAD_3DP_EXTRUDER0);

EXIT:
    return move_state;
}

// 移动到目标位置
// 目前只用到 回 home
void DualExtruder::MoveToDestination(uint8_t *data) {
  move_type_t move_type = (move_type_t)data[0];
  move_state_e move_state = MOVE_STATE_SUCCESS;

  switch (move_type) {
    case GO_HOME:
      move_state = GoHome();
      break;
    case MOVE_SYNC:
      break;
    case MOVE_ASYNC:
      break;
  }

  uint8_t buf[8], index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MOVE_TO_DEST);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (uint8_t)move_type;
    buf[index++] = (uint8_t)move_state;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 准备移动到目标位置
void DualExtruder::PrepareMoveToDestination(float position, float speed) {
  // 位移范围约束
  if (position > Z_MAX_POS) {
    position = Z_MAX_POS;
  } else if (position < 0) {
    position = 0;
  }

  DoBlockingMoveToZ(position - current_position_, speed);
  current_position_ = position;
}

// 阻塞运动到 Z 
// relative motion, the range of motion will not be checked here
// 相对运动，此处不会检查运动范围
void DualExtruder::DoBlockingMoveToZ(float length, float speed) {
  if (motor_state_) {
    return;
  }

  speed_ctrl_index_ = 0;

  float length_tmp = length;

  // 设置电机旋转方向
  // set motor rotation direction
  if (length < 0) {
    z_motor_dir_.Out(0);
    length = -length;
  }
  else {
    z_motor_dir_.Out(1);
  }

  // 转换为电机步骤数（脉冲数）
  // convert motion distance to number of pulses
  stepps_sum_ = Z_AXIS_STEPS_PER_UNIT * length + 0.5;
  uint32_t half_stepps_sum = stepps_sum_ / 2;

  // 计算加速和减速到目标速度所需的脉冲数
  // s = (1/2)at²
  // calculate the number of pulses needed to accelerate and decelerate to the target speed
  uint32_t acc_dec_stepps = ((speed * speed) / (2 * ACCELERATION)) * Z_AXIS_STEPS_PER_UNIT;
  float acc_dec_time;
  float acc_dec_time_quantum;

  // 若加速过程所需的步骤数 不超过总步骤数的一半
  if (acc_dec_stepps <= half_stepps_sum) {
    // calculation of acceleration and deceleration time
    // 计算加速时间和减速时间
    acc_dec_time = speed / ACCELERATION;
    // 速度变化量？
    acc_dec_time_quantum = acc_dec_time / 10;
    float acc_time = acc_dec_time_quantum;

    // calculate the number of pulses to be traveled for each portion of the acceleration process
    // 计算加速过程的每个部分要行进的脉冲数
    for (int32_t i = 0; i < 10; i++) {
      // 计算当前速度下的脉冲数
      speed_ctrl_buffer_[i].pulse_count = (ACCELERATION * (acc_time * acc_time) / 2) * Z_AXIS_STEPS_PER_UNIT;
      // 计算当前速度下的脉冲高低电平时间（50%占空比）
      speed_ctrl_buffer_[i].timer_time  = 1000000 / (ACCELERATION * acc_time * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // calculate the number of pulses that need to go for each share of time for uniform process
    // 计算均匀处理每一部分时间所需的脉冲数
    speed_ctrl_buffer_[9].pulse_count = stepps_sum_ - acc_dec_stepps;

    // calculate the number of pulses to be traveled for each portion of the decleration process
    // 计算减速过程的每个部分要行进的脉冲数
    acc_time = acc_dec_time_quantum;
    for (int32_t i = 0; i < 10; i++) {
      speed_ctrl_buffer_[10+i].pulse_count = speed_ctrl_buffer_[9].pulse_count + (speed*acc_time - ACCELERATION*(acc_time * acc_time)/2) * Z_AXIS_STEPS_PER_UNIT;
      speed_ctrl_buffer_[10+i].timer_time  = 1000000 / ((speed - ACCELERATION * (acc_time-acc_dec_time_quantum)) * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // just in case
    // 以防万一
    speed_ctrl_buffer_[19].pulse_count = stepps_sum_;
  } else {
    // 加速和减速时间的计算
    // calculation of acceleration and deceleration time
    acc_dec_time = sqrt(length/ACCELERATION);
    acc_dec_time_quantum = acc_dec_time / 10;
    float acc_time = acc_dec_time_quantum;
    float velocity = ACCELERATION * acc_dec_time;

    // calculate the number of pulses to be traveled for each portion of the acceleration process
    // 计算加速过程的每个部分要行进的脉冲数
    for (int32_t i = 0; i < 10; i++) {
      speed_ctrl_buffer_[i].pulse_count = (ACCELERATION * (acc_time * acc_time) / 2) * Z_AXIS_STEPS_PER_UNIT;
      speed_ctrl_buffer_[i].timer_time  = 1000000 / (ACCELERATION * acc_time * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // calculate the number of pulses to be traveled for each portion of the decleration process
    // 计算减速过程的每个部分要行进的脉冲数
    acc_time = acc_dec_time_quantum;
    for (int32_t i = 0; i < 10; i++) {
      speed_ctrl_buffer_[10+i].pulse_count = speed_ctrl_buffer_[9].pulse_count + (velocity*acc_time - ACCELERATION*(acc_time * acc_time)/2) * Z_AXIS_STEPS_PER_UNIT;
      speed_ctrl_buffer_[10+i].timer_time  = 1000000 / ((velocity - ACCELERATION * (acc_time-acc_dec_time_quantum)) * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // just in case
    // 以防万一
    speed_ctrl_buffer_[19].pulse_count = stepps_sum_;
  }

  // wakeup
  // 唤醒电机运动
  motor_state_ = 1;
  z_motor_cur_ctrl_.Out(0);
  z_motor_en_.Out(0);
  current_position_ += length_tmp;
  StepperTimerStart(speed_ctrl_buffer_[0].timer_time);
}

// 上报断料检测情况
void DualExtruder::ReportOutOfMaterial() {
  uint8_t buf[CAN_DATA_FRAME_LENGTH];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_CUT);
  if (msgid != INVALID_VALUE) {
    if (hw_ver_.GetVersion() < HW_VER_1) {
      buf[index++] = out_of_material_detect_0_.Read();
      buf[index++] = out_of_material_detect_1_.Read();
    }
    else {
      buf[index++] = !out_of_material_detect_0_.Read();
      buf[index++] = !out_of_material_detect_1_.Read();
    }
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 上报探针（探测）情况
// 模组上报接近开关状态
// 实际上报的有接近开关状态、光耦触发情况
void DualExtruder::ReportProbe() {
  uint8_t buf[CAN_DATA_FRAME_LENGTH];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PROBE);
  if (msgid != INVALID_VALUE) {
    buf[index++] = probe_proximity_switch_.Read();
    buf[index++] = probe_left_extruder_optocoupler_.Read();
    buf[index++] = probe_right_extruder_optocoupler_.Read();
    // buf[index++] = probe_left_extruder_conductive_.Read();
    // buf[index++] = probe_right_extruder_conductive_.Read();
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 风扇控制
void DualExtruder::FanCtrl(fan_e fan, uint8_t duty_cycle, uint16_t delay_sec_kill) {
  if (duty_cycle > 0 && duty_cycle < FAN_SPEED_MIN)
    duty_cycle = FAN_SPEED_MIN;

  duty_cycle = (uint8_t)(duty_cycle * 100 / 255);

  switch (fan) {
    case LEFT_MODEL_FAN:
      left_model_fan_.ChangePwm(duty_cycle, delay_sec_kill);
      break;
    case RIGHT_MODEL_FAN:
      right_model_fan_.ChangePwm(duty_cycle, delay_sec_kill);
      break;
    case NOZZLE_FAN:
      nozzle_fan_.ChangePwm(duty_cycle, delay_sec_kill);
      break;
    default:
      break;
  }
}

// 设置目标温度
void DualExtruder::SetTemperature(uint8_t *data) {
  temperature_0_.ChangeTarget(data[0] << 8 | data[1]);
  temperature_1_.ChangeTarget(data[2] << 8 | data[3]);
}

#define ERR_OVERTEMP_BIT_MASK         (0)
#define ERR_INVALID_NOZZLE_BIT_MASK   (1)

// 上报打印头温度
void DualExtruder::ReportTemprature() {
  int16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMPEARTURE);
  if (msgid != INVALID_VALUE) {
    uint16_t temp;
    uint8_t temp_error = 0;
    uint8_t buf[CAN_DATA_FRAME_LENGTH];
    uint8_t index = 0;

    if (nozzle_identify_0_.GetNozzleType() == NOZZLE_TYPE_MAX) {
      temp_error |= (1<<ERR_INVALID_NOZZLE_BIT_MASK);
    }

    // 获取处理左挤出机温度
    temp = temperature_0_.GetCurTemprature();

    // 若温度过高超过 1s，则降温至 0°，记录、反馈相应异常情况
    if (temp >= PROTECTION_TEMPERATURE * 10) {
      if (ELAPSED(millis(), overtemp_debounce_[0])) {
        temperature_0_.ChangeTarget(0);
        temp_error |= (1<<ERR_OVERTEMP_BIT_MASK);
      }
    }
    else {
      overtemp_debounce_[0] = millis() + OVER_TEMP_DEBOUNCE;
    }

    buf[index++] = temp >> 8;
    buf[index++] = temp;
    buf[index++] = temp_error;
    buf[index++] = temp_error;

    // 获取处理右挤出机温度
    temp_error = 0;
    if (nozzle_identify_1_.GetNozzleType() == NOZZLE_TYPE_MAX) {
      temp_error |= (1<<ERR_INVALID_NOZZLE_BIT_MASK);
    }

    temp = temperature_1_.GetCurTemprature();

    // 若温度过高超过 1s，则降温至 0°，记录、反馈相应异常情况
    if (temp >= PROTECTION_TEMPERATURE * 10) {
      if (ELAPSED(millis(), overtemp_debounce_[1])) {
        temperature_1_.ChangeTarget(0);
        temp_error |= (1<<ERR_OVERTEMP_BIT_MASK);
      }
    }
    else {
      overtemp_debounce_[1] = millis() + OVER_TEMP_DEBOUNCE;
    }

    buf[index++] = temp >> 8;
    buf[index++] = temp;
    buf[index++] = temp_error;
    buf[index++] = temp_error;

    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 激活挤出机
// 也就是选择当前活跃的挤出机
void DualExtruder::ActiveExtruder(uint8_t extruder) {
  if (extruder == TOOLHEAD_3DP_EXTRUDER0) {
    extruder_cs_0_.Out(1);
    extruder_cs_1_.Out(0);
  } else if (extruder == TOOLHEAD_3DP_EXTRUDER1) {
    extruder_cs_0_.Out(0);
    extruder_cs_1_.Out(1);
  }
}

// 设置挤出机状态检测控制
void DualExtruder::ExtruderStatusCheckCtrl(extruder_status_e status) {
  if (status > EXTRUDER_STATUS_IDLE) {
    return;
  }

  extruder_check_status_ = status;
}

// 挤出机状态检测例程
void DualExtruder::ExtruderStatusCheck() {
  uint8_t left_extruder_status;
  uint8_t right_extruder_status;

  switch (extruder_check_status_) {
    // 检测状态
    case EXTRUDER_STATUS_CHECK:
      // 确认当前活跃的挤出机
      left_extruder_status = probe_left_extruder_optocoupler_.Read();
      right_extruder_status = probe_right_extruder_optocoupler_.Read();
      if (left_extruder_status == 1 && right_extruder_status == 0) {
        active_extruder_ = TOOLHEAD_3DP_EXTRUDER0;
      } else if (left_extruder_status == 1 && right_extruder_status == 1) {
        active_extruder_ = TOOLHEAD_3DP_EXTRUDER1;
      } else {
        active_extruder_ = INVALID_EXTRUDER;
      }

      // 此部分也代表喷嘴切换中
      // 在此检测是否切换完成
      if ((active_extruder_ != target_extruder_) && (extruder_status_ == true)) {
        need_to_report_extruder_info_ = true;
        extruder_status_ = false;
      } else if ((active_extruder_ == target_extruder_) && (extruder_status_ == false)) {
        need_to_report_extruder_info_ = true;
        extruder_status_ = true;
      }

      // 若需要上报挤出机状态，则上报挤出机状态
      if (need_to_report_extruder_info_ == true) {
        need_to_report_extruder_info_ = false;
        ReportExtruderInfo();
      }

      break;

    // 空闲态
    case EXTRUDER_STATUS_IDLE:
      need_to_report_extruder_info_ = false;
      extruder_status_ = true;
      break;

    default:
      break;
  }
}

// 激活挤出机
// 未使用
void DualExtruder::ExtruderSwitching(uint8_t *data) {
  target_extruder_ = data[0];
  ActiveExtruder(target_extruder_);
}

// 切换喷嘴
void DualExtruder::ExtruderSwitcingWithMotor(uint8_t *data) {
  // 获取、选择目标喷嘴
  target_extruder_ = data[0];
  ActiveExtruder(target_extruder_);

  // won't move extruder if didn't home
  if (homed_state_) {
    extruder_check_status_ = EXTRUDER_STATUS_IDLE;
    // 切换到右喷嘴
    if (target_extruder_ == 1) {
      PrepareMoveToDestination(z_max_position_, 9);
    } else if (target_extruder_ == 0) {
      // 切换到左喷嘴
      PrepareMoveToDestination(0, 9);
    }
    MoveSync();
    extruder_check_status_ = EXTRUDER_STATUS_CHECK;
  }

  // 响应指令
  uint8_t buf[8], index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_SWITCH_EXTRUDER);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (uint8_t)target_extruder_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 上报喷嘴类型
void DualExtruder::ReportNozzleType() {
  uint8_t buf[CAN_DATA_FRAME_LENGTH];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_NOZZLE_TYPE);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (uint8_t)nozzle_identify_0_.GetNozzleType();
    buf[index++] = (uint8_t)nozzle_identify_1_.GetNozzleType();
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 挤出机状态上报
void DualExtruder::ReportExtruderInfo() {
  uint8_t buf[CAN_DATA_FRAME_LENGTH];
  uint8_t index = 0;

  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_EXTRUDER_INFO);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (active_extruder_ == target_extruder_) ? 0 : 1;
    buf[index++] = active_extruder_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

// 设置热端偏移量
void DualExtruder::SetHotendOffset(uint8_t *data) {
  uint8_t axis_index;
  float offset;

  axis_index = data[0];
  ((uint8_t *)&offset)[0] = data[1];
  ((uint8_t *)&offset)[1] = data[2];
  ((uint8_t *)&offset)[2] = data[3];
  ((uint8_t *)&offset)[3] = data[4];

  switch (axis_index) {
    case 0:
      registryInstance.cfg_.x_hotend_offset = offset;
      break;
    case 1:
      registryInstance.cfg_.y_hotend_offset = offset;
      break;
    case 2:
      registryInstance.cfg_.z_hotend_offset = offset;
      break;
    default:
      return;
      break;
  }

  registryInstance.SaveCfg();
}

// 上报热端偏移量
void DualExtruder::ReportHotendOffset() {
  float offset[3];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_HOTEND_OFFSET);
  if (msgid == INVALID_VALUE) {
    return ;
  }

  uint8_t u8DataBuf[8], i;
  offset[0] = registryInstance.cfg_.x_hotend_offset;
  offset[1] = registryInstance.cfg_.y_hotend_offset;
  offset[2] = registryInstance.cfg_.z_hotend_offset;

  for (i = 0; i < 3; i++) {
    u8DataBuf[0] = i;
    u8DataBuf[1] = ((uint8_t *)&offset[i])[0];
    u8DataBuf[2] = ((uint8_t *)&offset[i])[1];
    u8DataBuf[3] = ((uint8_t *)&offset[i])[2];
    u8DataBuf[4] = ((uint8_t *)&offset[i])[3];

    canbus_g.PushSendStandardData(msgid, u8DataBuf, 5);
  }
}

// 设置探针补偿值
void DualExtruder::SetProbeSensorCompensation(uint8_t *data) {
  uint8_t e;
  float compensation;

  e = data[0];
  ((uint8_t *)&compensation)[0] = data[1];
  ((uint8_t *)&compensation)[1] = data[2];
  ((uint8_t *)&compensation)[2] = data[3];
  ((uint8_t *)&compensation)[3] = data[4];

  switch (e) {
    case 0:
      registryInstance.cfg_.probe_sensor_compensation_0 = compensation;
      break;
    case 1:
      registryInstance.cfg_.probe_sensor_compensation_1 = compensation;
      break;
    default:
      return;
      break;
  }

  registryInstance.SaveCfg();
}

// 上报探针补偿值
void DualExtruder::ReportProbeSensorCompensation() {
  float compensation[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PROBE_SENSOR_COMPENSATION);
  if (msgid == INVALID_VALUE) {
    return ;
  }

  uint8_t u8DataBuf[8], i, j, u8Index = 0;
  compensation[0] = registryInstance.cfg_.probe_sensor_compensation_0;
  compensation[1] = registryInstance.cfg_.probe_sensor_compensation_1;

  for (i = 0; i < 2; i++) {
    u8DataBuf[0] = i;
    for (j = 0, u8Index = 1; j < 4; j ++) {
      u8DataBuf[u8Index++] = ((uint32_t)(compensation[i] * 1000)) >> (8 * (3 - j));
    }
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

// 设置单头双喷右喷嘴位置
void DualExtruder::SetRightExtruderPos(uint8_t *data) {
  raise_for_home_pos_ = (float)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]) / 1000;
  z_max_position_     = (float)((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]) / 1000;
}

// 上报单头双喷右喷嘴位置
void DualExtruder::ReportRightExtruderPos() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_RIGHT_EXTRUDER_POS);
  if (msgid != INVALID_VALUE) {
    uint32_t raise_for_home_pos_scaled = raise_for_home_pos_ * 1000;
    uint32_t z_max_position_scaled = z_max_position_ * 1000;

    uint8_t u8DataBuf[8], u8Index = 0;
    u8DataBuf[u8Index++] = (raise_for_home_pos_scaled >> 24) & 0xff;
    u8DataBuf[u8Index++] = (raise_for_home_pos_scaled >> 16) & 0xff;
    u8DataBuf[u8Index++] = (raise_for_home_pos_scaled >> 8) & 0xff;
    u8DataBuf[u8Index++] = raise_for_home_pos_scaled & 0xff;
    u8DataBuf[u8Index++] = (z_max_position_scaled >> 24) & 0xff;
    u8DataBuf[u8Index++] = (z_max_position_scaled >> 16) & 0xff;
    u8DataBuf[u8Index++] = (z_max_position_scaled >> 8) & 0xff;
    u8DataBuf[u8Index++] = z_max_position_scaled & 0xff;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

// 设置单头双喷接近开关电源
void DualExtruder::ProximitySwitchPowerCtrl(uint8_t state) {
  if (state == 0) {
    proximity_power_.Out(0);
  } else if (state == 1) {
    proximity_power_.Out(1);
  }

  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_PROXIMITY_SWITCH_POWER_CTRL);
  if (msgid != INVALID_VALUE) {
    uint8_t u8DataBuf[8], u8Index = 0;
    u8DataBuf[u8Index++] = state;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

// 上报硬件版本号
void DualExtruder::ReportHWVersion() {
  ModuleMacInfo *mac_info = (ModuleMacInfo *)FLASH_MODULE_PARA;
  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_GET_HW_VERSION);

  if (msgid != INVALID_VALUE) {
    buf[0] = hw_ver_.GetVersion() + mac_info->hw_version;
    // to have a simple checksum
    buf[1] = ~buf[0];
    canbus_g.PushSendStandardData(msgid, buf, 2);
  }
}

// 急停处理
void DualExtruder::EmergencyStop() {
  // 温度降到 0
  temperature_0_.ChangeTarget(0);
  temperature_1_.ChangeTarget(0);
  // 关闭风扇
  left_model_fan_.ChangePwm(0, 0);
  right_model_fan_.ChangePwm(0, 0);
  nozzle_fan_.ChangePwm(0, 0);
  // 不选中挤出机
  extruder_cs_0_.Out(0);
  extruder_cs_1_.Out(0);
}

// 例行程序
void DualExtruder::Loop() {
  // ADC 采样好后，更新温度、喷嘴类型
  if (hal_adc_status()) {
    temperature_0_.TemperatureOut();
    temperature_1_.TemperatureOut();

    nozzle_identify_0_.CheckLoop();
    nozzle_identify_1_.CheckLoop();

    hw_ver_.UpdateVersion();
  }

  // 500ms 上报一次打印头温度
  if (ELAPSED(millis(), temp_report_time_)) {
    temp_report_time_ = millis() + TEMP_REPORT_INTERVAL;
    ReportTemprature();
  }

  // 若断料检测引脚状态变化，则上报断料检测状态
  if (out_of_material_detect_0_.CheckStatusLoop() || out_of_material_detect_1_.CheckStatusLoop()) {
    ReportOutOfMaterial();
  }

  // 若接近开关传感器状态、左右光耦器状态 发生变化，则上报最新状态
  bool proximity_switch_status  = probe_proximity_switch_.CheckStatusLoop();
  bool left_optocoupler_status  = probe_left_extruder_optocoupler_.CheckStatusLoop();
  bool right_optocoupler_status = probe_right_extruder_optocoupler_.CheckStatusLoop();
  // bool left_conductive_status   = probe_left_extruder_conductive_.CheckStatusLoop();
  // bool right_conductive_status  = probe_right_extruder_conductive_.CheckStatusLoop();
  if (proximity_switch_status || left_optocoupler_status || right_optocoupler_status /*|| left_conductive_status || right_conductive_status*/) {
    ReportProbe();
  }

  // 挤出机状态检测
  // --- 检测当前挤出机状态，需要上报时上报挤出机状态
  ExtruderStatusCheck();
  // 风扇例行程序 -- 处理延时关闭情况
  left_model_fan_.Loop();
  right_model_fan_.Loop();
  nozzle_fan_.Loop();
}
