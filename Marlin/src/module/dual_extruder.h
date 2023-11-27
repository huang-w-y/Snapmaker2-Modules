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

#ifndef __DUAL_EXTRUDER_H_
#define __DUAL_EXTRUDER_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "src/device/analog_io_ctrl.h"
#include "module_base.h"
#include "src/device/temperature.h"
#include "../device/nozzle_identify.h"
#include "../device/hw_version.h"

#define CAN_DATA_FRAME_LENGTH     (8)
#define TOOLHEAD_3DP_EXTRUDER0    (0)
#define TOOLHEAD_3DP_EXTRUDER1    (1)
#define INVALID_EXTRUDER          0xFF
#define EXTRUDER_SWITCH_TIME      10000

#define HW_VERSION_ADC_PIN                      PA4
#define PROBE_PROXIMITY_SWITCH_PIN              PB6
#define PROBE_LEFT_EXTRUDER_OPTOCOUPLER_PIN     PA1
#define PROBE_RIGHT_EXTRUDER_OPTOCOUPLER_PIN    PA8
// #define PROBE_LEFT_EXTRUDER_CONDUCTIVE_PIN      PA4
// #define PROBE_RIGHT_EXTRUDER_CONDUCTIVE_PIN     PA4
#define OUT_OF_MATERIAL_DETECT_0_PIN            PA10
#define OUT_OF_MATERIAL_DETECT_1_PIN            PA2
#define EXTRUDER_0_CS_PIN                       PB7
#define EXTRUDER_1_CS_PIN                       PB11
#define TEMP_0_PIN                              PB1
#define TEMP_1_PIN                              PA3
#define HEATER_0_PIN                            PA9
#define HEATER_1_PIN                            PA0
#define NOZZLE_ID_0_PIN                         PA6
#define NOZZLE_ID_1_PIN                         PA5
#define LEFT_MODEL_FAN_PIN                      PA7
#define RIGHT_MODEL_FAN_PIN                     PB2
#define NOZZLE_FAN_PIN                          PB0
#define LIFT_MOTOR_DIR_PIN                      PB9
#define LIFT_MOTOR_STEP_PIN                     PB8
#define LIFT_MOTOR_ENABLE_PIN                   PB5
#define LIFT_MOTOR_CUR_CTRL_PIN                 PC13
#define PROXIMITY_SWITCH_PIN                    PC14

#define PROTECTION_TEMPERATURE  320

// 风扇类型
typedef enum {
  // 左挤出机风扇
  LEFT_MODEL_FAN,
  // 右挤出机风扇
  RIGHT_MODEL_FAN,
  // 喷嘴风扇
  NOZZLE_FAN
}fan_e;

// 喷嘴状态
typedef enum {
  EXTRUDER_STATUS_CHECK,
  EXTRUDER_STATUS_IDLE,
}extruder_status_e;

// 步进电机运动速度节点
typedef struct {
  // 脉冲个数
  uint32_t pulse_count;
  // 持续时间
  uint16_t timer_time;
}speed_node_t;

// 运动类型
typedef enum {
  GO_HOME,
  MOVE_SYNC,
  MOVE_ASYNC,
}move_type_t;

typedef enum {
  MOVE_STATE_SUCCESS,
  MOVE_STATE_FAIL,
}move_state_e;

class DualExtruder : public ModuleBase {
  public:
    DualExtruder () {
      active_extruder_    = TOOLHEAD_3DP_EXTRUDER0;
      target_extruder_    = TOOLHEAD_3DP_EXTRUDER0;
      nozzle_check_time_  = 0;
      temp_report_time_   = 0;
      extruder_check_status_    = EXTRUDER_STATUS_IDLE;
      extruder_switching_time_elapse_ = 0;
      need_to_report_extruder_info_ = false;
      extruder_status_ = true;
      end_stop_enable_ = false;
      step_timer_init_flag_ = false;
      stepps_count_ = 0;
      stepps_sum_ = 0;
      step_pin_state_ = 0;
      motor_state_ = 0;
      homed_state_ = 0;
      speed_ctrl_index_ = 0;
      raise_for_home_pos_ = 3.3;
      z_max_position_ = 5.0;
    }
    void Init();
    void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
    void Stepper();
    void StepperTimerStart(uint16_t time);
    void StepperTimerStop();
    void MoveSync();
    move_state_e GoHome();
    void MoveToDestination(uint8_t *data);
    void PrepareMoveToDestination(float position, float speed);
    void DoBlockingMoveToZ(float length, float speed);
    void ReportOutOfMaterial();
    void ReportProbe();
    void FanCtrl(fan_e fan, uint8_t duty_cycle, uint16_t delay_sec_kill);
    void SetTemperature(uint8_t *data);
    void ReportTemprature();
    void ActiveExtruder(uint8_t extruder);
    void ExtruderStatusCheckCtrl(extruder_status_e status);
    void ExtruderStatusCheck();
    void ExtruderSwitching(uint8_t *data);
    void ExtruderSwitcingWithMotor(uint8_t *data);
    void ReportNozzleType();
    void ReportExtruderInfo();
    void SetHotendOffset (uint8_t *data);
    void ReportHotendOffset();
    void SetProbeSensorCompensation(uint8_t *data);
    void ReportProbeSensorCompensation();
    void SetRightExtruderPos(uint8_t *data);
    void ReportRightExtruderPos();
    void ProximitySwitchPowerCtrl(uint8_t state);
    void ReportHWVersion();
    void EmergencyStop();
    void Loop();

    Fan left_model_fan_;
    Fan right_model_fan_;
    Fan nozzle_fan_;
    // 接近开关传感器
    SwitchInput probe_proximity_switch_;               // proximity switch sensor
    // 左右光耦器
    SwitchInput probe_left_extruder_optocoupler_;      // left extruder optocoupler sensor
    SwitchInput probe_right_extruder_optocoupler_;     // right extruder optocoupler sensor
    SwitchInput probe_left_extruder_conductive_;       // left extruder conductive sensor
    SwitchInput probe_right_extruder_conductive_;      // right extruder conductive sensor
    // 断料监测
    SwitchInput out_of_material_detect_0_;
    SwitchInput out_of_material_detect_1_;
    // 工作喷嘴片选
    SwitchOutput extruder_cs_0_;
    SwitchOutput extruder_cs_1_;
    // 左挤出机温度、或者说喷嘴温度，用于融化耗材的
    Temperature temperature_0_;
    // 右挤出机温度、或者说喷嘴温度，用于融化耗材的
    Temperature temperature_1_;
    NozzleIdentify nozzle_identify_0_;
    NozzleIdentify nozzle_identify_1_;

    // 切换喷嘴相关的电机控制引脚
    // 电机转动方向控制引脚--正转/反转
    SwitchOutput z_motor_dir_;
    // 步进信号引脚--旋转步数
    SwitchOutput z_motor_step_;
    // 使能信号引脚--启用/禁用步进电机旋转
    SwitchOutput z_motor_en_;
    // 电流控制引脚
    SwitchOutput z_motor_cur_ctrl_;
    SwitchInput limit_switch_;
    // 接近开关电源
    SwitchOutput proximity_power_;


  private:
    uint32_t temp_report_time_;
    uint8_t active_extruder_;
    uint8_t target_extruder_;
    uint32_t nozzle_check_time_;
    // 喷嘴检测状态：检测状态、空闲状态
    extruder_status_e extruder_check_status_;
    uint32_t extruder_switching_time_elapse_;
    bool need_to_report_extruder_info_;
    // 挤出机状态
    bool extruder_status_;
    // 控制喷嘴升降的电机的当前位置
    volatile float current_position_;
    // 限位检测使能标志
    volatile bool end_stop_enable_;
    // 电机转速控制缓冲区
    speed_node_t speed_ctrl_buffer_[20];
    // 电机转速控制缓冲区的索引值
    volatile uint8_t speed_ctrl_index_;
    // 当前电机运动步骤中已经累积的的脉冲计数值（已累积的步数）
    volatile uint32_t stepps_count_;
    // 当前电机运动步骤中所需的脉冲计数总值（总步数）
    volatile uint32_t stepps_sum_;
    // 当前输出的脉冲信号的引脚的高低电平
    volatile uint8_t step_pin_state_;
    volatile bool step_timer_init_flag_;
    // 电机状态：转动中/非转动中
    volatile uint8_t motor_state_;
    // 标记是否回 home
    volatile uint8_t homed_state_;
    volatile uint8_t hit_state_;
    // 回 home 后抬升的位置
    float raise_for_home_pos_;
    // 右喷嘴激活时往下走的距离
    float z_max_position_;

    uint32_t overtemp_debounce_[2];

    HWVersion hw_ver_;
};

#endif
