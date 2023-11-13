#include "hwy_test_light.h"
#include <src/HAL/hal_tim.h>
#include "src/registry/registry.h"
#include "src/core/can_bus.h"


#define TEST_LIGHT_STATE_OFF                        (0)     /**< 开灯 */
#define TEST_LIGHT_STATE_ON                         (1)     /**< 关灯 */

/**
 * @brief 初始化接口
 * 
 */
void HWYTestLight::Init()
{
    /* 灯状态初始化为关 */
    light_state_.Init(TEST_LIGHT_PIN, TEST_LIGHT_STATE_OFF, OUTPUT);
}


/**
 * @brief 模组例行程序
 * 
 */
void HWYTestLight::Loop()
{
    /* 灯控制例程，检测并执行延迟关闭功能 */
    light_state_.OutCtrlLoop();
}


/**
 * @brief 模组处理句柄
 * 
 * @param func_id
 * @param data 
 * @param data_len 
 */
void HWYTestLight::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len)
{
    if (NULL == data || 0 == data_len)
    {
        return;
    }

    /* 解析 FuncID，执行功能 */
    switch (func_id)
    {
        /* 设置灯状态 */
        case FUNC_SET_CROSSLIGHT:
        {
            SetLightState(data[0]);
        }
        break;

        /* 获取灯状态 */
        case FUNC_GET_CROSSLIGHT_STATE:
        {
            GetLightState();
        }
        break;
        
        default:
        {

        }
        break;
    }
}


/**
 * @brief 紧急处理接口
 * 
 */
void HWYTestLight::EmergencyStop()
{
    light_state_.Out(TEST_LIGHT_STATE_OFF);
}


/**
 * @brief 设置灯状态
 * 
 * @param state 
 */
void HWYTestLight::SetLightState(uint8_t state)
{
    if (0 == state)
    {
        light_state_.Out(TEST_LIGHT_STATE_OFF);
    }
    else
    {
        light_state_.Out(TEST_LIGHT_STATE_ON);
    }
}

/**
 * @brief 获取灯状态
 * 
 */
void HWYTestLight::GetLightState(void)
{
    uint8_t buf[1];
    uint8_t index = 0;
    uint8_t state = 0;
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_GET_CROSSLIGHT_STATE);

    state = digitalRead(TEST_LIGHT_PIN);
    if (TEST_LIGHT_STATE_OFF == state)
    {
        state = 0;
    }
    else
    {
        state = 1;
    }

    if (msgid != INVALID_VALUE)
    {
        buf[index++] = state;
        canbus_g.PushSendStandardData(msgid, buf, index);
    }
}

