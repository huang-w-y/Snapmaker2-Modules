#ifndef SNAPMAKERMODULES_HWY_TEST_LIGHT_H__
#define SNAPMAKERMODULES_HWY_TEST_LIGHT_H__

#include "src/configuration.h"
#include "module_base.h"
#include "src/device/switch.h"
#include "src/HAL/hal_pwm.h"

#define TEST_LIGHT_PIN      (PB5)

/**
 * @brief 定义 HWYTestLight 类
 * 
 */
class HWYTestLight : public ModuleBase {
public:
    void Init();
    void Loop();
    void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
    void EmergencyStop();

public:
    void SetLightState(uint8_t state);
    void GetLightState(void);

private:
    /* 灯状态 */
    SwitchOutput light_state_;
};



#endif  /**< SNAPMAKERMODULES_HWY_TEST_LIGHT_H__ */