#include "SG90.h"

/**
 * @brief  舵机初始化：配置LEDC定时器 + 通道
 * @严格对照官方文档流程
 */
void Servo_Init(void)
{
    // 1. 配置LEDC定时器（官方标准API）
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = SERVO_LEDC_MODE,
        .timer_num        = SERVO_LEDC_TIMER,
        .duty_resolution  = SERVO_PWM_RESOLUTION,
        .freq_hz          = SERVO_PWM_FREQ,
        .clk_cfg          = LEDC_USE_APB_CLK // 官方推荐时钟源
    };
    ledc_timer_config(&ledc_timer);

    // 2. 配置LEDC通道（官方标准API）
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = SERVO_LEDC_MODE,
        .channel        = SERVO_LEDC_CHANNEL,
        .timer_sel      = SERVO_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PWM_PIN,
        .duty           = 0, // 初始占空比0
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // 上电默认归位 25°
    Servo_SetAngle(25);
}

/**
 * @brief  角度 → PWM占空比 转换 + 输出
 * @param  angle 0-180度
 */
void Servo_SetAngle(uint8_t angle)
{
    // 限制角度范围
    if(angle > 180) angle = 180;

    // 公式：角度 → 脉冲宽度(us)
    uint32_t pulse_width = SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180;

    // 计算LEDC占空值 (50Hz = 20000us 一个周期)
    uint32_t duty = (pulse_width * (1 << SERVO_PWM_RESOLUTION)) / 20000;

    // 官方API：设置占空比 + 更新输出
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}