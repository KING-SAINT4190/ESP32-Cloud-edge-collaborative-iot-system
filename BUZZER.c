#include "BUZZER.h"

// 常用音调频率定义（Hz）
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_C5  523

/**
 * @brief  内部辅助函数：设置蜂鸣器频率并发声
 * @param  freq_hz: 频率(Hz)
 */
static void buzzer_set_freq(uint32_t freq_hz)
{
    // 1. 重新配置LEDC定时器频率
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .timer_num        = BUZZER_LEDC_TIMER,
        .duty_resolution  = BUZZER_PWM_RESOLUTION,
        .freq_hz          = freq_hz,
        .clk_cfg          = LEDC_USE_APB_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 2. 设置50%占空比（让蜂鸣器最响）
    uint32_t duty = (1 << BUZZER_PWM_RESOLUTION) / 2;
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, duty);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

/**
 * @brief  蜂鸣器初始化
 */
void Buzzer_Init(void)
{
    // 1. 配置LEDC定时器（初始频率随便设，后面会改）
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .timer_num        = BUZZER_LEDC_TIMER,
        .duty_resolution  = BUZZER_PWM_RESOLUTION,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_USE_APB_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 2. 配置LEDC通道
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_PIN,
        .duty           = 0, // 初始停止
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief  声音1：切换模式提示音（轻快上升调 C4 → E4 → G4）
 */
void Buzzer_StartupTone(void)
{
    buzzer_set_freq(NOTE_C4);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_freq(NOTE_E4);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_freq(NOTE_G4);
    vTaskDelay(pdMS_TO_TICKS(200));
    Buzzer_Stop();
}

/**
 * @brief  声音2：报警音（急促高低交替，用于舵机180°）
 */
void Buzzer_AlarmTone(void)
{
    for(int i=0; i<5; i++) // 循环5次
    {
        buzzer_set_freq(NOTE_A4);
        vTaskDelay(pdMS_TO_TICKS(100));
        buzzer_set_freq(NOTE_C5);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Buzzer_Stop();
}

/**
 * @brief  声音3：确认音（短促单音，用于舵机90°）
 */
void Buzzer_ConfirmTone(void)
{
    buzzer_set_freq(NOTE_E4);
    vTaskDelay(pdMS_TO_TICKS(100));
    Buzzer_Stop();
}

/**
 * @brief  声音4：错误音（下降调，用于舵机0°）
 */
void Buzzer_ErrorTone(void)
{
    buzzer_set_freq(NOTE_G4);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_freq(NOTE_E4);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_freq(NOTE_C4);
    vTaskDelay(pdMS_TO_TICKS(200));
    Buzzer_Stop();
}

/**
 * @brief  停止蜂鸣器（占空比设为0）
 */
void Buzzer_Stop(void)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}