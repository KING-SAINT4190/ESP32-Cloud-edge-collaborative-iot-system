/**
 * @file redray.c
 * @brief 基于RMT的红外接收实现 (UPD6122/NEC协议)
 */

#include "redray.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>

/* 日志标签，用于ESP_LOG系列宏输出调试信息 */
static const char *TAG = "REDRAY";

/* ======================== RMT硬件配置参数 ======================== */
#define RMT_RESOLUTION_HZ     (1000000)       /* RMT计数器分辨率：1 MHz，每个tick = 1微秒 */
#define RMT_RX_MEM_SYMBOLS    (64)            /* RMT接收通道使用的内存块符号数 */
#define RMT_RX_BUF_SIZE       (64)            /* 软件接收缓冲区大小（符号数） */

/* ======================== NEC协议时序参数（单位：微秒） ======================== */
/* NEC协议引导码：9ms低电平 + 4.5ms高电平（标准数据）或2.25ms高电平（重复码） */
#define NEC_LEADER_LOW_MIN    (8000)          /* 引导码低电平最小值 */
#define NEC_LEADER_LOW_MAX    (10000)         /* 引导码低电平最大值（标准9ms） */
#define NEC_LEADER_HIGH_NOR_MIN (4000)        /* 标准数据引导码高电平最小值 */
#define NEC_LEADER_HIGH_NOR_MAX (5000)        /* 标准数据引导码高电平最大值（标准4.5ms） */
#define NEC_LEADER_HIGH_REP_MIN (2000)        /* 重复码引导码高电平最小值 */
#define NEC_LEADER_HIGH_REP_MAX (2500)        /* 重复码引导码高电平最大值（标准2.25ms） */

/* NEC协议数据位时序：560us低电平 + 560us高电平（逻辑0）或1690us高电平（逻辑1） */
#define NEC_BIT_LOW_MIN       (400)           /* 数据位低电平最小值 */
#define NEC_BIT_LOW_MAX       (700)           /* 数据位低电平最大值（标准560us） */
#define NEC_BIT_ZERO_HIGH_MIN (400)           /* 逻辑"0"的高电平最小值 */
#define NEC_BIT_ZERO_HIGH_MAX (700)           /* 逻辑"0"的高电平最大值（标准560us） */
#define NEC_BIT_ONE_HIGH_MIN  (1200)          /* 逻辑"1"的高电平最小值 */
#define NEC_BIT_ONE_HIGH_MAX  (1800)          /* 逻辑"1"的高电平最大值（标准1690us） */

/* ======================== 静态全局变量 ======================== */
static rmt_channel_handle_t s_rmt_rx_channel = NULL;   /* RMT接收通道句柄 */
static TaskHandle_t         s_rx_task_handle = NULL;    /* 红外接收任务句柄 */
static SemaphoreHandle_t    s_rmt_rx_sem = NULL;        /* 二值信号量，用于ISR通知接收任务 */
static redray_callback_t    s_callback = NULL;          /* 用户注册的解码回调函数 */
static void                *s_user_data = NULL;         /* 用户自定义数据指针，透传给回调 */
static bool                 s_is_running = false;       /* 接收任务运行标志 */

/* 保存上一次成功解码的命令，用于处理NEC重复码 */
static uint16_t  s_last_user_code = 0;     /* 上一次的用户码（地址码） */
static uint8_t   s_last_key_code  = 0;     /* 上一次的键码（命令码） */
static bool      s_last_valid      = false; /* 上一次解码是否有效 */

/**
 * @brief 用于在ISR（中断服务程序）和接收任务之间传递RMT接收数据
 */
typedef struct {
    rmt_symbol_word_t symbols[RMT_RX_BUF_SIZE];  /* RMT符号字数组 */
    size_t num_symbols;                           /* 实际接收到的符号数量 */
} ir_rx_data_t;
static ir_rx_data_t s_rx_data;  /* 全局接收数据缓冲区 */

/**
 * @brief 解析RMT符号序列，提取NEC协议的用户码和键码
 * 
 * NEC协议数据帧格式（32位数据）：
 *   - 位0~7:   地址码低8位（先发送）
 *   - 位8~15:  地址码高8位
 *   - 位16~23: 命令码
 *   - 位24~31: 命令码的反码（用于校验）
 * 
 * @param symbols    RMT接收到的符号字数组
 * @param num        符号数量
 * @param user_code  输出参数：解析出的用户码（16位地址码）
 * @param key_code   输出参数：解析出的键码（8位命令码）
 * @param is_repeat  输出参数：是否为重复码
 * @return true      解析成功
 * @return false     解析失败（数据无效或格式错误）
 */
static bool parse_nec_frame(const rmt_symbol_word_t *symbols, size_t num,
                            uint16_t *user_code, uint8_t *key_code, bool *is_repeat)
{
    /* NEC协议至少需要4个符号才能构成完整帧（引导码 + 至少1个数据位） */
    if (num < 4) return false;

    /* 检查第一个符号是否为有效的引导码：先低电平后高电平 */
    if (symbols[0].level0 != 0 || symbols[0].level1 != 1) return false;

    uint32_t low_us  = symbols[0].duration0;   /* 引导码低电平持续时间 */
    uint32_t high_us = symbols[0].duration1;   /* 引导码高电平持续时间 */

    /* 验证引导码低电平是否在NEC协议允许范围内 */
    if (low_us < NEC_LEADER_LOW_MIN || low_us > NEC_LEADER_LOW_MAX) return false;

    /* 根据引导码高电平持续时间判断是标准数据还是重复码 */
    bool repeat = false;
    if (high_us >= NEC_LEADER_HIGH_NOR_MIN && high_us <= NEC_LEADER_HIGH_NOR_MAX) {
        repeat = false;  /* 标准数据：4.5ms高电平 */
    } else if (high_us >= NEC_LEADER_HIGH_REP_MIN && high_us <= NEC_LEADER_HIGH_REP_MAX) {
        repeat = true;   /* 重复码：2.25ms高电平 */
    } else {
        return false;    /* 高电平持续时间不在有效范围内 */
    }

    /* 如果是重复码，直接返回上一次保存的有效键值 */
    if (repeat) {
        if (s_last_valid) {
            *user_code = s_last_user_code;
            *key_code  = s_last_key_code;
            *is_repeat = true;
            return true;
        }
        return false;    /* 没有上一次有效记录，无法响应重复码 */
    }

    /* ========== 解析32位数据位 ========== */
    uint32_t raw_data = 0;   /* 原始32位数据 */
    size_t bit_idx = 0;      /* 当前处理的位索引（0~31） */
    size_t sym_idx = 1;      /* 从第1个符号开始（第0个是引导码） */

    /* 逐位解析32个数据位 */
    while (bit_idx < 32 && sym_idx < num) {
        const rmt_symbol_word_t *sym = &symbols[sym_idx];

        /* 每个数据位符号必须是先低后高 */
        if (sym->level0 != 0 || sym->level1 != 1) return false;

        uint32_t low_us  = sym->duration0;   /* 数据位低电平持续时间 */
        uint32_t high_us = sym->duration1;   /* 数据位高电平持续时间 */

        /* 验证低电平是否在NEC协议允许范围内 */
        if (low_us < NEC_BIT_LOW_MIN || low_us > NEC_BIT_LOW_MAX) return false;

        /* 根据高电平持续时间判断该位是逻辑"0"还是逻辑"1" */
        bool bit_val;
        if (high_us >= NEC_BIT_ZERO_HIGH_MIN && high_us <= NEC_BIT_ZERO_HIGH_MAX) {
            bit_val = 0;   /* 560us高电平 = 逻辑0 */
        } else if (high_us >= NEC_BIT_ONE_HIGH_MIN && high_us <= NEC_BIT_ONE_HIGH_MAX) {
            bit_val = 1;   /* 1690us高电平 = 逻辑1 */
        } else {
            return false;  /* 高电平持续时间不在有效范围内 */
        }

        /* 将解析出的位写入raw_data的对应位置（低位优先） */
        if (bit_val) raw_data |= (1UL << bit_idx);
        bit_idx++;
        sym_idx++;
    }

    /* 必须完整解析32位数据 */
    if (bit_idx != 32) return false;

    /* 从32位原始数据中提取各字段 */
    uint8_t addr_low  = (raw_data >> 0)  & 0xFF;   /* 地址码低8位 */
    uint8_t addr_high = (raw_data >> 8)  & 0xFF;   /* 地址码高8位 */
    uint8_t cmd       = (raw_data >> 16) & 0xFF;   /* 命令码 */
    uint8_t cmd_inv   = (raw_data >> 24) & 0xFF;   /* 命令码的反码 */

    /* 校验：命令码必须等于其反码的按位取反，否则数据无效 */
    if (cmd != (uint8_t)(~cmd_inv)) return false;

    /* 组合用户码（高8位地址 + 低8位地址），输出键码 */
    *user_code = (addr_high << 8) | addr_low;
    *key_code  = cmd;
    *is_repeat = false;
    return true;
}

/**
 * @brief RMT接收完成回调函数（在ISR上下文中执行）
 * 
 * 当RMT接收完一帧数据后，硬件触发此回调。
 * 回调中将接收到的符号数据从DMA缓冲区复制到全局缓冲区，
 * 然后释放信号量通知接收任务进行解码处理。
 * 
 * @param channel   触发回调的RMT通道句柄
 * @param edata     接收完成事件数据，包含符号指针和数量
 * @param user_arg  用户参数（未使用）
 * @return true     表示唤醒了高优先级任务
 */
static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_arg)
{
    BaseType_t high_task_wakeup = pdFALSE;

    /* 将接收到的符号数据从const指针复制到全局缓冲区，确保数据安全 */
    size_t copy_num = (edata->num_symbols < RMT_RX_BUF_SIZE) ? edata->num_symbols : RMT_RX_BUF_SIZE;
    memcpy(s_rx_data.symbols, edata->received_symbols, copy_num * sizeof(rmt_symbol_word_t));
    s_rx_data.num_symbols = copy_num;

    /* 释放信号量，通知接收任务有新数据可处理 */
    if (s_rmt_rx_sem) {
        xSemaphoreGiveFromISR(s_rmt_rx_sem, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

/**
 * @brief 红外接收任务
 * 
 * 该任务负责：
 *   1. 启动RMT接收
 *   2. 等待信号量（由ISR释放）
 *   3. 对接收到的符号序列进行NEC协议解码
 *   4. 调用用户回调函数通知解码结果
 *   5. 重新启动RMT接收，准备接收下一帧
 * 
 * @param arg 任务参数（未使用）
 */
static void redray_rx_task(void *arg)
{
    /* 配置RMT接收参数：信号范围0~30ms */
    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 0,
        .signal_range_max_ns = 30 * 1000000UL,   /* 最大等待30ms，超时自动停止接收 */
    };

    /* 启动第一次RMT接收 */
    esp_err_t err = rmt_receive(s_rmt_rx_channel, s_rx_data.symbols, sizeof(s_rx_data.symbols), &rx_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "首次rmt_receive失败: %s", esp_err_to_name(err));
    }

    /* 主循环：持续接收和解码红外信号 */
    while (s_is_running) {
        /* 阻塞等待信号量，直到ISR通知接收完成 */
        if (xSemaphoreTake(s_rmt_rx_sem, portMAX_DELAY) == pdTRUE) {
            size_t num_symbols = s_rx_data.num_symbols;
            if (num_symbols > 0) {
                uint16_t user_code;   /* 解码出的用户码 */
                uint8_t  key_code;    /* 解码出的键码 */
                bool     repeat;      /* 是否为重复码 */

                /* 尝试解析NEC协议帧 */
                if (parse_nec_frame(s_rx_data.symbols, num_symbols, &user_code, &key_code, &repeat)) {
                    /* 如果是新数据（非重复码），更新上一次记录 */
                    if (!repeat) {
                        s_last_user_code = user_code;
                        s_last_key_code  = key_code;
                        s_last_valid     = true;
                    }
                    /* 调用用户注册的回调函数，传递解码结果 */
                    if (s_callback) {
                        s_callback(user_code, key_code, repeat, s_user_data);
                    }
                } else {
                    ESP_LOGV(TAG, "解码失败，符号数=%d", num_symbols);
                }
            }

            /* 重新启动RMT接收，准备接收下一帧数据 */
            err = rmt_receive(s_rmt_rx_channel, s_rx_data.symbols, sizeof(s_rx_data.symbols), &rx_config);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "rmt_receive重启失败: %s，正在恢复...", esp_err_to_name(err));
                /* 恢复策略：先禁用再重新启用RMT通道 */
                rmt_disable(s_rmt_rx_channel);
                vTaskDelay(pdMS_TO_TICKS(10));
                rmt_enable(s_rmt_rx_channel);
                vTaskDelay(pdMS_TO_TICKS(10));
                /* 重试接收 */
                err = rmt_receive(s_rmt_rx_channel, s_rx_data.symbols, sizeof(s_rx_data.symbols), &rx_config);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "恢复失败，停止接收器");
                    break;  /* 恢复失败，退出循环 */
                }
            }
        }
    }
    /* 退出时自动删除任务 */
    vTaskDelete(NULL);
}

/**
 * @brief 初始化红外接收器
 * 
 * 该函数完成以下初始化步骤：
 *   1. 创建RMT接收通道并配置GPIO
 *   2. 启用RMT通道
 *   3. 注册接收完成回调函数
 *   4. 创建二值信号量用于ISR与任务同步
 * 
 * @param gpio_num  红外接收头连接的GPIO引脚号
 * @param callback  解码完成后的用户回调函数
 * @param user_data 用户自定义数据指针（透传给回调函数）
 * @return esp_err_t ESP_OK表示成功，否则返回错误码
 */
esp_err_t redray_init(int gpio_num, redray_callback_t callback, void *user_data)
{
    /* 参数有效性检查 */
    if (gpio_num < 0 || callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 保存用户回调及参数 */
    s_callback = callback;
    s_user_data = user_data;
    s_last_valid = false;  /* 清除上一次的有效记录 */

    /* 配置RMT接收通道 */
    rmt_rx_channel_config_t rx_ch_config = {
        .gpio_num = gpio_num,                    /* 红外接收引脚 */
        .clk_src = RMT_CLK_SRC_DEFAULT,          /* 使用默认时钟源 */
        .resolution_hz = RMT_RESOLUTION_HZ,      /* 分辨率1MHz，1tick=1us */
        .mem_block_symbols = RMT_RX_MEM_SYMBOLS, /* 内存块符号数 */
        .flags.invert_in = false,                /* 不反转输入信号 */
        .flags.with_dma = false,                 /* 不使用DMA */
    };
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_ch_config, &s_rmt_rx_channel), TAG, "创建RMT接收通道失败");

    /* 启用RMT接收通道 */
    ESP_RETURN_ON_ERROR(rmt_enable(s_rmt_rx_channel), TAG, "启用RMT接收通道失败");

    /* 注册RMT接收完成事件回调 */
    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = rmt_rx_done_callback,    /* 设置ISR回调函数 */
    };
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(s_rmt_rx_channel, &callbacks, NULL), TAG, "注册回调函数失败");

    /* 创建二值信号量，用于ISR和接收任务之间的同步 */
    s_rmt_rx_sem = xSemaphoreCreateBinary();
    if (s_rmt_rx_sem == NULL) {
        ESP_LOGE(TAG, "创建信号量失败");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "红外接收器初始化完成，GPIO%d", gpio_num);
    return ESP_OK;
}

/**
 * @brief 启动红外接收
 * 
 * 创建并启动红外接收任务，任务开始监听红外信号。
 * 多次调用不会重复创建任务。
 * 
 * @return esp_err_t ESP_OK表示成功，ESP_ERR_INVALID_STATE表示未初始化，
 *                   ESP_ERR_NO_MEM表示内存不足
 */
esp_err_t redray_start(void)
{
    /* 检查是否已初始化 */
    if (s_rmt_rx_channel == NULL || s_callback == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 仅在未运行时启动，避免重复创建任务 */
    if (!s_is_running) {
        s_is_running = true;
        /* 创建红外接收任务，栈大小4KB，优先级5 */
        BaseType_t ret = xTaskCreate(redray_rx_task, "redray_rx", 4096, NULL, 5, &s_rx_task_handle);
        if (ret != pdPASS) {
            s_is_running = false;
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "红外接收任务已启动");
    }
    return ESP_OK;
}

/**
 * @brief 停止红外接收
 * 
 * 停止接收任务，不再监听红外信号。
 * 多次调用安全。
 * 
 * @return esp_err_t 始终返回ESP_OK
 */
esp_err_t redray_stop(void)
{
    if (s_is_running) {
        s_is_running = false;  /* 设置退出标志，任务循环将退出 */
        if (s_rx_task_handle) {
            vTaskDelete(s_rx_task_handle);  /* 删除接收任务 */
            s_rx_task_handle = NULL;
        }
        ESP_LOGI(TAG, "红外接收器已停止");
    }
    return ESP_OK;
}

/**
 * @brief 反初始化红外接收器，释放所有资源
 * 
 * 执行以下清理操作：
 *   1. 停止接收任务
 *   2. 删除信号量
 *   3. 禁用并删除RMT通道
 *   4. 清除所有回调和数据指针
 * 
 * @return esp_err_t 始终返回ESP_OK
 */
esp_err_t redray_deinit(void)
{
    /* 先停止接收 */
    redray_stop();

    /* 删除信号量 */
    if (s_rmt_rx_sem) {
        vSemaphoreDelete(s_rmt_rx_sem);
        s_rmt_rx_sem = NULL;
    }

    /* 禁用并删除RMT通道 */
    if (s_rmt_rx_channel) {
        rmt_disable(s_rmt_rx_channel);
        rmt_del_channel(s_rmt_rx_channel);
        s_rmt_rx_channel = NULL;
    }

    /* 清除所有回调和数据指针 */
    s_callback = NULL;
    s_user_data = NULL;
    s_last_valid = false;

    ESP_LOGI(TAG, "红外接收器已反初始化");
    return ESP_OK;
}