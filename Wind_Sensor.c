#include "Wind_Sensor.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "WIND_SENSOR";

// 传感器配置
static uint8_t s_sensor_addr = WIND_SENSOR_DEFAULT_ADDR;
static uint32_t s_baud_rate = WIND_SENSOR_BAUDRATE;
static uart_port_t s_uart_num = WIND_SENSOR_UART_NUM;

// 计算 CRC16 (Modbus)
static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 发送 Modbus 请求，并等待响应
static esp_err_t modbus_request(uint8_t *req, size_t req_len,
                                uint8_t *rsp, size_t *rsp_len,
                                int timeout_ms)
{
    // 清空缓冲区
    uart_flush_input(s_uart_num);
    uart_write_bytes(s_uart_num, (const char *)req, req_len);

    // 等待响应
    int len = uart_read_bytes(s_uart_num, rsp, *rsp_len, pdMS_TO_TICKS(timeout_ms));
    if (len <= 0) {
        ESP_LOGE(TAG, "Modbus timeout, no response");
        return ESP_ERR_TIMEOUT;
    }
    *rsp_len = len;
    return ESP_OK;
}

// 读取保持寄存器 (功能码 0x03)
static esp_err_t read_holding_register(uint16_t reg_addr, uint16_t *value)
{
    uint8_t req[8];
    uint8_t rsp[32];
    size_t rsp_len = sizeof(rsp);

    // 构建请求帧
    req[0] = s_sensor_addr;          // 地址
    req[1] = 0x03;                   // 功能码：读保持寄存器
    req[2] = (reg_addr >> 8) & 0xFF; // 寄存器地址高字节
    req[3] = reg_addr & 0xFF;        // 寄存器地址低字节
    req[4] = 0x00;                   // 读取寄存器数量高字节
    req[5] = 0x01;                   // 读取1个寄存器
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    esp_err_t err = modbus_request(req, 8, rsp, &rsp_len, 500);
    if (err != ESP_OK) return err;

    // 检查响应长度（至少5字节：地址+功能码+字节数+2字节数据+2字节CRC）
    if (rsp_len < 5) {
        ESP_LOGE(TAG, "Response too short: %d", rsp_len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 检查地址和功能码
    if (rsp[0] != s_sensor_addr || rsp[1] != 0x03) {
        ESP_LOGE(TAG, "Invalid response header: addr=0x%02X, func=0x%02X", rsp[0], rsp[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 检查字节数（应为2）
    if (rsp[2] != 0x02) {
        ESP_LOGE(TAG, "Invalid data byte count: %d", rsp[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 计算CRC并验证
    uint16_t crc_calc = modbus_crc16(rsp, rsp_len - 2);
    uint16_t crc_recv = rsp[rsp_len-2] | (rsp[rsp_len-1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC error: calc=0x%04X, recv=0x%04X", crc_calc, crc_recv);
        return ESP_ERR_INVALID_CRC;
    }

    // 提取寄存器值（高字节在前）
    *value = (rsp[3] << 8) | rsp[4];
    return ESP_OK;
}

// 初始化 UART (RS485 半双工)
esp_err_t wind_sensor_init(uint8_t addr, uint32_t baud_rate)
{
    s_sensor_addr = addr;
    s_baud_rate = baud_rate;

    uart_config_t uart_config = {
        .baud_rate = s_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装驱动
    esp_err_t err = uart_driver_install(s_uart_num, 256, 256, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(s_uart_num, &uart_config);
    if (err != ESP_OK) return err;
    
    // 设置引脚（仅 TXD 和 RXD，无 RTS）
    err = uart_set_pin(s_uart_num,
                       WIND_SENSOR_UART_TXD_PIN,
                       WIND_SENSOR_UART_RXD_PIN,
                       UART_PIN_NO_CHANGE,   // RTS 不使用
                       UART_PIN_NO_CHANGE);  // CTS 不使用
    if (err != ESP_OK) return err;

    // 注意：不要设置 UART_MODE_RS485_HALF_DUPLEX，使用普通模式即可
    // 自动收发模块会自行处理方向切换

    ESP_LOGI(TAG, "Wind sensor initialized (auto-direction mode): addr=%d, baud=%d", addr, baud_rate);
    return ESP_OK;
}

// 读取风速（单位：m/s）
esp_err_t wind_sensor_read_speed(float *wind_speed_mps, int timeout_ms)
{
    if (!wind_speed_mps) return ESP_ERR_INVALID_ARG;

    uint16_t reg_value = 0;
    esp_err_t err = read_holding_register(0x0000, &reg_value);
    if (err != ESP_OK) return err;

    // 根据手册：上传值为真实值的10倍
    *wind_speed_mps = reg_value / 10.0f;
    ESP_LOGD(TAG, "Raw register: 0x%04X (%d) -> Wind speed: %.1f m/s", reg_value, reg_value, *wind_speed_mps);
    return ESP_OK;
}