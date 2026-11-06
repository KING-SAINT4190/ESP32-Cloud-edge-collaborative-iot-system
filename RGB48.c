#include "RGB48.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "esp_check.h"
#include "driver/gpio.h"
//#include "esp_common.h"
#define RMT_RESOLUTION_HZ 10000000


static rmt_channel_handle_t s_rmt_chan = NULL;
static rmt_encoder_handle_t s_rmt_encoder = NULL;

static const rmt_bytes_encoder_config_t s_rgb48_encoder_cfg = {
    .bit0 = {
        .duration0 = 0.35 * RMT_RESOLUTION_HZ / 1000000,
        .level0 = 1,
        .duration1 = 0.8 * RMT_RESOLUTION_HZ / 1000000,
        .level1 = 0,
    },
    .bit1 = {
        .duration0 = 0.7 * RMT_RESOLUTION_HZ / 1000000,
        .level0 = 1,
        .duration1 = 0.6 * RMT_RESOLUTION_HZ / 1000000,
        .level1 = 0,
    },
    .flags.msb_first = 1,
};


void RGB48_init(void)
{
    rmt_tx_channel_config_t tx_chan_cfg = {
        .gpio_num = RGB_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &s_rmt_chan));
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&s_rgb48_encoder_cfg, &s_rmt_encoder));
    ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));
}

void RGB48_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t rgb48[3] = {g, r, b};
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };
    ESP_ERROR_CHECK(rmt_transmit(s_rmt_chan, s_rmt_encoder, rgb48, sizeof(rgb48), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_rmt_chan, portMAX_DELAY));
}

void RGB48_clear(void)
{
    RGB48_set_color(0, 0, 0);
}
