#include "KNN.h"
#include "knn_model.h"

void knn_init(void)
{
    /* 模型数据静态内联在 knn_model.h 中，无需运行时初始化 */
}

uint16_t knn_model_predict(float temp, float humi, float rain, float wind)
{
    float input[KNN_FEATURE_DIM] = {temp, humi, rain, wind};
    return knn_predict(input);
}

knn_result_t knn_model_predict_detailed(float temp, float humi, float rain, float wind)
{
    uint16_t label = knn_model_predict(temp, humi, rain, wind);

    uint8_t window = KNN_WINDOW(label);
    uint8_t fan    = KNN_FAN(label);
    uint8_t humid  = KNN_HUMIDIFIER(label);

    knn_result_t result = {
        .label         = label,
        .window_open   = (window > 0),
        .window_half   = (window == 1),
        .fan_on        = (fan == 1),
        .humidifier_on = (humid == 1),
    };
    return result;
}
