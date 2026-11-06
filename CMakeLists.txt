idf_component_register(
    SRCS "hello_world_main.c" 
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_rmt freertos esp_driver_gpio RGB48 OLED esp_driver_i2c SG90 BUZZER DHT11 redray esp_timer esp_adc rain weather LLM esp_wifi esp_http_client nvs_flash esp_event esp-tls Wind_Sensor wet fan KNN MQTT
)