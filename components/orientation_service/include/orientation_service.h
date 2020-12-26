#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define DUMP_ORIENTATION(VALUES) ({                                                             \
    ESP_LOGI(TAG, "x_rads: %.5f, y_rads: %.5f, z_rads: %.5f, ax_rads: %.5f, ay_rads: %.5f",     \
        (VALUES).x_rads,                                                                        \
        (VALUES).y_rads,                                                                        \
        (VALUES).z_rads,                                                                        \
        (VALUES).ax_rads,                                                                       \
        (VALUES).ay_rads                                                                        \
    );                                                                                          \
})

typedef struct {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
} gyro_correction_t;

typedef struct {
    float ax_rads;
    float ay_rads;
    float x_rads;
    float y_rads;
    float z_rads;
} orientation_t;

esp_err_t init_orientation_service();

esp_err_t get_orientation(orientation_t* orientation);

esp_err_t stop_orientation_service();


