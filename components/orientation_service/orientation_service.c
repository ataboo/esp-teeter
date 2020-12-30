#include "orientation_service.h"
#include "mpu_6050.h"
#include "teeter_common.h"

#define ESP_LOCAL_LOG_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char* TAG = "ORIENTATION_SERVICE";
static orientation_t _orientation;
static SemaphoreHandle_t read_lock;
static bool _running = false;
static int64_t _last_update = 0;
static int64_t _current_time;
static int64_t _delta;
static float _gx_rads, _gy_rads, _gz_rads;
static gyro_correction_t _correction;

static float map(float value, float r1_min, float r1_max, float r2_min, float r2_max) {
    return value / (r1_max - r1_min) * (r2_max - r2_min) + r2_min;
}

static esp_err_t measure_gyro_correction(gyro_correction_t* correction) {
    mpu_6050_values_t values;

    ESP_LOGI(TAG, "Callibrating...");

    for(int i=0; i<1000; i++) {
        mpu_6050_read(&values);    
        correction->gx = (correction->gx * i + values.gx) / (i+1);
        correction->gy = (correction->gy * i + values.gy) / (i+1);
        correction->gz = (correction->gz * i + values.gz) / (i+1);

        correction->ax = (correction->ax * i + values.ax) / (i+1);
        correction->ay = (correction->ay * i + values.ay) / (i+1);
        correction->az = (correction->az * i + (values.az-1)) / (i+1);
    }

    ESP_LOGI(TAG, "Gyro correction:  gx: %.5f, gy: %.5f, gz: %.5f", correction->gx, correction->gy, correction->gz);
    ESP_LOGI(TAG, "Accel correction: ax: %.5f, ay: %.5f, az: %.5f", correction->ax, correction->ay, correction->az);

    return ESP_OK;
}

static void update_orientation(mpu_6050_values_t* gyro_values) {
    _current_time = esp_timer_get_time();
    if (_last_update > 0) {
        _delta = _current_time - _last_update;
        _gx_rads = DEG_TO_RAD * gyro_values->gx * _delta / 1e6;
        _gy_rads = DEG_TO_RAD * gyro_values->gy * _delta / 1e6;
        _gz_rads = DEG_TO_RAD * gyro_values->gz * _delta / 1e6;

        _orientation.ax_rads = atan2(gyro_values->ay, gyro_values->az);
        _orientation.ay_rads = atan2(-gyro_values->ax, gyro_values->az);

        _orientation.x_rads = 0.98 * (_gx_rads + _orientation.x_rads) + 0.02 * _orientation.ax_rads;
        _orientation.y_rads = 0.98 * (_gy_rads + _orientation.y_rads) + 0.02 * _orientation.ay_rads;
        _orientation.z_rads += _gz_rads;
    }

    _last_update = _current_time;
}

static void read_task(void* arg) {
    mpu_6050_values_t values;
    esp_err_t ret;

    while(_running) {
        ret = mpu_6050_read(&values);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyro: %d", ret);
            continue;
        }

        values.ax -= _correction.ax;
        values.ay -= _correction.ay;
        values.az -= _correction.az;
        values.gx -= _correction.gz;
        values.gy -= _correction.gy;
        values.gz -= _correction.gz;

        xSemaphoreTake(read_lock, portMAX_DELAY);
        update_orientation(&values);
        xSemaphoreGive(read_lock);
    }

    mpu_6050_deinit();

    vTaskDelete(NULL);
}

esp_err_t init_orientation_service(gyro_correction_t* correction, bool measure_correction) {
    _orientation = (orientation_t){ };
    read_lock = xSemaphoreCreateMutex();
    _running = true;
    
    esp_err_t ret = mpu_6050_init();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to init mpu6050");
        return ret;
    }
    
    _correction = (gyro_correction_t){ };
    
    if (correction != NULL) {
        if (measure_correction) {
            measure_gyro_correction(correction);
        }

        memcpy(&_correction, correction, sizeof(gyro_correction_t));
    }

    xTaskCreatePinnedToCore(read_task, "main read task", 4096, NULL, 1, NULL, 1);

    return ESP_OK;
}

esp_err_t get_orientation(orientation_t* orientation) {
    xSemaphoreTake(read_lock, portMAX_DELAY);
    memcpy(orientation, &_orientation, sizeof(orientation_t));
    xSemaphoreGive(read_lock);

    return ESP_OK;
}

esp_err_t stop_orientation_service() {
    _running = false;

    return ESP_OK;
}