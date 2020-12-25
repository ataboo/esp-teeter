#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu_6050.h"
#include "esp_log.h"

static const char* TAG = "ESP TEETER";

// static void dump_gyro_vals(mpu_6050_values_t values) {
//     ESP_LOGI(TAG, "%x, %x, %x, %x, %x, %x", values.raw_accel[0], values.raw_accel[1], values.raw_accel[2], values.raw_accel[3], values.raw_accel[4], values.raw_accel[5]);
// }

void app_main(void)
{
    mpu_6050_values_t values;

    esp_err_t ret = mpu_6050_init();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init gyro: %d", ret);   
    }

    while(true) {
        esp_err_t ret = mpu_6050_read(&values);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyro: %d", ret);
        }
        ESP_LOGI(TAG, "ax: %.5f, ay: %.5f, az: %.5f, gx: %.5f, gy: %.5f, gz: %.5f", values.ax_fl, values.ay_fl, values.az_fl, values.gx_fl, values.gy_fl, values.gz_fl);
        // dump_gyro_vals(values);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
