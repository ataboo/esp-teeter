#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "orientation_service.h"
#include "esp_log.h"

static const char* TAG = "ESP TEETER";

void app_main(void)
{
    orientation_t orientation;
    gyro_correction_t correction;

    ESP_ERROR_CHECK(init_orientation_service(&correction, true));
    ESP_LOGI(TAG, "orientation initialized");

    while(true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        get_orientation(&orientation);

        DUMP_ORIENTATION(orientation);
    }
}
