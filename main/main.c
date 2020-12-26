#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "orientation_service.h"
#include "esp_log.h"

static const char* TAG = "ESP TEETER";

void app_main(void)
{
    init_orientation_service();
    orientation_t orientation;

    while(true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        get_orientation(&orientation);

        DUMP_ORIENTATION(orientation);
    }
}
