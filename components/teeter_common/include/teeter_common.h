#pragma once

#define RETURN_IF_NOT_OK(ERROR, DESCRIPTION) ({              \
    if ((ERROR) != ESP_OK) {                                 \
        ESP_LOGE(TAG, "Error returned: %s", (DESCRIPTION));  \
        return (ERROR);                                      \
    }                                                        \
})  
