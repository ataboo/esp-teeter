#pragma once

#include "sdkconfig.h"
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "teeter_common.h"

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define GYRO_READ_BUFFER_LEN            16
#define GYRO_ERROR_SAMPLE_COUNT         500

#define MPU6050_RA_WHO_AM_I             0x75
#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_RA_GYRO_CFG             0x1B
#define MPU6050_RA_ACCEL_CFG            0x1C
#define MPU6050_RA_GYRO_XOUT            0x43
#define MPU6050_RA_ACCEL_XOUT           0x3B
#define MPU6050_RA_CONFIG               0x1A
#define MPU6050_RA_SMPL_DIV             0x19

#define MPU6050_RA_INT_CFG              0x37
#define MPU6050_RA_INT_EN               0x38

// 0b11000000
#define MPU6050_INT_CFG_BYTE            0x40
// 0b00000001
#define MPU6050_INT_EN_BYTE             0x01

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CFG_DLPF_BIT            2
#define MPU6050_CFG_DLPF_LENGTH         3

#define MPU6050_GYRO_RANGE_BIT          4
#define MPU6050_GYRO_RANGE_LENGTH       2

#define MPU6050_ACCEL_RANGE_BIT         4
#define MPU6050_ACCEL_RANGE_LENGTH      2

#define MPU6050_RDY_INT_BIT             0
#define MPU6050_RDY_INT_LENGTH          1

#define MPU6050_PWR1_XAXIS_CLK          0x01

#ifdef CONFIG_GYRO_RANGE_0
    #define MPU6050_GYRO_LSB      131
    #define MPU6050_GYRO_RNG      0
#endif
#ifdef CONFIG_GYRO_RANGE_1
    #define MPU6050_GYRO_LSB      65.5
    #define MPU6050_GYRO_RNG      1
#endif
#ifdef CONFIG_GYRO_RANGE_2
    #define MPU6050_GYRO_LSB      32.8
    #define MPU6050_GYRO_RNG      2
#endif
#ifdef CONFIG_GYRO_RANGE_3
    #define MPU6050_GYRO_LSB      16.4
    #define MPU6050_GYRO_RNG      3
#endif

#ifdef CONFIG_ACCEL_RANGE_0
    #define MPU6050_ACCEL_LSB     16384
    #define MPU6050_ACCEL_RNG     0
#endif
#ifdef CONFIG_ACCEL_RANGE_1
    #define MPU6050_ACCEL_LSB     8192
    #define MPU6050_ACCEL_RNG     1
#endif
#ifdef CONFIG_ACCEL_RANGE_2
    #define MPU6050_ACCEL_LSB     4096
    #define MPU6050_ACCEL_RNG     2
#endif
#ifdef CONFIG_ACCEL_RANGE_3
    #define MPU6050_ACCEL_LSB     2048
    #define MPU6050_ACCEL_RNG     3
#endif

#ifdef CONFIG_I2C_DEVICE_NUM_0
    #define MPU6050_I2C_NUM       I2C_NUM_0
#endif

#ifdef CONFIG_I2C_DEVICE_NUM_1
    #define MPU6050_I2C_NUM       I2C_NUM_1
#endif

#if CONFIG_DLPF_CFG_LEVEL == 0 || CONFIG_DPLF_CFG_LEVEL == 7
    #define MPU6050_BASE_SMPL_DIV_MULTIPLIER 8
#else
    #define MPU6050_BASE_SMPL_DIV_MULTIPLIER 1
#endif

#define MPU6050_SMPL_DIV 1 * MPU6050_BASE_SMPL_DIV_MULTIPLIER                

#define DUMP_GYRO_VALUES(VALUES) ({                                                 \
    ESP_LOGI(TAG, "ax: %.5f, ay: %.5f, az: %.5f, gx: %.5f, gy: %.5f, gz: %.5f",     \
        (VALUES).ax,                                                                \
        (VALUES).ay,                                                                \
        (VALUES).az,                                                                \
        (VALUES).gx,                                                                \
        (VALUES).gy,                                                                \
        (VALUES).gz                                                                 \
    );                                                                              \
})

#define CREATE_MPU6050_QUEUE() ({return xQueueCreate(8, sizeof(mpu_6050_values_t));})

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} mpu_6050_values_t;

esp_err_t mpu_6050_init();

esp_err_t mpu_6050_deinit();

esp_err_t mpu_6050_read(mpu_6050_values_t* values);
