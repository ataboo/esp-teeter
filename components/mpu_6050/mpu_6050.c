#include "mpu_6050.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char* TAG = "MPU_6050";

static uint8_t io_buffer[GYRO_READ_BUFFER_LEN];

static esp_err_t write_byte(uint8_t regAddr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    RETURN_IF_NOT_OK(i2c_master_start(cmd), "i2c start");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, (CONFIG_GYRO_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK), "i2c address write");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK), "i2c select register");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, data, I2C_MASTER_ACK), "i2c write data");
    RETURN_IF_NOT_OK(i2c_master_stop(cmd), "i2c stop write");
    esp_err_t ret = i2c_master_cmd_begin(GYRO_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t select_register(uint8_t regAddr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    RETURN_IF_NOT_OK(i2c_master_start(cmd), "i2c start");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, (CONFIG_GYRO_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK), "i2c address write");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK), "i2c select register");
    RETURN_IF_NOT_OK(i2c_master_stop(cmd), "i2c stop");
    esp_err_t ret = i2c_master_cmd_begin(GYRO_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t read_bytes(uint8_t regAddr, size_t size, uint8_t* buffer) {
    if (size == 0 || size > GYRO_READ_BUFFER_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    RETURN_IF_NOT_OK(select_register(regAddr), "read select register");

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    RETURN_IF_NOT_OK(i2c_master_start(cmd), "read start command");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, (CONFIG_GYRO_I2C_ADDRESS << 1) | I2C_MASTER_READ, 1), "read gyro address");

    if (size > 1) {
        RETURN_IF_NOT_OK(i2c_master_read(cmd, buffer, size-1, I2C_MASTER_ACK), "read single byte");
    }
    RETURN_IF_NOT_OK(i2c_master_read_byte(cmd, buffer + size - 1, I2C_MASTER_NACK), "read multiple bytes");

    RETURN_IF_NOT_OK(i2c_master_stop(cmd), "read stop");
    esp_err_t ret = i2c_master_cmd_begin(GYRO_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t read_byte(uint8_t regAddr, uint8_t *data) {
    return read_bytes(regAddr, 1, data);
}

static uint8_t combine_bit_slice_into_int(uint8_t subject, uint8_t data, uint8_t start, uint8_t length) {
    uint8_t mask = ((1<<length)-1) << (start - length + 1);  // Create a mask of 1's from start to end
    data <<= (start - length + 1);  // Shift data to start position
    data &= mask;  // Keep masked bits in data
    subject &= ~(mask);  // Keep non-masked bits in subject

    return subject | data;  // Combine subject and data
}

static esp_err_t set_bits(uint8_t regAddr, uint8_t data, uint8_t start, uint8_t length) {
    uint8_t val_byte = 0;
    RETURN_IF_NOT_OK(read_byte(regAddr, &val_byte), "read byte for set bit");

    val_byte = combine_bit_slice_into_int(val_byte, data, start, length);

    return write_byte(regAddr, val_byte);
}

static esp_err_t reset_gyro() {
    return write_byte(MPU6050_RA_PWR_MGMT_1, 1<<MPU6050_PWR1_DEVICE_RESET_BIT);
}

static esp_err_t set_gyro_config_values() {
    // Set the clock source as x-axis gyro.  Clear the reset bit as well.
    esp_err_t ret = write_byte(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_XAXIS_CLK);

    // Set the gyro and accel range.
    ret |= set_bits(MPU6050_RA_GYRO_CFG, MPU6050_GYRO_RNG, MPU6050_GYRO_RANGE_BIT, MPU6050_GYRO_RANGE_LENGTH);
    ret |= set_bits(MPU6050_RA_ACCEL_CFG, MPU6050_ACCEL_RNG, MPU6050_ACCEL_RANGE_BIT, MPU6050_ACCEL_RANGE_LENGTH);

    // Set the low pass filter.
    ret |= set_bits(MPU6050_RA_CONFIG, CONFIG_DLPF_CFG_LEVEL, MPU6050_CFG_DLPF_BIT, MPU6050_CFG_DLPF_LENGTH);

    return ret;
}

esp_err_t mpu_6050_init() {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)CONFIG_GYRO_SDA,
        .scl_io_num = (gpio_num_t)CONFIG_GYRO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };

    config.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(GYRO_I2C_NUM, &config));
    ESP_ERROR_CHECK(i2c_driver_install(GYRO_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

    esp_err_t ret = reset_gyro();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset gyro.");
        return ret;
    }

    vTaskDelay(50/portTICK_PERIOD_MS);

    ret = set_gyro_config_values();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro config values.");
        return ret;
    }

    vTaskDelay(50/portTICK_PERIOD_MS);

    return ESP_OK;
}

esp_err_t mpu_6050_read(mpu_6050_values_t* values) {
    esp_err_t ret = read_bytes(MPU6050_RA_ACCEL_XOUT, 6, io_buffer);
    values->ax_raw = (int16_t)(io_buffer[0]<<8 | io_buffer[1]);
    values->ay_raw = (int16_t)(io_buffer[2]<<8 | io_buffer[3]);
    values->az_raw = (int16_t)(io_buffer[4]<<8 | io_buffer[5]);

    values->ax_fl = (float)values->ax_raw / MPU6050_ACCEL_LSB;
    values->ay_fl = (float)values->ay_raw / MPU6050_ACCEL_LSB;
    values->az_fl = (float)values->az_raw / MPU6050_ACCEL_LSB;
    
    // if (values->norm_accel_x == 0 || values->norm_accel_y == 0) {
    //     return ESP_OK;
    // }

    ret |= read_bytes(MPU6050_RA_GYRO_XOUT, 6, io_buffer);

    values->gx_raw = (int16_t)(io_buffer[0]<<8 | io_buffer[1]);
    values->gy_raw = (int16_t)(io_buffer[2]<<8 | io_buffer[3]);
    values->gz_raw = (int16_t)(io_buffer[4]<<8 | io_buffer[5]);

    values->gx_fl = values->gx_raw / MPU6050_GYRO_LSB;
    values->gy_fl = values->gy_raw / MPU6050_GYRO_LSB;
    values->gz_fl = values->gz_raw / MPU6050_GYRO_LSB;

    // values->accel_x_component = ((atan((values->norm_accel_y) / sqrt(pow((values->norm_accel_x), 2) + pow((values->norm_accel_z), 2))) * RAD_TO_DEG));
    // values->accel_y_component = ((atan(-1 * (values->norm_accel_x) / sqrt(pow((values->norm_accel_y), 2) + pow((values->norm_accel_z), 2))) * RAD_TO_DEG));

    // if (apply_error) {
    //     values->accel_x_component -= state->error->accel_err_x;
    //     values->accel_y_component -= state->error->accel_err_y;

    //     values->norm_gyro_x -= state->error->gyro_err_x;
    //     values->norm_gyro_y -= state->error->gyro_err_y;
    //     values->norm_gyro_z -= state->error->gyro_err_z;
    // }

    // state->current_micros = esp_timer_get_time();
    // values->delta_micros = state->current_micros - state->last_micros;
    // state->last_micros = state->current_micros;

    // state->gyro_roll += values->norm_gyro_x * values->delta_micros / 1e6;
    // state->gyro_pitch += values->norm_gyro_y * values->delta_micros / 1e6;
    // state->gyro_yaw += values->norm_gyro_z * values->delta_micros / 1e6; 

    // values->roll_rads = 0.96 * state->gyro_roll + 0.04 * values->accel_x_component;
    // values->pitch_rads = 0.96 * state->gyro_pitch + 0.04 * values->accel_y_component;
    // values->yaw_rads = state->gyro_yaw;

    return ret;
}