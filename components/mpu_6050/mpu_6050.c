#include "mpu_6050.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include <string.h>
#include "driver/gpio.h"

static const char* TAG = "MPU_6050";

static uint8_t io_buffer[GYRO_READ_BUFFER_LEN];

static SemaphoreHandle_t isr_semaphore = NULL;
static SemaphoreHandle_t read_lock = NULL;

#ifdef CONFIG_MPU6050_INTERRUPT
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    xSemaphoreGiveFromISR(isr_semaphore, NULL);
}
#endif

static esp_err_t write_byte(uint8_t regAddr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    RETURN_IF_NOT_OK(i2c_master_start(cmd), "i2c start");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, (CONFIG_GYRO_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK), "i2c address write");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK), "i2c select register");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, data, I2C_MASTER_ACK), "i2c write data");
    RETURN_IF_NOT_OK(i2c_master_stop(cmd), "i2c stop write");
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t select_register(uint8_t regAddr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    RETURN_IF_NOT_OK(i2c_master_start(cmd), "i2c start");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, (CONFIG_GYRO_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK), "i2c address write");
    RETURN_IF_NOT_OK(i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK), "i2c select register");
    RETURN_IF_NOT_OK(i2c_master_stop(cmd), "i2c stop");
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
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
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
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
    RETURN_IF_NOT_OK(write_byte(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_XAXIS_CLK), "set power management");

    // Set the gyro and accel range.
    RETURN_IF_NOT_OK(set_bits(MPU6050_RA_GYRO_CFG, MPU6050_GYRO_RNG, MPU6050_GYRO_RANGE_BIT, MPU6050_GYRO_RANGE_LENGTH), "set gyro config");
    RETURN_IF_NOT_OK(set_bits(MPU6050_RA_ACCEL_CFG, MPU6050_ACCEL_RNG, MPU6050_ACCEL_RANGE_BIT, MPU6050_ACCEL_RANGE_LENGTH), "set accel config");

    // Set the low pass filter.
    RETURN_IF_NOT_OK(set_bits(MPU6050_RA_CONFIG, CONFIG_DLPF_CFG_LEVEL, MPU6050_CFG_DLPF_BIT, MPU6050_CFG_DLPF_LENGTH), "set low pass filter");

    #ifdef CONFIG_MPU6050_INTERRUPT
        RETURN_IF_NOT_OK(write_byte(MPU6050_RA_INT_CFG, MPU6050_INT_CFG_BYTE), "set interrupt config register");
        RETURN_IF_NOT_OK(write_byte(MPU6050_RA_INT_EN, MPU6050_INT_EN_BYTE), "set interrupt enable register");
    #else
        RETURN_IF_NOT_OK(set_bits(MPU6050_RA_INT_EN, 0, MPU6050_RDY_INT_BIT, MPU_6050_RDY_INT_LENGTH), "write disable interrupt");
    #endif

    RETURN_IF_NOT_OK(write_byte(MPU6050_RA_SMPL_DIV, MPU6050_SMPL_DIV), "set sample rate divider");

    return ESP_OK;
}

esp_err_t mpu_6050_init() {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)CONFIG_MPU6050_SDA_PIN_NUM,
        .scl_io_num = (gpio_num_t)CONFIG_MPU6050_SCL_PIN_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };

    config.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(MPU6050_I2C_NUM, &config));
    ESP_ERROR_CHECK(i2c_driver_install(MPU6050_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

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

    read_lock = xSemaphoreCreateMutex();

#ifdef CONFIG_MPU6050_INTERRUPT
    isr_semaphore = xSemaphoreCreateBinary();
    xSemaphoreTake(isr_semaphore, 0);

    gpio_config_t int_pin_cfg = {
        .pin_bit_mask = (1ULL<<CONFIG_MPU6050_INT_PIN_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = false,
        .pull_up_en = false,
        .intr_type = GPIO_INTR_POSEDGE
    };

    RETURN_IF_NOT_OK(gpio_install_isr_service(0), "isr service install");
    RETURN_IF_NOT_OK(gpio_config(&int_pin_cfg), "init gpio interrupt pin");
    RETURN_IF_NOT_OK(gpio_isr_handler_add(CONFIG_MPU6050_INT_PIN_NUM, gpio_isr_handler, NULL), "ISR handler add");
#endif
    
    return ESP_OK;
}

static esp_err_t read_accel(mpu_6050_values_t* values) {
    RETURN_IF_NOT_OK(read_bytes(MPU6050_RA_ACCEL_XOUT, 6, io_buffer), "read accel values");
    values->ax = (float)((int16_t)(io_buffer[0]<<8 | io_buffer[1])) / MPU6050_ACCEL_LSB;
    values->ay = (float)((int16_t)(io_buffer[2]<<8 | io_buffer[3])) / MPU6050_ACCEL_LSB;
    values->az = (float)((int16_t)(io_buffer[4]<<8 | io_buffer[5])) / MPU6050_ACCEL_LSB;

    return ESP_OK;
}

static esp_err_t read_gyro(mpu_6050_values_t* values) {
    RETURN_IF_NOT_OK(read_bytes(MPU6050_RA_GYRO_XOUT, 6, io_buffer), "read gyro values");
    values->gx = (float)((int16_t)(io_buffer[0]<<8 | io_buffer[1])) / MPU6050_GYRO_LSB;
    values->gy = (float)((int16_t)(io_buffer[2]<<8 | io_buffer[3])) / MPU6050_GYRO_LSB;
    values->gz = (float)((int16_t)(io_buffer[4]<<8 | io_buffer[5])) / MPU6050_GYRO_LSB;

    return ESP_OK;     
}

esp_err_t mpu_6050_read(mpu_6050_values_t* values) {
    esp_err_t ret;

#ifdef CONFIG_MPU6050_INTERRUPT
    xSemaphoreTake(isr_semaphore, portMAX_DELAY);
#endif

    xSemaphoreTake(read_lock, portMAX_DELAY);
    ret = read_accel(values);
    ret |= read_gyro(values);
    xSemaphoreGive(read_lock);

    return ret;
}

esp_err_t mpu_6050_deinit() {
    i2c_driver_delete(MPU6050_I2C_NUM);

#ifdef CONFIG_MPU6050_INTERRUPT
    gpio_intr_disable(CONFIG_MPU6050_INT_PIN_NUM);
    gpio_isr_handler_remove(CONFIG_MPU6050_INT_PIN_NUM);
    gpio_uninstall_isr_service();
#endif

    return ESP_OK;
}