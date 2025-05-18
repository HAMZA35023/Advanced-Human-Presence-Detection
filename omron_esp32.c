#include "omron_esp32.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "OMRON"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define D6T_ADDR 0x0A
#define D6T_CMD  0x4D
#define D6T_SET_ADD 0x01

#define N_ROW 32
#define N_PIXEL (32 * 32)
#define N_READ ((N_PIXEL + 1) * 2 + 1)

static uint8_t rbuf[N_READ];
static double ptat;
static double pix_data[N_PIXEL];

#define D6T_IIR 0x00
#define D6T_AVERAGE 0x04

static uint8_t calc_crc(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        uint8_t temp = data;
        data <<= 1;
        if (temp & 0x80) {
            data ^= 0x07;
        }
    }
    return data;
}

static bool D6T_checkPEC(uint8_t buf[], int n) {
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);
    for (int i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    if (crc != buf[n]) {
        ESP_LOGW(TAG, "PEC check failed: calculated=0x%02X, received=0x%02X", crc, buf[n]);
        return false;
    }
    return true;
}

static int16_t conv8us_s16_le(uint8_t* buf, int n) {
    uint16_t ret = buf[n] + (buf[n + 1] << 8);
    return (int16_t)ret;
}

static esp_err_t d6t_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, D6T_ADDR, data, sizeof(data), 100 / portTICK_PERIOD_MS);
}

static esp_err_t d6t_read_data() {
    uint8_t cmd = D6T_CMD;
    return i2c_master_write_read_device(I2C_MASTER_NUM, D6T_ADDR, &cmd, 1, rbuf, N_READ, 200 / portTICK_PERIOD_MS);
}

esp_err_t omron_i2c_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    esp_err_t err;
    err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(350));
    uint8_t config = ((D6T_IIR << 4) & 0xF0) | (D6T_AVERAGE & 0x0F);
    return d6t_write_register(D6T_SET_ADD, config);
}

void omron_i2c_scan() {
    printf("Starting I2C scan...\n");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Found I2C device at 0x%02X\n", addr);
        }
    }
    printf("I2C scan complete.\n");
}

esp_err_t omron_read() {
    memset(rbuf, 0, N_READ);
    esp_err_t ret = d6t_read_data();
    if (ret != ESP_OK) return ret;

    if (!D6T_checkPEC(rbuf, N_READ - 1)) {
        return ESP_ERR_INVALID_CRC;
    }

    ptat = (double)conv8us_s16_le(rbuf, 0) / 10.0;
    for (int i = 0; i < N_PIXEL; i++) {
        int16_t temp = conv8us_s16_le(rbuf, 2 + 2 * i);
        pix_data[i] = (double)temp / 10.0;
    }

    printf("PTAT: %.1f [°C], Temperature:\n", ptat);
    for (int i = 0; i < N_PIXEL; i++) {
        printf("%.1f, ", pix_data[i]);
        if ((i + 1) % 32 == 0) printf("\n");
    }

    return ESP_OK;
}
