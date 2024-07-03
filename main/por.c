#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "unistd.h"

#include "simple_read.c"

#define GPIO_NUM_4 26
#define GPIO_NUM_22 22 // Define the I2C pins for your setup
#define GPIO_NUM_21 21

#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define LCD_ADDR 0x27             // I2C address of the LCD1602

#define SLAVE_ADDRESS_LCD 0x4E >> 1

void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_put_cur(int row, int col);

esp_err_t err;
static const char *TAG = "LCD1602 Example";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &conf);

    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

// esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_adress,
//                                      const uint8_t *write_buffer, size_t write_size,
//                                      TickType_t tikes_to_wait)

void lcd_send_cmd(char cmd)
{
    esp_err_t err;
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = data_u | 0x08; // en=0, rs=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = data_l | 0x08; // en=0, rs=0

    err = i2c_master_write_to_device(I2C_NUM_0, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
    if (err != 0)
    {
        ESP_LOGI(TAG, "Error command");
    }
}

void lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; // en=1, rs=0
    data_t[1] = data_u | 0x09; // en=0, rs=0
    data_t[2] = data_l | 0x0D; // en=1, rs=0
    data_t[3] = data_l | 0x09; // en=0, rs=0
    err = i2c_master_write_to_device(I2C_NUM_0, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
    if (err != 0)
    {
        ESP_LOGI(TAG, "Error  data");
    }
}

void lcd_init(void)
{
    usleep(50000); // wait for >40ms
    lcd_send_cmd(0x30);
    usleep(4500); // wait for >4.1ms
    lcd_send_cmd(0x30);
    usleep(200); // wait for >100us
    lcd_send_cmd(0x30);
    usleep(200);
    lcd_send_cmd(0x20); // 4bit mode
    usleep(200);

    // dislay initialisation
    lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    usleep(1000);
    lcd_send_cmd(0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
    usleep(1000);
    lcd_send_cmd(0x01); // clear display
    usleep(1000);
    usleep(1000);
    lcd_send_cmd(0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    usleep(1000);
    lcd_send_cmd(0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
    usleep(2000);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
    case 0:
        col |= 0x80;
        break;
    case 1:
        col |= 0xC0;
        break;
    }

    lcd_send_cmd(col);
}

void lcd_send_string(char *str)
{
    while (*str)
    {
        lcd_send_data(*str++);
    }
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    usleep(5000);
}

char buffer[20];
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    while (1)
    {
        lcd_init();
        lcd_clear();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sprintf(buffer, "Tem = %d Celsius", DHT11_read().temperature);
        lcd_put_cur(0, 0);
        lcd_send_string(buffer);

        sprintf(buffer, "Hum = %d Percent", DHT11_read().humidity);
        lcd_put_cur(1, 0);
        lcd_send_string(buffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        DHT11_init(GPIO_NUM_4);

        printf("Temperature is %d \n", DHT11_read().temperature);
        printf("Humidity is %d\n", DHT11_read().humidity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
