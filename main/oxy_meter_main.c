#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "oxy_meter.h"

static const char *TAG = "oxy_meter_main";

/* I2C总线初始化函数 */
i2c_master_bus_handle_t i2c_master_bus_init(int i2c_port)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus =  NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = i2c_port,
        .sda_io_num = i2c_port ? OXY_METER_I2C1_SDA : OXY_METER_I2C0_SDA,
        .scl_io_num = i2c_port ? OXY_METER_I2C1_SCL : OXY_METER_I2C0_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    return i2c_bus;
}

void app_main(void)
{
    /* 初始化两个I2C总线 */
    i2c_master_bus_handle_t i2c0_bus = i2c_master_bus_init(0);
    i2c_master_bus_handle_t i2c1_bus = i2c_master_bus_init(1);
    /* 初始化显示任务和传感器测量任务 */
    oxy_meter_display_task_init(i2c0_bus);
    oxy_meter_sensor_task_init(i2c1_bus);

    float spo2 = 0;
    int32_t heart_rate = 0;
    /* 循环读取测量结果并显示 */
    while (1) {
        oxy_meter_max30102_read_blood_oxygen(&spo2, &heart_rate);
        oxy_meter_display_blood_oxygen(spo2, heart_rate);
    }
}
