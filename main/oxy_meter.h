#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

#define MAX30102_INTR_GPIO   42
#define OXY_METER_I2C0_SCL   4
#define OXY_METER_I2C0_SDA   5
#define OXY_METER_I2C1_SCL   1
#define OXY_METER_I2C1_SDA   2


void oxy_meter_display_task_init(i2c_master_bus_handle_t i2c_bus);

void oxy_meter_display_blood_oxygen(float spo2, int32_t hr);

void oxy_meter_sensor_task_init(i2c_master_bus_handle_t i2c_bus);

void oxy_meter_max30102_read_blood_oxygen(float *spo2, int32_t *hr);
