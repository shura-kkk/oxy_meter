#pragma once

#include "driver/i2c_master.h"
#include "esp_log.h"

esp_err_t i2c_bus_add_max30102_dev(i2c_master_bus_handle_t i2c_bus);
