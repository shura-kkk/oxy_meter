#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "oxy_meter.h"
#include "driver_max30102.h"
#include "driver_max30102_fifo.h"
#include "driver_max30102_impl_esp.h"
#include "algorithm.h"
#include "esp_check.h"

const static char *TAG = "oxy_meter_sensor_task";

#define BUF_LEN     1024

static SemaphoreHandle_t s_intr_sem = NULL;
static QueueHandle_t s_queue = NULL;
static uint32_t *gs_raw_red = NULL;
static uint32_t *gs_raw_ir = NULL;

typedef struct {
    float spo2;
    int8_t spo2_valid;
    int32_t heart_rate;
    int8_t hr_valid;
} max30102_calc_result_t;

static void max30102_event_callback(uint8_t type)
{
    static uint32_t cnt = 0;
    switch (type) {
    case MAX30102_INTERRUPT_STATUS_FIFO_FULL :
        max30102_interface_debug_print("irq fifo full\n");
        break;
    case MAX30102_INTERRUPT_STATUS_PPG_RDY :
        uint8_t res = 0;
        /* read data */
        uint32_t len = 32;
        res = max30102_fifo_read(gs_raw_red + cnt, gs_raw_ir + cnt, &len);
        if (res != 0)  {
            max30102_interface_debug_print("read failed.\n");
            break;
        }
        cnt += len;
        if (cnt >= 50) {
            max30102_calc_result_t result = {};
            maxim_heart_rate_and_oxygen_saturation(gs_raw_ir, cnt, gs_raw_red, 
                                                    &result.spo2, &result.spo2_valid, 
                                                    &result.heart_rate, &result.hr_valid);
            ESP_LOGI(TAG, "%"PRIu32" Result get!", cnt);
            xQueueOverwrite(s_queue, &result);
            cnt = 0;
        }
        break;
    case MAX30102_INTERRUPT_STATUS_ALC_OVF :
        max30102_interface_debug_print("irq alc ovf.\n");
        break;
    case MAX30102_INTERRUPT_STATUS_PWR_RDY :
        max30102_interface_debug_print("irq pwr rdy.\n");
        break;
    case MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY :
        max30102_interface_debug_print("irq die temp rdy.\n");
        break;
    default :
        max30102_interface_debug_print("unknow code.\n");
        break;
    }
}

static void IRAM_ATTR max30102_gpio_intr_handler(void *arg)
{
    BaseType_t need_yield;
    xSemaphoreGiveFromISR(s_intr_sem, &need_yield);
}

static void gpio_intr_init(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << MAX30102_INTR_GPIO,
        .pull_up_en = true,
    };
    //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_intr_type(MAX30102_INTR_GPIO, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MAX30102_INTR_GPIO, max30102_gpio_intr_handler, NULL));
}

static void oxy_meter_sensor_task(void *arg)
{
    while (1) {
        xSemaphoreTake(s_intr_sem, portMAX_DELAY);
        max30102_fifo_irq_handler();
    }
}

void oxy_meter_sensor_task_init(i2c_master_bus_handle_t i2c_bus)
{
    s_intr_sem = xSemaphoreCreateBinary();
    if (!s_intr_sem) {
        ESP_LOGE(TAG, "no mem for interrupt semaphore");
        abort();
    }
    s_queue = xQueueCreate(1, sizeof(max30102_calc_result_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "no mem for interrupt queue");
        abort();
    }
    gs_raw_red = calloc(1, BUF_LEN * sizeof(uint32_t));
    gs_raw_ir = calloc(1, BUF_LEN * sizeof(uint32_t));
    if (gs_raw_red && gs_raw_ir == 0) {
        ESP_LOGE(TAG, "no mem for data buffer");
        abort();
    }

    gpio_intr_init();
    ESP_ERROR_CHECK(i2c_bus_add_max30102_dev(i2c_bus));
    max30102_fifo_init(max30102_event_callback);

    xTaskCreate(oxy_meter_sensor_task, "oxy_meter_sensor_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "max30102 sensor task started");
}

void oxy_meter_max30102_read_blood_oxygen(float *spo2, int32_t *hr)
{
    max30102_calc_result_t res = {};
    xQueueReset(s_queue);  // 清除遗留数据
    xQueueReceive(s_queue, &res, portMAX_DELAY);
    if (res.spo2_valid) {
        *spo2 = res.spo2;
    } else {
        *spo2 = -1;
    }
    if (res.hr_valid) {
        *hr = res.heart_rate / 2;
    } else {
        *hr = -1;
    }
}
