/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_continuous.h"
#include <string.h>
static const char *TAG = "example";

#define EXPOSURE_ADC_FRAME_SIZE 32
#define EXPOSURE_ADC_UNIT ADC_UNIT_1
#define EXPOSURE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXPOSURE_ADC_ATTENUATION ADC_ATTEN_DB_0
#define EXPOSURE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH
#define EXPOSURE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
volatile int s_exposure = 1000;
static adc_channel_t channel[] = { ADC_CHANNEL_3 };

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void* user_data)
{
    BaseType_t mustYield = pdFALSE;
    TaskHandle_t task_handle = (TaskHandle_t) user_data;
    vTaskNotifyGiveFromISR(task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

static void exposure_adc_init(adc_channel_t* channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size=1024,
        .conv_frame_size=EXPOSURE_ADC_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_config = {
        .sample_freq_hz = 20000,
        .conv_mode = EXPOSURE_ADC_CONV_MODE,
        .format = EXPOSURE_ADC_OUTPUT_TYPE,
    };
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_config.pattern_num = channel_num;
    for (int i=0; i<channel_num; i++) 
    {
        adc_pattern[i].atten = EXPOSURE_ADC_ATTENUATION;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXPOSURE_ADC_UNIT;
        adc_pattern[i].bit_width = EXPOSURE_ADC_BIT_WIDTH;
    }
    dig_config.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_config));
    *out_handle = handle;

}
#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT  -100

#define EXAMPLE_GPIO_A 17
#define EXAMPLE_GPIO_B 25
#define TRIGGER_GPIO 23
#define TRIGGER_TIMEBASE_RESOLUTION_HZ 100000
#define TRIGGER_TIMEBASE_PERIOD 20000 // 200ms

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}
/*
static bool timer_stopped(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    xQueueSendFromISR(queue, &(edata->count_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

static bool timer_empty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    xQueueSendFromISR(queue, &(edata->count_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}
*/

void exposure_adc_task()
{
    uint8_t buffer[EXPOSURE_ADC_FRAME_SIZE] = {0};
    memset(buffer, 0xCC,EXPOSURE_ADC_FRAME_SIZE);
    adc_continuous_handle_t handle = NULL;
    exposure_adc_init(channel, 1, &handle);
    adc_continuous_evt_cbs_t callbacks = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &callbacks, xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            uint32_t length;
            int ret = adc_continuous_read(handle, buffer, EXPOSURE_ADC_FRAME_SIZE , &length, 0);
            if (ESP_OK != ret) break;
                    
            for (int i=0; i<length; i+=SOC_ADC_DIGI_RESULT_BYTES) 
            {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&buffer[i];
                uint32_t c = p->type1.channel;
                uint32_t d = p->type1.data;
                s_exposure = d;
                ESP_LOGI("EXPOSURE", "%lu => %lu", c, d);
            }
            vTaskDelay(1);
        }
    }
}
void app_main(void)
{
    TaskHandle_t exposure_adc_task_handle;
    xTaskCreate( exposure_adc_task, "EXPOSURE TASK", 1<<12, NULL, tskIDLE_PRIORITY, &exposure_adc_task_handle );


    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = 100,
        .low_limit = -100,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_GPIO_A,
        .level_gpio_num = EXAMPLE_GPIO_B
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 1));
    
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // Setup MCPWM
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TRIGGER_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = TRIGGER_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = TRIGGER_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));


    mcpwm_sync_handle_t sync = NULL;
    mcpwm_soft_sync_config_t sync_config = {};
    ESP_ERROR_CHECK(mcpwm_new_soft_sync_src(&sync_config,&sync));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 1000));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                MCPWM_GEN_TIMER_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));


    mcpwm_timer_sync_phase_config_t phase_config = {
        .sync_src = sync,
        .count_value = 0,
        .direction = MCPWM_TIMER_DIRECTION_UP 
    };
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timer, &phase_config));

    //mcpwm_timer_event_callbacks_t timer_cbs = {
    //    .on_stop = timer_stopped, 
    //};
    //mcpwm_timer_register_event_callbacks(timer, &timer_cbs, queue);
    
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    //ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_STOP_FULL));
    
    
    // Report counter value
    int pulse_count = 0;
    int event_count = 0;
    while (1) {
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
            ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
            if (event_count > 0)
            {
                ESP_ERROR_CHECK(mcpwm_timer_set_period(timer, s_exposure));
                ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_STOP_FULL));
                pcnt_unit_clear_count(pcnt_unit);
            }
        }
        else
        {
            pcnt_unit_get_count(pcnt_unit, &pulse_count);
            ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        }
    }
}
