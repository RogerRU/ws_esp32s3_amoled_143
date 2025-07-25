
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "lvgl.h"
#include "DRIVERS/TOUCH/touch_bsp.h"
#include "ws_esp32s3_amoled_143.h"
#include "rotation.h"

extern volatile uint32_t touch_count;
extern SemaphoreHandle_t i2c_mutex;

static const char *TAG = "WS_AMOLED";
static SemaphoreHandle_t lvgl_mux = NULL;


rotation_task_t task = { 0 };

uint8_t *buf1 = NULL;
uint8_t *buf2 = NULL;

static lv_indev_t *indev_touchpad = NULL;

esp_err_t amoled_set_brightness(uint8_t percent) {

    task.percent = percent;
    task.update_brightness = 1;
    lcd_bus_lock_release(&task.copy_lock);

    return ESP_OK;
}


static void amoled_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    rotation_task_t *r_task = (rotation_task_t *)lv_display_get_user_data(disp);
    lcd_bus_lock_acquire(&r_task->tx_color_lock);

    r_task->last_update = (uint8_t)lv_display_flush_is_last(disp);
    r_task->partial_buf = px_map;
    r_task->x_start = area->x1;
    r_task->y_start = area->y1;
    r_task->x_end = area->x2;
    r_task->y_end = area->y2;
    r_task->rotation = (uint8_t)lv_display_get_rotation(disp);
    lcd_bus_lock_release(&r_task->copy_lock);
}

static void amoled_lvgl_rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);

    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // round the start of coordinate down to the nearest 2M number
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    // round the end of coordinate up to the nearest 2N+1 number
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}


uint16_t last_tp_x = 0;
uint16_t last_tp_y = 0;


static void amoled_lvgl_touch_cb(lv_indev_t *drv, lv_indev_data_t *data)
{
    uint16_t tp_x;
    uint16_t tp_y;

    uint8_t win = 0;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        win = getTouch(&tp_x, &tp_y);
        xSemaphoreGive(i2c_mutex);
    }

    if (win) {
        last_tp_x = tp_x;
        last_tp_y = tp_y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }

    data->point.x = last_tp_x;
    data->point.y = last_tp_y;
}


static void amoled_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(AMOLED_LVGL_TICK_PERIOD_MS); /* Tell LVGL how many milliseconds has elapsed */
}


static void amoled_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = AMOLED_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        task_delay_ms = lv_timer_handler();
        if (task_delay_ms > AMOLED_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = AMOLED_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < AMOLED_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = AMOLED_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


void amoled_lcd_init()
{

    ESP_LOGI("DISPLAY", "Free heap before init: %d", (int)esp_get_free_heap_size());

    int buffer_size = AMOLED_LCD_H_RES * AMOLED_LCD_V_RES * 3;

    task.active_fb = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    task.idle_fb = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);

    lcd_bus_lock_init(&task.copy_lock);
    lcd_bus_lock_init(&task.tx_color_lock);
    lcd_bus_event_init(&task.copy_task_exit);
    lcd_bus_event_init(&task.swap_bufs);
    lcd_bus_event_set(&task.swap_bufs);
    lcd_bus_lock_init(&task.init_lock);
    lcd_bus_lock_acquire(&task.init_lock);


    lv_init();
    task.disp = lv_display_create(AMOLED_LCD_H_RES, AMOLED_LCD_V_RES);
    lv_display_set_user_data(task.disp, &task);
    lv_display_set_flush_cb(task.disp, amoled_lvgl_flush_cb);
    lv_display_add_event_cb(task.disp, amoled_lvgl_rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    lv_display_set_rotation(task.disp, LV_DISPLAY_ROTATION_0);

    ESP_LOGI(TAG, "Initialize LVGL library");

    int mm = heap_caps_get_largest_free_block( MALLOC_CAP_DMA);
    buf1 = heap_caps_malloc(buffer_size / 10, MALLOC_CAP_INTERNAL);
    buf2 = heap_caps_malloc(buffer_size / 10, MALLOC_CAP_INTERNAL);

    // initialize LVGL draw buffers
    lv_display_set_buffers(task.disp, buf1, buf2, buffer_size / 10, LV_DISPLAY_RENDER_MODE_PARTIAL);

    xTaskCreatePinnedToCore(
        lcd_bus_copy_task, "lcd_task", AMOLED_LVGL_TASK_STACK_SIZE / sizeof(StackType_t),
        &task, ESP_TASK_PRIO_MAX - 1, &task.copy_task_handle, 0);


    lcd_bus_lock_acquire(&task.init_lock);
    lcd__bus_lock_release(&task.init_lock);
    lcd_bus_lock_delete(&task.init_lock);

    if (task.err == 0) {
        ESP_LOGI(TAG, "Install LVGL tick timer");

        // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
        const esp_timer_create_args_t lvgl_tick_timer_args = {
            .callback = &amoled_increase_lvgl_tick,
            .name = "lvgl_tick"};
        esp_timer_handle_t lvgl_tick_timer = NULL;
        ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, AMOLED_LVGL_TICK_PERIOD_MS * 1000));

        ESP_LOGW(TAG, " Display Initialized! ");

        lvgl_mux = xSemaphoreCreateMutex();
        assert(lvgl_mux);

        amoled_touch_init(); // Touch initialization

        xTaskCreatePinnedToCore(
            amoled_lvgl_port_task, "lvgl_task", AMOLED_LVGL_TASK_STACK_SIZE / sizeof(StackType_t),
            &task, AMOLED_LVGL_TASK_PRIORITY, NULL, 1);


        lv_display_set_rotation(task.disp, LV_DISPLAY_ROTATION_90);
    } else {
        ESP_LOGW(TAG, " Display Initialization FAILED! ");
    }
}


void amoled_touch_init()
{
    Touch_Init();
    indev_touchpad = lv_indev_create();

    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_touchpad, amoled_lvgl_touch_cb);
}
