#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"



#define LCD_HOST    SPI2_HOST



#define SH8601_ID 0x86
#define CO5300_ID 0xff

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL       (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL       (16)
#endif

#define AMOLED_LCD_BK_LIGHT_ON_LEVEL  1
#define AMOLED_LCD_BK_LIGHT_OFF_LEVEL !AMOLED_LCD_BK_LIGHT_ON_LEVEL
#define AMOLED_PIN_NUM_LCD_CS            (GPIO_NUM_9)
#define AMOLED_PIN_NUM_LCD_PCLK          (GPIO_NUM_10)
#define AMOLED_PIN_NUM_LCD_DATA0         (GPIO_NUM_11)
#define AMOLED_PIN_NUM_LCD_DATA1         (GPIO_NUM_12)
#define AMOLED_PIN_NUM_LCD_DATA2         (GPIO_NUM_13)
#define AMOLED_PIN_NUM_LCD_DATA3         (GPIO_NUM_14)
#define AMOLED_PIN_NUM_LCD_RST           (GPIO_NUM_21)
#define AMOLED_PIN_NUM_BK_LIGHT          (-1)

// The pixel number in horizontal and vertical
#define AMOLED_LCD_H_RES              466
#define AMOLED_LCD_V_RES              466


#define AMOLED_LVGL_BUF_HEIGHT        (AMOLED_LCD_V_RES / 4)
#define AMOLED_LVGL_TICK_PERIOD_MS    2
#define AMOLED_LVGL_TASK_MAX_DELAY_MS 500
#define AMOLED_LVGL_TASK_MIN_DELAY_MS 1
#define AMOLED_LVGL_TASK_STACK_SIZE   (8 * 1024)
#define AMOLED_LVGL_TASK_PRIORITY     2

#define LVGL_TICK_PERIOD_MS            2


//void rotate_image_buffer(lv_color_t* img_buf, uint16_t width, uint16_t height, lv_display_rotation_t rotation, lv_color_t* rotated_buf);
void  amoled_lcd_init(void);
void  amoled_touch_init(void);
esp_err_t amoled_set_brightness(uint8_t percent);
static void amoled_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static void amoled_lvgl_rounder_cb(lv_event_t *e);
static void amoled_lvgl_touch_cb(lv_indev_t *drv, lv_indev_data_t *data);
static void amoled_lvgl_port_task(void *arg);
static void amoled_increase_lvgl_tick(void *arg);





