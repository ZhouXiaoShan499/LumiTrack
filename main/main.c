/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief LumiTrack - Smart Focus Light UI with camera live view
 * @details Displays current activity status, focus timer, and daily total.
 *          Press button to toggle between main UI and camera live preview.
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ── Camera (V4L2) ── */
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <linux/videodev2.h>
#include "esp_video_init.h"
#include "esp_video_device.h"

#if BSP_CAPS_IMU
#include "qma6100p.h"
#endif
#if BSP_CAPS_BUTTONS
#include "iot_button.h"
#endif

static const char *TAG = "LumiTrack";

static lv_disp_t *display;
static lv_disp_rotation_t rotation = LV_DISPLAY_ROTATION_0;

/* ── LumiTrack UI Objects ── */
static lv_obj_t *lbl_title;
static lv_obj_t *lbl_status_mode;       /* "当前状态：自动识别 · 看书模式" */
static lv_obj_t *lbl_focus_time;        /* large timer (reused from old code) */
static lv_obj_t *lbl_focus_desc;        /* "本次专注时长 | 实时统计" */
static lv_obj_t *lbl_daily_stats;       /* "📊 今日累计  专注 04:12  休息 00:45" */
static lv_obj_t *lbl_quick_tags;        /* "🏷️ 看书 学习 看电脑 看手机" */
static lv_obj_t *lbl_light_status;      /* light bar text (reused from old code) */
static lv_obj_t *lbl_perf;              /* optional FPS/CPU debug bar */

/* ── Screen management ── */
static lv_obj_t *main_screen = NULL;
static lv_obj_t *camera_screen = NULL;
static lv_obj_t *settings_screen = NULL;
static lv_obj_t *cam_img = NULL;
static bool is_camera_view = false;
static bool is_settings_view = false;

/* ── Settings menu ── */
#define SETTINGS_MENU_COUNT 5
static const char *settings_menu_items[SETTINGS_MENU_COUNT] = {
    "自定义行为标签",
    "自动识别传感器",
    "屏幕亮度调节",
    "本地数据导出",
    "设备恢复出厂"
};
static int settings_menu_index = 0;
static lv_obj_t *settings_menu_list = NULL;

/* ── Camera frame buffer ── */
#define CAM_WIDTH  240
#define CAM_HEIGHT 240
static uint8_t *cam_frame_buf[2] = {NULL, NULL};
static int cam_frame_ready = -1;  /* index of latest ready frame, or -1 */
static SemaphoreHandle_t cam_frame_mutex = NULL;

static lv_image_dsc_t cam_img_dsc;

/* ── Timer state ── */
static uint32_t focus_seconds = 0;       /* Current focus session seconds */
static uint32_t daily_seconds = 15120;   /* Today's total seconds (04:12:00) */

#if BSP_CAPS_IMU
static qma6100p_dev_t *imu = NULL;
#endif
#if BSP_CAPS_BUTTONS
static button_handle_t btn_handles[BSP_BUTTON_NUM] = {0};
static int btn_cnt = 0;
#endif

/*******************************************************************************
 * Helper: format seconds to HH:MM:SS
 *******************************************************************************/
static void format_time(uint32_t total_sec, char *buf, size_t buf_size)
{
    uint32_t h = total_sec / 3600;
    uint32_t m = (total_sec % 3600) / 60;
    uint32_t s = total_sec % 60;
    snprintf(buf, buf_size, "%02lu:%02lu:%02lu", h, m, s);
}

/*******************************************************************************
 * LVGL timer callback: update focus & daily time every second
 *******************************************************************************/
static void lumi_timer_cb(lv_timer_t *timer)
{
    focus_seconds++;
    daily_seconds++;

    char buf[16], line[64];
    format_time(focus_seconds, buf, sizeof(buf));
    if (!is_camera_view) {
        lv_label_set_text(lbl_focus_time, buf);
    }

    format_time(daily_seconds, buf, sizeof(buf));
    if (!is_camera_view) {
        snprintf(line, sizeof(line), "today  %s  |  rest", buf);
        lv_label_set_text(lbl_daily_stats, line);
    }
}

/*******************************************************************************
 * Private functions – Button & IMU (kept for hardware compatibility)
 *******************************************************************************/

static uint16_t app_lvgl_get_rotation_degrees(lv_disp_rotation_t rotation)
{
    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        return 0;
    case LV_DISPLAY_ROTATION_90:
        return 90;
    case LV_DISPLAY_ROTATION_180:
        return 180;
    case LV_DISPLAY_ROTATION_270:
        return 270;
    }
    return 0;
}

#if BSP_CAPS_BUTTONS

/*******************************************************************************
 * Button callback: toggle between LumiTrack main UI and camera live view
 *******************************************************************************/
static void app_hw_btn_toggle_view_cb(void *button_handle, void *usr_data)
{
    if (is_settings_view) {
        /* Exit settings mode */
        is_settings_view = false;
        bsp_display_lock(0);
        lv_scr_load_anim(main_screen, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
        ESP_LOGI(TAG, "Exited settings mode");
        bsp_display_unlock();
        return;
    }

    if (is_camera_view) {
        /* Go to settings from camera view */
        is_camera_view = false;
        is_settings_view = true;
        bsp_display_lock(0);
        lv_scr_load_anim(settings_screen, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
        ESP_LOGI(TAG, "Switched to settings view");
        bsp_display_unlock();
    } else {
        /* Toggle between main UI and camera view */
        is_camera_view = !is_camera_view;
        bsp_display_lock(0);
        if (is_camera_view) {
            lv_scr_load_anim(camera_screen, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
            ESP_LOGI(TAG, "Switched to camera live view");
        } else {
            lv_scr_load_anim(main_screen, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
            ESP_LOGI(TAG, "Switched to LumiTrack main UI");
        }
        bsp_display_unlock();
    }
}

static void app_hw_btn_settings_up_cb(void *button_handle, void *usr_data)
{
    if (!is_settings_view) return;

    settings_menu_index--;
    if (settings_menu_index < 0) {
        settings_menu_index = SETTINGS_MENU_COUNT - 1;
    }
    bsp_display_lock(0);
    /* Update selection highlight */
    for (int i = 0; i < SETTINGS_MENU_COUNT; i++) {
        lv_obj_t *item = lv_obj_get_child(settings_menu_list, i);
        if (item) {
            lv_obj_t *label = lv_obj_get_child(item, 0);
            if (label) {
                if (i == settings_menu_index) {
                    lv_obj_set_style_text_color(label, lv_color_hex(0x00E5FF), 0);
                } else {
                    lv_obj_set_style_text_color(label, lv_color_hex(0xAAAAAA), 0);
                }
            }
        }
    }
    bsp_display_unlock();
    ESP_LOGI(TAG, "Settings menu up: index=%d", settings_menu_index);
}

static void app_hw_btn_settings_down_cb(void *button_handle, void *usr_data)
{
    if (!is_settings_view) return;

    settings_menu_index++;
    if (settings_menu_index >= SETTINGS_MENU_COUNT) {
        settings_menu_index = 0;
    }
    bsp_display_lock(0);
    /* Update selection highlight */
    for (int i = 0; i < SETTINGS_MENU_COUNT; i++) {
        lv_obj_t *item = lv_obj_get_child(settings_menu_list, i);
        if (item) {
            lv_obj_t *label = lv_obj_get_child(item, 0);
            if (label) {
                if (i == settings_menu_index) {
                    lv_obj_set_style_text_color(label, lv_color_hex(0x00E5FF), 0);
                } else {
                    lv_obj_set_style_text_color(label, lv_color_hex(0xAAAAAA), 0);
                }
            }
        }
    }
    bsp_display_unlock();
    ESP_LOGI(TAG, "Settings menu down: index=%d", settings_menu_index);
}

static void app_hw_btn_settings_confirm_cb(void *button_handle, void *usr_data)
{
    if (!is_settings_view) return;

    ESP_LOGI(TAG, "Settings confirm: %s", settings_menu_items[settings_menu_index]);
    /* TODO: Enter sub-page for selected item */
}

static void app_hw_btn_rotate_right_cb(void *button_handle, void *usr_data)
{
    if (rotation == LV_DISPLAY_ROTATION_270) {
        rotation = LV_DISPLAY_ROTATION_0;
    } else {
        rotation++;
    }
    bsp_display_lock(0);
    bsp_display_rotate(display, rotation);
    bsp_display_unlock();
    ESP_LOGI(TAG, "Button rotate right – Rotation: %d", app_lvgl_get_rotation_degrees(rotation));
}

static void app_hw_btn_rotate_left_cb(void *button_handle, void *usr_data)
{
    if (rotation == LV_DISPLAY_ROTATION_0) {
        rotation = LV_DISPLAY_ROTATION_270;
    } else {
        rotation--;
    }
    bsp_display_lock(0);
    bsp_display_rotate(display, rotation);
    bsp_display_unlock();
    ESP_LOGI(TAG, "Button rotate left – Rotation: %d", app_lvgl_get_rotation_degrees(rotation));
}

static void app_hw_btn_init(void)
{
    ESP_ERROR_CHECK(bsp_iot_button_create(btn_handles, &btn_cnt, BSP_BUTTON_NUM));
    ESP_LOGI(TAG, "Created %d hardware buttons", btn_cnt);

#if CONFIG_BSP_SELECT_ESP32_S3_EYE
    /* BSP_BUTTON_1 → toggle view / enter settings */
    if (btn_handles[BSP_BUTTON_1]) {
        iot_button_register_cb(btn_handles[BSP_BUTTON_1], BUTTON_PRESS_DOWN, NULL, app_hw_btn_toggle_view_cb, NULL);
    }
    /* BSP_BUTTON_2 → rotate left / settings up */
    if (btn_handles[BSP_BUTTON_2]) {
        iot_button_register_cb(btn_handles[BSP_BUTTON_2], BUTTON_PRESS_DOWN, NULL, app_hw_btn_rotate_left_cb, NULL);
        iot_button_register_cb(btn_handles[BSP_BUTTON_2], BUTTON_LONG_PRESS_START, NULL, app_hw_btn_settings_up_cb, NULL);
    }
    /* BSP_BUTTON_5 → rotate right / settings down */
    if (btn_handles[BSP_BUTTON_5]) {
        iot_button_register_cb(btn_handles[BSP_BUTTON_5], BUTTON_PRESS_DOWN, NULL, app_hw_btn_rotate_right_cb, NULL);
        iot_button_register_cb(btn_handles[BSP_BUTTON_5], BUTTON_LONG_PRESS_START, NULL, app_hw_btn_settings_down_cb, NULL);
    }
    /* Additional button for confirm in settings (using short press of btn2 in settings mode) */
#else
    if (btn_cnt >= 1) {
        iot_button_register_cb(btn_handles[0], BUTTON_PRESS_DOWN, NULL, app_hw_btn_toggle_view_cb, NULL);
        iot_button_register_cb(btn_handles[0], BUTTON_LONG_PRESS_START, NULL, app_hw_btn_settings_confirm_cb, NULL);
    }
    if (btn_cnt >= 2) {
        iot_button_register_cb(btn_handles[1], BUTTON_PRESS_DOWN, NULL, app_hw_btn_rotate_right_cb, NULL);
        iot_button_register_cb(btn_handles[1], BUTTON_LONG_PRESS_START, NULL, app_hw_btn_settings_down_cb, NULL);
    }
    if (btn_cnt >= 3) {
        iot_button_register_cb(btn_handles[2], BUTTON_PRESS_DOWN, NULL, app_hw_btn_settings_up_cb, NULL);
    }
#endif
}
#endif /* BSP_CAPS_BUTTONS */

#if BSP_CAPS_IMU
static void app_imu_init(void)
{
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();

    qma6100p_cfg_t cfg = {
        .i2c_bus_handle = i2c_handle,
        .i2c_addr = QMA6100P_I2C_ADDR,
        .sensor_id = QMA6100P_CHIP_ID,
    };

    ESP_ERROR_CHECK(qma6100p_create(&cfg, &imu));

    if (imu) {
        qma6100p_acce_cfg_t acce_cfg = {
            .odr = QMA6100P_ODR_100HZ,
            .range = QMA6100P_ACCE_RANGE_4G,
            .bandwidth = QMA6100P_ACCE_BW_NORMAL,
        };
        ESP_ERROR_CHECK(qma6100p_config_acce(imu, &acce_cfg));
        ESP_ERROR_CHECK(qma6100p_acce_enable(imu));
        ESP_LOGI(TAG, "QMA6100P accelerometer initialized");
    }
}

static void low_pass_filter(float *out, float in, float alpha)
{
    *out = *out + alpha * (in - *out);
}

static void app_imu_read(void)
{
    static float flt_x = 0.0f;
    static float flt_y = 0.0f;
    static float flt_z = 0.0f;
    static float prev_angle_xy = 0.0f;
    static float cumulative_angle = 0.0f;
    static int direction_state = 0;
    static int direction_stable_count = 0;
    static int stable_count = 0;
    static lv_disp_rotation_t last_direction = LV_DISPLAY_ROTATION_0;
    static int64_t last_switch_time = 0;

    const float FILTER_ALPHA = 0.2f;
    const float THRESHOLD = 0.5f;
    const int STABLE_COUNT_REQUIRED = 5;
    const int64_t MIN_SWITCH_INTERVAL_US = 500000;

    qma6100p_sensor_data_t data;
    ESP_ERROR_CHECK(qma6100p_get_sensor_data(imu, &data));

    low_pass_filter(&flt_x, data.acce_x, FILTER_ALPHA);
    low_pass_filter(&flt_y, data.acce_y, FILTER_ALPHA);
    low_pass_filter(&flt_z, data.acce_z, FILTER_ALPHA);

    lv_disp_rotation_t target = rotation;
    if (flt_y < -THRESHOLD) {
        target = LV_DISPLAY_ROTATION_0;
    } else if (flt_y > THRESHOLD) {
        target = LV_DISPLAY_ROTATION_180;
    } else if (flt_x > THRESHOLD) {
        target = LV_DISPLAY_ROTATION_270;
    } else if (flt_x < -THRESHOLD) {
        target = LV_DISPLAY_ROTATION_90;
    }

    float angle_xy = atan2f(flt_y, flt_x) * 180.0f / M_PI;
    float angle_delta = angle_xy - prev_angle_xy;
    if (angle_delta > 180.0f) {
        angle_delta -= 360.0f;
    } else if (angle_delta < -180.0f) {
        angle_delta += 360.0f;
    }

    const float ANGLE_THRESHOLD = 15.0f;
    int current_dir = 0;
    if (angle_delta > ANGLE_THRESHOLD) {
        current_dir = 1;
    } else if (angle_delta < -ANGLE_THRESHOLD) {
        current_dir = -1;
    }

    if (current_dir != 0) {
        cumulative_angle += angle_delta;
    }

    if (current_dir != 0 && current_dir == direction_state) {
        direction_stable_count++;
    } else if (current_dir != 0 && current_dir != direction_state) {
        direction_state = current_dir;
        direction_stable_count = 0;
    } else if (current_dir == 0) {
        direction_stable_count = 0;
    }

    prev_angle_xy = angle_xy;

    int64_t now = esp_timer_get_time();

    if (target != last_direction) {
        stable_count = 0;
        last_direction = target;
    } else if (target != rotation) {
        stable_count++;
        if (stable_count >= STABLE_COUNT_REQUIRED &&
            (now - last_switch_time) > MIN_SWITCH_INTERVAL_US) {

            int switch_dir = 0;
            if (rotation == LV_DISPLAY_ROTATION_0 && target == LV_DISPLAY_ROTATION_90) switch_dir = -1;
            else if (rotation == LV_DISPLAY_ROTATION_0 && target == LV_DISPLAY_ROTATION_270) switch_dir = 1;
            else if (rotation == LV_DISPLAY_ROTATION_90 && target == LV_DISPLAY_ROTATION_0) switch_dir = 1;
            else if (rotation == LV_DISPLAY_ROTATION_90 && target == LV_DISPLAY_ROTATION_180) switch_dir = -1;
            else if (rotation == LV_DISPLAY_ROTATION_180 && target == LV_DISPLAY_ROTATION_90) switch_dir = 1;
            else if (rotation == LV_DISPLAY_ROTATION_180 && target == LV_DISPLAY_ROTATION_270) switch_dir = -1;
            else if (rotation == LV_DISPLAY_ROTATION_270 && target == LV_DISPLAY_ROTATION_0) switch_dir = -1;
            else if (rotation == LV_DISPLAY_ROTATION_270 && target == LV_DISPLAY_ROTATION_180) switch_dir = 1;

            rotation = target;
            last_switch_time = now;
            stable_count = 0;

            bsp_display_lock(0);
            bsp_display_rotate(display, rotation);
            bsp_display_unlock();

            if (switch_dir == 1) {
                ESP_LOGI(TAG, ">>> [CW] Auto-rotate to %d", app_lvgl_get_rotation_degrees(rotation));
            } else if (switch_dir == -1) {
                ESP_LOGI(TAG, ">>> [CCW] Auto-rotate to %d", app_lvgl_get_rotation_degrees(rotation));
            } else {
                ESP_LOGI(TAG, "Auto-rotate to %d", app_lvgl_get_rotation_degrees(rotation));
            }
        }
    }
}
#endif /* BSP_CAPS_IMU */

/*******************************************************************************
 * Camera DVP init
 *******************************************************************************/
static esp_err_t app_camera_init(void)
{
    ESP_LOGI(TAG, "Initializing camera (OV2640 via DVP)...");

    /* ESP32-S3-EYE DVP camera pin configuration */
    static esp_video_init_dvp_config_t dvp_cfg = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = 0,
                .scl_pin = 5,
                .sda_pin = 4,
            },
            .freq = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
        .dvp_pin = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                11,   /* D0 */
                9,    /* D1 */
                8,    /* D2 */
                10,   /* D3 */
                12,   /* D4 */
                18,   /* D5 */
                17,   /* D6 */
                16,   /* D7 */
            },
            .vsync_io = 6,
            .de_io = 7,
            .pclk_io = 13,
            .xclk_io = 15,
        },
        .xclk_freq = 20000000,  /* 20 MHz */
    };

    static const esp_video_init_config_t video_config = {
        .dvp = &dvp_cfg,
    };

    esp_err_t ret = esp_video_init(&video_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

/*******************************************************************************
 * Camera capture task (runs in separate FreeRTOS task)
 * Uses V4L2 API to capture frames from DVP video device /dev/video2
 *******************************************************************************/
static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Camera capture task started");

    const char *dev_path = ESP_VIDEO_DVP_DEVICE_NAME; /* "/dev/video2" */
    const int buf_count = 2;
    uint8_t *v4l2_bufs[buf_count];

    int fd = open(dev_path, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open %s", dev_path);
        vTaskDelete(NULL);
        return;
    }

    /* ── Set format: RGB565, 240x240 ── */
    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix.width = CAM_WIDTH,
        .fmt.pix.height = CAM_HEIGHT,
        .fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565,
    };
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "Failed to set format, trying JPEG...");
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
            ESP_LOGE(TAG, "Failed to set any format");
            close(fd);
            vTaskDelete(NULL);
            return;
        }
    }
    ESP_LOGI(TAG, "Camera format: %" PRIu32 "x%" PRIu32 " pixfmt=0x%08" PRIx32,
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             fmt.fmt.pix.pixelformat);

    /* ── Request MMAP buffers ── */
    struct v4l2_requestbuffers req = {
        .count = buf_count,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "Failed to request buffers");
        close(fd);
        vTaskDelete(NULL);
        return;
    }

    for (int i = 0; i < buf_count; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "Failed to query buffer %d", i);
            close(fd);
            vTaskDelete(NULL);
            return;
        }

        v4l2_bufs[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd, buf.m.offset);
        if (v4l2_bufs[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "Failed to mmap buffer %d", i);
            close(fd);
            vTaskDelete(NULL);
            return;
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "Failed to queue buffer %d", i);
            close(fd);
            vTaskDelete(NULL);
            return;
        }
    }

    /* ── Start streaming ── */
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "Failed to start stream");
        close(fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Camera stream started");

    int frame_index = 0;

    /* ── Capture loop ── */
    while (1) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };

        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF failed");
            break;
        }

        if (buf.flags & V4L2_BUF_FLAG_DONE && buf.bytesused > 0) {
            /* Copy frame data into display buffer */
            int dst = frame_index % 2;
            size_t copy_len = buf.bytesused;
            if (copy_len > CAM_WIDTH * CAM_HEIGHT * 2) {
                copy_len = CAM_WIDTH * CAM_HEIGHT * 2;
            }

            if (xSemaphoreTake(cam_frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (cam_frame_buf[dst]) {
                    memcpy(cam_frame_buf[dst], v4l2_bufs[buf.index], copy_len);
                    cam_frame_ready = dst;
                }
                xSemaphoreGive(cam_frame_mutex);
            }

            frame_index++;
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QBUF failed");
            break;
        }
    }

    /* Cleanup (unreachable in normal operation but kept for completeness) */
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    close(fd);
    ESP_LOGW(TAG, "Camera capture task exiting");
    vTaskDelete(NULL);
}

/*******************************************************************************
 * LVGL timer: refresh camera image on screen (runs every ~50ms = ~20fps)
 *******************************************************************************/
static void camera_lvgl_update_cb(lv_timer_t *timer)
{
    if (!is_camera_view || !cam_img) {
        return;
    }

    int ready_idx = -1;
    if (xSemaphoreTake(cam_frame_mutex, 0) == pdTRUE) {
        ready_idx = cam_frame_ready;
        if (ready_idx >= 0) {
            cam_frame_ready = -1;
        }
        xSemaphoreGive(cam_frame_mutex);
    }

    if (ready_idx >= 0 && cam_frame_buf[ready_idx]) {
        bsp_display_lock(0);
        cam_img_dsc.data = cam_frame_buf[ready_idx];
        /* Re-set source to trigger LVGL to re-decode/render the new frame */
        lv_image_set_src(cam_img, &cam_img_dsc);
        lv_obj_invalidate(cam_img);
        bsp_display_unlock();
    }
}

/*******************************************************************************
 * LumiTrack UI – Main Screen  (4-zone OSD layout for 240×240)
 * ┌────────────────────┐
 * │     LumiTrack      │  title
 * │ ────────────────── │  divider
 * │ 状态: 自动识别·看书│  Zone1 - status bar
 * │ ┌─────────────────┐│
 * │ │   01:28:46      ││  Zone2 - focus timer box
 * │ │ 专注时长 | 实时  ││
 * │ └─────────────────┘│
 * │ 📊 今日 04:12:00   │  Zone3 - daily stats
 * │ 🏷 看书 学习 ...   │         quick tags
 * │ ┌─────────────────┐│
 * │ │💡 关灯|调暗|调亮││  Zone4 - light bar
 * │ │      |自动       ││
 * │ └─────────────────┘│
 * │  30fps  5% CPU     │  optional debug row
 * └────────────────────┘
 *******************************************************************************/
static void app_lvgl_display(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();
    main_screen = scr;

    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0A0A1A), 0);
    lv_obj_set_style_pad_all(scr, 6, 0);          /* 6 px global inset */

    /* === Anchor: top bar === */
    lv_obj_t *prev = NULL;   /* previous widget for relative layout */

    /* ── Title ── */
    lbl_title = lv_label_create(scr);
    lv_label_set_text(lbl_title, "LumiTrack");
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_16, 0);
    lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 6);
    prev = lbl_title;

    /* ── Divider ── */
    lv_obj_t *div = lv_obj_create(scr);
    lv_obj_set_size(div, 210, 1);
    lv_obj_set_style_bg_color(div, lv_color_hex(0x1A3A4A), 0);
    lv_obj_set_style_border_width(div, 0, 0);
    lv_obj_set_style_radius(div, 0, 0);
    lv_obj_align_to(div, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    prev = div;

    /* ── Zone 1: Status bar ── */
    lbl_status_mode = lv_label_create(scr);
    lv_label_set_text(lbl_status_mode, "auto det: read");
    lv_obj_set_style_text_color(lbl_status_mode, lv_color_hex(0x33CC99), 0);  /* theme green */
    lv_obj_set_style_text_font(lbl_status_mode, &lv_font_montserrat_12, 0);
    lv_obj_align_to(lbl_status_mode, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    prev = lbl_status_mode;

    /* ── Zone 2: Focus timer highlight box ── */
    lv_obj_t *time_box = lv_obj_create(scr);
    lv_obj_set_size(time_box, 220, 52);
    lv_obj_set_style_bg_color(time_box, lv_color_hex(0x102530), 0);           /* dark teal background */
    lv_obj_set_style_border_color(time_box, lv_color_hex(0x1A5A4A), 0);
    lv_obj_set_style_border_width(time_box, 1, 0);
    lv_obj_set_style_radius(time_box, 6, 0);
    lv_obj_set_style_pad_all(time_box, 4, 0);
    lv_obj_align_to(time_box, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    prev = time_box;

    /* Timer value (large) */
    lbl_focus_time = lv_label_create(time_box);
    lv_label_set_text(lbl_focus_time, "00:00:00");
    lv_obj_set_style_text_color(lbl_focus_time, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_focus_time, &lv_font_montserrat_24, 0);
    lv_obj_align(lbl_focus_time, LV_ALIGN_TOP_LEFT, 4, 4);

    /* Timer description (small, right side) */
    lbl_focus_desc = lv_label_create(time_box);
    lv_label_set_text(lbl_focus_desc, "focus  |  live");
    lv_obj_set_style_text_color(lbl_focus_desc, lv_color_hex(0x668888), 0);
    lv_obj_set_style_text_font(lbl_focus_desc, &lv_font_montserrat_12, 0);
    lv_obj_align_to(lbl_focus_desc, lbl_focus_time, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /* ── Zone 3: Daily stats row ── */
    lbl_daily_stats = lv_label_create(scr);
    char init_line[48];
    snprintf(init_line, sizeof(init_line), "today 04:12:00  |  rest");
    lv_label_set_text(lbl_daily_stats, init_line);
    lv_obj_set_style_text_color(lbl_daily_stats, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(lbl_daily_stats, &lv_font_montserrat_12, 0);
    lv_obj_align_to(lbl_daily_stats, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    prev = lbl_daily_stats;

    /* Quick tags row */
    lbl_quick_tags = lv_label_create(scr);
    lv_label_set_text(lbl_quick_tags, "read  study  pc  phone");
    lv_obj_set_style_text_color(lbl_quick_tags, lv_color_hex(0x667777), 0);
    lv_obj_set_style_text_font(lbl_quick_tags, &lv_font_montserrat_12, 0);
    lv_obj_align_to(lbl_quick_tags, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    prev = lbl_quick_tags;

    /* ── Separator before light bar ── */
    lv_obj_t *sep_light = lv_obj_create(scr);
    lv_obj_set_size(sep_light, 210, 1);
    lv_obj_set_style_bg_color(sep_light, lv_color_hex(0x1A3A4A), 0);
    lv_obj_set_style_border_width(sep_light, 0, 0);
    lv_obj_set_style_radius(sep_light, 0, 0);
    lv_obj_align_to(sep_light, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    prev = sep_light;

    /* ── Zone 4: Light bar fixed bottom ── */
    lv_obj_t *light_box = lv_obj_create(scr);
    lv_obj_set_size(light_box, 220, 28);
    lv_obj_set_style_bg_color(light_box, lv_color_hex(0x1A1A0A), 0);           /* warm dark */
    lv_obj_set_style_border_color(light_box, lv_color_hex(0x3A3A1A), 0);
    lv_obj_set_style_border_width(light_box, 1, 0);
    lv_obj_set_style_radius(light_box, 6, 0);
    lv_obj_set_style_pad_all(light_box, 2, 0);
    lv_obj_align_to(light_box, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

    lbl_light_status = lv_label_create(light_box);
    lv_label_set_text(lbl_light_status, "off  |  dim*  |  bright  |  auto");
    lv_obj_set_style_text_color(lbl_light_status, lv_color_hex(0xCCAA66), 0);   /* warm amber */
    lv_obj_set_style_text_font(lbl_light_status, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_light_status);
    prev = light_box;

    /* ── Optional debug row ── */
    lbl_perf = lv_label_create(scr);
    lv_label_set_text(lbl_perf, "30fps  5% cpu");
    lv_obj_set_style_text_color(lbl_perf, lv_color_hex(0x334455), 0);
    lv_obj_set_style_text_font(lbl_perf, &lv_font_montserrat_12, 0);
    lv_obj_align_to(lbl_perf, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);

    bsp_display_unlock();
}

/*******************************************************************************
 * Camera UI – Full-screen live preview screen
 *******************************************************************************/
static void app_camera_lvgl_screen(void)
{
    bsp_display_lock(0);

    camera_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(camera_screen, lv_color_black(), 0);
    lv_obj_set_style_pad_all(camera_screen, 0, 0);

    /* Full-screen camera image */
    cam_img = lv_image_create(camera_screen);
    lv_obj_set_size(cam_img, LV_PCT(100), LV_PCT(100));
    lv_obj_align(cam_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(cam_img, 0, 0);
    lv_obj_set_style_border_width(cam_img, 0, 0);

    /* Set the initial empty image source */
    lv_image_set_src(cam_img, &cam_img_dsc);

    /* Hint text overlay (shown when no frame yet) */
    lv_obj_t *hint = lv_label_create(camera_screen);
    lv_label_set_text(hint, "Camera Starting...");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(hint, LV_OBJ_FLAG_FLOATING);

    bsp_display_unlock();
}

/*******************************************************************************
 * Settings UI – Device Settings Screen
 * ┌─────────────────────────────┐
 * │         设备设置            │
 * │                             │
 * │ ▶ 自定义行为标签（新增/删除）│
 * │ ▶ 自动识别传感器开关        │
 * │ ▶ 屏幕亮度调节              │
 * │ ▶ 本地数据导出              │
 * │ ▶ 设备恢复出厂              │
 * │                             │
 * │ 上下选菜单，确认进入子页面   │
 * │                             │
 * ├─────────────────────────────┤
 * │ ○关灯  ◎调暗  ●调亮         │
 * └─────────────────────────────┘
 *******************************************************************************/
static void app_settings_lvgl_screen(void)
{
    bsp_display_lock(0);

    settings_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(settings_screen, lv_color_hex(0x0A0A1A), 0);
    lv_obj_set_style_pad_all(settings_screen, 6, 0);

    /* ── Title ── */
    lv_obj_t *title = lv_label_create(settings_screen);
    lv_label_set_text(title, "设备设置");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    /* ── Divider ── */
    lv_obj_t *div = lv_obj_create(settings_screen);
    lv_obj_set_size(div, 210, 1);
    lv_obj_set_style_bg_color(div, lv_color_hex(0x1A3A4A), 0);
    lv_obj_set_style_border_width(div, 0, 0);
    lv_obj_set_style_radius(div, 0, 0);
    lv_obj_align_to(div, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

    /* ── Settings menu list ── */
    settings_menu_list = lv_obj_create(settings_screen);
    lv_obj_set_size(settings_menu_list, 220, 120);
    lv_obj_set_style_bg_color(settings_menu_list, lv_color_hex(0x102530), 0);
    lv_obj_set_style_border_color(settings_menu_list, lv_color_hex(0x1A5A4A), 0);
    lv_obj_set_style_border_width(settings_menu_list, 1, 0);
    lv_obj_set_style_radius(settings_menu_list, 6, 0);
    lv_obj_set_style_pad_all(settings_menu_list, 4, 0);
    lv_obj_set_flex_flow(settings_menu_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(settings_menu_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align_to(settings_menu_list, div, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

    /* Create menu items */
    for (int i = 0; i < SETTINGS_MENU_COUNT; i++) {
        lv_obj_t *item = lv_obj_create(settings_menu_list);
        lv_obj_set_size(item, LV_PCT(100), 22);
        lv_obj_set_style_bg_color(item, lv_color_hex(0x00000000), 0);
        lv_obj_set_style_border_width(item, 0, 0);
        lv_obj_set_style_pad_all(item, 0, 0);

        lv_obj_t *label = lv_label_create(item);
        lv_label_set_text(label, settings_menu_items[i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_center(label);
    }

    /* Highlight current selection */
    {
        lv_obj_t *first_item = lv_obj_get_child(settings_menu_list, 0);
        if (first_item) {
            lv_obj_t *first_label = lv_obj_get_child(first_item, 0);
            if (first_label) {
                lv_obj_set_style_text_color(first_label, lv_color_hex(0x00E5FF), 0);
            }
        }
    }

    /* ── Instruction text ── */
    lv_obj_t *instruction = lv_label_create(settings_screen);
    lv_label_set_text(instruction, "上下选菜单，确认进入子页面");
    lv_obj_set_style_text_color(instruction, lv_color_hex(0x667777), 0);
    lv_obj_set_style_text_font(instruction, &lv_font_montserrat_10, 0);
    lv_obj_align_to(instruction, settings_menu_list, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);

    /* ── Light control bar at bottom ── */
    lv_obj_t *light_bar = lv_obj_create(settings_screen);
    lv_obj_set_size(light_bar, 220, 30);
    lv_obj_set_style_bg_color(light_bar, lv_color_hex(0x1A1A0A), 0);
    lv_obj_set_style_border_color(light_bar, lv_color_hex(0x3A3A1A), 0);
    lv_obj_set_style_border_width(light_bar, 1, 0);
    lv_obj_set_style_radius(light_bar, 6, 0);
    lv_obj_set_style_pad_all(light_bar, 2, 0);
    lv_obj_align_to(light_bar, instruction, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

    lv_obj_t *light_label = lv_label_create(light_bar);
    lv_label_set_text(light_label, "○关灯  ◎调暗  ●调亮");
    lv_obj_set_style_text_color(light_label, lv_color_hex(0xCCAA66), 0);
    lv_obj_set_style_text_font(light_label, &lv_font_montserrat_11, 0);
    lv_obj_center(light_label);

    bsp_display_unlock();
}

/*******************************************************************************
 * app_main
 *******************************************************************************/
void app_main(void)
{
    /* Initialize display and LVGL */
    display = bsp_display_start();

    /* Set display brightness to 100% */
    bsp_display_backlight_on();

#if BSP_CAPS_IMU
    app_imu_init();
#endif

#if BSP_CAPS_BUTTONS
    app_hw_btn_init();
#endif

    /* Create LumiTrack UI */
    app_lvgl_display();

    /* Create camera screen (hidden initially) */
    app_camera_lvgl_screen();

    /* Create settings screen (hidden initially) */
    app_settings_lvgl_screen();

    /* Initialize camera frame buffers for LVGL display */
    cam_frame_buf[0] = heap_caps_malloc(CAM_WIDTH * CAM_HEIGHT * 2,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    cam_frame_buf[1] = heap_caps_malloc(CAM_WIDTH * CAM_HEIGHT * 2,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    cam_frame_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(cam_frame_mutex);  /* Give initial token so first take succeeds */

    /* Setup camera image descriptor */
    cam_img_dsc.header.w = CAM_WIDTH;
    cam_img_dsc.header.h = CAM_HEIGHT;
    cam_img_dsc.header.stride = CAM_WIDTH * 2;
    cam_img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    cam_img_dsc.data = cam_frame_buf[0];
    cam_img_dsc.data_size = CAM_WIDTH * CAM_HEIGHT * 2;

    /* Initialize camera hardware */
    esp_err_t cam_err = app_camera_init();
    if (cam_err == ESP_OK) {
        /* Start camera capture task */
        xTaskCreatePinnedToCore(camera_capture_task, "cam_cap", 4096, NULL, 5,
                                NULL, 1);  /* Run on core 1 */
    } else {
        ESP_LOGE(TAG, "Camera init failed, camera view will show black screen");
    }

    /* Create 1-second timer to update focus & daily counters */
    lv_timer_create(lumi_timer_cb, 1000, NULL);

    /* Create 50ms timer to refresh camera image on display */
    lv_timer_create(camera_lvgl_update_cb, 50, NULL);

    ESP_LOGI(TAG, "LumiTrack UI initialized. Press [BTN1] to toggle camera view.");

#if BSP_CAPS_IMU
    while (1) {
        app_imu_read();
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#else
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
}