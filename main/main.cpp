/* Edge Impulse Espressif ESP32 Standalone Inference ESP IDF Example
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include <stdio.h>

#include <esp_log.h>

#include <esp_system.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"

#include "esp_camera.h"

#include "driver/mcpwm_prelude.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// Display Stuff
#define tag "SSD1306"


static const char *TAG = "my AI App";

// SERVO DEFINITIONS
#define SERVO_MIN_PULSEWIDTH_US 1000  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2000  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle
#define SERVO_PULSE_GPIO             2        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

// CAMERA DEFINITIONS FOR AI THINKER BOARD
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#define LED_PIN GPIO_NUM_21

// My definitions 

uint8_t logoMischianti [1024] = {
// 'logoBN128x64, 128x64px
// 'IMG_4300(1)(1)', 128x64px
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc3, 0xfe, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xc0, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfc, 0x1f, 0xfc, 0xff, 0x87, 0xf0, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xfc, 0x73, 0x00, 0x18, 0xf8, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf9, 0x8f, 0xbf, 0xfe, 0x7c, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xc0, 0x00, 0x00, 0x03, 0xcd, 0xff, 0xfd, 0x3c, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xe0, 0x03, 0xff, 0xff, 0xfb, 0xdf, 0xcf, 0x7c, 0x9e, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x03, 0xff, 0xff, 0xff, 0xf7, 0x9f, 0xff, 0x3f, 0x1e, 
0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0xff, 0xc0, 0x3f, 0xff, 0xfb, 0x8f, 0xff, 0xbf, 0x4f, 
0x00, 0x00, 0x00, 0x00, 0x0f, 0xf8, 0x1f, 0xf0, 0xff, 0xff, 0xff, 0xf3, 0x8f, 0x00, 0xbd, 0x4f, 
0x00, 0x00, 0x00, 0x00, 0x7f, 0x81, 0xfc, 0x1f, 0xff, 0xff, 0xf6, 0x18, 0x00, 0xf8, 0xbd, 0x4f, 
0x00, 0x00, 0x00, 0x03, 0xfc, 0x1f, 0xcf, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x7f, 0xf8, 0x91, 0xcf, 
0x00, 0x00, 0x00, 0x0f, 0xe0, 0xfc, 0xff, 0xff, 0xfe, 0x01, 0x00, 0x1f, 0xff, 0xc0, 0x00, 0xce, 
0x00, 0x00, 0x00, 0x7f, 0x07, 0xff, 0xff, 0xfc, 0x03, 0x80, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x1e, 
0x00, 0x00, 0x01, 0xfc, 0x3e, 0xff, 0xff, 0x06, 0xc0, 0x03, 0xff, 0x00, 0x07, 0xc0, 0x00, 0x3c, 
0x00, 0x00, 0x03, 0xf0, 0xff, 0xff, 0x80, 0xf8, 0x00, 0xff, 0xff, 0x8f, 0xc0, 0x60, 0x00, 0x3c, 
0x00, 0x00, 0x0f, 0x83, 0xdf, 0xf0, 0x3c, 0x00, 0x1f, 0xff, 0xff, 0x9f, 0xdf, 0xc0, 0x20, 0x38, 
0x00, 0x00, 0x3e, 0x0f, 0x7c, 0x0b, 0x80, 0x3f, 0x7f, 0xf0, 0x7f, 0xbf, 0xff, 0x83, 0xe0, 0x78, 
0x00, 0x03, 0xfc, 0x3b, 0xe1, 0xc0, 0x0f, 0xff, 0x18, 0x00, 0x7f, 0x3f, 0xff, 0x0e, 0xc0, 0x78, 
0x00, 0x1f, 0xf0, 0xed, 0xd0, 0x03, 0xff, 0xfe, 0x01, 0xc0, 0x7e, 0x3f, 0x80, 0x2f, 0xc0, 0xf0, 
0x00, 0x1f, 0x81, 0x9b, 0x43, 0xef, 0xff, 0x00, 0x3f, 0x80, 0xfc, 0x7f, 0x01, 0xf3, 0x80, 0xf0, 
0x00, 0x1e, 0x00, 0x0f, 0x8f, 0xe3, 0x80, 0x00, 0x3f, 0x87, 0xf8, 0x7f, 0x3b, 0xe7, 0x01, 0xe0, 
0x00, 0x3c, 0x0f, 0x01, 0x9f, 0xc0, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0xfe, 0x03, 0x9e, 0x03, 0xc0, 
0x00, 0x3d, 0xff, 0x82, 0x1f, 0xc7, 0xf0, 0x00, 0x7f, 0xff, 0x00, 0xfe, 0x4f, 0x78, 0x07, 0x80, 
0x00, 0x79, 0xff, 0xc1, 0x3f, 0x8f, 0xe1, 0xf0, 0xff, 0xff, 0x80, 0xfc, 0x3e, 0x60, 0x0f, 0x00, 
0x00, 0x78, 0xff, 0xe0, 0x3f, 0x9f, 0xff, 0xf0, 0xfe, 0xff, 0xe1, 0xfc, 0xff, 0x80, 0x1e, 0xf0, 
0x00, 0x70, 0xff, 0xf0, 0x3f, 0x9f, 0xff, 0xe0, 0xfe, 0x7f, 0xf1, 0xfc, 0xfe, 0x00, 0x3e, 0xf0, 
0x00, 0xf0, 0xff, 0xf8, 0x7f, 0x3f, 0xfe, 0x01, 0xfc, 0x1f, 0xff, 0xf8, 0xf8, 0x00, 0xff, 0xe0, 
0x00, 0xf1, 0xff, 0xfc, 0x7f, 0x3f, 0x80, 0x01, 0xfc, 0x0f, 0xff, 0xc1, 0xc0, 0x03, 0xff, 0xe0, 
0x01, 0xe1, 0xff, 0xfe, 0x7e, 0x7f, 0x83, 0xe3, 0xf8, 0x43, 0xf0, 0x02, 0x00, 0x00, 0x00, 0xf0, 
0x01, 0xe3, 0xfb, 0xff, 0xfe, 0x7f, 0x18, 0x03, 0xf8, 0xe0, 0x00, 0x3e, 0x00, 0x03, 0xf8, 0x78, 
0x03, 0xc3, 0xf9, 0xff, 0xfc, 0x7f, 0x00, 0xf7, 0xf0, 0xd8, 0x0b, 0xf8, 0x00, 0x1f, 0xf8, 0x78, 
0x03, 0xc7, 0xf0, 0xff, 0xfc, 0xff, 0x3f, 0xe7, 0xc1, 0xdb, 0x7f, 0x80, 0x00, 0x3f, 0xf8, 0x78, 
0x07, 0x87, 0xf0, 0x7f, 0xfc, 0xff, 0xff, 0x80, 0x02, 0x3e, 0xfc, 0x00, 0x01, 0x3e, 0x40, 0x78, 
0x07, 0x8f, 0xe0, 0x3f, 0xf9, 0xff, 0xfc, 0x00, 0x7d, 0xbf, 0x80, 0x00, 0x0f, 0x7c, 0x00, 0x78, 
0x07, 0x8f, 0xe1, 0x1f, 0xf9, 0xff, 0x00, 0x3e, 0x0f, 0xf8, 0x00, 0x0f, 0x8f, 0x7c, 0x38, 0xf0, 
0x0f, 0x0f, 0xc0, 0x8f, 0xf0, 0x00, 0x07, 0x8a, 0xff, 0x00, 0x0b, 0xef, 0x9f, 0x7b, 0xf8, 0xf0, 
0x0f, 0x1f, 0xc3, 0x03, 0xf0, 0x00, 0xe0, 0xef, 0xc0, 0x07, 0xfb, 0xdf, 0xde, 0xfb, 0xf1, 0xe0, 
0x1e, 0x1f, 0xc3, 0xe3, 0xe0, 0x08, 0x00, 0x20, 0x00, 0x7f, 0xfb, 0xdf, 0xfe, 0xfb, 0xf1, 0xe0, 
0x1e, 0x3f, 0x80, 0x10, 0x00, 0x07, 0xff, 0x01, 0xf8, 0x7f, 0xc7, 0xdf, 0xfe, 0xf1, 0xf1, 0xe0, 
0x3e, 0x3f, 0x86, 0x00, 0x0f, 0xff, 0xe0, 0x03, 0xf8, 0xff, 0x07, 0xbf, 0xfd, 0xf1, 0xe3, 0xc0, 
0x3c, 0x20, 0x0f, 0xe0, 0x7f, 0xc0, 0x0f, 0x83, 0xf8, 0x0f, 0x07, 0xbd, 0xfd, 0xff, 0xe3, 0xc0, 
0x3c, 0x00, 0x7f, 0xff, 0xf8, 0x03, 0xff, 0x87, 0xfc, 0x1f, 0x0f, 0xbc, 0xfd, 0xff, 0xc7, 0x80, 
0x78, 0x3f, 0xfe, 0x40, 0x00, 0xf7, 0xfc, 0x0f, 0xfc, 0x1e, 0x0f, 0x7c, 0xf9, 0xfc, 0x07, 0x80, 
0x7f, 0xff, 0xf0, 0x38, 0xf1, 0xf7, 0x80, 0x1f, 0x7c, 0x1e, 0x0f, 0x78, 0x70, 0x07, 0x8f, 0x00, 
0xff, 0xff, 0xc3, 0xfd, 0xf1, 0xe7, 0x9f, 0x1f, 0xfc, 0x3e, 0x1f, 0x78, 0x07, 0xff, 0x0f, 0x00, 
0xfe, 0x17, 0x8f, 0xfd, 0xe1, 0xef, 0xfe, 0x3f, 0xfc, 0x3c, 0x1e, 0x03, 0xff, 0xfc, 0x1e, 0x00, 
0xc0, 0x77, 0x0f, 0xbd, 0xe3, 0xef, 0xf8, 0x7f, 0x7c, 0x3c, 0x01, 0xff, 0xff, 0xf0, 0x3c, 0x00, 
0x00, 0x0f, 0x1f, 0x3b, 0xff, 0xcf, 0x00, 0xf8, 0x7c, 0x30, 0x7f, 0xff, 0xff, 0x00, 0xf8, 0x00, 
0x00, 0x0f, 0x1f, 0x7b, 0xff, 0xdf, 0x0c, 0xf0, 0x70, 0x1f, 0xff, 0xff, 0xe0, 0x03, 0xf0, 0x00, 
0x00, 0x0e, 0x1e, 0x03, 0xff, 0xdf, 0xfd, 0xe0, 0x0f, 0xff, 0xff, 0xfc, 0x00, 0x3f, 0xc0, 0x00, 
0x00, 0x1e, 0x3e, 0x07, 0xc7, 0x9f, 0xfd, 0x07, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xff, 0x00, 0x00, 
0x00, 0x1e, 0x3e, 0x07, 0x87, 0xbf, 0x80, 0xff, 0xff, 0xff, 0xc0, 0x03, 0xff, 0xf8, 0x00, 0x00, 
0x00, 0x3c, 0x7c, 0x07, 0x8f, 0xa0, 0x7f, 0xff, 0xff, 0xe0, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 
0x00, 0x3c, 0x7c, 0x6f, 0x8f, 0x3f, 0xff, 0xff, 0xf8, 0x00, 0x1f, 0xff, 0x80, 0x00, 0x00, 0x00, 
0x00, 0x3c, 0x7c, 0xef, 0x00, 0x7f, 0xff, 0xfc, 0x00, 0x07, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x78, 0xf9, 0xe8, 0x00, 0x7f, 0xff, 0x80, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x78, 0xff, 0xc0, 0x60, 0x7f, 0xc0, 0x01, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x78, 0xff, 0x8f, 0xe0, 0xe0, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x78, 0xfc, 0x3f, 0xe0, 0x00, 0x7f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x3c, 0x00, 0xfc, 0xf8, 0x3f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x3f, 0x9f, 0xf0, 0x7f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x0f, 0xff, 0xc0, 0x3f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xfc, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// TODO: Remove
#define FEATURE_LEN 96*96

// Init features to 0
static const float features[FEATURE_LEN] = {
};

static uint8_t image[FEATURE_LEN*3] = { 0 };

//declare fb_pointer
static camera_fb_t * framebuffer_ptr;


// CAMERA STUFF
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_96X96,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// a helper function for the servo; taken from the example page.
static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ei_printf("Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

void setup_led() {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_rom_gpio_pad_select_gpio(LED_PIN);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    gpio_pad_select_gpio(LED_PIN);
#endif
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;

    // read byte for byte
    while (bytes_left != 0) {
        // grab the values and convert to r/g/b
        uint8_t r, g, b;
        r = image[pixel_ix];
        g = image[pixel_ix + 1];
        b = image[pixel_ix + 2];

        // then convert to out_ptr format
        float pixel_f = (r << 16) + (g << 8) + b;
        out_ptr[out_ptr_ix] = pixel_f;

        // and go to the next pixel
        out_ptr_ix++;
        pixel_ix += 3;
        bytes_left--;
    }
    ei_printf("DONE!");
    return 0;
}

void print_inference_result(ei_impulse_result_t result) {

    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    // Print the prediction results (object detection)
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

}




void initDisplay(void) {


}

extern "C" int app_main()
{
    //Display init
    SSD1306_t dev;
	int center, top, bottom;
	char lineChar[20];
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
	ssd1306_init(&dev, 128, 64);

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);

    ssd1306_bitmaps(&dev, 0, 0, logoMischianti, 128, 64, false);


    ESP_LOGI(TAG, "Create timer and operator");
    //Servo init
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .intr_priority = NULL,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config;
    comparator_config.intr_priority = NULL;
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    //control loop
    int angle = 0;
    int step = 5;

    //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply

    //Camera init
    if(ESP_OK != init_camera()) {
        return ESP_FAIL;
    }
    //Rest init
    setup_led();

    ei_impulse_result_t result = { nullptr };

    ei_printf("Edge Impulse standalone inferencing (Espressif ESP32)\n");

    while (true)
    {
        // blink LED
        gpio_set_level(LED_PIN, 1);
        
        //Camera loop part
        
        ei_printf("Taking picture...\n");
        framebuffer_ptr = esp_camera_fb_get();

        // use pic->buf to access the image
        ei_printf("Picture taken! Its size was: %zu bytes\n", framebuffer_ptr->len);
        bool converted = fmt2rgb888(framebuffer_ptr->buf, framebuffer_ptr->len, PIXFORMAT_RGB565, image);
        //CAMERA: Return the frame buffer to camera driver
        esp_camera_fb_return(framebuffer_ptr);

        ei_printf("TEST of image data[0]: %02X\r\n", image[0]);
        ei_printf("TEST of image data[9215]: %02X\r\n", image[9215]);
        ei_printf("TEST of image data[27647]: %02X\r\n", image[27647]);

        if(!converted){
            ei_printf("ERR: Conversion failed\n");
            return false;
        }

        ei_printf("converted to rgb888\n");
        // the features are stored into flash, and we don't want to load everything into RAM
        signal_t features_signal;
        features_signal.total_length = sizeof(features) / sizeof(features[0]);
        features_signal.get_data = &raw_feature_get_data;

        // invoke the impulse (---INFERENCE---)
        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
        if (res != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", res);
            return res;
        }

        

        //Check how many boxes
        if(result.bounding_boxes_count >= 1) {
            angle =  -38;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
            angle = 50;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
        }
        print_inference_result(result);
        
        //Draw boxes on screen
        ei_printf("Draw Boxes on Screen...\r\n");

        uint8_t tempArray[1024] = {0};

        ssd1306_clear_screen(&dev, false);
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            //for some reason there are 0 bounding boxes
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) {
                continue;
            }
            ei_printf("DEBUG! Duck found!\r\n");
            
            //TODO: Make real function outta this.
            //OLED painting algo: 
            // in: tempArray, bb.x, bb.y, bb.width, bb.height
            //out: tempArray
            for(int i = 0; i < 16; ++i) {
                for(int j = 0; j < 64; ++j) {  //That's b/c ... TODO
                    //recalculate the 96*96 info to 64*64 to match the display TODO: This is hardcoded. Better use the size info, not 0.66.
                    int bb_x_64 = int(0.66*bb.x);
                    int bb_y_64 = int(0.66*bb.y);
                    int bb_width_64 = int(0.66*bb.width);
                    int bb_height_64 = int(0.66*bb.height);
                    // Now the conditions for drawing the lines
                    // Left vertical line
                    if(i==bb_x_64/8 && j>=bb_y_64 && j<=bb_y_64+bb_height_64) { // we interpret the x axis on the display as a row of bytes, thus divison by 8
                        int shift = 8 - bb_x_64 % 8;
                        tempArray[j*16+i] = tempArray[j*16+i] | 0x01 << shift;
                    }
                    // Right vertical line
                    if(i==(bb_x_64+bb_width_64)/8 && j>=bb_y_64 && j<=bb_y_64+bb_height_64) { //similar to left
                        int shift = 8 - (bb_x_64+bb_width_64) % 8;
                        tempArray[j*16+i] = tempArray[j*16+i] | 0x01 << shift;
                    }
                    // horizontal lines
                    if((j==bb_y_64 || j==bb_y_64+bb_height_64) && i>=bb_x_64/8 && i<=bb_x_64/8+bb_width_64/8) { //Y interpreted as 64 "collumns" as "indices" for the x lines
                        tempArray[j*16+i] = 0xFF;
                    }
                    //draw right border line:
                    if(i==8) {
                        tempArray[j*16+i] = tempArray[j*16+i] | 0x01 << 7;
                    }
                }
            }
            //END ALGO

            //ei_printf("Now Printing!!\r\n");
            ei_printf("X-Offset: %d\r\n", bb.x);
            // ei_printf("  %s (%f) [ x: %u, y: %u]\r\n",
            //         bb.label,
            //         bb.value,
            //         bb.x,
            //         bb.y);
        }

        ssd1306_bitmaps(&dev, 0, 0, tempArray, 128, 64, false);

        ei_printf("Boxes drawn...\r\n");
        
        gpio_set_level(LED_PIN, 0);
        
        
        
    }
}

