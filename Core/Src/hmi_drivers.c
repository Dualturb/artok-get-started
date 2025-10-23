/**
 * @file hmi_drivers.c
 * @brief Unified HMI drivers for the Artok-Runtime.
 *
 * This file contains the low-level hardware driver implementations for the
 * display and touchpad. It correctly separates the hardware abstraction from the UI library.
 */

/*********************
 * INCLUDES
 *********************/
#include "hmi_drivers.h"
#include "hmi_lcd.h"
#include "xpt2046.h"
#include "w25qxx.h"
#include "artok_runtime.h"
#include "ili9341.h"
#include <stdbool.h>

/**********************
 * TYPEDEFS
 **********************/
// Define the LCD_Area_t struct here for use in the static functions.
typedef struct {
	int16_t x1;
	int16_t y1;
	int16_t x2;
	int16_t y2;
} LCD_Area_t;

/**********************
 * STATIC PROTOTYPES
 **********************/
// Raw hardware functions
static void disp_init(void);
static void touchpad_init(void);
static uint32_t read_flash_wrapper(uint8_t* p_buffer, uint32_t address, uint32_t size);

// Artok-Runtime compatible callback wrappers
static void artok_disp_flush_cb(void *p_disp_drv, int32_t area_x1, int32_t area_y1, int32_t area_x2, int32_t area_y2, void *p_color);
static void artok_read_input_cb(void *p_indev_drv, void *p_data);

// Touchpad-related helpers
static void ConvXPTtoILI(uint16_t *x, uint16_t *y);

/**********************
 * STATIC VARIABLES
 **********************/
// LVGL draw buffers (double buffering)
static uint8_t buf_2_1[LVGL_DRAW_BUFFER_SIZE];
static uint8_t buf_2_2[LVGL_DRAW_BUFFER_SIZE];

static HMI_Hardware_Interface_t hmi_interface;
// This variable will track the current press state.
static bool touch_state = false;
/**********************
 * GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Initializes all HMI drivers and returns the hardware interface struct.
 * @return A pointer to the populated HMI_Hardware_Interface_t struct.
 */
HMI_Hardware_Interface_t *hmi_drivers_init(void)
{
    // Initialize the physical display and touchpad hardware
    disp_init();
    touchpad_init();

    // Populate the HMI_Hardware_Interface_t struct with our hardware-specific
    // function pointers and buffer addresses.
    hmi_interface.read_flash = read_flash_wrapper;
    hmi_interface.disp_flush_cb = artok_disp_flush_cb;
    hmi_interface.disp_buffer_1 = buf_2_1;
    hmi_interface.disp_buffer_2 = buf_2_2;
    hmi_interface.disp_buffer_size_bytes = LVGL_DRAW_BUFFER_SIZE;
    hmi_interface.read_input_cb = artok_read_input_cb;

    return &hmi_interface;
}


/**********************
 * STATIC FUNCTIONS
 **********************/

/**
 * @brief Initializes the physical LCD display.
 */
static void disp_init(void)
{
	ILI9341_Init();
}

/**
 * @brief LVGL-compatible wrapper for the display flush callback.
 */
static void artok_disp_flush_cb(void *p_disp_drv, int32_t area_x1, int32_t area_y1, int32_t area_x2, int32_t area_y2, void *p_color)
{
    // This function acts as the bridge. It takes the LVGL-provided void pointers
    // and correctly casts and passes them to the low-level hardware function.
	LCD_flush_dma(area_x1, area_y1, area_x2, area_y2, p_color);
}
/**
 * @brief Initializes the physical touchpad hardware.
 */
static void touchpad_init(void)
{
    XPT2046_Init();
}

/**
 * @brief LVGL-compatible wrapper for the input read callback.
 */
static void artok_read_input_cb(void *p_indev_drv, void *p_data)
{
	HMI_Touch_Data_t *data = (HMI_Touch_Data_t *)p_data;
    uint16_t raw_x = 0, raw_y = 0;

    // Read the pressure (Z-axis) from the touch controller.
    uint16_t z_value = XPT2046_ReadZ();

    // Determine the current state based on the z_value
    bool current_press_detected = (z_value > Z_THRESHOLD);

    // Print the Z-axis value for debugging purposes.
//    printf("Z-AXIS: %hu\n", z_value);

    // Check if the state has changed from released to pressed
    if (current_press_detected && !touch_state) {
        // This is a new press event.
        if (XPT2046_ReadXY(&raw_x, &raw_y)) {
            printf("RAW X: %hu, RAW Y: %hu\n", raw_x, raw_y);
            ConvXPTtoILI(&raw_x, &raw_y);
            data->x = raw_x;
            data->y = raw_y;
            data->pressed = true; // This is a confirmed press
            printf("Converted X: %hu, Converted Y: %hu\n", raw_x, raw_y);
            touch_state = true; // Update the static state
        }
    }
    // Check if the state has changed from pressed to released
    else if (!current_press_detected && touch_state) {
        // This is a new release event.
        data->pressed = false; // We are no longer pressed
        touch_state = false; // Update the static state
    }
    // If the state is still pressed, we report the position
    else if (current_press_detected && touch_state) {
        if (XPT2046_ReadXY(&raw_x, &raw_y)) {
            // **CORRECTION:** Convert the raw coordinates to display coordinates.
            ConvXPTtoILI(&raw_x, &raw_y);
            data->x = raw_x;
            data->y = raw_y;
            data->pressed = true;
        }
    }
    // If we're not touching and weren't touching before, we do nothing.
    else {
        data->pressed = false;
    }

    // Print the final pressed state that LVGL will receive.
    printf("LVGL Pressed State: %s\n", data->pressed ? "true" : "false");
}
/**
 * @brief Converts raw XPT2046 coordinates to display coordinates.
 * @param x The raw X coordinate.
 * @param y The raw Y coordinate.
 */
static void ConvXPTtoILI(uint16_t *x, uint16_t *y)
{
    int32_t tx, ty;

    // The raw XPT_Y value (from the touchpad's Y channel) maps to the display's X axis.
    // The touch range for Y is [350, 3600], and the display's X range is [0, 240].
    tx = (((int32_t)*y - XPT_YMIN) * ART_LCD_WIDTH) / (XPT_YMAX - XPT_YMIN);

    // The raw XPT_X value (from the touchpad's X channel) maps to the display's Y axis.
    // The touch range for X is [350, 3800], and the display's Y range is [0, 320].
    // This axis needs to be inverted.
    ty = (((int32_t)*x - XPT_XMIN) * ART_LCD_HEIGHT) / (XPT_XMAX - XPT_XMIN);

    // Clamp values to stay within display bounds
    if (tx < 0) tx = 0;
    if (tx >= ART_LCD_WIDTH) tx = ART_LCD_WIDTH - 1;
    if (ty < 0) ty = 0;
    if (ty >= ART_LCD_HEIGHT) ty = ART_LCD_HEIGHT - 1;

    // Update the original pointers with the new, converted values
    *x = (uint16_t)tx;
    *y = (uint16_t)ty;
}

static void ConvXPTtoILI2(uint16_t *x, uint16_t *y)
{
    // User-provided calibration values
    const uint16_t XPT_CAL_XMIN = 248;
    const uint16_t XPT_CAL_YMIN = 589;
    const uint16_t XPT_CAL_XMAX = 3584;
    const uint16_t XPT_CAL_YMAX = 3712;

    int32_t tx, ty;

    // The raw XPT_Y value maps to the display's X axis.
    tx = (((int32_t)*y - XPT_CAL_YMIN) * ART_LCD_WIDTH) / (XPT_CAL_YMAX - XPT_CAL_YMIN);

    // The raw XPT_X value maps to the display's Y axis.
    ty = (((int32_t)*x - XPT_CAL_XMIN) * ART_LCD_HEIGHT) / (XPT_CAL_XMAX - XPT_CAL_XMIN);

    // Clamp values to stay within display bounds
    if (tx < 0) tx = 0;
    if (tx >= ART_LCD_WIDTH) tx = ART_LCD_WIDTH - 1;
    if (ty < 0) ty = 0;
    if (ty >= ART_LCD_HEIGHT) ty = ART_LCD_HEIGHT - 1;

    // Update the original pointers with the new, converted values
    *x = (uint16_t)tx;
    *y = (uint16_t)ty;
}

/**
 * @brief Wrapper for the flash read function to match Artok-Runtime API.
 */
static uint32_t read_flash_wrapper(uint8_t* p_buffer, uint32_t address, uint32_t size)
{
    return W25Q_Read(p_buffer, address, size);
}
