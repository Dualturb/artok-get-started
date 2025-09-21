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

// This type must match the one provided by the LVGL runtime.
typedef struct {
    int16_t x;
    int16_t y;
    bool pressed;
} TouchData_t;

/**********************
 * STATIC PROTOTYPES
 **********************/
// Raw hardware functions
static void disp_init(void);
static void touchpad_init(void);
static void read_flash_wrapper(uint8_t* p_buffer, uint32_t address, uint32_t size);

// Artok-Runtime compatible callback wrappers
static void artok_disp_flush_cb(void *p_disp_drv, int32_t area_x1, int32_t area_y1, int32_t area_x2, int32_t area_y2, void *p_color);
static void artok_read_input_cb(void *p_indev_drv, void *p_data);

// Touchpad-related helpers
static bool touchpad_is_pressed(void);
static void ConvXPTtoILI(uint16_t *x, uint16_t *y);

/**********************
 * STATIC VARIABLES
 **********************/
// LVGL draw buffers (double buffering)
static uint8_t buf_2_1[LVGL_DRAW_BUFFER_SIZE];
static uint8_t buf_2_2[LVGL_DRAW_BUFFER_SIZE];

static HMI_Hardware_Interface_t hmi_interface;

/**********************
 * EXTERN VARIABLES
 **********************/
// The touch_event_flag is set by the EXTI interrupt in main.c
extern volatile bool touch_event_flag;

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
	LCD_flush(area_x1, area_y1, area_x2, area_y2, p_color);
	ART_FlushComplete();
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
    TouchData_t *data = (TouchData_t *)p_data;
    uint16_t raw_x = 0, raw_y = 0;

    data->pressed = false; // Assume no press by default

    // Check the touch event flag instead of directly polling the pin.
    if (touch_event_flag) {
        // A touch has occurred, so read the coordinates from the touch IC
        if (XPT2046_Read(&raw_x, &raw_y)) {
            // If the read was successful, report the press
            ConvXPTtoILI(&raw_x, &raw_y);
            data->x = raw_x;
            data->y = raw_y;
            data->pressed = true;
        } else {
            // The pin was low, but the read failed (e.g., pressure too low)
            data->pressed = false;
        }

        // Reset the flag after the read attempt, regardless of success.
        touch_event_flag = false;
    }
}

/**
 * @brief Converts raw XPT2046 coordinates to display coordinates.
 * @param x The raw X coordinate.
 * @param y The raw Y coordinate.
 */
static void ConvXPTtoILI(uint16_t *x, uint16_t *y)
{
    int16_t tx, ty;
    tx = (int16_t)(((int32_t)*x - XPT_XMIN) * ART_LCD_WIDTH / XPT_WIDTH);
    tx = (tx < 0) ? 0 : tx;
    tx = (tx >= ART_LCD_WIDTH) ? ART_LCD_WIDTH - 1 : tx;
    ty = (int16_t)(((int32_t)*y - XPT_YMIN) * ART_LCD_HEIGHT / XPT_HEIGHT);
    ty = (ty < 0) ? 0 : ty;
    ty = (ty >= ART_LCD_HEIGHT) ? ART_LCD_HEIGHT - 1 : ty;
    *x = tx;
    *y = ty;
}

/**
 * @brief Checks the state of the TP_IRQ pin.
 * @return True if the pin is low (touch detected), false otherwise.
 */
static bool touchpad_is_pressed(void)
{
    return HAL_GPIO_ReadPin(TP_IRQ_GPIO_Port, TP_IRQ_Pin) == GPIO_PIN_RESET;
}

/**
 * @brief Wrapper for the flash read function to match Artok-Runtime API.
 */
static void read_flash_wrapper(uint8_t* p_buffer, uint32_t address, uint32_t size)
{
    W25Q_Read(p_buffer, address, size);
}
