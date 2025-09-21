/**
 * @file hmi_drivers.h
 *
 * This header file contains the public API for the HMI's
 * display and input device drivers.
 */

#ifndef HMI_DRIVERS_H
#define HMI_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 * INCLUDES
 *********************/
#include <stdbool.h>
#include <stdint.h>

#include "artok_runtime.h" // Includes the definition for HMI_Hardware_Interface_t

/**********************
 * DEFINES
 **********************/

/**
 * @brief Size of the LVGL draw buffer in bytes.
 *
 * This buffer size is determined by the display width and the color depth,
 * and is used for rendering a portion of the screen before flushing.
 * Here we use two rows for efficiency.
 */
#define LVGL_DRAW_BUFFER_SIZE (ART_LCD_WIDTH * 2 * (ART_COLOR_DEPTH / 8))


/**********************
 * GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Initializes all HMI drivers and returns the hardware interface struct.
 *
 * This is the entry point for HMI driver initialization. It sets up the
 * physical hardware and populates the `HMI_Hardware_Interface_t` struct with
 * pointers to the appropriate callback functions and buffers.
 *
 * @return A pointer to the populated HMI_Hardware_Interface_t struct.
 */
HMI_Hardware_Interface_t* hmi_drivers_init(void);


#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* HMI_DRIVERS_H */
