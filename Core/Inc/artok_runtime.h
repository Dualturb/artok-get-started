/**
 * @file artok_runtime.h
 * @brief Defines the core API and hardware abstraction layer for the ARTOK HMI runtime.
 *
 * This header provides the necessary structures and functions to initialize,
 * run, and manage the HMI, bridging the gap between the generated UI
 * and the target hardware.
 */

#ifndef INC_ARTOK_RUNTIME_H_
#define INC_ARTOK_RUNTIME_H_

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief HMI color depth configuration.
 *
 * This define controls the color depth used by the LVGL UI library.
 * The artok-runtime will automatically handle conversion to the
 * hardware's native format (e.g., RGB565).
 *
 * Options:
 * 1 (1 byte per pixel)
 * 8 (RGB332)
 * 16 (RGB565)
 * 32 (ARGB8888)
 */
#define ART_COLOR_DEPTH 16

/**
 * @brief HMI display dimensions.
 *
 * These defines control the resolution of the display and are used
 * internally by the runtime to configure the UI library.
 */
#define ART_LCD_WIDTH 240
#define ART_LCD_HEIGHT 320

/**
 * @brief Generic data structure for touch input.
 */
typedef struct {
    int16_t x;
    int16_t y;
    bool pressed;
} HMI_Touch_Data_t;

/**
 * @brief Represents the hardware abstraction layer (HAL) interface.
 *
 * This struct provides function pointers and data addresses that the HMI runtime
 * will use to interact with the target hardware's peripherals.
 */
typedef struct {
    /** A function pointer for reading data from non-volatile memory (e.g., flash). */
    uint32_t (*read_flash)(uint8_t* p_buffer, uint32_t address, uint32_t size);

    /** The display flush callback provided by the customer.
     * It's responsible for sending rendered pixels to the physical display. */
    void (*disp_flush_cb)(void *p_disp_drv, int32_t area_x1, int32_t area_y1, int32_t area_x2, int32_t area_y2, void *p_color);

    /** The main display buffer. LVGL will render into this. */
    void *disp_buffer_1;

    /** An optional second buffer for double-buffering. */
    void *disp_buffer_2;

    /** The size of the display buffer in bytes. */
    uint32_t disp_buffer_size_bytes;
    
    /** The input read callback provided by the customer.
     * It's responsible for reporting touch input data (x, y, and button state). */
    void (*read_input_cb)(void *p_indev_drv, void *p_data);
} HMI_Hardware_Interface_t;

/**
 * @brief Initializes and starts the HMI runtime.
 *
 * This function handles the parsing of the UI binary, building of the UI,
 * and initialization of the event handling system.
 *
 * @param ui_binary_addr The starting address of the UI binary in flash.
 * @param p_hardware_interface A pointer to the hardware abstraction interface.
 * @return true if the HMI runtime started successfully, false otherwise.
 */
bool ART_StartHMI(uint32_t ui_binary_addr, const HMI_Hardware_Interface_t* p_hardware_interface);

/**
 * @brief The main loop for the HMI runtime.
 *
 * This function should be called continuously to process LVGL and Lua tasks,
 * ensuring the UI remains responsive.
 */
void ART_MainLoop(void);

/**
 * @brief Cleans up all resources used by the HMI runtime.
 */
void ART_Cleanup(void);

/**
 * @brief Increments the HMI runtime's internal tick counter.
 *
 * This function serves as a portable interface for the system timer.
 * It should be called periodically (e.g., every 1ms) from a system
 * timer interrupt in the main application.
 *
 * @param ms The number of milliseconds to increment the tick by.
 */
void ART_IncTick(uint32_t ms);


/**
 * @brief Notifies the ARTOK runtime that a display flush operation has completed.
 *
 * This function should be called by the firmware's display driver from its
 * DMA transfer complete callback. It signals that the hardware is ready
 * for the next rendering operation. This function is the sole public API
 * for this purpose, keeping the firmware decoupled from the UI library details.
 */
void ART_FlushComplete(void);

#endif /* INC_ARTOK_RUNTIME_H_ */
