/**
 * @file xpt2046.c
 * @brief Low-level driver for the XPT2046 touch controller.
 *
 * This file provides the implementation for initializing the XPT2046 chip
 * and reading touch coordinates via the SPI bus.
 */

#include "xpt2046.h"
#include "main.h"
#include <stdbool.h>

/**********************
 * EXTERN VARIABLES
 **********************/
// External declaration for the SPI handle used to communicate with the XPT2046.
// Assumes the touch controller is on SPI3.
extern SPI_HandleTypeDef hspi3;


/**********************
 * STATIC PROTOTYPES
 **********************/
// Functions to control the Chip Select (CS) pin
static void XPT2046_CS_Select(void);
static void XPT2046_CS_Deselect(void);

// Function to read a single axis from the touch controller
static uint16_t XPT2046_Read_Axis(uint8_t command);

// Helper function to average the best two of three readings
static int16_t besttwoavg(int16_t x, int16_t y, int16_t z);


/**********************
 * GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Initializes the XPT2046 touch controller.
 */
void XPT2046_Init(void) {
    // No specific software initialization is required for the XPT2046 itself.
    // The hardware is already configured via the GPIO_Init() and SPI_Init()
    // calls. We just ensure the Chip Select pin is in its default (inactive) state.
    XPT2046_CS_Deselect();
    HAL_Delay(10);
}

/**
 * @brief Reads the touch coordinates from the XPT2046.
 *
 * This function should only be called when a touch event is detected
 * (i.e., when the PENIRQ line goes low). It performs the SPI transactions
 * to get the x and y coordinates.
 *
 * @param x A pointer to a variable to store the raw X coordinate.
 * @param y A pointer to a variable to store the raw Y coordinate.
 * @return Returns true if a valid touch is read, false otherwise.
 */
bool XPT2046_Read(uint16_t *x, uint16_t *y) {
    // The check for a touch event is now handled by the artok_read_input_cb
    // function. We assume a touch is present here.

    // Select the XPT2046 chip and hold CS low for the entire transaction.
    XPT2046_CS_Select();
    HAL_Delay(1);

    // Perform a Z-axis pressure check before reading coordinates.
    // If pressure is too low, it's a false touch.
    int16_t z1 = XPT2046_Read_Axis(0xb1);
    int16_t z2 = XPT2046_Read_Axis(0xc1);

    int32_t z = z1 + 4095 - z2;
    if (z < Z_THRESHOLD) {
        XPT2046_CS_Deselect(); // Release CS on failure
        return false;
    }

    // Read X and Y coordinates multiple times to average the values.
    int16_t data[6];
    XPT2046_Read_Axis(0x91);  // Dummy read to settle ADC
    data[0] = XPT2046_Read_Axis(0x91);
    data[1] = XPT2046_Read_Axis(0xd1);
    data[2] = XPT2046_Read_Axis(0x91);
    data[3] = XPT2046_Read_Axis(0xd1);
    data[4] = XPT2046_Read_Axis(0x91);
    data[5] = XPT2046_Read_Axis(0xd1);

    // Deselect the chip.
    XPT2046_CS_Deselect();

    // Average the coordinates using the best two out of three readings.
    int16_t intx = besttwoavg(data[0], data[2], data[4]);
    int16_t inty = besttwoavg(data[1], data[3], data[5]);

    // Check if the coordinates are within a reasonable range before returning.
    if (!XPT2046_IsReasonable(intx, inty)) {
        return false;
    }

    *x = intx;
    *y = inty;

    return true;
}


/**********************
 * STATIC FUNCTIONS
 **********************/

/**
 * @brief Selects the XPT2046 Chip Select pin (active low).
 */
static void XPT2046_CS_Select(void) {
    HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Deselects the XPT2046 Chip Select pin.
 */
static void XPT2046_CS_Deselect(void) {
    HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Reads a single axis value from the XPT2046.
 *
 * @param command The command byte for the desired axis (CMD_RDX or CMD_RDY).
 * @return The 12-bit ADC value.
 */
static uint16_t XPT2046_Read_Axis(uint8_t command) {
    uint8_t tx_buf[1] = {command};
    uint8_t rx_buf[2];

    // Transmit the command byte
    HAL_SPI_Transmit(&hspi3, tx_buf, 1, HAL_MAX_DELAY);

    // Receive the two data bytes while sending dummy bytes to clock them in.
    uint8_t dummy_tx[] = {0x00, 0x00};
    HAL_SPI_TransmitReceive(&hspi3, dummy_tx, rx_buf, 2, HAL_MAX_DELAY);

    // The first received byte is garbage. The next two bytes contain the 12-bit data.
    uint16_t data = (rx_buf[0] << 8) | rx_buf[1];
    return data >> 3;
}

/**
 * @brief Helper function to average the best two of three values.
 */
static int16_t besttwoavg(int16_t x, int16_t y, int16_t z) {
    int16_t da, db, dc;
    if (x > y) da = x - y; else da = y - x;
    if (x > z) db = x - z; else db = z - x;
    if (z > y) dc = z - y; else dc = y - z;

    if (da <= db && da <= dc) return (x + y) >> 1;
    if (db <= da && db <= dc) return (x + z) >> 1;
    return (y + z) >> 1;
}

/**
 * @brief Checks if the raw coordinates are within a reasonable range.
 * @param x The raw x-coordinate.
 * @param y The raw y-coordinate.
 * @return 1 if reasonable, 0 otherwise.
 */
uint8_t XPT2046_IsReasonable(uint16_t x, uint16_t y)
{
    if (x >= XPT_XMIN && x <= XPT_XMAX && y >= XPT_YMIN && y <= XPT_YMAX) {
        return 1;
    }
    return 0;
}
