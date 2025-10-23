/**
 * @file xpt2046.c
 * @brief Source file for the XPT2046 touch controller driver.
 *
 * This file contains the implementation of functions for communicating with the
 * XPT2046 touch controller. It uses a pressure-based polling method rather than
 * relying on the IRQ pin.
 */

#include "xpt2046.h"
#include "main.h" // For SPI and HAL functions

// Extern declaration for the SPI handle
extern SPI_HandleTypeDef hspi3;

// Commands for the XPT2046
#define CMD_X_MEASURE 0x90
#define CMD_Y_MEASURE 0xD0
#define CMD_Z1_MEASURE 0xB1
#define CMD_Z2_MEASURE 0xC1

// Private function prototypes
static void XPT2046_WriteByte(uint8_t data);
static uint16_t XPT2046_ReadAD(uint8_t command);
static int16_t besttwoavg(int16_t x, int16_t y, int16_t z);

/**
 * @brief Initializes the XPT2046 driver.
 *
 * This function is used to signal that the driver is ready.
 */
void XPT2046_Init(void)
{
    // A dummy read to wake up the controller
    XPT2046_ReadAD(CMD_X_MEASURE);
}

/**
 * @brief Reads the raw pressure value (Z-axis) from the XPT2046.
 * @return The raw Z-axis value.
 */
uint16_t XPT2046_ReadZ(void)
{
    // Z-axis measurement is more reliable for press detection
    uint16_t z1 = XPT2046_ReadAD(CMD_Z1_MEASURE);
    uint16_t z2 = XPT2046_ReadAD(CMD_Z2_MEASURE);
    uint16_t z_val = (z1 + 4095) - z2;
    return z_val;
}

/**
 * @brief Reads the X and Y coordinates from the XPT2046.
 * @param x Pointer to store the raw X coordinate.
 * @param y Pointer to store the raw Y coordinate.
 * @return true if a valid read occurred, false otherwise.
 */
bool XPT2046_ReadXY(uint16_t *x, uint16_t *y)
{
    uint16_t tx_samples[3];
    uint16_t ty_samples[3];

    // Read multiple times to filter out noise
    tx_samples[0] = XPT2046_ReadAD(CMD_X_MEASURE);
    ty_samples[0] = XPT2046_ReadAD(CMD_Y_MEASURE);
    tx_samples[1] = XPT2046_ReadAD(CMD_X_MEASURE);
    ty_samples[1] = XPT2046_ReadAD(CMD_Y_MEASURE);
    tx_samples[2] = XPT2046_ReadAD(CMD_X_MEASURE);
    ty_samples[2] = XPT2046_ReadAD(CMD_Y_MEASURE);

    // Get the averaged values
    *x = besttwoavg(tx_samples[0], tx_samples[1], tx_samples[2]);
    *y = besttwoavg(ty_samples[0], ty_samples[1], ty_samples[2]);

    // Simple validation based on a wide range.
    // Specific calibration is done in hmi_drivers.c
    if (*x >= XPT_XMIN && *x <= XPT_XMAX && *y >= XPT_YMIN && *y <= XPT_YMAX) {
        return true;
    }

    return false;
}

/**
 * @brief Writes a single byte to the XPT2046 via SPI.
 * @param data The byte to send.
 */
static void XPT2046_WriteByte(uint8_t data)
{
    HAL_SPI_Transmit(&hspi3, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reads a 12-bit ADC value from the XPT2046.
 * @param command The command byte for the desired measurement.
 * @return The 12-bit ADC value.
 */
static uint16_t XPT2046_ReadAD(uint8_t command)
{
    uint8_t tx_buf[1];
    uint8_t rx_buf[2];
    uint16_t result;

    tx_buf[0] = command;

    HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_buf, 1, 100);
    HAL_SPI_Receive(&hspi3, rx_buf, 2, 100);
    HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);

    result = (rx_buf[0] << 8) | rx_buf[1];
    result >>= 3; // The XPT2046 returns a 16-bit value, but only 12 bits are valid.
    return result;
}

/**
 * @brief Averages the two closest values from three measurements to reduce noise.
 */
static int16_t besttwoavg(int16_t x, int16_t y, int16_t z)
{
    int16_t da, db, dc;
    int16_t reta;

    if (x > y) da = x - y;
    else da = y - x;
    if (x > z) db = x - z;
    else db = z - x;
    if (z > y) dc = z - y;
    else dc = y - z;

    if (da <= db && da <= dc) {
        reta = (x + y) >> 1;
    } else if (db <= da && db <= dc) {
        reta = (x + z) >> 1;
    } else {
        reta = (y + z) >> 1;
    }
    return (reta);
}
