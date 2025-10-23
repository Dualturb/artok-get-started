/*
 * w25qxx.h
 *
 *  Created on: Apr 27, 2025
 *      Author: halton
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

#include "stm32f1xx_hal.h" // Include HAL for SPI handle type and GPIO
#include "main.h"

// --- W25Q Command Definitions ---
#define W25Q_CMD_WRITE_ENABLE     0x06 // Write Enable
#define W25Q_CMD_VOLATILE_SR_WRITE_ENABLE 0x50 // Volatile SR Write Enable
#define W25Q_CMD_WRITE_DISABLE    0x04 // Write Disable
#define W25Q_CMD_READ_STATUS_REG1 0x05 // Read Status Register 1
#define W25Q_CMD_READ_STATUS_REG2 0x35 // Read Status Register 2
#define W25Q_CMD_READ_STATUS_REG3 0x15 // Read Status Register 3
#define W25Q_CMD_WRITE_STATUS_REG1 0x01 // Write Status Register 1
#define W25Q_CMD_WRITE_STATUS_REG2 0x31 // Write Status Register 2
#define W25Q_CMD_WRITE_STATUS_REG3 0x11 // Write Status Register 3
#define W25Q_CMD_READ_DATA        0x03 // Read Data
#define W25Q_CMD_FAST_READ        0x0B // Fast Read (requires dummy byte)
#define W25Q_CMD_PAGE_PROGRAM     0x02 // Page Program
#define W25Q_CMD_SECTOR_ERASE     0x20 // Sector Erase (4KB)
#define W25Q_CMD_BLOCK_ERASE_32KB 0x52 // Block Erase (32KB)
#define W25Q_CMD_BLOCK_ERASE_64KB 0xD8 // Block Erase (64KB)
#define W25Q_CMD_CHIP_ERASE       0xC7 // Chip Erase (or 0x60)
#define W25Q_CMD_POWER_DOWN       0xB9 // Power Down
#define W25Q_CMD_RELEASE_POWER_DOWN 0xAB // Release Power Down / Device ID
#define W25Q_CMD_MANUFACTURER_DEVICE_ID 0x90 // Manufacturer / Device ID
#define W25Q_CMD_JEDEC_ID         0x9F // JEDEC ID

// --- Status Register 1 Bits ---
#define W25Q_SR1_BUSY             0x01 // Busy bit (1=Busy, 0=Ready)
#define W25Q_SR1_WEL              0x02 // Write Enable Latch (1=Enabled, 0=Disabled)
#define W25Q_SR1_BP0              0x04 // Block Protect Bit 0
#define W25Q_SR1_BP1              0x08 // Block Protect Bit 1
#define W25Q_SR1_BP2              0x10 // Block Protect Bit 2
#define W25Q_SR1_TB               0x20 // Top/Bottom Protect
#define W25Q_SR1_SEC              0x40 // Sector Protect
#define W25Q_SR1_SPRJ             0x80 // Status Register Protect

// --- Status Register 2 Bits ---
#define W25Q_SR2_SRL              0x01 // Status Register Lock
#define W25Q_SR2_QE               0x02 // Quad Enable (Set this for Quad SPI mode)
// ... other bits in SR2 ...


// --- Memory Organization ---
#define W25Q_PAGE_SIZE            256     // Bytes per page
#define W25Q_SECTOR_SIZE          4096    // Bytes per sector (16 pages)
#define W25Q_BLOCK_SIZE_32KB      32768   // Bytes per 32KB block (8 sectors)
#define W25Q_BLOCK_SIZE_64KB      65536   // Bytes per 64KB block (16 sectors)
#define W25Q_SECTORS_PER_BLOCK_64KB 16    // Sectors per 64KB block
#define W25Q_BLOCKS_64KB          256     // Number of 64KB blocks (128Mb / 64KB = 2048 sectors / 16 sectors/block = 128 blocks)
#define W25Q_SECTOR_COUNT         4096    // Total number of 4KB sectors (128Mb / 4KB = 16384 KB / 4KB = 4096 sectors)
#define W25Q_PAGE_COUNT           65536   // Total number of pages (128Mb / 256 Bytes = 16777216 / 256 = 65536 pages)

// --- Total Capacity (128 Megabits = 16 Megabytes) ---
#define W25Q_CAPACITY             16777216 // Total bytes (16 * 1024 * 1024)

// --- SPI Handle ---
// Declare the SPI handle used for communication with W25Q128
extern SPI_HandleTypeDef hspi1;

// --- Chip Select (CS) Pin Definitions ---
// Define the GPIO port and pin for the W25Q128 Chip Select
#define W25Q_CS_GPIO_PORT         F_CS_GPIO_Port // Adjust if your CS pin is on a different port
#define W25Q_CS_PIN               F_CS_Pin // Adjust if your CS pin is different (e.g., FLSH_CSn_Pin from main.c)

// --- Chip Select Control Macros ---
#define W25Q_CS_LOW()             HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_PIN, GPIO_PIN_RESET)
#define W25Q_CS_HIGH()            HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_PIN, GPIO_PIN_SET)

// --- Function Prototypes ---

/**
 * @brief Initializes the W25QXX flash chip.
 * @retval None
 */
void W25Q_Init(void);

/**
 * @brief Resets the W25QXX flash chip using the software reset command.
 * @retval None
 */
void W25Q_Reset(void);

/**
 * @brief Reads the JEDEC ID of the W25QXX flash chip.
 * @retval JEDEC ID (Manufacturer ID, Memory Type, Capacity)
 */
uint32_t W25Q_ReadID(void);

/**
 * @brief Reads data from the W25QXX flash chip.
 * @param pData Pointer to the buffer to store the read data.
 * @param ReadAddr Start address to read from.
 * @param Size Number of bytes to read.
 * @retval The actual number of bytes read (Size on success, 0 on failure).
 */
uint32_t W25Q_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);

/**
 * @brief Writes data to a page of the W25QXX flash chip.
 * Note: The target sector must be erased before writing.
 * @param pData Pointer to the buffer containing the data to write.
 * @param WriteAddr Start address to write to (should be page aligned for optimal performance).
 * @param Size Number of bytes to write (should not exceed page boundaries).
 * @retval None
 */
void W25Q_WritePage(uint8_t* pData, uint32_t WriteAddr, uint16_t Size);


/**
 * @brief Writes data to the W25QXX flash chip. Handles page boundaries.
 * Note: The target sectors must be erased before writing.
 * @param pData Pointer to the buffer containing the data to write.
 * @param WriteAddr Start address to write to.
 * @param Size Number of bytes to write.
 * @retval None
 */
void W25Q_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);


/**
 * @brief Erases a sector (4KB) of the W25QXX flash chip.
 * @param SectorAddr Address within the sector to erase.
 * @retval None
 */
void W25Q_EraseSector(uint32_t SectorAddr);

/**
 * @brief Erases a 32KB block of the W25QXX flash chip.
 * @param BlockAddr Address within the 32KB block to erase.
 * @retval None
 */
void W25Q_EraseBlock32KB(uint32_t BlockAddr);


/**
 * @brief Erases a 64KB block of the W25QXX flash chip.
 * @param BlockAddr Address within the 64KB block to erase.
 * @retval None
 */
void W25Q_EraseBlock64KB(uint32_t BlockAddr);


/**
 * @brief Erases the entire W25QXX flash chip.
 * @retval None
 */
void W25Q_EraseChip(void);


#endif /* INC_W25QXX_H_ */
