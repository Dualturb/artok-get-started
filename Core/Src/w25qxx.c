/*
 * w25qxx.c
 *
 *  Created on: Apr 27, 2025
 *      Author: halton
 */
#include "main.h"
#include "w25qxx.h"

extern SPI_HandleTypeDef hspi1;
#define W25Q_SPI hspi1

#define numBLOCK 32  // number of total blocks for 16Mb flash

void W25Q_Delay(uint32_t time)
{
	HAL_Delay(time);
}

void csLOW (void)
{
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void csHIGH (void)
{
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void SPI_Write (uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}

void SPI_Read (uint8_t *data, uint8_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}

// --- Private Helper Functions ---

/**
 * @brief Sends a single byte over SPI.
 * @param byte The byte to send.
 * @retval The byte received from SPI.
 */
static uint8_t SPI_SendByte(uint8_t byte) {
    uint8_t rx_byte = 0;
    // Use HAL_SPI_TransmitReceive for full duplex communication
    // Timeout of HAL_MAX_DELAY means wait indefinitely. Adjust if needed.
    if (HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_byte, 1, HAL_MAX_DELAY) != HAL_OK) {
        // Handle SPI error (e.g., call Error_Handler())
        // Error_Handler();
    }
    return rx_byte;
}

/**
 * @brief Reads the Status Register 1 of the W25QXX flash chip.
 * @retval Status Register 1 value.
 */
static uint8_t W25Q_ReadStatusRegister1(void) {
    uint8_t status = 0;
    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_READ_STATUS_REG1); // Send Read Status Register 1 command
    status = SPI_SendByte(0xFF); // Send dummy byte to clock out status
    W25Q_CS_HIGH(); // Deselect flash chip
    return status;
}

/**
 * @brief Reads the Status Register 2 of the W25QXX flash chip.
 * @retval Status Register 2 value.
 */
static uint8_t W25Q_ReadStatusRegister2(void) {
    uint8_t status = 0;
    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_READ_STATUS_REG2); // Send Read Status Register 2 command
    status = SPI_SendByte(0xFF); // Send dummy byte to clock out status
    W25Q_CS_HIGH(); // Deselect flash chip
    return status;
}

/**
 * @brief Sends the Write Enable command to the W25QXX flash chip.
 * Must be called before any write or erase operation.
 * @retval None
 */
static void W25Q_WriteEnable(void) {
    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_WRITE_ENABLE); // Send Write Enable command
    W25Q_CS_HIGH(); // Deselect flash chip
}


/**
 * @brief Waits for the W25QXX flash chip to finish a write or erase operation.
 * Checks the BUSY bit in Status Register 1.
 * @retval None
 */
static void W25Q_WaitForWriteEnd(void) {
    // Wait for the BUSY bit (bit 0) in Status Register 1 to become 0
    while ((W25Q_ReadStatusRegister1() & W25Q_SR1_BUSY) != 0) {
        // Add a small delay or yield if using an RTOS to avoid blocking
        // HAL_Delay(1); // Example delay
    }
}


// --- Public Functions Implementation ---

/**
 * @brief Initializes the W25QXX flash chip.
 * @retval None
 */
void W25Q_Init(void) {
    // Ensure CS pin is configured as GPIO Output Push Pull and is initially high
    // This should be done in MX_GPIO_Init in main.c
    W25Q_CS_HIGH(); // Ensure CS is high initially

    // Optional: Perform a software reset
    W25Q_Reset();

    // Optional: Read ID to verify communication
     uint32_t id = W25Q_ReadID();
    // You can print or check the ID here for debugging
}

/**
 * @brief Resets the W25QXX flash chip using the software reset command.
 * @retval None
 */
void W25Q_Reset(void) {
    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(0x66); // Send Enable Reset command
    W25Q_CS_HIGH(); // Deselect flash chip
    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(0x99); // Send Reset Device command
    W25Q_CS_HIGH(); // Deselect flash chip
    HAL_Delay(1); // Wait for reset to complete (datasheet recommends >200us)
}


/**
 * @brief Reads the JEDEC ID of the W25QXX flash chip.
 * @retval JEDEC ID (Manufacturer ID, Memory Type, Capacity)
 */
uint32_t W25Q_ReadID(void) {
    uint8_t id_bytes[3] = {0};
    uint32_t jedec_id = 0;

    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_JEDEC_ID); // Send JEDEC ID command
    id_bytes[0] = SPI_SendByte(0xFF); // Read Manufacturer ID
    id_bytes[1] = SPI_SendByte(0xFF); // Read Memory Type
    id_bytes[2] = SPI_SendByte(0xFF); // Read Capacity
    W25Q_CS_HIGH(); // Deselect flash chip

    jedec_id = (uint32_t)(id_bytes[0] << 16) | (uint32_t)(id_bytes[1] << 8) | id_bytes[2];
    return jedec_id;
}

/**
 * @brief Reads data from the W25QXX flash chip.
 * @param pData Pointer to the buffer to store the read data.
 * @param ReadAddr Start address to read from (24-bit).
 * @param Size Number of bytes to read.
 * @retval The actual number of bytes read (Size on success, 0 on failure).
 */
uint32_t W25Q_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size) {
	uint32_t bytes_read = 0;
    W25Q_WaitForWriteEnd(); // Ensure flash is not busy

    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_READ_DATA); // Send Read Data command

    // Send 24-bit address (MSB first)
    SPI_SendByte((ReadAddr >> 16) & 0xFF);
    SPI_SendByte((ReadAddr >> 8) & 0xFF);
    SPI_SendByte(ReadAddr & 0xFF);

    // Read data bytes
    if (HAL_SPI_Receive(&hspi1, pData, Size, HAL_MAX_DELAY) == HAL_OK) {
        bytes_read = Size; // Success: return the requested size
    } else {
        // SPI read failed, return 0 bytes read
        // You may want to log an error here
        bytes_read = 0;
    }

    W25Q_CS_HIGH(); // Deselect flash chip
    return bytes_read;
}

/**
 * @brief Writes data to a page of the W25QXX flash chip.
 * Note: The target sector must be erased before writing.
 * @param pData Pointer to the buffer containing the data to write.
 * @param WriteAddr Start address to write to (should be page aligned for optimal performance).
 * @param Size Number of bytes to write (should not exceed page boundaries, max 256).
 * @retval None
 */
void W25Q_WritePage(uint8_t* pData, uint32_t WriteAddr, uint16_t Size) {
    if (Size == 0 || Size > W25Q_PAGE_SIZE) return; // Basic validation

    W25Q_WaitForWriteEnd(); // Ensure flash is not busy
    W25Q_WriteEnable();     // Send Write Enable command

    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_PAGE_PROGRAM); // Send Page Program command

    // Send 24-bit address (MSB first)
    SPI_SendByte((WriteAddr >> 16) & 0xFF);
    SPI_SendByte((WriteAddr >> 8) & 0xFF);
    SPI_SendByte(WriteAddr & 0xFF);

    // Send data bytes
    if (HAL_SPI_Transmit(&hspi1, pData, Size, HAL_MAX_DELAY) != HAL_OK) {
         // Handle SPI error
         // Error_Handler();
    }

    W25Q_CS_HIGH(); // Deselect flash chip
    W25Q_WaitForWriteEnd(); // Wait for programming to complete
}


/**
 * @brief Erases a sector (4KB) of the W25QXX flash chip.
 * @param SectorAddr Address within the sector to erase (e.g., start address of the sector).
 * @retval None
 */
void W25Q_EraseSector(uint32_t SectorAddr) {
    // Sector erase address must be sector aligned (multiple of 4096)
    uint32_t aligned_address = SectorAddr & ~(W25Q_SECTOR_SIZE - 1);

    W25Q_WaitForWriteEnd(); // Ensure flash is not busy
    W25Q_WriteEnable();     // Send Write Enable command

    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_SECTOR_ERASE); // Send Sector Erase command

    // Send 24-bit address (MSB first)
    SPI_SendByte((aligned_address >> 16) & 0xFF);
    SPI_SendByte((aligned_address >> 8) & 0xFF);
    SPI_SendByte(aligned_address & 0xFF);

    W25Q_CS_HIGH(); // Deselect flash chip
    W25Q_WaitForWriteEnd(); // Wait for erase to complete (can take tens or hundreds of ms)
}


/**
 * @brief Erases a 32KB block of the W25QXX flash chip.
 * @param BlockAddr Address within the 32KB block to erase.
 * @retval None
 */
void W25Q_EraseBlock32KB(uint32_t BlockAddr) {
     // 32KB block erase address must be 32KB aligned
    uint32_t aligned_address = BlockAddr & ~(W25Q_BLOCK_SIZE_32KB - 1);

    W25Q_WaitForWriteEnd(); // Ensure flash is not busy
    W25Q_WriteEnable();     // Send Write Enable command

    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_BLOCK_ERASE_32KB); // Send 32KB Block Erase command

    // Send 24-bit address (MSB first)
    SPI_SendByte((aligned_address >> 16) & 0xFF);
    SPI_SendByte((aligned_address >> 8) & 0xFF);
    SPI_SendByte(aligned_address & 0xFF);

    W25Q_CS_HIGH(); // Deselect flash chip
    W25Q_WaitForWriteEnd(); // Wait for erase to complete
}


/**
 * @brief Erases a 64KB block of the W25QXX flash chip.
 * @param BlockAddr Address within the 64KB block to erase.
 * @retval None
 */
void W25Q_EraseBlock64KB(uint32_t BlockAddr) {
    // 64KB block erase address must be 64KB aligned
    uint32_t aligned_address = BlockAddr & ~(W25Q_BLOCK_SIZE_64KB - 1);

    W25Q_WaitForWriteEnd(); // Ensure flash is not busy
    W25Q_WriteEnable();     // Send Write Enable command

    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_BLOCK_ERASE_64KB); // Send 64KB Block Erase command

    // Send 24-bit address (MSB first)
    SPI_SendByte((aligned_address >> 16) & 0xFF);
    SPI_SendByte((aligned_address >> 8) & 0xFF);
    SPI_SendByte(aligned_address & 0xFF);

    W25Q_CS_HIGH(); // Deselect flash chip
    W25Q_WaitForWriteEnd(); // Wait for erase to complete
}


/**
 * @brief Erases the entire W25QXX flash chip.
 * @retval None
 */
void W25Q_EraseChip(void) {
    W25Q_WaitForWriteEnd(); // Ensure flash is not busy
    W25Q_WriteEnable();     // Send Write Enable command

    W25Q_CS_LOW(); // Select flash chip
    SPI_SendByte(W25Q_CMD_CHIP_ERASE); // Send Chip Erase command
    W25Q_CS_HIGH(); // Deselect flash chip

    W25Q_WaitForWriteEnd(); // Wait for erase to complete (can take seconds)
}



