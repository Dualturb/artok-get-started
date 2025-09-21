/*
 * flush.h
 *
 *  Created on: May 10, 2025
 *      Author: halton
 */

#ifndef INC_FLUSH_H_
#define INC_FLUSH_H_
// Include necessary headers from your project (modify based on your project structure)
#include "main.h" // Generally includes basic types and HAL defines
#include <stdbool.h> // For bool type

// Define the size of the RX ring buffer (must be a power of 2)
#ifndef RX_RING_BUFFER_SIZE
#define RX_RING_BUFFER_SIZE 256 // Example size: 256 bytes
#endif

// Declare the buffer and pointers (defined in flush.c)
extern uint8_t rx_ring_buffer[RX_RING_BUFFER_SIZE];
extern volatile uint16_t rx_write_ptr; // Index where the next incoming byte will be written by ISR
extern volatile uint16_t rx_read_ptr;  // Index where the next byte to be processed will be read from by main loop

// Declare ring buffer helper functions (implemented in flush.c)
bool is_rx_buffer_empty(void);
bool is_rx_buffer_full(void); // Needed by the ISR


// --- Flash Programmer States ---
typedef enum {
    PROG_STATE_IDLE = 0,
    PROG_STATE_WAIT_START2,         // After receiving 0xAA
    PROG_STATE_WAIT_OPERATION_CMD,  // NEW STATE: After receiving 0x55 and sending ACK_START_ACKNOWLEDGED (0x04)
    PROG_STATE_WAIT_ADDR1,          // After receiving Operation Type (0x00) and sending ACK_OK (0x01)
    PROG_STATE_WAIT_ADDR2,          // After receiving Address byte 1
    PROG_STATE_WAIT_ADDR3,          // After receiving Address byte 2
    PROG_STATE_WAIT_SIZE1,          // After receiving Address byte 3
    PROG_STATE_WAIT_SIZE2,          // After receiving Size byte 1
    PROG_STATE_WAIT_DATA,           // After receiving Size byte 2 and sending ACK_READY (0x02)
    PROG_STATE_PROGRAMMING,         // In the process of programming a data chunk
    PROG_STATE_WAIT_END1,           // After programming the last data chunk
    PROG_STATE_WAIT_END2,           // After receiving END1 (0x55)
    PROG_STATE_ERROR                // Error state
} FlashProgState_t;

// --- Protocol Commands ---
#define PROG_CMD_START1     0xAA
#define PROG_CMD_START2     0x55
#define PROG_CMD_END1       0x55
#define PROG_CMD_END2       0xAA

// --- Operation Types ---
#define PROG_OP_PROGRAM     0x00
// #define PROG_OP_ERASE_SECTOR 0x01 // Example for future operations

// --- Acknowledge (ACK) Types ---
#define PROG_ACK_OK                   0x01 // Command received and processed, waiting for next step (used after Op Type)
#define PROG_ACK_READY                0x02 // Ready to receive data bytes (used after Address/Size)
#define PROG_ACK_DONE                 0x03 // Operation completed successfully (used after End Command)
#define PROG_ACK_START_ACKNOWLEDGED   0x04 // NEW ACK: Start command (AA 55) received and acknowledged
#define PROG_ACK_ERROR                0xFF // Protocol error occurred


// --- Public Function Prototypes ---

// Function to initialize the flash programmer module
void Flash_Programmer_Init(void);

// Function to process data from the ring buffer and advance the state machine
// Call this periodically from the main loop.
void Flash_Programmer_ProcessRxBuffer(void);

// Function to be called periodically in the main loop to process programming states (erase/write)
// This function needs to check if prog_state is PROG_STATE_PROGRAMMING and do the work.
// This is separate from processing data from the ring buffer.
void Flash_Programmer_Process(void);

// Function to get the current programming state (optional)
FlashProgState_t Flash_Programmer_GetState(void);

// Callback function to be called from the UART's RxCpltCallback ISR.
void Flash_Programmer_RxCallback(uint8_t received_byte);

#endif /* INC_FLUSH_H_ */
