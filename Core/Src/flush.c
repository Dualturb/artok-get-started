// --- flush.c (Updated Protocol) ---

/*
 * flush.c (Lean ISR + Ring Buffer + Updated Protocol Version)
 *
 * Created on: May 15, 2025
 * Author: halton
 *
 * This file implements the flash programmer state machine and helper functions.
 * It uses a ring buffer to receive data from the UART ISR and processes the
 * protocol state machine in the main loop.
 * Updated to handle Operation Type as a separate command after Start Command.
 */

#include "flush.h"
#include "w25qxx.h" // Include your W25Qxx driver header
#include <string.h> // For memcpy, memset
#include <stdio.h>  // For printf (optional, for debugging)
#include <stdbool.h>

// Include STM32 HAL UART header for polling transmit functions and flag macros
// We are keeping polling for transmit ACKs for simplicity.
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx.h" // Needed for direct register access if using CLEAR_BIT etc.


// --- Ring Buffer Definition ---
// The buffer and pointers are defined here and accessed by ISR (stm32f1xx_it.c) and main loop.
#ifndef RX_RING_BUFFER_SIZE
#define RX_RING_BUFFER_SIZE 256 // Choose a size (must be a power of 2)
#endif

uint8_t rx_ring_buffer[RX_RING_BUFFER_SIZE]; // The buffer array
volatile uint16_t rx_write_ptr = 0; // Index where the next incoming byte will be written by ISR
volatile uint16_t rx_read_ptr = 0;  // Index where the next byte to be processed will be read from by main loop


// Helper function to check if buffer is empty
bool is_rx_buffer_empty(void) {
    return rx_read_ptr == rx_write_ptr;
}

// Helper function to check if buffer is full (important for overflow handling)
bool is_rx_buffer_full(void) {
    // Buffer is considered full when the write pointer is one position behind the read pointer (wrapping around).
    // This leaves one empty slot to distinguish between full and empty.
    return ((rx_write_ptr + 1) & (RX_RING_BUFFER_SIZE - 1)) == rx_read_ptr;
}


// --- Private Variables (for State Machine) ---
static FlashProgState_t prog_state = PROG_STATE_IDLE;
static uint8_t current_operation_type = 0xFF; // Variable to store the received operation type
static uint32_t target_flash_address = 0;
static uint16_t data_size_to_receive = 0;
static uint16_t received_data_count = 0; // Total bytes received for the current operation
static bool erase_done_for_current_session = false; // Flag to ensure erase happens only once

// Buffer to store incoming data before writing to flash (used by state machine).
// Data is moved from the ring buffer to this temporary buffer before programming flash.
#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE 1024 // Example size: 4 W25Qxx pages. Adjust based on your RAM.
#endif
static uint8_t rx_buffer[RX_BUFFER_SIZE]={0};


// --- Externs ---
extern UART_HandleTypeDef huart1; // Your UART handle from main.c
extern uint8_t uart_rx_byte_isr;


// --- Private Function Prototypes ---
static void Send_Ack(uint8_t ack_byte); // We are keeping polling for sending ACKs
static void Enter_Error_State(void); // Error state handler (called from main loop processing)
static void Reset_Programmer(void); // Resets state machine variables and temporary buffer
static void Program_Rx_Buffer_To_Flash(void); // Handles flash write from rx_buffer (called from main loop processing)


// --- Public Functions ---

// Function to initialize the flash programmer module
void Flash_Programmer_Init(void) {
    prog_state = PROG_STATE_IDLE;
    Reset_Programmer(); // Reset state variables and temporary buffer

    // Initialize ring buffer pointers (ISR will start writing, main loop will start reading from 0)
    rx_read_ptr = 0;
    rx_write_ptr = 0;

    // Start the UART receive in interrupt mode
    // The HAL library will call HAL_UART_RxCpltCallback each time a byte is received
    // and place it in the uart_rx_byte_isr variable.
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte_isr, 1);
}

// --- ISR Callback Handler ---
// This function is the bridge between the HAL's interrupt-driven receive
// and our ring buffer implementation. It must be as fast as possible.
// This function should be placed in your stm32f1xx_it.c file.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart1.Instance) {
        Flash_Programmer_RxCallback(uart_rx_byte_isr);

        // Resume listening for the next byte
        HAL_UART_Receive_IT(huart, &uart_rx_byte_isr, 1);
    }
}


// --- Public Function for Ring Buffer Management (Called by ISR) ---
// This function adds a received byte to the ring buffer and is called from the UART ISR.
void Flash_Programmer_RxCallback(uint8_t received_byte) {
    if (!is_rx_buffer_full()) {
        rx_ring_buffer[rx_write_ptr] = received_byte;
        rx_write_ptr = (rx_write_ptr + 1) & (RX_RING_BUFFER_SIZE - 1);
    } else {
        // Buffer is full, discard the byte or handle overflow.
        // For this application, we simply discard. A more robust system might
        // send an error signal to the host.
        // For now, we do nothing to keep the ISR lean.
        // The main loop state machine will detect protocol errors when bytes are missed.
    }
}


// Function to process data from the ring buffer and advance the state machine
// This function should be called periodically from the main loop (e.g., in while(1)).
// It processes all available bytes from the ring buffer during each call.
void Flash_Programmer_ProcessRxBuffer(void) {
    uint8_t received_byte;

    // Process all available bytes from the ring buffer
    while (!is_rx_buffer_empty()) {
        // Read byte from the ring buffer
        received_byte = rx_ring_buffer[rx_read_ptr];

        // Increment the read pointer (with wrap-around using the buffer size mask)
        rx_read_ptr = (rx_read_ptr + 1) & (RX_RING_BUFFER_SIZE - 1);

        // --- State Machine (processes bytes read from the ring buffer) ---
        // This logic is similar to the previous Poll/Callback logic, but reads from the buffer.
        // PROG_STATE_PROGRAMMING and PROG_STATE_ERROR processing are handled separately below
        // or in the Flash_Programmer_Process function if kept separate.

        // Only process incoming bytes if not currently in a busy processing state (like PROGRAMMING)
        // or already in a terminal error state.
        if (prog_state != PROG_STATE_PROGRAMMING && prog_state != PROG_STATE_ERROR) {
            switch (prog_state) {
                case PROG_STATE_IDLE:
                    if (received_byte == PROG_CMD_START1) { // Received 0xAA
                        prog_state = PROG_STATE_WAIT_START2;
                         printf("DEBUG ProcessRx: State IDLE -> WAIT_START2 on 0xAA\r\n");
                    } else {
                        // Ignore unexpected bytes in IDLE
                    }
                    break;

                case PROG_STATE_WAIT_START2:
                    if (received_byte == PROG_CMD_START2) { // Received 0x55
                        // Start command received (AA 55)
                        prog_state = PROG_STATE_WAIT_OPERATION_CMD; // NEW STATE: Wait for Operation Type command
                        Reset_Programmer(); // Reset variables (also clears rx_buffer)
                        Send_Ack(PROG_ACK_START_ACKNOWLEDGED); // NEW ACK: Acknowledge Start Command
                         printf("DEBUG ProcessRx: State WAIT_START2 -> WAIT_OPERATION_CMD on 0x55, Sent ACK_START_ACKNOWLEDGED.\r\n");
                         printf("Programmer variables reset.\r\n"); // Print from Reset_Programmer
                    } else {
                        // Incorrect start sequence
                        Enter_Error_State(); // Enters error state and sends ACK_ERROR
                         printf("DEBUG ProcessRx: State -> ERROR (Bad Start2)\r\n");
                    }
                    break;

                case PROG_STATE_WAIT_OPERATION_CMD: // NEW STATE: Waiting for Operation Type Command (e.g., 0x00 for Program)
                    current_operation_type = received_byte; // The received byte is the operation type
                     printf("DEBUG ProcessRx: State WAIT_OPERATION_CMD, Received Op Type 0x%02X\r\n", current_operation_type);
                    switch (current_operation_type) {
                        case PROG_OP_PROGRAM: // Received 0x00
                            prog_state = PROG_STATE_WAIT_ADDR1; // Proceed to get address
                            Send_Ack(PROG_ACK_OK); // Send ACK_OK (0x01) to acknowledge Operation Type
                            // printf("DEBUG ProcessRx: State -> WAIT_ADDR1, Sent ACK_OK.\r\n"); // Print is in Send_Ack_Polling
                            break;
                        // Add cases for other operations here (e.g., case PROG_OP_ERASE_SECTOR:)
                        default:
                            // Unknown operation type
                            Enter_Error_State(); // Enters error state and sends ACK_ERROR (0xFF)
                             printf("DEBUG ProcessRx: State -> ERROR (Unknown Op Type 0x%02X)\r\r\n", current_operation_type);
                            break;
                    }
                    break; // End of PROG_STATE_WAIT_OPERATION_CMD case

                case PROG_STATE_WAIT_ADDR1: // Waiting for Address MSB (Byte 1 of 3)
                    target_flash_address = (uint32_t)received_byte << 16;
                    prog_state = PROG_STATE_WAIT_ADDR2;
                     printf("DEBUG ProcessRx: Received Addr1 (MSB).\r\n");
                    break;

                case PROG_STATE_WAIT_ADDR2: // Waiting for Address Mid byte (Byte 2 of 3)
                    target_flash_address |= (uint32_t)received_byte << 8;
                    prog_state = PROG_STATE_WAIT_ADDR3;
                     printf("DEBUG ProcessRx: Received Addr2.\r\n");
                    break;

                case PROG_STATE_WAIT_ADDR3: // Waiting for Address LSB (Byte 3 of 3)
                    target_flash_address |= (uint32_t)received_byte;
                    // Address received, validate and transition (for PROGRAM operation)
                     if (current_operation_type == PROG_OP_PROGRAM) {
                          if (target_flash_address >= W25Q_CAPACITY) { // Basic bounds check
                               Enter_Error_State();
                               printf("DEBUG ProcessRx: State -> ERROR (Address Out of Bounds 0x%lX)\r\n", target_flash_address); // Error state will also print
                          } else {
                             prog_state = PROG_STATE_WAIT_SIZE1; // Transition to waiting for Size MSB
                              printf("DEBUG ProcessRx: State -> WAIT_SIZE1, Full Address 0x%lX.\r\n", target_flash_address);
                          }
                     } else {
                          // Should not happen if state machine logic is correct
                         Enter_Error_State();
                         printf("DEBUG ProcessRx: State -> ERROR (Addr3 Unexpected Op 0x%02X)\r\n", current_operation_type); // Error state will also print
                     }
                    break;

                case PROG_STATE_WAIT_SIZE1: // Waiting for Size MSB (Byte 1 of 2)
                    data_size_to_receive = (uint16_t)received_byte << 8;
                    prog_state = PROG_STATE_WAIT_SIZE2;
                     printf("DEBUG ProcessRx: Received Size1 (MSB).\r\n");
                    break;

                case PROG_STATE_WAIT_SIZE2: // Waiting for Size LSB (Byte 2 of 2)
                    data_size_to_receive |= (uint16_t)received_byte;
                    // Validate size (e.g., not too large, not zero, fits within address bounds)
                    if (data_size_to_receive == 0 || data_size_to_receive > W25Q_CAPACITY || (target_flash_address + data_size_to_receive) > W25Q_CAPACITY) {
                         Enter_Error_State();
                         printf("DEBUG ProcessRx: State -> ERROR (Invalid Size %u or Bounds), Resetting\r\n", data_size_to_receive); // Error state will also print
                    } else {
                        prog_state = PROG_STATE_WAIT_DATA; // Ready for data bytes
                        received_data_count = 0; // Reset data counter for this new data block
                        // Note: erase_done_for_current_session is reset in Reset_Programmer (called at start and on AA 55)
                        Send_Ack(PROG_ACK_READY); // Send ACK_READY (0x02)
                         printf("DEBUG ProcessRx: State -> WAIT_DATA, Full Size %u, Sent ACK_READY.\r\n", data_size_to_receive);
                    }
                    break;

                case PROG_STATE_WAIT_DATA: // Waiting for Data Bytes
                    // Store the received byte in the temporary rx_buffer
                    if (received_data_count < data_size_to_receive) {
                         uint16_t buffer_index = received_data_count % RX_BUFFER_SIZE;
                         rx_buffer[buffer_index] = received_byte;
                         received_data_count++;

                         // If temporary buffer is full or last byte received, trigger programming in main loop
                         // Use '>' 0' check to handle cases where data_size_to_receive is exactly RX_BUFFER_SIZE * N
                         if ((received_data_count > 0 && received_data_count % RX_BUFFER_SIZE == 0) || (received_data_count == data_size_to_receive)) {
                             prog_state = PROG_STATE_PROGRAMMING; // Transition to main loop handling
                              printf("DEBUG ProcessRx: State -> PROGRAMMING (Buffer full or last chunk)\r\n");
                              // We have a buffer full or the last chunk. Exit this while loop to let
                              // Flash_Programmer_Process handle the programming. It will run on the next
                              // iteration of the main while loop.
                              return; // Exit ProcessRxBuffer function for now
                         }
                    } else {
                        // Received more data than expected - protocol error
                        Enter_Error_State();
                        printf("DEBUG ProcessRx: State -> ERROR (Received too much data), Resetting\r\n"); // Error state will also print
                    }
                    break; // Exit switch case, main loop will handle PROG_STATE_PROGRAMMING state via Flash_Programmer_Process

                case PROG_STATE_WAIT_END1: // Waiting for End Command Byte 1 (0x55)
                    if (received_byte == PROG_CMD_END1) {
                        prog_state = PROG_STATE_WAIT_END2;
                         printf("DEBUG ProcessRx: Received END1.\r\n");
                    } else {
                        Enter_Error_State();
                        printf("DEBUG ProcessRx: State -> ERROR (Bad End1), Resetting\r\n"); // Error state will also print
                    }
                    break;

                case PROG_STATE_WAIT_END2: // Waiting for End Command Byte 2 (0xAA)
                    if (received_byte == PROG_CMD_END2) {
                        // End command received.
                        // Check if the operation completed successfully in terms of data count (for Program)
                         if (current_operation_type == PROG_OP_PROGRAM && received_data_count != data_size_to_receive) {
                             // Size mismatch for Program operation - Protocol Error
                             Enter_Error_State();
                             printf("ERROR: Program Size Mismatch. Received %u, Expected %u\r\n", received_data_count, data_size_to_receive); // Keep this print
                         } else {
                             // Operation completed successfully from UART protocol perspective.
                             // The actual flash operation (Erase/Program) should have finished by now
                             // in the Flash_Programmer_Process function (if separate).
                             Send_Ack(PROG_ACK_DONE); // Send ACK_DONE (0x03)
                             printf("Flash Programming Complete (Address: 0x%lX, Size: %u)\r\n", target_flash_address, data_size_to_receive); // Keep this print
                         }
                         prog_state = PROG_STATE_IDLE; // Return to idle
                         Reset_Programmer(); // Reset variables for next session
                         printf("DEBUG ProcessRx: State -> IDLE (Done).\r\n");
                    } else {
                        Enter_Error_State();
                        printf("DEBUG ProcessRx: State -> ERROR (Bad End2), Resetting\r\n"); // Error state will also print
                    }
                    break;

                // PROG_STATE_ERROR is a terminal state for protocol processing
                case PROG_STATE_ERROR:
                     // Ignore any incoming bytes while in ERROR state, unless a specific error recovery is needed.
                    break;

                default:
                    // Should not happen - Unknown state
                    Enter_Error_State(); // Enter error state
                    printf("DEBUG ProcessRx: State -> ERROR (Unknown State %d)\r\n", prog_state);
                    break;
            } // End of switch (prog_state)
        } // End of if (!PROG_STATE_PROGRAMMING && !PROG_STATE_ERROR)
    } // End of while (!is_rx_buffer_empty()) - Process all available bytes from ring buffer
} // End of Flash_Programmer_ProcessRxBuffer


// Function to be called periodically in the main loop to process programming states
// This handles the actual flash programming (erase/write) which can take time.
// This function needs to check if prog_state is PROG_STATE_PROGRAMMING and do the work.
// It should NOT poll for UART data directly, but rather process the rx_buffer filled by ProcessRxBuffer.
void Flash_Programmer_Process(void) {
    switch (prog_state) {
        case PROG_STATE_PROGRAMMING: {
            // We have a buffer full or the last chunk of data to program in rx_buffer.
            // Determine how many bytes are in the current chunk buffer to program.
            uint16_t bytes_in_buffer = (received_data_count > 0 && received_data_count % RX_BUFFER_SIZE == 0)
                                       ? RX_BUFFER_SIZE
                                       : (uint16_t)(received_data_count % RX_BUFFER_SIZE);

             // Adjust for the case where received_data_count is an exact multiple of RX_BUFFER_SIZE and it's the last chunk
             if (received_data_count > 0 && received_data_count == data_size_to_receive && (data_size_to_receive % RX_BUFFER_SIZE == 0)) {
                 bytes_in_buffer = RX_BUFFER_SIZE;
             } else if (received_data_count > 0 && received_data_count == data_size_to_receive && (data_size_to_receive % RX_BUFFER_SIZE != 0)) {
                  // Last chunk, not a full buffer
                  bytes_in_buffer = data_size_to_receive % RX_BUFFER_SIZE;
             } else if (received_data_count > 0 && received_data_count % RX_BUFFER_SIZE == 0 && received_data_count < data_size_to_receive) {
                  // Full buffer chunk, but not the last one
                  bytes_in_buffer = RX_BUFFER_SIZE;
             } else if (received_data_count > 0 && received_data_count % RX_BUFFER_SIZE != 0 && received_data_count < data_size_to_receive) {
                  // Not a full buffer chunk, and not the last one (shouldn't happen with current logic, buffer should be full or last)
                  // This case indicates an issue with the state transition logic.
                  // Handle defensively, but it suggests a problem in ProcessRxBuffer logic.
                  printf("ERROR: PROG_STATE_PROGRAMMING reached in unexpected buffer state.\r\n");
                  Enter_Error_State(); // Use polling version
                  break; // Exit case
             }


            if (bytes_in_buffer > 0) {
                // Calculate the address for this chunk.
                // The address is the base address + total bytes received SO FAR - bytes in the CURRENT buffer chunk.
                uint32_t current_program_address = target_flash_address + (received_data_count - bytes_in_buffer);

                // --- Perform Flash Erase (Only once per programming session) ---
                if (!erase_done_for_current_session) {
                    // Calculate the range of sectors to erase for the *entire* program operation
                    uint32_t start_address = target_flash_address;
                    uint32_t end_address = target_flash_address + data_size_to_receive -1; // Last byte address

                    // Align start address down to the nearest sector boundary (4KB for W25Qxx)
                    uint32_t start_sector_address = start_address & ~(W25Q_SECTOR_SIZE - 1);

                    // Align end address up to the nearest sector boundary (4KB)
                    uint32_t end_sector_address = end_address & ~(W25Q_SECTOR_SIZE - 1);
                    if (data_size_to_receive > 0 && (end_address % W25Q_SECTOR_SIZE) != (W25Q_SECTOR_SIZE - 1) ) {
                         // If data was received and end address is not the last byte of its sector, include the next sector boundary
                         end_sector_address += W25Q_SECTOR_SIZE;
                    }
                    // Handle the case where size is 0 gracefully if needed, depends on protocol.
                    // If size is 0, start_address == end_address - 1, start_sector_address == end_sector_address.
                    // The loop should handle this (sector_addr < end_sector_address).
                    if (data_size_to_receive == 0) {
                         end_sector_address = start_sector_address; // If size is 0, no sectors to erase.
                    }


                    // Ensure end_sector_address does not exceed flash capacity
                    if (end_sector_address > W25Q_CAPACITY) {
                         end_sector_address = W25Q_CAPACITY; // Cap at max flash address
                    }


                    printf("Erasing flash from 0x%lX to 0x%lX (Sectors 0x%lX to 0x%lX)...\r\n",
                           start_address, end_address, start_sector_address, end_sector_address - (end_sector_address > start_sector_address ? W25Q_SECTOR_SIZE : 0)); // Print actual sector range start/end (end is inclusive of last byte, sector print is inclusive of last sector start)


                    // Iterate through and erase required sectors
                    for (uint32_t sector_addr = start_sector_address; sector_addr < end_sector_address; sector_addr += W25Q_SECTOR_SIZE) {
                         // Add a check to avoid erasing beyond capacity if end_sector_address was capped
                         if (sector_addr >= W25Q_CAPACITY) break;

                         printf("  Erasing sector 0x%lX...\r\n", sector_addr);
                         W25Q_EraseSector(sector_addr); // Call the driver's erase function (This might be blocking)
                         // Add error checking for W25Q_EraseSector if possible (depends on driver return)
                         printf("  Sector 0x%lX erased.\r\r\n", sector_addr);
                    }

                    erase_done_for_current_session = true; // Mark erase as complete for this session
                    printf("Flash erase complete for session.\r\n");
                }

                // --- Program the buffer chunk to flash ---
                // W25Q_WritePage programs up to W25Q_PAGE_SIZE (256) bytes at a time.
                // You might need to call W25Q_WritePage multiple times if bytes_in_buffer > W25Q_PAGE_SIZE.
                uint16_t bytes_left_in_chunk = bytes_in_buffer;
                uint16_t offset_in_buffer = 0;

                printf("Programming %u bytes from buffer to flash address 0x%lX...\r\n", bytes_in_buffer, current_program_address);

                while(bytes_left_in_chunk > 0) {
                    uint16_t bytes_to_write_this_page = (bytes_left_in_chunk > W25Q_PAGE_SIZE) ? W25Q_PAGE_SIZE : bytes_left_in_chunk;
                    uint32_t current_page_address = current_program_address + offset_in_buffer;

                    // Make sure W25Q_WritePage handles addresses and page boundaries correctly.
                    W25Q_WritePage(&rx_buffer[offset_in_buffer], current_page_address, bytes_to_write_this_page); // This might be blocking

                    bytes_left_in_chunk -= bytes_to_write_this_page;
                    offset_in_buffer += bytes_to_write_this_page;
                }
                printf("Chunk programmed.\r\n");


                // Programming of this buffer chunk is complete.
                // The state was transitioned to PROG_STATE_PROGRAMMING in ProcessRxBuffer.
                // Now, transition back to WAITING FOR DATA or END.
                if (received_data_count == data_size_to_receive) {
                    // All data received and programmed, wait for END command.
                    prog_state = PROG_STATE_WAIT_END1;
                     printf("State -> WAIT_END1 (All data programmed)\r\n");
                } else {
                    // More data is expected from the host for the next chunk.
                    prog_state = PROG_STATE_WAIT_DATA;
                     printf("State -> WAIT_DATA (Chunk programmed, more data expected)\r\n");
                }

                // Send ACK for the programmed chunk (optional, for host feedback)
                // Send_Ack(PROG_ACK_OK); // Acknowledge chunk programmed if needed by host
            } else {
                 // Should not happen if state logic in ProcessRxBuffer is correct
                 printf("ERROR: PROG_STATE_PROGRAMMING reached with no bytes to program in buffer.\r\n");
                 Enter_Error_State(); // Use polling version
            }
            break;
        } // End of PROG_STATE_PROGRAMMING case block

        case PROG_STATE_ERROR:
            // Error handling in main loop (e.g., flashing an LED, waiting for reset)
            break;

        default:
            // Other states don't require main loop processing in Flash_Programmer_Process
            break;
    }
}


// Function to get the current programming state (optional)
FlashProgState_t Flash_Programmer_GetState(void) {
    return prog_state;
}


// --- Private Helper Functions (Polling Transmit Versions) ---

// Function to send an acknowledge/status byte over UART (Polling version)
// This is called from the main loop processing functions.
static void Send_Ack(uint8_t ack_byte) {
    // Wait for the Transmit Data Register Empty flag - ready to accept data
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET) {
        // Optional: Add a timeout here to avoid infinite loop if TXE never sets
        // In a cooperative multitasking system (like simple main loops with LVGL),
        // you might yield here or break after a certain time to allow other tasks to run.
        // For sending ACKs, blocking is usually acceptable as they are short messages.
    }
    huart1.Instance->DR = ack_byte; // Put the byte in the Data Register for transmission

    // Optional: Wait for the Transmission Complete flag if needed for timing confirmation
    // This confirms the byte has finished being shifted out by the hardware.
    // while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
    //    // Optional: Add a timeout here
    // }
     printf("DEBUG SendAck: Sent ACK 0x%02X (Polling)\r\n", ack_byte); // Keep this print
}

// Function to enter the error state (Called from main loop processing)
static void Enter_Error_State(void) {
    if (prog_state != PROG_STATE_ERROR) {
        prog_state = PROG_STATE_ERROR;
        printf("Entering ERROR state.\r\n"); // Keep this print
        Send_Ack(PROG_ACK_ERROR); // Send error ACK (0xFF) using polling
        Reset_Programmer(); // Reset variables (also clears rx_buffer and ring buffer)
        printf("Programmer variables reset.\r\r\n"); // Keep this print
    }
}

// Function to reset programming variables and temporary buffer
static void Reset_Programmer(void) {
    current_operation_type = 0xFF;
    target_flash_address = 0;
    data_size_to_receive = 0;
    received_data_count = 0;
    erase_done_for_current_session = false;
    memset(rx_buffer, 0, RX_BUFFER_SIZE); // Clears the temporary data buffer

    // Also reset ring buffer pointers when the programmer is reset
    rx_read_ptr = 0;
    rx_write_ptr = 0;

    // printf("Programmer variables reset.\r\n"); // Make this print conditional if you want less output in main loop
}
