/*
 * flasher.h
 *
 *  Created on: Sep 26, 2025
 *      Author: halton
 */

#ifndef INC_FLASHER_H_
#define INC_FLASHER_H_
#include <stdbool.h>
#include <stdint.h>

// --- Ring Buffer Definitions ---
#define RX_RING_BUFFER_SIZE 256 // Must be a power of 2

// External variables for the ring buffer (accessed by stm32f1xx_it.c ISR and flasher.c)
extern uint8_t g_rx_ring_buffer[RX_RING_BUFFER_SIZE];
extern volatile uint16_t g_rx_write_ptr;
extern volatile uint16_t g_rx_read_ptr;
extern uint8_t g_uart_rx_byte_isr; // The single byte container for the HAL interrupt

/**
 * @brief Initializes the artok-flasher library and registers all hardware callbacks.
 * This should be called once at system startup.
 */
void Flasher_Setup(void);

/**
 * @brief Runs the artok-flasher state machine.
 * This should be called periodically in the main application loop.
 */
void Flasher_Run(void);

/**
 * @brief Handles the completion of a UART RX transfer (called from the HAL callback).
 * This function processes the incoming byte and immediately prepares for the next.
 * @param huart The UART handle (e.g., &huart1)
 */
void Flasher_UartRxCpltCallback(void);


#endif /* INC_FLASHER_H_ */
