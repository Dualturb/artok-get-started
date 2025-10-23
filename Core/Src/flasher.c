/*
 * flasher.c
 *
 * This file implements the flasher abstraction layer, providing the necessary
 * hardware interface to the `artok-flasher` library.
 */


#include "artok_flasher.h"
#include "flasher.h"
#include "main.h" // We need access to the HAL handles and GPIO defines

// Extern declarations from main.c for the peripheral handlers
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1; // Assuming a UART handler for debug/response
// External reference to the built-in W25Qxx driver defined in the library (.a file)
extern const Atk_FlasherDriver_t w25qxx_driver;

// --- Ring Buffer Implementation (Global/Externalized for IT handler) ---
// Note: These variables are defined here and extern'd in flasher.h
uint8_t g_rx_ring_buffer[RX_RING_BUFFER_SIZE];
volatile uint16_t g_rx_write_ptr = 0; // Index where the next incoming byte will be written by ISR
volatile uint16_t g_rx_read_ptr = 0;  // Index where the next byte to be processed will be read from by main loop
uint8_t g_uart_rx_byte_isr; // The single byte container for the HAL interrupt

// Helper function to check if buffer is empty
static bool is_rx_buffer_empty(void) {
    return g_rx_read_ptr == g_rx_write_ptr;
}

// Helper function to check if buffer is full
static bool is_rx_buffer_full(void) {
    // Leaves one slot empty to distinguish between full and empty
    return ((g_rx_write_ptr + 1) & (RX_RING_BUFFER_SIZE - 1)) == g_rx_read_ptr;
}

// --- Private Hardware Interface Functions ---
// These functions bridge the STM32 HAL and the library's required API.
// They are passed into the `flasher_init` function.

static void preinit(void) {
    // Optional: Add any hardware-specific pre-initialization here if needed.
    // For this example, we don't need to do anything.
}

static void send_response_byte(uint8_t byte) {
    // This function sends a single byte back to the host, typically over UART.
    HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
}

static void device_select(void) {
    // Assert the SPI Chip Select pin to begin a transaction.
    HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);
}

static void device_deselect(void) {
    // Deassert the SPI Chip Select pin to end a transaction.
    HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
}

static void transmit_data(const uint8_t* tx_data, uint16_t size) {
    HAL_SPI_Transmit(&hspi1, (uint8_t*)tx_data, size, HAL_MAX_DELAY);
}

static void receive_data(uint8_t* rx_data, uint16_t size) {
    HAL_SPI_Receive(&hspi1, rx_data, size, HAL_MAX_DELAY);
}

static void transmit_receive(const uint8_t* tx_data, uint8_t* rx_data, uint16_t size) {
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data, rx_data, size, HAL_MAX_DELAY);
}

static void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

const Atk_FlasherInterface_t hardware_interface = {
    .preinit = preinit,
    .send_response_byte = send_response_byte,
    .device_select = device_select,
    .device_deselect = device_deselect,
    .transmit_data = transmit_data,
    .receive_data = receive_data,
    .transmit_receive = transmit_receive,
    .delay_ms = delay_ms
};

// --- Public API Wrapper Functions ---

void Flasher_Setup(void) {
    // 1. Initialize the artok-flasher library
    flasher_init(&hardware_interface, &w25qxx_driver);

    // 2. Initialize ring buffer pointers
    g_rx_read_ptr = 0;
    g_rx_write_ptr = 0;

    // 3. Start the initial non-blocking UART receive interrupt
    HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte_isr, 1);
}

void Flasher_Run(void) {
    uint8_t received_byte;

    // 1. Process all available bytes from the ring buffer
    // This is safe because it runs in the main loop context.
    while (!is_rx_buffer_empty()) {
        received_byte = g_rx_ring_buffer[g_rx_read_ptr];
        g_rx_read_ptr = (g_rx_read_ptr + 1) & (RX_RING_BUFFER_SIZE - 1);

        // Feed the byte into the library's state machine.
        flasher_process_byte(received_byte);
    }

    // 2. Execute the library's main loop processing (handles flash I/O like erasing/writing chunks)
    flasher_process();
}

void Flasher_UartRxCpltCallback(void) {
    // This function must be extremely fast as it runs in Interrupt Context!

    // 1. Push the received byte into the ring buffer
    if (!is_rx_buffer_full()) {
        g_rx_ring_buffer[g_rx_write_ptr] = g_uart_rx_byte_isr;
        g_rx_write_ptr = (g_rx_write_ptr + 1) & (RX_RING_BUFFER_SIZE - 1);
    } else {
        // Buffer is full, discard byte.
    }

    // 2. CRITICAL: Re-enable the UART interrupt to receive the next byte.
    HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte_isr, 1);
}
