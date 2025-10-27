#ifndef UART_H
#define UART_H

#include <stdint.h>

/**
 * @brief Initialize UART
 *
 * @param baudrate Baud rate (e.g., 9600, 115200)
 * Configures UART for serial communication at specified baud rate.
 */
void uart_init(uint32_t baudrate);

/**
 * @brief Send a single character via UART
 *
 * @param data Character to send
 */
void uart_putc(char data);

/**
 * @brief Send a string via UART
 *
 * @param str Null-terminated string to send
 */
void uart_puts(const char *str);

/**
 * @brief Send an unsigned integer via UART
 *
 * @param value Integer value to send (as decimal string)
 */
void uart_put_uint(uint16_t value);

/**
 * @brief Send a hex value via UART
 *
 * @param value 8-bit value to send as hex (e.g., 0x2A)
 * Outputs 2-digit hexadecimal representation.
 */
void uart_put_hex(uint8_t value);

/**
 * @brief Send newline (CRLF) via UART
 */
void uart_newline(void);

#endif // UART_H
