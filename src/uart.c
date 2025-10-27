#include "uart.h"
#include <avr/io.h>

#define F_CPU 16000000UL

/**
 * @brief Initialize UART
 *
 * @param baudrate Baud rate (e.g., 9600, 115200)
 * Configures UART for serial communication at specified baud rate.
 */
void uart_init(uint32_t baudrate)
{
    // Calculate UBRR value
    uint16_t ubrr = (F_CPU / (16UL * baudrate)) - 1;

    // Set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;

    // Enable transmitter
    UCSR0B = (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/**
 * @brief Send a single character via UART
 *
 * @param data Character to send
 */
void uart_putc(char data)
{
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)))
        ;

    // Put data into buffer, sends the data
    UDR0 = data;
}

/**
 * @brief Send a string via UART
 *
 * @param str Null-terminated string to send
 */
void uart_puts(const char *str)
{
    while (*str)
    {
        uart_putc(*str);
        str++;
    }
}

/**
 * @brief Send an unsigned integer via UART
 *
 * @param value Integer value to send (as decimal string)
 */
void uart_put_uint(uint16_t value)
{
    char buffer[6]; // Max 5 digits + null terminator
    uint8_t i = 0;

    // Handle zero case
    if (value == 0)
    {
        uart_putc('0');
        return;
    }

    // Convert to string (reversed)
    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }

    // Print in correct order
    while (i > 0)
    {
        uart_putc(buffer[--i]);
    }
}

/**
 * @brief Send newline (CRLF) via UART
 */
void uart_newline(void)
{
    uart_putc('\r');
    uart_putc('\n');
}
