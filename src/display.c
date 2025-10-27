#include "display.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

// I2C/TWI pin definitions
#define I2C_SDA_PIN PC4 // SDA (Data line)
#define I2C_SCL_PIN PC5 // SCL (Clock line)

// I2C clock frequency
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define I2C_FREQ 100000UL // 100 kHz

// LCD I2C address and constants
#define LCD_I2C_ADDRESS 0x27

// LCD commands
#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME 0x02
#define LCD_CMD_ENTRY_MODE 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_DDRAM_ADDR 0x80

// LCD control bits
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_READ_WRITE 0x02
#define LCD_REGISTER_SELECT 0x01

// LCD display control flags
#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_OFF 0x00

// LCD function set flags
#define LCD_4BIT_MODE 0x00
#define LCD_2_LINE 0x08
#define LCD_5x8_DOTS 0x00

// LCD entry mode flags
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// 7-segment display I2C address (PCF8574/PCF8575 or similar port expander)
#define SEVENSEG_I2C_ADDRESS 0x20 // Default address for PCF8574

// 7-segment digit patterns for common cathode (0-9)
// Bit mapping: DP G F E D C B A
static const uint8_t sevenseg_digits[10] = {
    0x3F, // 0: segments A B C D E F
    0x06, // 1: segments B C
    0x5B, // 2: segments A B D E G
    0x4F, // 3: segments A B C D G
    0x66, // 4: segments B C F G
    0x6D, // 5: segments A C D F G
    0x7D, // 6: segments A C D E F G
    0x07, // 7: segments A B C
    0x7F, // 8: segments A B C D E F G
    0x6F  // 9: segments A B C D F G
};

// Forward declarations
static void i2c_init(void);
static void lcd_init(void);
static void sevenseg_init(void);

/**
 * @brief Initialize I2C/TWI hardware
 *
 * Configures the TWI (Two-Wire Interface) for I2C communication at 100 kHz.
 * Enables internal pull-ups on SDA (PC4) and SCL (PC5) pins.
 *
 * TWI bit rate formula:
 * SCL_freq = F_CPU / (16 + 2 × TWBR × prescaler)
 *
 * With prescaler = 1 (TWPS1:0 = 00):
 * TWBR = ((F_CPU / I2C_FREQ) - 16) / 2
 * TWBR = ((16000000 / 100000) - 16) / 2 = (160 - 16) / 2 = 72
 */
static void i2c_init(void)
{
    // Enable internal pull-ups on SDA (PC4) and SCL (PC5)
    // Note: DDR should be input (0) for I2C pins
    DDRC &= ~(1 << I2C_SDA_PIN); // PC4 (SDA) as input
    DDRC &= ~(1 << I2C_SCL_PIN); // PC5 (SCL) as input

    PORTC |= (1 << I2C_SDA_PIN); // Enable pull-up on PC4 (SDA)
    PORTC |= (1 << I2C_SCL_PIN); // Enable pull-up on PC5 (SCL)

    // Set TWI bit rate register for 100 kHz
    // TWBR = ((F_CPU / I2C_FREQ) - 16) / 2
    TWBR = 72;

    // Set TWI prescaler to 1 (TWPS1:0 = 00)
    TWSR = 0x00; // Clear prescaler bits and status bits

    // Enable TWI module
    TWCR = (1 << TWEN);
}

/**
 * @brief Blocking I2C transmit helper
 *
 * Sends data to I2C device with START, address, data, and STOP conditions.
 *
 * @param address I2C device address (7-bit)
 * @param data Byte to transmit
 * @return uint8_t 0 on success, non-zero on error
 */
static uint8_t i2c_transmit(uint8_t address, uint8_t data)
{
    // Send START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ; // Wait for TWINT flag

    // Send device address with write bit (0)
    TWDR = (address << 1);
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ; // Wait for TWINT flag

    // Send data byte
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ; // Wait for TWINT flag

    // Send STOP condition
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

    return 0; // Success
}

/**
 * @brief Send nibble (4 bits) to LCD via I2C
 *
 * @param nibble 4-bit data to send (in upper nibble)
 * @param mode Control bits (RS, RW, Backlight)
 */
static void lcd_send_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = nibble | mode | LCD_BACKLIGHT;

    // Send with Enable high
    i2c_transmit(LCD_I2C_ADDRESS, data | LCD_ENABLE);
    _delay_us(1);

    // Send with Enable low (latch data)
    i2c_transmit(LCD_I2C_ADDRESS, data & ~LCD_ENABLE);
    _delay_us(50);
}

/**
 * @brief Send byte to LCD in 4-bit mode
 *
 * @param byte Data byte to send
 * @param mode Control bits (RS for command/data selection)
 */
static void lcd_send_byte(uint8_t byte, uint8_t mode)
{
    // Send upper nibble
    lcd_send_nibble(byte & 0xF0, mode);

    // Send lower nibble
    lcd_send_nibble((byte << 4) & 0xF0, mode);
}

/**
 * @brief Send command to LCD
 *
 * @param cmd Command byte
 */
static void lcd_send_command(uint8_t cmd)
{
    lcd_send_byte(cmd, 0); // RS = 0 for command
    _delay_ms(2);
}

/**
 * @brief Send data (character) to LCD
 *
 * @param data Character to display
 */
static void lcd_send_data(uint8_t data)
{
    lcd_send_byte(data, LCD_REGISTER_SELECT); // RS = 1 for data
}

/**
 * @brief Initialize LCD in 4-bit mode
 *
 * Initializes a 2x16 I2C LCD at address 0x27.
 * Uses standard HD44780 initialization sequence for 4-bit mode.
 */
static void lcd_init(void)
{
    // Wait for LCD to power up
    _delay_ms(50);

    // Initialize in 4-bit mode (special sequence)
    lcd_send_nibble(0x30, 0); // Function set: 8-bit mode (initial)
    _delay_ms(5);

    lcd_send_nibble(0x30, 0); // Function set: 8-bit mode (repeat)
    _delay_ms(1);

    lcd_send_nibble(0x30, 0); // Function set: 8-bit mode (repeat)
    _delay_ms(1);

    lcd_send_nibble(0x20, 0); // Function set: 4-bit mode
    _delay_ms(1);

    // Now in 4-bit mode, send full commands
    lcd_send_command(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8_DOTS);
    lcd_send_command(LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    lcd_send_command(LCD_CMD_CLEAR);
    _delay_ms(2);
    lcd_send_command(LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
}

/**
 * @brief Write text to specific line on LCD
 *
 * @param row Line number (0 or 1 for 2x16 LCD)
 * @param text Null-terminated string to display (max 16 characters)
 */
static void lcd_write_line(uint8_t row, const char *text)
{
    // Set cursor position (DDRAM address)
    // Line 0: 0x00-0x0F, Line 1: 0x40-0x4F
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    lcd_send_command(LCD_CMD_SET_DDRAM_ADDR | address);

    // Write characters (max 16)
    uint8_t count = 0;
    while (*text && count < 16)
    {
        lcd_send_data(*text);
        text++;
        count++;
    }

    // Fill remaining positions with spaces
    while (count < 16)
    {
        lcd_send_data(' ');
        count++;
    }
}

/**
 * @brief Initialize the display
 *
 * Configures the display hardware (LCD, LED, 7-segment, etc.).
 * Sets up communication pins and initializes display controller.
 */
void display_init(void)
{
    // Initialize I2C/TWI hardware
    i2c_init();

    // Initialize LCD
    lcd_init();

    // Initialize 7-segment display
    sevenseg_init();
}

/**
 * @brief Start display operation
 *
 * Turns on the display and prepares it for output.
 */
void display_start(void)
{
    // TODO: Implement display start sequence
}

/**
 * @brief Update display with new data
 *
 * @param distance Distance in centimeters
 * @param frequency Frequency in Hz
 * Updates the display with distance and frequency information.
 * Line 0: "Dist: xx cm"
 * Line 1: "Freq: yyyy Hz"
 */
void display_update(uint16_t distance, uint16_t frequency)
{
    char buffer[17];

    // Format and display distance on line 0
    snprintf(buffer, sizeof(buffer), "Dist: %u cm", distance);
    lcd_write_line(0, buffer);

    // Format and display frequency on line 1
    snprintf(buffer, sizeof(buffer), "Freq: %u Hz", frequency);
    lcd_write_line(1, buffer);
}

/**
 * @brief Clear the display
 *
 * Clears all content from the display.
 */
void display_clear(void)
{
    lcd_send_command(LCD_CMD_CLEAR);
    _delay_ms(2);
}

/**
 * @brief Display refresh ISR hook
 *
 * Called periodically to refresh multiplexed displays.
 * This function should be called from a timer ISR for displays requiring refresh.
 */
void display_isr_handler(void)
{
    // TODO: Implement if needed for multiplexed displays
}

/**
 * @brief Initialize 7-segment display port expander
 *
 * Sets up the I2C port expander for 7-segment display.
 * Clears the display initially.
 */
static void sevenseg_init(void)
{
    // Clear display (all segments off)
    i2c_transmit(SEVENSEG_I2C_ADDRESS, 0x00);
}

/**
 * @brief Display a digit on 7-segment display
 *
 * Sends the segment pattern for the specified digit to the I2C port expander.
 * Only displays single digits 0-9. Values >= 10 will display 0.
 *
 * @param value Digit to display (0-9)
 */
void sevenseg_display(uint8_t value)
{
    // Clamp value to 0-9 range
    if (value > 9)
    {
        value = 0;
    }

    // Send segment pattern to port expander
    i2c_transmit(SEVENSEG_I2C_ADDRESS, sevenseg_digits[value]);
}

/**
 * @brief Display filter size on 7-segment display
 *
 * Shows the current filter size on the 7-segment display.
 * For values > 9, displays the last digit (ones place).
 * For example: 15 displays as "5", 10 displays as "0".
 *
 * @param size Filter size to display (0-15 typical range)
 */
void display_filter_size(uint8_t size)
{
    // Extract last digit (ones place) using modulo 10
    uint8_t digit = size % 10;

    // Display the digit on 7-segment
    sevenseg_display(digit);
}
