#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

/**
 * @brief Initialize the display
 *
 * Configures the display hardware (LCD, LED, 7-segment, etc.).
 * Sets up communication pins and initializes display controller.
 */
void display_init(void);

/**
 * @brief Start display operation
 *
 * Turns on the display and prepares it for output.
 */
void display_start(void);

/**
 * @brief Update display with new data
 *
 * @param distance Distance in centimeters
 * @param frequency Frequency in Hz
 * Updates the display with distance and frequency information.
 * Line 0: "Dist: xx cm"
 * Line 1: "Freq: yyyy Hz"
 */
void display_update(uint16_t distance, uint16_t frequency);

/**
 * @brief Clear the display
 *
 * Clears all content from the display.
 */
void display_clear(void);

/**
 * @brief Display refresh ISR hook
 *
 * Called periodically to refresh multiplexed displays.
 * This function should be called from a timer ISR for displays requiring refresh.
 */
void display_isr_handler(void);

/**
 * @brief Display a digit on 7-segment display
 *
 * @param value Digit to display (0-9)
 * Sends the segment pattern for the specified digit to the I2C port expander.
 * Values > 9 will display 0.
 */
void sevenseg_display(uint8_t value);

#endif // DISPLAY_H
