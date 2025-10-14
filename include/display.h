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
 * @param value Value to display (interpretation depends on display type)
 * Updates the display with the provided value.
 */
void display_update(uint16_t value);

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

#endif // DISPLAY_H
