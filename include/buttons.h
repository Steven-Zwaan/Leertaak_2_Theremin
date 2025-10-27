#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize button inputs
 *
 * Configures GPIO pins for button reading.
 * Sets up pull-up resistors and interrupt handling if needed.
 */
void buttons_init(void);

/**
 * @brief Read button state
 *
 * @param button_id Button identifier (0-based index)
 * @return bool true if button is pressed, false otherwise
 * Returns the current state of the specified button.
 */
bool buttons_read(uint8_t button_id);

/**
 * @brief Update button states
 *
 * Polls all buttons and updates their debounced states.
 * Should be called periodically from main loop or timer.
 */
void buttons_update(void);

/**
 * @brief Check if button was just pressed
 *
 * @param button_id Button identifier (0-based index)
 * @return bool true if button was pressed since last check
 * Detects rising edge (button press event).
 */
bool buttons_is_pressed(uint8_t button_id);

/**
 * @brief Check if button was just released
 *
 * @param button_id Button identifier (0-based index)
 * @return bool true if button was released since last check
 * Detects falling edge (button release event).
 */
bool buttons_is_released(uint8_t button_id);

/**
 * @brief Button interrupt ISR hook
 *
 * Called when button state changes (if using interrupts).
 * This function should be called from the external interrupt or pin change ISR.
 */
void buttons_isr_handler(void);

/**
 * @brief Test buttons and output to UART
 *
 * Reads raw button states and prints debug info via UART.
 * Useful for testing button wiring and functionality.
 */
void buttons_test(void);

#endif // BUTTONS_H
