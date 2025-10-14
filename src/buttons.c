#include "buttons.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Button pin definitions
#define BUTTON_0_PIN PD4 // PCINT20
#define BUTTON_1_PIN PD5 // PCINT21

// Button state variables
static volatile uint8_t button_state = 0;
static volatile uint8_t button_pressed = 0;
static volatile uint8_t button_released = 0;

/**
 * @brief Initialize button inputs
 *
 * Configures PD4 and PD5 as inputs with internal pull-ups.
 * Sets up Pin Change Interrupt for PCINT20 (PD4) and PCINT21 (PD5).
 */
void buttons_init(void)
{
    // Configure PD4 and PD5 as inputs
    DDRD &= ~(1 << BUTTON_0_PIN); // PD4 as input
    DDRD &= ~(1 << BUTTON_1_PIN); // PD5 as input

    // Enable internal pull-ups on PD4 and PD5
    PORTD |= (1 << BUTTON_0_PIN); // Pull-up on PD4
    PORTD |= (1 << BUTTON_1_PIN); // Pull-up on PD5

    // Configure Pin Change Interrupt
    // Enable PCINT2 (for PCINT23:16, which includes PD0-PD7)
    PCICR |= (1 << PCIE2);

    // Enable PCINT20 (PD4) and PCINT21 (PD5) in mask register
    PCMSK2 |= (1 << PCINT20); // Enable PD4
    PCMSK2 |= (1 << PCINT21); // Enable PD5

    // Initialize button states
    button_state = 0;
    button_pressed = 0;
    button_released = 0;
}

/**
 * @brief Read button state
 *
 * @param button_id Button identifier (0 or 1)
 * @return bool true if button is pressed (pin LOW), false otherwise
 * Returns the current state of the specified button.
 */
bool buttons_read(uint8_t button_id)
{
    if (button_id == 0)
    {
        // Button is pressed when pin is LOW (active low with pull-up)
        return !(PIND & (1 << BUTTON_0_PIN));
    }
    else if (button_id == 1)
    {
        return !(PIND & (1 << BUTTON_1_PIN));
    }
    return false;
}

/**
 * @brief Update button states
 *
 * Polls all buttons and updates their debounced states.
 * Should be called periodically from main loop or timer.
 */
void buttons_update(void)
{
    // TODO: Implement debouncing logic
}

/**
 * @brief Check if button was just pressed
 *
 * @param button_id Button identifier (0 or 1)
 * @return bool true if button was pressed since last check
 * Detects rising edge (button press event).
 */
bool buttons_is_pressed(uint8_t button_id)
{
    if (button_id > 1)
        return false;

    uint8_t mask = (1 << button_id);
    if (button_pressed & mask)
    {
        button_pressed &= ~mask; // Clear the flag
        return true;
    }
    return false;
}

/**
 * @brief Check if button was just released
 *
 * @param button_id Button identifier (0 or 1)
 * @return bool true if button was released since last check
 * Detects falling edge (button release event).
 */
bool buttons_is_released(uint8_t button_id)
{
    if (button_id > 1)
        return false;

    uint8_t mask = (1 << button_id);
    if (button_released & mask)
    {
        button_released &= ~mask; // Clear the flag
        return true;
    }
    return false;
}

/**
 * @brief Button interrupt ISR hook
 *
 * Called when button state changes (if using interrupts).
 * This function should be called from the external interrupt or pin change ISR.
 */
void buttons_isr_handler(void)
{
    // TODO: Implement interrupt handler logic
    // Will be called from ISR(PCINT2_vect)
}
