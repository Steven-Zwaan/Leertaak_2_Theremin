#include "buttons.h"
#include "filter.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Button pin definitions
#define BUTTON_0_PIN PD4 // PCINT20 - Decrease filter size
#define BUTTON_1_PIN PD5 // PCINT21 - Increase filter size

// Debounce time in milliseconds
#define DEBOUNCE_TIME_MS 50

// Button state variables
static volatile uint8_t button_state = 0;
static volatile uint8_t button_pressed = 0;
static volatile uint8_t button_released = 0;

// Previous button states for edge detection
static volatile uint8_t prev_button0_state = 0;
static volatile uint8_t prev_button1_state = 0;

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

/**
 * @brief Pin Change Interrupt ISR for PCINT2 (Port D)
 *
 * Detects pin changes on PD4 and PD5 with edge detection.
 * Uses simple state tracking to detect button press events.
 * - PD4 (Button 0): Decrease filter size
 * - PD5 (Button 1): Increase filter size
 */
ISR(PCINT2_vect)
{
    // Read current pin states (LOW = pressed due to pull-up)
    uint8_t button0_current = !(PIND & (1 << BUTTON_0_PIN));
    uint8_t button1_current = !(PIND & (1 << BUTTON_1_PIN));

    // Check Button 0 (PD4) - Decrease filter size
    // Detect falling edge (transition from released to pressed)
    if (button0_current && !prev_button0_state)
    {
        // Button press detected
        prev_button0_state = 1;
        button_pressed |= (1 << 0);

        // Decrease filter size (minimum 1)
        uint8_t current_size = filter_get_size();
        if (current_size > 1)
        {
            filter_set_size(current_size - 1);

            // Log button press and new filter size via UART
            extern void uart_puts(const char *str);
            extern void uart_put_uint(uint16_t value);
            extern void uart_newline(void);
            uart_puts("[BTN0] Filter size decreased to ");
            uart_put_uint(current_size - 1);
            uart_newline();
        }
    }
    else if (!button0_current && prev_button0_state)
    {
        // Button released
        prev_button0_state = 0;
        button_released |= (1 << 0);
    }

    // Check Button 1 (PD5) - Increase filter size
    // Detect falling edge (transition from released to pressed)
    if (button1_current && !prev_button1_state)
    {
        // Button press detected
        prev_button1_state = 1;
        button_pressed |= (1 << 1);

        // Increase filter size (maximum 9)
        uint8_t current_size = filter_get_size();
        if (current_size < 9)
        {
            filter_set_size(current_size + 1);

            // Log button press and new filter size via UART
            extern void uart_puts(const char *str);
            extern void uart_put_uint(uint16_t value);
            extern void uart_newline(void);
            uart_puts("[BTN1] Filter size increased to ");
            uart_put_uint(current_size + 1);
            uart_newline();
        }
    }
    else if (!button1_current && prev_button1_state)
    {
        // Button released
        prev_button1_state = 0;
        button_released |= (1 << 1);
    }
}

/**
 * @brief Test buttons and output to UART
 *
 * Reads raw button states and prints debug info via UART.
 * Useful for testing button wiring and functionality.
 */
void buttons_test(void)
{
    // Note: This requires uart.h to be included
    // Read raw button states (active LOW - 0 when pressed)
    uint8_t btn0 = (PIND & (1 << BUTTON_0_PIN)) ? 0 : 1;
    uint8_t btn1 = (PIND & (1 << BUTTON_1_PIN)) ? 0 : 1;

    if (btn0 || btn1)
    {
        // Only output if at least one button is pressed
        extern void uart_puts(const char *str);
        extern void uart_newline(void);

        uart_puts("BTN0:");
        uart_puts(btn0 ? "PRESSED " : "RELEASED ");
        uart_puts("BTN1:");
        uart_puts(btn1 ? "PRESSED" : "RELEASED");
        uart_newline();
    }
}
