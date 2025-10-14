#include "ping.h"
#include <avr/io.h>
#include <util/delay.h>

// State machine states
typedef enum {
    IDLE,
    START_PULSE,
    WAIT_ECHO,
    MEASURE
} PingState;

// State machine variables
static volatile PingState state = IDLE;
static volatile uint32_t timestamp = 0;
static volatile uint16_t distance_cm = 0;

/**
 * @brief Initialize the PING sensor
 *
 * Configures PB1 as output (trigger pin) and necessary timer/interrupt setup.
 */
void ping_init(void) {
    // Configure PB1 as output (trigger pin)
    DDRB |= (1 << DDB1);
    
    // Initialize trigger pin low
    PORTB &= ~(1 << PORTB1);
    
    // Initialize state machine
    state = IDLE;
}

/**
 * @brief Trigger PING sensor measurement
 *
 * Sets PB1 high for 10µs to trigger the ultrasonic sensor.
 * Non-blocking state machine starter.
 * Note: Uses busy-loop delay for 10µs pulse (minimal blocking acceptable for precision).
 */
void ping_trigger(void) {
    // Set PB1 high
    PORTB |= (1 << PORTB1);
    
    // Save timestamp (placeholder - actual implementation depends on timer setup)
    timestamp = 0;  // TODO: Replace with actual timer counter value
    
    // Set state to START_PULSE
    state = START_PULSE;
    
    // 10µs delay using busy-loop
    // Note: For true non-blocking, this could be replaced with a timer interrupt
    // but 10µs is minimal and acceptable for pulse generation
    _delay_us(10);
    
    // Set PB1 low to complete the trigger pulse
    PORTB &= ~(1 << PORTB1);
    
    // Transition to next state
    state = WAIT_ECHO;
}

/**
 * @brief Start PING measurement
 *
 * Triggers the ultrasonic sensor to start a distance measurement.
 */
void ping_start(void) {
    if (state == IDLE) {
        ping_trigger();
    }
}

/**
 * @brief Read PING distance
 *
 * @return uint16_t Distance in centimeters
 */
uint16_t ping_read(void) {
    return distance_cm;
}

/**
 * @brief PING echo ISR hook
 *
 * Called on rising/falling edge of echo pin.
 */
void ping_isr_handler(void) {
    // TODO: Implement echo pin interrupt handling
    // - On rising edge: start timer/counter
    // - On falling edge: calculate distance based on pulse width
}
