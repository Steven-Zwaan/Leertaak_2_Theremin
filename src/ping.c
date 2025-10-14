#include "ping.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// State machine states
typedef enum
{
    IDLE,
    START_PULSE,
    WAIT_ECHO,
    MEASURE
} PingState;

// State machine variables
static volatile PingState state = IDLE;
static volatile uint32_t timestamp = 0;
static volatile uint16_t distance_cm = 0;

// Input-capture timing variables
static volatile uint16_t icr1_start = 0;        // Rising edge timestamp
static volatile uint16_t icr1_stop = 0;         // Falling edge timestamp
static volatile uint8_t ping_measure_ready = 0; // Measurement complete flag

/**
 * @brief Configure Timer1 for input-capture mode
 *
 * Configures Timer1 in normal mode with prescaler 8, input capture on rising edge.
 * Input capture pin: ICP1 (PB0 on ATmega328P)
 */
static void ping_timer1_init(void)
{
    // Set Timer1 to normal mode (WGM11:10 = 00)
    TCCR1A = 0;

    // Configure Timer1 Control Register B:
    // - CS11 = 1: Prescaler = 8 (16MHz / 8 = 2MHz, 0.5µs resolution)
    // - ICES1 = 1: Input Capture Edge Select = rising edge
    // - WGM13:12 = 00: Normal mode (combined with TCCR1A)
    TCCR1B = (1 << CS11) | (1 << ICES1);

    // Clear Timer1 counter
    TCNT1 = 0;

    // Enable Input Capture Interrupt (ICIE1)
    TIMSK1 = (1 << ICIE1);
}

/**
 * @brief Initialize the PING sensor
 *
 * Configures PB1 as output (trigger pin) and necessary timer/interrupt setup.
 */
void ping_init(void)
{
    // Configure PB1 as output (trigger pin)
    DDRB |= (1 << DDB1);

    // Initialize trigger pin low
    PORTB &= ~(1 << PORTB1);

    // Initialize Timer1 for input-capture
    ping_timer1_init();

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
void ping_trigger(void)
{
    // Set PB1 high
    PORTB |= (1 << PORTB1);

    // Save timestamp (placeholder - actual implementation depends on timer setup)
    timestamp = 0; // TODO: Replace with actual timer counter value

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
void ping_start(void)
{
    if (state == IDLE)
    {
        ping_trigger();
    }
}

/**
 * @brief Read PING distance
 *
 * @return uint16_t Distance in centimeters
 */
uint16_t ping_read(void)
{
    return distance_cm;
}

/**
 * @brief PING echo ISR hook
 *
 * Called on rising/falling edge of echo pin.
 */
void ping_isr_handler(void)
{
    // TODO: Implement echo pin interrupt handling
    // - On rising edge: start timer/counter
    // - On falling edge: calculate distance based on pulse width
}

/**
 * @brief Timer1 Input Capture ISR
 *
 * Handles both rising and falling edges of the echo pulse.
 * - Rising edge: Records start time (ICR1) and toggles to falling edge detection
 * - Falling edge: Records stop time (ICR1), toggles back to rising edge, sets measurement ready flag
 */
ISR(TIMER1_CAPT_vect)
{
    // Check current edge detection setting
    if (TCCR1B & (1 << ICES1))
    {
        // Currently detecting rising edge
        // Record the start timestamp from ICR1
        icr1_start = ICR1;

        // Toggle ICES1 to detect falling edge next
        TCCR1B &= ~(1 << ICES1);
    }
    else
    {
        // Currently detecting falling edge
        // Record the stop timestamp from ICR1
        icr1_stop = ICR1;

        // Toggle ICES1 back to detect rising edge
        TCCR1B |= (1 << ICES1);

        // Set measurement ready flag
        ping_measure_ready = 1;
    }
}
