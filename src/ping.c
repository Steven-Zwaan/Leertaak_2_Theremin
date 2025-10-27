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
static volatile uint8_t ping_timeout = 0;       // Timeout flag

// Timeout constants
#define PING_TIMEOUT_MS 33   // 33ms timeout for echo
#define PING_MAX_DISTANCE 66 // Maximum valid distance (cm)

// Frequency output variable
static volatile uint16_t frequency_hz = 0; // Last computed frequency in Hz

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

    // Clear flags
    ping_measure_ready = 0;
    ping_timeout = 0;

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
 * @brief Compute distance from timer ticks
 *
 * Converts timer ticks to distance in centimeters.
 * With prescaler 8 at 16MHz: 1 tick = 0.5µs
 * Sound speed: ~340 m/s → 29µs per cm (round trip: 58µs per cm)
 *
 * @param ticks Number of timer ticks (pulse width)
 * @return uint16_t Distance in centimeters
 */
static uint16_t ping_compute_distance(uint16_t ticks)
{
    // Convert ticks to microseconds
    // Prescaler 8 at 16MHz: 1 tick = 0.5µs
    // Therefore: microseconds = ticks * 0.5 = ticks / 2
    uint16_t microseconds = ticks / 2;

    // Convert microseconds to centimeters
    // Sound travels 58µs per cm (round trip)
    uint16_t distance_cm = microseconds / 58;

    return distance_cm;
}

/**
 * @brief Map distance to frequency for theremin
 *
 * Maps measured distance to a frequency value using linear interpolation.
 * Formula: f = fmax - (fmax - fmin) * (distance / distMax)
 * Closer distance → higher frequency, farther distance → lower frequency
 *
 * @param dist_cm Distance in centimeters
 * @return uint16_t Frequency in Hz (clamped between fmin and fmax)
 */
static uint16_t map_distance_to_freq(uint16_t dist_cm)
{
    // Define mapping parameters
    const uint16_t fmin = 230;   // Minimum frequency (Hz) - far distance
    const uint16_t fmax = 1400;  // Maximum frequency (Hz) - close distance
    const uint16_t distMax = 65; // Maximum distance for mapping (cm)

    // Clamp distance to maximum
    if (dist_cm > distMax)
    {
        dist_cm = distMax;
    }

    // Calculate frequency using linear mapping
    // f = fmax - (fmax - fmin) * (distance / distMax)
    // Rearranged to avoid floating point: f = fmax - ((fmax - fmin) * distance) / distMax
    uint16_t frequency = fmax - (((uint32_t)(fmax - fmin) * dist_cm) / distMax);

    // Clamp frequency to valid range
    if (frequency < fmin)
    {
        frequency = fmin;
    }
    if (frequency > fmax)
    {
        frequency = fmax;
    }

    return frequency;
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
 * @brief Check if ping timeout occurred
 *
 * @return uint8_t 1 if timeout, 0 if normal
 * Returns whether the last ping measurement timed out.
 */
uint8_t ping_is_timeout(void)
{
    return ping_timeout;
}

/**
 * @brief Handle ping timeout
 *
 * Should be called from main loop if no echo received within timeout period.
 * Sets distance to invalid value and sets timeout flag.
 */
void ping_handle_timeout(void)
{
    ping_timeout = 1;
    distance_cm = PING_MAX_DISTANCE + 1; // Set to invalid distance
    state = IDLE;                        // Reset state machine
}

/**
 * @brief Get the last computed frequency
 *
 * Returns the frequency calculated from the most recent distance measurement.
 * The frequency is mapped from distance using the theremin mapping function.
 *
 * @return uint16_t Frequency in Hz (230-1400 Hz range)
 */
uint16_t ping_get_frequency(void)
{
    return frequency_hz;
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
