#include "buzzer.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// PWM pin definition
#define BUZZER_PIN PD3  // OC2B (Timer2 Channel B)

// Current frequency variable
static volatile uint16_t current_frequency = 0;

/**
 * @brief Initialize Timer2 for Fast PWM
 *
 * Configures Timer2 in Fast PWM mode with non-inverting output on OC2B (PD3).
 * - Fast PWM mode (WGM22:0 = 011)
 * - Non-inverting mode on OC2B (COM2B1:0 = 10)
 * - No prescaler (CS22:0 = 001) for maximum frequency resolution
 */
static void pwm_init(void)
{
    // Configure PD3 (OC2B) as output
    DDRD |= (1 << BUZZER_PIN);
    
    // Configure Timer2 Control Register A (TCCR2A)
    // COM2B1 = 1, COM2B0 = 0: Non-inverting mode on OC2B
    // WGM21 = 1, WGM20 = 1: Fast PWM mode (part 1)
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    
    // Configure Timer2 Control Register B (TCCR2B)
    // WGM22 = 0: Fast PWM mode (part 2) - mode 3 (TOP = 0xFF)
    // CS22:0 = 001: No prescaler (clk/1)
    TCCR2B = (1 << CS20);
    
    // Set initial duty cycle to 50% (OCR2B = 128)
    OCR2B = 128;
    
    // Optional: Enable Timer2 overflow interrupt if needed
    // TIMSK2 |= (1 << TOIE2);
}

/**
 * @brief Initialize the buzzer
 *
 * Configures the PWM timer and GPIO pin for buzzer control.
 * Sets up the necessary registers for tone generation.
 */
void buzzer_init(void)
{
    // Initialize PWM for buzzer control
    pwm_init();
    
    // Initialize frequency
    current_frequency = 0;
    
    // Start with buzzer off
    buzzer_stop();
}

/**
 * @brief Start buzzer with frequency
 *
 * @param frequency Frequency in Hz for the tone
 * Starts the buzzer playing at the specified frequency.
 */
void buzzer_start(uint16_t frequency)
{
    current_frequency = frequency;
    
    // Enable PWM output by setting non-inverting mode
    TCCR2A |= (1 << COM2B1);
    
    // Set 50% duty cycle
    OCR2B = 128;
    
    // TODO: Adjust timer frequency based on desired tone frequency
    // This basic implementation uses fixed PWM frequency
}

/**
 * @brief Update buzzer frequency
 *
 * @param frequency New frequency in Hz
 * Updates the buzzer to play at a new frequency without stopping.
 */
void buzzer_update(uint16_t frequency)
{
    current_frequency = frequency;
    
    // TODO: Implement frequency adjustment
    // This may require changing prescaler or using a different approach
    // For accurate frequency generation, consider using Timer1 with CTC mode
}

/**
 * @brief Stop the buzzer
 *
 * Stops the buzzer output (silence).
 */
void buzzer_stop(void)
{
    // Disconnect OC2B from the timer (normal port operation)
    TCCR2A &= ~(1 << COM2B1);
    
    // Set pin low
    PORTD &= ~(1 << BUZZER_PIN);
    
    current_frequency = 0;
}

/**
 * @brief Buzzer timer ISR hook
 *
 * Called from the timer overflow interrupt if needed.
 * Can be used for advanced buzzer control.
 */
void buzzer_isr_handler(void)
{
    // TODO: Implement if needed for advanced control
}

/**
 * @brief Timer2 Overflow ISR (optional)
 *
 * Can be used for frequency generation or other timing needs.
 */
// ISR(TIMER2_OVF_vect)
// {
//     buzzer_isr_handler();
// }
