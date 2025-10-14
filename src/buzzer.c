#include "buzzer.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// PWM pin definition
#define BUZZER_PIN PD3 // OC2B (Timer2 Channel B)

// Timer0 frequency calculation constants
#define F_CPU 16000000UL // 16MHz CPU clock
#define TIMER0_PRESCALER 256
#define TIMER0_FREQ_CONSTANT (F_CPU / (2UL * TIMER0_PRESCALER)) // 31250 Hz

// Frequency limits for Timer0 with prescaler 256
#define MIN_FREQUENCY 122   // OCR0A = 255 → ~122 Hz
#define MAX_FREQUENCY 15625 // OCR0A = 0 → ~31250 Hz (theoretical max)

// Current frequency variable
static volatile uint16_t current_frequency = 0;

// Toggle state for tone generation (gating PWM on/off)
static volatile uint8_t tone_output_enabled = 0;

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
 * @brief Initialize Timer0 for CTC mode (tone generation)
 *
 * Configures Timer0 in CTC (Clear Timer on Compare Match) mode for generating
 * tone frequencies via interrupt-based toggling.
 * - CTC mode (WGM01 = 1, WGM00 = 0)
 * - Prescaler 256 (CS02 = 1, CS01 = 0, CS00 = 0)
 * - OCR0A determines the half-period of the tone frequency
 * - Compare Match A interrupt enabled (OCIE0A)
 *
 * Timer frequency = F_CPU / prescaler / (2 * (OCR0A + 1))
 * With F_CPU = 16MHz, prescaler = 256:
 * Timer frequency = 16,000,000 / 256 / (2 * (OCR0A + 1))
 * Example: OCR0A = 121 → ~512 Hz
 */
static void tone_timer0_init(void)
{
    // Configure Timer0 Control Register A (TCCR0A)
    // WGM01 = 1, WGM00 = 0: CTC mode (Clear Timer on Compare Match)
    // COM0A and COM0B bits = 0: Normal port operation (no output compare pins)
    TCCR0A = (1 << WGM01);

    // Configure Timer0 Control Register B (TCCR0B)
    // WGM02 = 0: CTC mode (part 2)
    // CS02 = 1, CS01 = 0, CS00 = 0: Prescaler = 256 (clk/256)
    TCCR0B = (1 << CS02);

    // Set initial OCR0A value (determines half-period)
    // This will be updated based on desired frequency
    OCR0A = 121; // Example: ~512 Hz tone

    // Enable Timer0 Compare Match A interrupt (OCIE0A)
    TIMSK0 |= (1 << OCIE0A);
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

    // Initialize Timer0 for tone frequency generation
    tone_timer0_init();

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
 * @brief Set volume duty cycle
 *
 * Sets the PWM duty cycle for volume control by updating OCR2B.
 * OCR2B is an 8-bit register, so no atomic access is needed.
 *
 * @param duty Duty cycle value (0-255)
 *             0 = 0% duty (silent), 255 = 100% duty (max volume)
 */
void buzzer_set_volume_duty(uint8_t duty)
{
    // Update OCR2B register with new duty cycle
    // OCR2B is 8-bit, so atomic access is inherent (single instruction)
    OCR2B = duty;
}

/**
 * @brief Set buzzer frequency
 *
 * Calculates and sets the OCR0A value to generate the desired tone frequency.
 * Uses Timer0 in CTC mode with prescaler 256.
 *
 * Formula: OCR0A = (F_CPU / (2 × prescaler × frequency)) - 1
 * With F_CPU = 16MHz, prescaler = 256:
 * OCR0A = (31250 / frequency) - 1
 *
 * @param freq_hz Desired frequency in Hz (122 - 15625 Hz range)
 *                Frequencies outside this range will be clamped
 */
void buzzer_set_frequency(uint16_t freq_hz)
{
    uint8_t ocr_value;

    // Clamp frequency to valid range
    if (freq_hz < MIN_FREQUENCY)
    {
        freq_hz = MIN_FREQUENCY;
    }
    if (freq_hz > MAX_FREQUENCY)
    {
        freq_hz = MAX_FREQUENCY;
    }

    // Calculate OCR0A value from frequency
    // OCR0A = (TIMER0_FREQ_CONSTANT / freq_hz) - 1
    // Ensure result fits in 8-bit register (0-255)
    uint32_t temp = TIMER0_FREQ_CONSTANT / freq_hz;
    if (temp > 0)
    {
        temp = temp - 1;
    }

    // Clamp to 8-bit range
    if (temp > 255)
    {
        ocr_value = 255;
    }
    else
    {
        ocr_value = (uint8_t)temp;
    }

    // Update OCR0A atomically (8-bit register, single instruction write)
    OCR0A = ocr_value;

    // Store current frequency
    current_frequency = freq_hz;
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
 * @brief Timer0 Compare Match A ISR
 *
 * Toggles the PWM output enable/disable to create the tone frequency.
 * This ISR is called every half-period of the desired tone.
 * By gating the PWM on and off, we create an audible tone at the frequency
 * determined by OCR0A while maintaining volume control via PWM duty cycle.
 *
 * The toggle is implemented by manipulating the COM2B1 bit in TCCR2A:
 * - COM2B1 = 1: OC2B connected (PWM output enabled)
 * - COM2B1 = 0: OC2B disconnected (PWM output disabled)
 */
ISR(TIMER0_COMPA_vect)
{
    // Toggle the output enable state
    tone_output_enabled = !tone_output_enabled;

    if (tone_output_enabled)
    {
        // Enable OC2B output by setting COM2B1 bit
        // This connects the PWM to the output pin
        TCCR2A |= (1 << COM2B1);
    }
    else
    {
        // Disable OC2B output by clearing COM2B1 bit
        // This disconnects the PWM from the output pin
        TCCR2A &= ~(1 << COM2B1);

        // Optionally set pin low when disabled for cleaner output
        PORTD &= ~(1 << BUZZER_PIN);
    }
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
