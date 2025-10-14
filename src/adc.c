#include "adc.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Global variable to store the last ADC reading
static volatile uint16_t adc_value = 0;

// Callback function pointer for VolumeRegelaar API
static volatile adc_callback_t adc_callback = NULL;

/**
 * @brief Initialize the ADC peripheral in freerunning mode
 *
 * Configuration:
 * - ADMUX: ADLAR=1 (8-bit left-adjusted), REFS0=1 (AVCC reference)
 * - ADCSRA: ADEN=1 (enable), ADSC=1 (start), ADATE=1 (auto-trigger),
 *           ADIE=1 (interrupt enable), prescaler=128 (bits 2:0 = 111)
 * - ADCSRB: Auto-trigger source = free running mode (bits 2:0 = 000)
 */
void adc_init(void)
{
    // Configure ADMUX register
    // REFS1:0 = 01 -> AVCC with external capacitor at AREF pin
    // ADLAR = 1 -> Left adjust result (8-bit mode, read ADCH only)
    // MUX3:0 = 0000 -> Start with ADC0 (can be changed later)
    ADMUX = (1 << REFS0) | (1 << ADLAR);

    // Configure ADCSRA register
    // ADEN = 1 -> Enable ADC
    // ADSC = 1 -> Start conversion (first conversion after enable)
    // ADATE = 1 -> Auto trigger enable (freerunning mode)
    // ADIE = 1 -> ADC interrupt enable
    // ADPS2:0 = 111 -> Prescaler 128 (16MHz/128 = 125kHz ADC clock)
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |
             (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Configure ADCSRB register
    // ADTS2:0 = 000 -> Free running mode (auto-trigger source)
    ADCSRB = 0x00;
}

/**
 * @brief Start ADC conversion on specific channel
 *
 * @param channel ADC channel to read from (0-7)
 */
void adc_start(uint8_t channel)
{
    // Clear the channel bits and set new channel (MUX3:0)
    // Keep REFS and ADLAR bits unchanged
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    // In freerunning mode, conversion continues automatically
    // But we can restart with ADSC if needed
    ADCSRA |= (1 << ADSC);
}

/**
 * @brief Read the last ADC value
 *
 * @return uint16_t The last converted ADC value
 *         In 8-bit mode (ADLAR=1), returns ADCH (0-255)
 *         Cast to uint16_t for compatibility
 */
uint16_t adc_read(void)
{
    // In 8-bit mode, only ADCH is valid
    // Return the stored value from ISR
    return adc_value;
}

/**
 * @brief Register a callback function for ADC conversions
 *
 * @param callback Pointer to callback function
 */
void adc_set_callback(adc_callback_t callback)
{
    adc_callback = callback;
}

/**
 * @brief ADC conversion complete ISR hook
 *
 * This function is called from the ADC interrupt service routine
 * Stores the converted value for later retrieval
 */
void adc_isr_handler(void)
{
    // In 8-bit left-adjusted mode, read only ADCH
    uint8_t val = ADCH;
    
    // Store value in volatile buffer
    adc_value = val;
    
    // Call registered callback if available (VolumeRegelaar API)
    if (adc_callback != NULL)
    {
        adc_callback(val);
    }
}

/**
 * @brief ADC Conversion Complete Interrupt Service Routine
 *
 * This ISR is automatically called when ADC conversion completes.
 * Reads ADCH and calls the registered callback with minimal processing.
 */
ISR(ADC_vect)
{
    adc_isr_handler();
}
