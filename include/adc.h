#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/**
 * @brief Initialize the ADC peripheral
 *
 * Configures the ADC for reading analog values.
 * Sets up the necessary registers and prescaler.
 */
void adc_init(void);

/**
 * @brief Start ADC conversion
 *
 * @param channel ADC channel to read from (0-7)
 * Initiates an ADC conversion on the specified channel.
 */
void adc_start(uint8_t channel);

/**
 * @brief Read ADC value
 *
 * @return uint16_t The converted ADC value (0-1023 for 10-bit ADC)
 * Returns the last converted ADC value.
 */
uint16_t adc_read(void);

/**
 * @brief ADC conversion complete ISR hook
 *
 * Called when ADC conversion is complete.
 * This function should be called from the ADC interrupt service routine.
 */
void adc_isr_handler(void);

#endif // ADC_H
