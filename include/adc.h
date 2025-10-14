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

/**
 * @brief Callback function type for ADC conversions
 *
 * @param value The ADC conversion result (0-255 in 8-bit mode)
 * This callback is invoked from the ADC ISR with the converted value.
 */
typedef void (*adc_callback_t)(uint8_t value);

/**
 * @brief Register a callback function for ADC conversions
 *
 * @param callback Pointer to callback function to be called when ADC conversion completes
 * The callback will be invoked from the ISR context, so it should be kept minimal.
 */
void adc_set_callback(adc_callback_t callback);

#endif // ADC_H
