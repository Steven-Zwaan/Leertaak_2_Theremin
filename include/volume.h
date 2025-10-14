#ifndef VOLUME_H
#define VOLUME_H

#include <stdint.h>

/**
 * @brief Initialize the volume module
 * 
 * Sets up initial volume state and configuration.
 */
void volume_init(void);

/**
 * @brief Set volume from ADC value
 * 
 * @param adc_value ADC reading (0-255 in 8-bit mode)
 * Maps the ADC value to internal volume representation (0-100%).
 * This function can be called from ISR context.
 */
void volume_set_from_adc(uint8_t adc_value);

/**
 * @brief Get current volume as percentage
 * 
 * @return uint8_t Volume percentage (0-100)
 * Returns the current volume level as a percentage.
 */
uint8_t volume_get_percent(void);

/**
 * @brief Get volume as PWM duty cycle value
 * 
 * @return uint8_t PWM duty cycle (0-255)
 * Returns the volume mapped to 8-bit PWM duty cycle for direct use with PWM hardware.
 */
uint8_t volume_get_duty(void);

#endif // VOLUME_H
