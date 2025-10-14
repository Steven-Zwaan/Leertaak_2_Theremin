#include "volume.h"
#include <stdint.h>

// Volatile storage for volume values (accessed from ISR and main loop)
static volatile uint8_t volume_adc_raw = 0;      // Raw ADC value (0-255)
static volatile uint8_t volume_percent = 0;      // Volume as percentage (0-100)
static volatile uint8_t volume_duty = 0;         // Volume as PWM duty cycle (0-255)

/**
 * @brief Initialize the volume module
 */
void volume_init(void)
{
    volume_adc_raw = 0;
    volume_percent = 0;
    volume_duty = 0;
}

/**
 * @brief Set volume from ADC value
 * 
 * @param adc_value ADC reading (0-255 in 8-bit mode)
 * 
 * Maps ADC input (0-255) to:
 * - Volume percentage (0-100%)
 * - PWM duty cycle (0-255)
 * 
 * Mapping: 
 * - 0-255 ADC -> 0-100% volume
 * - 0-255 ADC -> 0-255 PWM duty (direct mapping)
 */
void volume_set_from_adc(uint8_t adc_value)
{
    // Store raw ADC value
    volume_adc_raw = adc_value;
    
    // Map 0-255 to 0-100 percent
    // Formula: (adc_value * 100) / 255
    // Using uint16_t for intermediate calculation to prevent overflow
    volume_percent = (uint8_t)(((uint16_t)adc_value * 100) / 255);
    
    // Map to PWM duty cycle (0-255)
    // Direct mapping: ADC value is already 0-255
    volume_duty = adc_value;
}

/**
 * @brief Get current volume as percentage
 * 
 * @return uint8_t Volume percentage (0-100)
 */
uint8_t volume_get_percent(void)
{
    return volume_percent;
}

/**
 * @brief Get volume as PWM duty cycle value
 * 
 * @return uint8_t PWM duty cycle (0-255)
 * 
 * Returns the volume mapped to 8-bit PWM duty cycle.
 * This value can be directly written to PWM registers (e.g., OCRxx).
 */
uint8_t volume_get_duty(void)
{
    return volume_duty;
}
