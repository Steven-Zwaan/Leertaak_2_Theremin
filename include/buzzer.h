#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/**
 * @brief Initialize the buzzer
 *
 * Configures the PWM timer and GPIO pin for buzzer control.
 * Sets up the necessary registers for tone generation.
 */
void buzzer_init(void);

/**
 * @brief Start buzzer with frequency
 *
 * @param frequency Frequency in Hz for the tone
 * Starts the buzzer playing at the specified frequency.
 */
void buzzer_start(uint16_t frequency);

/**
 * @brief Update buzzer frequency
 *
 * @param frequency New frequency in Hz
 * Updates the buzzer to play at a new frequency without stopping.
 */
void buzzer_update(uint16_t frequency);

/**
 * @brief Stop the buzzer
 *
 * Stops the buzzer output (silence).
 */
void buzzer_stop(void);

/**
 * @brief Buzzer timer ISR hook
 *
 * Called from the timer overflow interrupt if needed.
 * Can be used for advanced buzzer control.
 */
void buzzer_isr_handler(void);

#endif // BUZZER_H
