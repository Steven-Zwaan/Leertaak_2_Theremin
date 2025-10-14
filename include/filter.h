#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/**
 * @brief Initialize the filter
 *
 * Sets up the filter parameters and initial state.
 * Clears any previous filter history.
 */
void filter_init(void);

/**
 * @brief Update filter with new value
 *
 * @param raw_value The raw input value to be filtered
 * @return uint16_t The filtered output value
 * Processes the input through the filter algorithm (e.g., moving average, low-pass).
 */
uint16_t filter_update(uint16_t raw_value);

/**
 * @brief Read last filtered value
 *
 * @return uint16_t The last filtered output value
 * Returns the most recent filtered value without updating.
 */
uint16_t filter_read(void);

/**
 * @brief Reset filter state
 *
 * Clears the filter history and resets to initial state.
 */
void filter_reset(void);

#endif // FILTER_H
