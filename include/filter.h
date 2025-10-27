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

/**
 * @brief Set the maximum filter size
 *
 * @param n The desired maximum filter size (1-9)
 * Sets the runtime configurable maximum size of the filter buffer.
 * Size is clamped to valid range [1, 9] for 7-segment display compatibility.
 */
void filter_set_size(uint8_t n);

/**
 * @brief Get the current maximum filter size
 *
 * @return uint8_t The current maximum filter size (1-9)
 * Returns the runtime configured maximum size of the filter buffer.
 */
uint8_t filter_get_size(void);

#endif // FILTER_H
