#include "filter.h"
#include <stdint.h>

// Maximum size of the filter buffer
#define MAX_FILTER_SIZE 15

/**
 * @brief Filter item structure
 *
 * Contains a value and its age for median filtering.
 * Age is used to track how long a value has been in the buffer.
 */
typedef struct
{
    uint8_t age;    // Age of the value (for tracking/replacement)
    uint16_t value; // The actual data value
} filter_item;

// Filter buffer and state
static filter_item filter_buffer[MAX_FILTER_SIZE];
static uint8_t current_size = 0;

/**
 * @brief Initialize the filter
 *
 * Sets up the filter parameters and initial state.
 * Clears any previous filter history.
 */
void filter_init(void)
{
    // Reset current size
    current_size = 0;

    // Clear all filter items
    for (uint8_t i = 0; i < MAX_FILTER_SIZE; i++)
    {
        filter_buffer[i].age = 0;
        filter_buffer[i].value = 0;
    }
}

/**
 * @brief Add new value to filter buffer with age rotation
 *
 * Increments age of all existing items, then either:
 * - If buffer not full: adds new item with age=0 at current_size position
 * - If buffer full: overwrites the oldest item (max age) with new value at age=0
 *
 * @param val The new value to add to the filter buffer
 */
static void filter_add(uint16_t val)
{
    // Increment age for all current items
    for (uint8_t i = 0; i < current_size; i++)
    {
        filter_buffer[i].age++;
    }

    if (current_size < MAX_FILTER_SIZE)
    {
        // Buffer not full: add new element at the end
        filter_buffer[current_size].age = 0;
        filter_buffer[current_size].value = val;
        current_size++;
    }
    else
    {
        // Buffer full: find and replace the oldest item (highest age)
        uint8_t oldest_index = 0;
        uint8_t max_age = filter_buffer[0].age;

        for (uint8_t i = 1; i < MAX_FILTER_SIZE; i++)
        {
            if (filter_buffer[i].age > max_age)
            {
                max_age = filter_buffer[i].age;
                oldest_index = i;
            }
        }

        // Replace oldest item with new value
        filter_buffer[oldest_index].age = 0;
        filter_buffer[oldest_index].value = val;
    }
}

/**
 * @brief Update filter with new value
 *
 * @param raw_value The raw input value to be filtered
 * @return uint16_t The filtered output value
 * Processes the input through the filter algorithm (e.g., moving average, low-pass).
 */
uint16_t filter_update(uint16_t raw_value)
{
    // TODO: Implement median filter update logic
    return raw_value;
}

/**
 * @brief Read last filtered value
 *
 * @return uint16_t The last filtered output value
 * Returns the most recent filtered value without updating.
 */
uint16_t filter_read(void)
{
    // TODO: Implement filtered value read
    return 0;
}

/**
 * @brief Reset filter state
 *
 * Clears the filter history and resets to initial state.
 */
void filter_reset(void)
{
    filter_init();
}
