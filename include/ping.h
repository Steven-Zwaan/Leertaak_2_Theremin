#ifndef PING_H
#define PING_H

#include <stdint.h>

/**
 * @brief Initialize the PING sensor
 *
 * Configures the GPIO pins and timer for ultrasonic distance measurement.
 */
void ping_init(void);

/**
 * @brief Start PING measurement
 *
 * Triggers the ultrasonic sensor to start a distance measurement.
 * Sends a pulse to the PING sensor.
 */
void ping_start(void);

/**
 * @brief Read PING distance
 *
 * @return uint16_t Distance in centimeters
 * Returns the measured distance from the ultrasonic sensor.
 */
uint16_t ping_read(void);

/**
 * @brief Check if ping timeout occurred
 *
 * @return uint8_t 1 if timeout, 0 if normal
 * Returns whether the last ping measurement timed out.
 */
uint8_t ping_is_timeout(void);

/**
 * @brief Handle ping timeout
 *
 * Should be called from main loop if no echo received within timeout period.
 * Sets distance to invalid value and marks timeout condition.
 */
void ping_handle_timeout(void);

/**
 * @brief Get the last computed frequency
 *
 * @return uint16_t Frequency in Hz (230-1400 Hz range)
 * Returns the frequency calculated from the most recent distance measurement.
 */
uint16_t ping_get_frequency(void);

/**
 * @brief PING echo ISR hook
 *
 * Called on rising/falling edge of echo pin.
 * This function should be called from the external interrupt or pin change ISR.
 */
void ping_isr_handler(void);

#endif // PING_H
