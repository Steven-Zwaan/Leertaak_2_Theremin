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
 * @brief PING echo ISR hook
 *
 * Called on rising/falling edge of echo pin.
 * This function should be called from the external interrupt or pin change ISR.
 */
void ping_isr_handler(void);

#endif // PING_H
