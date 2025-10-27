#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "adc.h"
#include "ping.h"
#include "filter.h"
#include "volume.h"
#include "buzzer.h"
#include "display.h"
#include "buttons.h"
#include "uart.h"

// ADC channel for volume control
#define VOLUME_ADC_CHANNEL 0

// Debug logging interval (ms)
#define DEBUG_LOG_INTERVAL 250

// Measurement ready flag (checked in main loop)
static volatile uint8_t measurement_ready = 0;

/**
 * @brief ADC callback for volume control
 *
 * Called when ADC conversion completes (from ISR context).
 * Updates volume from ADC reading.
 */
void adc_volume_callback(uint8_t adc_value)
{
  volume_set_from_adc(adc_value);
}

/**
 * @brief Send debug log via UART
 *
 * Prints current system state: distance, frequency, and volume.
 */
static void debug_log(uint16_t distance, uint16_t frequency, uint8_t volume)
{
  uart_puts("D:");
  uart_put_uint(distance);
  uart_puts("cm F:");
  uart_put_uint(frequency);
  uart_puts("Hz V:");
  uart_put_uint(volume);
  uart_puts("%");
  uart_newline();
}

/**
 * @brief Check if ping measurement is ready
 *
 * This is a wrapper to check the internal ping_measure_ready flag.
 * Since the flag is internal to ping.c, we'll trigger via ping_start
 * and check if new data is available via ping_read changes.
 */
static uint8_t check_ping_ready(void)
{
  // For simplicity, we'll use a periodic trigger
  // In a real implementation, check the internal ISR flag
  return 1; // Assume ready for periodic processing
}

int main(void)
{
  // Initialize all modules
  uart_init(9600); // Initialize UART at 9600 baud
  adc_init();
  ping_init();
  filter_init();
  volume_init();
  buzzer_init();
  display_init();
  buttons_init();

  // Register ADC callback for volume control
  adc_set_callback(adc_volume_callback);

  // Enable global interrupts
  sei();

  // Initialize display
  display_clear();

  // Send startup message
  uart_puts("=== Theremin Debug Started ===");
  uart_newline();

  // Debug logging counter
  uint8_t log_counter = 0;

  // Main control loop
  while (1)
  {
    // Trigger ping measurement periodically
    ping_start();
    _delay_ms(60); // Wait for measurement (>50ms for ping sensor)

    // Read distance from ping sensor
    uint16_t distance = ping_read();

    // Process distance through median filter
    uint16_t filtered_distance = filter_update(distance);

    // Get frequency from ping module (already mapped)
    uint16_t frequency = ping_get_frequency();

    // Set buzzer frequency
    buzzer_set_frequency(frequency);

    // Read volume and set buzzer duty cycle
    uint8_t volume_duty = volume_get_duty();
    uint8_t volume_percent = volume_get_percent();
    buzzer_set_volume_duty(volume_duty);

    // Update LCD display with distance and frequency
    display_update(filtered_distance, frequency);

    // Update 7-segment display with current filter size
    uint8_t filter_size = filter_get_size();
    display_filter_size(filter_size);

    // Start ADC conversion for volume control
    adc_start(VOLUME_ADC_CHANNEL);

    // Debug logging every ~250ms (approx every 2 loop iterations)
    log_counter++;
    if (log_counter >= 2)
    {
      debug_log(filtered_distance, frequency, volume_percent);
      log_counter = 0;
    }

    // Small delay to prevent overwhelming the displays
    _delay_ms(50);
  }
}
