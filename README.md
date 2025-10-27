# Theremin - Microcontroller Project

A digital theremin implementation using ATmega328P microcontroller with ultrasonic distance sensing, median filtering, PWM tone generation, and multi-display output.

## Features

-   **Ultrasonic Distance Sensing**: HC-SR04/PING sensor with Timer1 input capture
-   **Median Filter**: Configurable filter size (1-15 samples) for noise reduction
-   **Frequency Mapping**: Linear distance-to-frequency mapping (230-1400 Hz)
-   **PWM Tone Generation**: Dual-timer approach (Timer0 CTC + Timer2 PWM)
-   **Volume Control**: ADC-based volume adjustment with 8-bit resolution
-   **Dual Display**: I2C LCD (2x16) + 7-segment display
-   **Button Control**: Pin-change interrupts for filter size adjustment
-   **UART Debug**: Serial logging at 9600 baud
-   **Safety**: Timeout handling and automatic muting

## Hardware Requirements

-   **MCU**: ATmega328P (Arduino Uno compatible)
-   **Sensors**:
    -   HC-SR04 or PING ultrasonic sensor
    -   Potentiometer for volume control (connected to ADC)
-   **Output**:
    -   Piezo buzzer or speaker (PWM output)
    -   I2C LCD 2x16 (address 0x27)
    -   7-segment display via I2C port expander (address 0x20)
-   **Input**: 2 push buttons for filter size control

## Pinout

### ATmega328P Pin Configuration

| Pin     | Function | Module  | Description                |
| ------- | -------- | ------- | -------------------------- |
| **PB0** | ICP1     | Ping    | Input Capture (Echo)       |
| **PB1** | GPIO     | Ping    | Trigger pulse output       |
| **PD3** | OC2B     | Buzzer  | PWM tone output            |
| **PD4** | PCINT20  | Buttons | Button 0 (decrease filter) |
| **PD5** | PCINT21  | Buttons | Button 1 (increase filter) |
| **PC0** | ADC0     | Volume  | Volume potentiometer       |
| **PC4** | SDA      | Display | I2C Data (LCD + 7-seg)     |
| **PC5** | SCL      | Display | I2C Clock (LCD + 7-seg)    |
| **PD1** | TXD      | UART    | Serial debug output        |

### I2C Devices

| Device    | Address | Description                           |
| --------- | ------- | ------------------------------------- |
| LCD       | 0x27    | 2x16 character LCD (PCF8574 backpack) |
| 7-Segment | 0x20    | Port expander (PCF8574)               |

## Build Instructions

### Prerequisites

Install AVR toolchain:

```bash
# On Ubuntu/Debian
sudo apt-get install gcc-avr avr-libc avrdude

# On macOS with Homebrew
brew install avr-gcc avrdude

# On Windows
# Download and install WinAVR or use Arduino IDE's bundled tools
```

### Build with PlatformIO (Recommended)

```bash
# Build the project
pio run

# Build and upload
pio run --target upload

# Monitor serial output
pio device monitor --baud 9600
```

### Build with avr-gcc

```bash
# Compile all source files
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os \
  -I./include \
  -c src/*.c

# Link object files
avr-gcc -mmcu=atmega328p -o theremin.elf *.o

# Generate hex file
avr-objcopy -O ihex -R .eeprom theremin.elf theremin.hex

# Check size
avr-size --format=avr --mcu=atmega328p theremin.elf
```

### Flash with avrdude

```bash
# Flash to Arduino Uno (using Arduino bootloader)
avrdude -c arduino -p atmega328p -P /dev/ttyUSB0 -b 115200 \
  -U flash:w:theremin.hex:i

# Flash with USBasp programmer
avrdude -c usbasp -p atmega328p \
  -U flash:w:theremin.hex:i

# Flash with AVR ISP mkII
avrdude -c avrispmkii -p atmega328p -P usb \
  -U flash:w:theremin.hex:i

# Windows (adjust COM port)
avrdude -c arduino -p atmega328p -P COM3 -b 115200 ^
  -U flash:w:theremin.hex:i
```

### Set Fuses (Optional)

For standalone operation (not using Arduino bootloader):

```bash
# Set fuses for 16MHz external crystal, no bootloader
avrdude -c usbasp -p atmega328p \
  -U lfuse:w:0xFF:m \
  -U hfuse:w:0xD9:m \
  -U efuse:w:0xFF:m
```

## Module Overview

### Core Modules

| Module      | Files         | Description                                               |
| ----------- | ------------- | --------------------------------------------------------- |
| **Ping**    | `ping.c/h`    | Ultrasonic distance measurement with Timer1 input capture |
| **Filter**  | `filter.c/h`  | Median filter with age-based buffer management            |
| **Buzzer**  | `buzzer.c/h`  | Dual-timer tone generation (Timer0+Timer2)                |
| **Volume**  | `volume.c/h`  | ADC-based volume control with duty cycle mapping          |
| **Display** | `display.c/h` | I2C LCD + 7-segment display drivers                       |
| **Buttons** | `buttons.c/h` | Pin-change interrupt button handling                      |
| **ADC**     | `adc.c/h`     | ADC with callback support                                 |
| **UART**    | `uart.c/h`    | Serial debug output                                       |

### Control Flow

```
Ping Sensor → Distance → Median Filter → Frequency Mapping
                                              ↓
ADC (Volume) → Volume Module → PWM Duty ← Buzzer ← Frequency
                                              ↓
                                          Displays
                                    (LCD + 7-Segment)
```

## Usage

### Normal Operation

1. Power on the device
2. Place hand near ultrasonic sensor (5-65 cm range)
3. Adjust volume potentiometer
4. Use buttons to change filter size (1-15)
5. Monitor LCD for distance and frequency
6. Monitor 7-segment for filter size

### Serial Debug Output

Connect serial monitor at **9600 baud**:

```
=== Theremin Debug Started ===
D:45cm F:440Hz V:75%
D:23cm F:612Hz V:67%
D:67cm F:230Hz V:0% [TIMEOUT]
```

-   **D**: Distance in centimeters
-   **F**: Frequency in Hertz
-   **V**: Volume percentage
-   **[TIMEOUT]**: No valid measurement (buzzer muted)

### Button Functions

-   **Button 0 (PD4)**: Decrease filter size
-   **Button 1 (PD5)**: Increase filter size
-   Debounce time: 50ms
-   Range: 1-15 samples

## Configuration

### Frequency Mapping

Edit `src/ping.c` - `map_distance_to_freq()`:

```c
const uint16_t fmin = 230;   // Minimum frequency (Hz)
const uint16_t fmax = 1400;  // Maximum frequency (Hz)
const uint16_t distMax = 65; // Maximum distance (cm)
```

### Filter Size

Default: 15 samples
Adjust at runtime with buttons or in `src/filter.c`:

```c
static uint8_t max_size = MAX_FILTER_SIZE;  // 1-15
```

### Timeout

Edit `src/ping.c`:

```c
#define PING_TIMEOUT_MS 33  // Timeout period
#define PING_MAX_DISTANCE 66  // Maximum valid distance
```

## Troubleshooting

### No Sound

-   Check buzzer connection to PD3
-   Verify volume potentiometer (PC0)
-   Check serial output for [TIMEOUT] messages
-   Ensure distance is 5-65 cm

### Display Not Working

-   Verify I2C connections (SDA=PC4, SCL=PC5)
-   Check I2C addresses (LCD=0x27, 7-seg=0x20)
-   Enable pull-up resistors on I2C lines
-   Verify 5V power to displays

### Buttons Not Responding

-   Check button connections (PD4, PD5)
-   Verify internal pull-ups are enabled
-   Ensure buttons are normally open (active low)
-   Check debounce timing

### Distance Reading Issues

-   Verify trigger pin (PB1) and echo pin (PB0)
-   Check sensor 5V power and ground
-   Keep sensor perpendicular to target
-   Avoid acoustic interference

## Project Structure

```
Theremin/
├── include/           # Header files
│   ├── adc.h
│   ├── buttons.h
│   ├── buzzer.h
│   ├── display.h
│   ├── filter.h
│   ├── ping.h
│   ├── uart.h
│   └── volume.h
├── src/               # Implementation files
│   ├── adc.c
│   ├── buttons.c
│   ├── buzzer.c
│   ├── display.c
│   ├── filter.c
│   ├── main.c
│   ├── ping.c
│   ├── uart.c
│   └── volume.c
├── platformio.ini     # PlatformIO configuration
└── README.md          # This file
```

## License

Educational project for microcontroller course at Windesheim.

## Authors

-   Steven Zwaan
-   Developed for Leertaak 2 - Microcontrollers

## Version History

-   **v1.0** - Initial integration with all modules functional
-   **v1.1** - Added timeout handling and safety features
-   **v1.2** - Added UART debug logging
