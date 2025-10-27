# Theremin - Microcontroller Project

A digital theremin implementation using ATmega328P microcontroller with ultrasonic distance sensing, median filtering, PWM tone generation, and multi-display output.

## Features

-   **Ultrasonic Distance Sensing**: HC-SR04/PING sensor with Timer1 input capture
-   **Median Filter**: Configurable filter size (1-9 samples) for noise reduction
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

## Unit Testing

### Host-Side Testing

The project includes unit tests that run on your development machine without hardware, testing the filter logic and distance-to-frequency mapping algorithms.

#### Test Files

-   `test/filter_test.c` - Tests median filter with 14 test cases
-   `test/mapping_test.c` - Tests distance-to-frequency mapping with 16 test cases

#### Run Tests with PlatformIO

```bash
# Run all tests on native platform
pio test -e native

# Run specific test
pio test -e native -f filter_test
pio test -e native -f mapping_test
```

#### Run Tests with Make

**Linux/macOS:**

```bash
# Download Unity framework (first time only)
make unity

# Build and run all tests
make test

# Clean build artifacts
make clean
```

**Windows (PowerShell):**

```powershell
# Download Unity framework (first time only)
make -f Makefile.win unity

# Build and run all tests
make -f Makefile.win test

# Clean build artifacts
make -f Makefile.win clean
```

#### Manual Compilation

**Prerequisites:**

-   GCC compiler (MinGW on Windows)
-   Unity framework: https://github.com/ThrowTheSwitch/Unity

```bash
# Compile filter test
gcc -std=c11 -Wall -Wextra -I./include -I./unity/src \
  -o filter_test test/filter_test.c unity/src/unity.c -lm

# Compile mapping test
gcc -std=c11 -Wall -Wextra -I./include -I./unity/src \
  -o mapping_test test/mapping_test.c unity/src/unity.c -lm

# Run tests
./filter_test
./mapping_test
```

#### Test Coverage

**Filter Tests:**

-   Initialization and single value
-   Median calculation (odd/even count)
-   Outlier rejection
-   Buffer overflow handling
-   Age-based rotation
-   Noise reduction and spike rejection

**Mapping Tests:**

-   Boundary conditions (min/max distance)
-   Linearity verification
-   Out-of-range handling
-   Musical note coverage
-   Overflow protection

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
4. Use buttons to change filter size (1-9)
5. Monitor LCD, 7-segment display, and serial output for real-time data

### Controlling the Theremin

#### Changing Frequency

The frequency is controlled by the **distance** between your hand and the ultrasonic sensor:

-   **Close distance (5 cm)**: Higher frequency (~1400 Hz)
-   **Far distance (65 cm)**: Lower frequency (~230 Hz)
-   **Linear mapping**: Distance is linearly mapped to frequency
-   Move your hand closer or farther from the sensor to change the pitch
-   Valid range: 5-65 cm (measurements outside this range will timeout)

#### Adjusting Volume

Volume is controlled by a **potentiometer** connected to ADC0 (PC0):

-   **Turn potentiometer clockwise**: Increase volume (0-100%)
-   **Turn potentiometer counterclockwise**: Decrease volume
-   **8-bit resolution**: 256 volume levels
-   Volume affects the PWM duty cycle of the buzzer output
-   Real-time volume adjustment without interrupting the tone

#### Changing Filter Size

The median filter size can be adjusted using **two buttons**:

-   **Button 0 (PD4)**: Decrease filter size (minimum: 1 sample)
-   **Button 1 (PD5)**: Increase filter size (maximum: 9 samples)
-   **Debounce protection**: 50ms debounce prevents accidental multiple presses
-   **Effect**: Larger filter sizes provide smoother frequency transitions but slower response
-   **Recommendation**: Use filter size 5-7 for optimal noise reduction

### Reading System Values

The theremin provides real-time feedback through three output interfaces:

#### 7-Segment Display

The 7-segment display shows the **current filter size**:

-   **Display location**: I2C address 0x20 (PCF8574 port expander)
-   **Range**: 1-9 (single digit)
-   **Updates**: Immediately when filter size is changed via buttons
-   **Example**: Display shows "5" when filter is set to 5 samples

#### LCD Display (2x16)

The LCD shows **distance and frequency** on two lines:

-   **Display location**: I2C address 0x27 (PCF8574 LCD backpack)
-   **Line 1**: Distance measurement
    -   Format: `Dist: XX cm` (e.g., "Dist: 45 cm")
    -   Updates continuously based on sensor readings
    -   Shows "TIMEOUT" if no valid measurement
-   **Line 2**: Frequency output
    -   Format: `Freq: XXXX Hz` (e.g., "Freq: 440 Hz")
    -   Calculated from filtered distance measurement
    -   Updates in real-time as you move your hand
-   **Refresh rate**: Updated every measurement cycle (~30ms)

#### Serial Monitor (UART)

Connect a serial monitor at **9600 baud** to view detailed debug information:

```
=== Theremin Debug Started ===
D:45cm F:440Hz V:75% FILTER: 5
D:23cm F:612Hz V:67% FILTER: 5
D:67cm F:230Hz V:0% [TIMEOUT]
D:34cm F:589Hz V:100% FILTER: 7
```

**Serial output format:**

-   **D:XXcm** - Distance in centimeters from ultrasonic sensor
-   **F:XXXHz** - Calculated frequency in Hertz
-   **V:XX%** - Volume percentage (0-100%)
-   **FILTER: X** - Current median filter size (1-9)
-   **[TIMEOUT]** - Indicates no valid measurement (buzzer muted)

**To monitor serial output:**

```bash
# Using PlatformIO
pio device monitor --baud 9600

# Using Arduino IDE Serial Monitor
# Set to 9600 baud

# Using screen (Linux/macOS)
screen /dev/ttyUSB0 9600

# Using PuTTY (Windows)
# Set COM port and 9600 baud
```

### Button Functions

-   **Button 0 (PD4)**: Decrease filter size
-   **Button 1 (PD5)**: Increase filter size
-   Debounce time: 50ms
-   Range: 1-9 samples

## Configuration

### Frequency Mapping

Edit `src/ping.c` - `map_distance_to_freq()`:

```c
const uint16_t fmin = 230;   // Minimum frequency (Hz)
const uint16_t fmax = 1400;  // Maximum frequency (Hz)
const uint16_t distMax = 65; // Maximum distance (cm)
```

### Filter Size

Default: 9 samples
Adjust at runtime with buttons or in `src/filter.c`:

```c
static uint8_t max_size = MAX_FILTER_SIZE;  // 1-9
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
├── test/              # Unit tests (host-side)
│   ├── filter_test.c
│   └── mapping_test.c
├── platformio.ini     # PlatformIO configuration
├── Makefile           # Unix/macOS test build
├── Makefile.win       # Windows test build
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
-   **v1.3** - Added host-side unit tests with Unity framework
