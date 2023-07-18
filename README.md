# STARLIGHT Rocket Development Board

The STARLIGHT Rocket Development Board is a versatile platform designed for model rocketry projects. This repository contains all the necessary information and resources to get started with the STARLIGHT board.

## Features

- Powerful RP2040 microcontroller for fast code execution and easy programming in Python.
- ICM-42605 6-axis IMU with gyroscope and accelerometer.
- Level shifter for compatibility with 5V servos and the 3.3V RP2040.
- Dual temperature sensors for redundancy and overheating protection.
- Pressure sensor for altitude determination and flight tracking.
- 6x 3.3V GPIO pins, with exposed SPI, I2C, and UART interfaces for increased versatility.
- 2x 5V GPIO pins enabled by the on-board level shifter.
- 16MB flash storage for storing flight data and firmware.
- Indicator LEDs for visual feedback.
- Micro-USB port for easy programming.
- Wide input voltage range of 5-18V.
- Dual voltage regulators for both 3.3V and 5V directly on the board.
- Two servo outputs for optional thrust loop vector control interface.
- M3 mounting holes (mounting hardware not included).
- Digital guide included.

## Getting Started

To start using the STARLIGHT board, follow these steps:

1. **Hardware Setup:** Connect the necessary peripherals to the board, such as igniters, servos, and sensors, as outlined in the documentation.
2. **Firmware Development:** Write your own firmware to control the board using the provided library. Refer to the digital guide included in this repository for detailed instructions.
3. **Programming the Board:** Connect the board to your computer via the Micro-USB port and upload your firmware using your preferred programming tool.
4. **Testing and Integration:** Test the functionality of your firmware on the board and integrate it into your model rocket project.
5. **Contribute:** If you discover any issues or have improvements to suggest, feel free to submit a pull request to this repository.

## Repository Contents

- `/docs`: Contains the digital guide with detailed documentation and examples for using the STARLIGHT board.
- `README.md`: This file, providing an overview of the repository and instructions for getting started.

## License

This project is licensed under the [MIT License](LICENSE), allowing for both personal and commercial use.

## Purchase

The STARLIGHT board is available for purchase [here]() at the price of $49 USD.

For further details and support, please refer to the documentation in the `/docs` directory.

Happy rocketry hacking with the STARLIGHT board!
