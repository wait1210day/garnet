# Garnet

Garnet is the firmware for a digital GaN four-switch buck-boost power module
built around the **STM32G474**, written in **C++23** and running on **FreeRTOS**.

## Features

- **Adaptive buck-boost control** with automatic selection of buck, boost,
and transition regions.
- **Constant-voltage regulation** with hardware peak-current control, approximately
500 kHz switching, and a 250 kHz digital control loop.
- **Soft startup and voltage changes**, including startup into a precharged output, 
and passthrough mode with soft start.
- **Hardware protection** using programmable DAC thresholds and HRTIM fault shutdown.

## Development status

The project is in active development at the early prototype stage:

- [x] Buck/boost voltage regulation and operating-region transitions.
- [x] Regulated and passthrough startup, output shutdown, and gate-driver power sequencing.
- [x] Zero current detection and diode emulation.
- [x] Pulse-skipping mode for light loads (experimental).
- [x] Initial hardware overvoltage protection, serial commands, and measurement logging.
- [x] FreeRTOS integration, CMake builds, and MPU task stack guards.
- [ ] Constant-current regulation.
- [ ] ... and more

## License and acknowledgements

Original firmware code is licensed under the **GNU General Public License v3.0 or later**
as indicated in the source headers.

Third-party components retain their respective copyrights and licenses.

Retain the applicable copyright and license notices when redistributing these components.

The modulation strategy is independently implemented for STM32G474 HRTIM and draws inspiration
from the operating principles described in the **LT8390** datasheet. LT8390 and related trademarks
belong to Analog Devices, Inc. Garnet is an independent project with no affiliation, sponsorship,
or endorsement by Analog Devices.
