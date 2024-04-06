<a name="readme-top"></a>

<h1 align="center">
  <br> STM32_FreeRTOS-Kernel <br>
</h1>

<div align="center">

[![Orel138 - STM32_FreeRTOS-Kernel](https://img.shields.io/static/v1?label=Orel138&message=STM32_FreeRTOS-Kernel&color=blue&logo=github)](https://github.com/Orel138/STM32_FreeRTOS-Kernel "Go to GitHub repo")
[![stars - STM32_FreeRTOS-Kernel](https://img.shields.io/github/stars/Orel138/STM32_FreeRTOS-Kernel?style=social)](https://github.com/Orel138/STM32_FreeRTOS-Kernel)
[![forks - STM32_FreeRTOS-Kernel](https://img.shields.io/github/forks/Orel138/STM32_FreeRTOS-Kernel?style=social)](https://github.com/Orel138/STM32_FreeRTOS-Kernel)

[![Open in Visual Studio Code](https://img.shields.io/static/v1?logo=visualstudiocode&label=&message=Open%20in%20Visual%20Studio%20Code&labelColor=2c2c32&color=007acc&logoColor=007acc)](https://open.vscode.dev/Orel138/STM32_FreeRTOS-Kernel)
[![license](https://custom-icon-badges.demolab.com/github/license/Orel138/STM32_FreeRTOS-Kernel?logo=law&logoColor=white)](https://github.com/Orel138/STM32_FreeRTOS-Kernel/blob/main/LICENSE "license MIT")
[![issues](https://custom-icon-badges.demolab.com/github/issues-raw/Orel138/STM32_FreeRTOS-Kernel?logo=issue)](https://github.com/Orel138/STM32_FreeRTOS-Kernel/issues "issues")
[![Use this template](https://img.shields.io/badge/Use_as_template-2ea44f?style=flat&color=blue)](https://github.com/Orel138/STM32_FreeRTOS-Kernel/generate)

[![STM32](https://img.shields.io/badge/STM32-message?style=flat&logo=stmicroelectronics&color=%2303234B)](https://st.com "STM32")


</div>

<div align="center">
  <h4>
    <a href="#about">About</a> |
    <a href="#requirements">Requirements</a> |
    <a href="#installation">Installation</a> |
    <a href="#usage">Usage</a> |
    <a href="#references">References</a> |
    <a href="#license">License</a>
  </h4>
</div>

<div align="center">
  <sub>Built by
  <a href="https://orel138.github.io">Orel138</a> and
  <a href="https://github.com/orel138/STM32_FreeRTOS-Kernel/graphs/contributors">contributors </a>
</div>
<br>

### A Template for STM32 Development with the FreeRTOS kernel
This project acts as a foundational template for initiating development with STM32 microcontrollers utilizing the Amazon FreeRTOS kernel, providing a basic framework that can be readily adapted and expanded.

## Table of Contents

- [About](#about)
- [Architecture Overview](#architecture-overview)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [References](#references)
- [Contributing](#contributing)
- [License](#license)

## About

_STM32_FreeRTOS-Kernel_ is a simple project designed to build projects for STM32 microcontrollers.

This project uses Amazon's FreeRTOS kernel middleware in the form of a git submodule rather than that offered by STMicroelectronics with the STM32CubeMX software.

### Features
- A command line interface (CLI) allowing basic functions, but adaptable to any use.
- A "Heartbeat" FreeRTOS task flashing the user LED.
- (In *FreeRTOS-Sensors* application only) Two FreeRTOS tasks:
    
    a. A task that reads the temperature sensor value and sends it to a FreeRTOS queue.

    b. A task that reads the temperature value from the FreeRTOS queue and displays it on an OLED screen.

### Flexibility and Compatibility
**Board Family Adaptability**: The project is structured to allow easy adaptation to different STM32 family boards (such as U5, L4, H7, etc.).

> [!IMPORTANT]
> **Driver Compatibility**: Switching between different STM32 families involves updating the drivers (HAL/BSP and CMSIS Device) to match the specific requirements of the target board.

### Alignment with STMicroelectronics' STM32Cube Topology
_STM32_FreeRTOS-Kernel_ follows the topology of the official packages provided by STMicroelectronics, known as ["STM32Cube."](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer)

This means that the project is compatible with various STM32Cube Firmware Packages such as **STM32Cube Firmware Packages** (e.g., [STM32CubeH7](https://github.com/STMicroelectronics/STM32CubeH7), [STM32CubeWB](https://github.com/STMicroelectronics/STM32CubeWB)) or **STM32Cube Extension Softwares** (e.g., [X-CUBE-AZURE](https://github.com/STMicroelectronics/x-cube-azure-telematics), [X-CUBE-FREERTOS](https://github.com/STMicroelectronics/x-cube-freertos)), enabling further expansion and customization for specific applications.
<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Architecture Overview

### Software Bill Of Materials
This Software Bill Of Materials (SBOM) itemizes the software components included in this package, detailing the copyright holder and licensing terms for each.

|   Component   |   Version |   Copyright |   License |
|  ---          |    :-:    |     :-:     |       --: |
|   CMSIS Core  |   x   |   ARM Limited |   [Apache License 2.0](https://opensource.org/license/apache-2-0) |
|   CMSIS Device STM32WB5   |   x   |   ARM Limited |   [Apache License 2.0](https://opensource.org/license/apache-2-0) |
|   Drivers HAL/LL STM32WB5   |   x   |   STMicroelectronics |   [BSD-3-Clause](https://opensource.org/license/BSD-3-Clause) |
|   BSP STM32WB5MM-DK Board   |   x   |   STMicroelectronics |   [BSD-3-Clause](https://opensource.org/license/BSD-3-Clause) |
|   BSP Components   |   x   |   STMicroelectronics |   [BSD-3-Clause](https://opensource.org/license/BSD-3-Clause) |
|   FreeRTOS-Kernel   |   x   |   Amazon.com |   [MIT](https://opensource.org/license/MIT) |
|   Applications projects   |   x   |   Amazon.com, STMicroelectronics, Orel138 |   [MIT](https://opensource.org/license/MIT) |

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Requirements

### Boards available
- STM32WB55
  - [STM32WB5MM-DK](https://www.st.com/en/evaluation-tools/stm32wb5mm-dk.html)

### Development toolchains and compilers

Before you begin, ensure you have met the following softwares installed on your computer :

- STM32CubeIDE v1.14.1
- STM32CubeMX v6.11.0
- STM32CubeProgrammer v2.16.0

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

### Installation

To install STM32_FreeRTOS-Kernel, follow these steps:

1. Clone the repository
   ```bash
   git clone https://github.com/Orel138/STM32_FreeRTOS-Kernel.git
   ```
2. Navigate to the project directory
   ```bash
   cd STM32_FreeRTOS-Kernel
   ```
3. Run the following command.
   ```bash
   git submodule update --init
   ```
4. Open the project with STM32CubeIDE (open the *.project* or *.cproject* file)/
   Projects for STM32CubeIDE are located in *Projects/{Board}/Applications/{FreeRTOS-xxxxxxxx}/STM32CubeIDE/*
5. Build the project in STM32CubeIDE.
6. Debug in the IDE or use STM32cubeProgrammer to flash the executable.
4. The executable (`.elf` file) for the example will be in `Projects/{Board}/Applications/{FreeRTOS-xxxxxxxx}/STM32CubeIDE/Debug` folder.
- You can now drag and drop the `.elf` file in your board and see the user LED blink.
- Or you can modify the project for your STM32 target with STM32CubeIDE and STM32CubeMX.

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Usage

- WIP

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## References

- WIP

## Contributing
We welcome your contributions to _STM32_FreeRTOS-Kernel_.

To contribute:
1. Fork the repository.
2. Create a new branch: git checkout -b [branch-name].
3. Make your changes and commit them: git commit -m '[commit-message]'.
4. Push to the original branch: git push origin [project-name]/[location].
5. Create the pull request.

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## License
_STM32_FreeRTOS-Kernel_ is released under the [MIT license](/LICENSE) © [Orel138](https://github.com/Orel138).

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

> [!TIP]
> I trust you'll find this project enjoyable. Should you appreciate the project, bestowing a small ⭐ on it is a meaningful gesture, signifying: **My efforts are recognized.** Your support would be greatly valued. _Many thanks!_