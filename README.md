# nturt_stm32_rear_box

## Introduction

This is the repository for the firmware in the STM32G431 of the rear box in Epsilon4 Formula car. The codebase is built around HAL, with coderdbc and nturt_stm32_module as library dependencies.

## Installation
The dependent libraries are not in the repo by default. They have to be installed seperately after you have cloned the repository before you can build the binary. Please follow the steps below:
1. run `git submodule init` and `git submodule update` to clone the 2 submodule "nturt_can_config" and "nturt_stm32_module". They should then live in the Lib/ directory afterwards.
2. open the .ioc file in the repo and click "Generate Code" to generate the HAL drivers and the FreeRTOS source code.
3. Go to Lib/ntur_can_config and run the bash scripts that generate the coderdbc C source files. The relevant steps is documented in the ReadMe of that repository. If you cannot run the bash scripts, you may contact the team members of electrical systems group.
4. Verify that all source code are in place by building in CubeIDE.

## Peripheral Configuration

### GPIO

- Pins are denoted as `PIN`, e.g. `5V`, `PA0`, etc.
