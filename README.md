# STM32 FreeRTOS Temperature Monitor with LCD Touch and FRAM Logging

This project is an embedded IoT application developed on STM32 using STM32CubeIDE, HAL drivers, and FreeRTOS.

## Features

- Reads internal STM32 temperature sensor using ADC
- Displays real-time temperature and ADC value on ST7789 LCD
- Touch UI with HOME, PLAY, and PAUSE modes
- PLAY mode heats the STM32 chip and monitors internal ADC data
- Saves ADC values to external FRAM when temperature exceeds 38°C
- PAUSE mode displays saved over-threshold records from FRAM
- Uses FreeRTOS with multiple tasks for heating, sensing, and LCD/touch handling

## Hardware / Platform

- STM32F405RGTX
- ST7789 LCD
- Touch screen interface
- External FRAM via I2C
- ADC internal temperature sensor

## Tools

- STM32CubeIDE
- STM32 HAL Driver
- FreeRTOS
- C language

## Project Structure

```text
Core/           Main application source code
Drivers/        STM32 HAL and CMSIS drivers
Middlewares/    FreeRTOS middleware
IOT2.ioc        STM32CubeMX configuration file
```

## Demo Video

[Watch the demo video on Google Drive](https://drive.google.com/file/d/1s3vPyeEUkB8SpDj9e1Nro_5s-1otYczW/view?usp=drive_link)

## How to Open

1. Open STM32CubeIDE.
2. Choose `File > Import > Existing Projects into Workspace`.
3. Select this project folder.
4. Build and flash to the STM32 board.