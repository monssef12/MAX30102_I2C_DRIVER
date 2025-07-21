# MAX30102 Sensor Driver (STM32 - HAL)

This repository contains a lightweight C driver for the MAX30102 heart rate and SpO₂ sensor, designed to work with STM32 microcontrollers using the HAL library.

---

## 📌 Features

- I2C communication with MAX30102
- Configuration for SpO₂ and Heart Rate modes
- Reads RED and IR values from FIFO
- Supports temperature reading
- Uses 18-bit ADC data for signal processing
- UART output for debugging/logging

---

## 🚀 Getting Started

### ✅ Requirements

- STM32 MCU (tested with STM32F series)
- STM32CubeMX-generated project using HAL
- I2C peripheral configured and initialized
- UART configured (optional, for debug)

---

### 🧩 Files Included

| File              | Description                                |
|------------------|--------------------------------------------|
| `max30102.c/h`    | Sensor driver implementation and headers   |
| `main.c`          | Example usage: init, read, UART print      |
| `README.md`       | Project overview and instructions          |

---

## 📖 How It Works

### 🔄 Data Acquisition

- The MAX30102 outputs **18-bit ADC values** for RED and IR light absorption.
- The driver reads the FIFO and reconstructs full values from the 3-byte format.
- Example output:
