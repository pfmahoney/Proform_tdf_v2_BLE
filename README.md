# Proform_tdf_v2_BLE

# ProForm TdF Pro 4.0 BLE Conversion

This project replaces the original iFit-based display on a ProForm Tour de France Pro 4.0 / TdF v2 indoor bicycle trainer with a modern ESP32-based controller and display. It converts the trainer from a proprietary Wi-Fi–dependent design into a native Bluetooth Low Energy (BLE) smart trainer compatible with modern training platforms.

The system removes the need for QDOMYOS_ZWIFT or other Wi-Fi bridges by advertising, receiving, and broadcasting BLE data directly. Training applications such as Zwift and Rouvy can control resistance and elevation while receiving power and cadence data in real time.

---

## Background

The ProForm TdF Pro 4.0 relies on an embedded iFit display and a Wi-Fi communication model that is increasingly unreliable and unsupported. This project replaces the entire control and communication layer while retaining the original mechanical resistance hardware.

The goal is to extend the useful life of otherwise functional hardware and restore compatibility with current training software.

---

## Features

- Native BLE advertising and pairing
- Direct compatibility with Zwift and Rouvy
- Bidirectional control of resistance and simulated grade
- Real-time reporting of power and cadence
- Replacement of the original console with a Waveshare ESP32-S3 LCD
- Standalone operation without phone or bridge applications

---

## System Overview

- **Controller / Display**: Waveshare ESP32-S3 LCD
- **Communication**: Bluetooth Low Energy
- **Control Loop**:
  - BLE commands from training software drive resistance and elevation
  - Sensor data is processed and reported as power and cadence
- **Trainer Interface**:
  - Direct electrical interface to existing ProForm resistance hardware
  - No modification to the mechanical drivetrain

---

## Compatibility

### Hardware
- ProForm Tour de France Pro 4.0
- may work with other Proform TdF models but I haven't tested

### Software
- Zwift
- Rouvy
- MyWhoosh

Other BLE-capable training applications may work but have not been fully tested.

---
