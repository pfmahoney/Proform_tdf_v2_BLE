# Proform_tdf_v2_BLE

# ProForm TdF Pro 4.0 BLE Conversion

This project replaces the original iFit-based display on a ProForm Tour de France Pro 4.0 bicycle trainer from approximately 2012-2016 with a modern ESP32-based controller and display. It converts the trainer from a proprietary Wi-Fi–dependent design into a native Bluetooth Low Energy (BLE) smart trainer which is directly compatible with modern training platforms.  It also resolves an issue where the built-in original Proform iFit display gets stuck in an endless boot-loop and the bike stops working because the display doesn't work any more.   

The system removes the need for QDOMYOS_ZWIFT or other Wi-Fi bridges to connect to online cycling apps like Zwift and Rouvy by advertising, receiving, and broadcasting BLE data directly. Training applications such as Zwift and Rouvy can control resistance and elevation while receiving power and cadence data in real time.

---

## Background

The ProForm TdF Pro 4.0 relies on an embedded iFit display and a Wi-Fi communication model that is increasingly unreliable and unsupported. This project replaces the entire control and communication layer while retaining the original mechanical resistance hardware.

The goal is to extend the useful life of otherwise functional hardware and restore compatibility with current training software.

Specifically the goal is to replace the "ProForm TDF Pro 4.0 Bike 7" Display Touchscreen Console Assembly Ebpf01914" with a ~$45 "Waveshare ESP32-S3 7 LCD"

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
- ProForm Tour de France Pro 4.0 from approximately 2012 to 2016
- may work with other Proform TdF models but I haven't tested

### Software
- Zwift
- Rouvy
- MyWhoosh

Other BLE-capable training applications may work but have not been fully tested.

---

## Acknowledgements

This project was inspired by the work of @kevinmott09:
https://github.com/kevinmott09/proformBLE

While this implementation is a complete rewrite, the original project demonstrated that replacing the factory iFit console on the ProForm TdF platform was feasible and served as the initial inspiration and I used his reverse-engineering of the cadence and resistance values as well as the RS-485 connectivity as the basis for my code.  Without the example from @kevinmott09, there's a high probability my Proform TdF v4 Pro would be headed over to  metal scrap recycling.

