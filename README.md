# CANFD-Sender

A CAN / CAN FD message transmission and replay tool for the **Teensy 4.1** microcontroller, built with PlatformIO and the FlexCAN_T4 library.

## Hardware

- Teensy 4.1 (Andy's dodgy dev board - purple)
- LEMO connector for CAN / CAN FD bus
- Black button (pin 29) for start/stop control
- LED indicator (pin 13) shows transmission status
- SD card slot for ASC file replay

## Getting Started

1. Connect LEMO to a CAN or CAN FD bus
2. Open a serial terminal at **115200 baud** (USB port)
3. Configure the bus settings using keyboard commands (press `h` for help)
4. Press `p` or the black button to start/stop sending

## Features

### CAN Modes
- **CAN FD** - 64-byte payload, flexible data rates (default)
- **Standard CAN** - 8-byte payload, classic CAN

### Baud Rates (Nominal)
1 Mbps, 500 Kbps, 250 Kbps, 125 Kbps, 100 Kbps

### Data Rates (CAN FD only)
5 Mbps, 4 Mbps (default), 2 Mbps, 1 Mbps

### TX Frequencies
- **100 Hz** (10 ms) - default
- **50 Hz** (20 ms)
- **10 Hz** (100 ms)
- **1 Hz** (1000 ms)

### Payload Modes
- **FF mode** - all bytes filled with `0xFF`
- **Incrementing mode** - payload bytes increment with each frame

### Multiple Frames
Send 1-100 simultaneous frames with sequential CAN IDs starting from `0x555`.

### Bus Load Monitoring
Press `l` to run a 3-second bus load measurement showing frames/second and estimated bus load percentage for both your device and other traffic on the bus.

### Statistics
Press `s` during transmission to view session duration, total frames sent, average frames/second, and per-ID frame counts.

### ASC File Replay
Replay pre-recorded CAN message logs from `.asc` files stored on the SD card.

1. Press `a` to enter ASC replay mode
2. Select a file from the list (`1`-`9`)
3. Press `p` or the button to play/pause
4. Replay loops automatically at end of file
5. Press `x` to exit back to normal mode

Supports both standard CAN and CAN FD frames in ASC format. The tool auto-detects FD content and warns if the device is in standard CAN mode.

## Keyboard Controls

| Key | Function |
|-----|----------|
| `m` | Toggle CAN FD / Standard CAN mode |
| `b` | Cycle baud rate |
| `d` | Cycle data rate (CAN FD only) |
| `f` | Cycle TX frequency |
| `1` | FF payload mode |
| `2` | Incrementing payload mode |
| `+` | Add a frame (max 100) |
| `-` | Remove a frame (min 1) |
| `p` | Start / stop transmission |
| `s` | Show statistics |
| `l` | Measure bus load (3 seconds) |
| `a` | Enter ASC replay mode |
| `h` | Display help menu |

> Mode, baud rate, and data rate cannot be changed while transmitting.

## Build

Requires [PlatformIO](https://platformio.org/). Build and upload with:

```
pio run --target upload
```
