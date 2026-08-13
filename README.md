# SREC-Loader
This repository contains code for a simple host-side loader, that uploads SREC-encoded executable to a UART-connected device running a SREC bootloader.

## Usage

To upload, you need a SREC dump of your image.

Usage:
```
./uart_flasher [-b] [--baud <rate>] <filename> <port identifier> [run command]
```

- `-b`, `--binary` enables the (custom) SREC-200 binary protocol. Without this switch, the original textual SREC upload is used.
- `--baud <rate>` switches the binary transfer to a higher baud rate after the initial 115200 baud handshake. This requires a matching SREC-200 bootloader. Maximum supported baud rate (aligned with the PL USB-TTL converters) is `921600`.
- `filename` is an absolute or relative path to the SREC, ELF, IMG, or BIN image. ELF/IMG/BIN inputs are supported only in binary mode; raw IMG/BIN files are loaded at `0x8000` and started at `0x8000`.
- `port identifier` is a name of, or a path to the serial port to be used (e.g., `COM4`, `/dev/ttyUSB0`, ...); please note, that MS Windows requires the COM port name in the form of `\\.\COM12` for ports with identifier larger than 9
- `run command` - command to execute after loading is done
  - when not specified, the loader ends after the upload is done
  - `local` - runs only a local listener (in-program loop that reads characters from the port)
  - `putty` - attempts to find PuTTY installation and run it after the loading is done
  
Examples:
```
./uart_flasher kernel.srec COM4 putty
./uart_flasher -b kernel.srec COM4 putty
./uart_flasher -b --baud 921600 kernel.elf COM4 putty
./uart_flasher -b --baud 921600 kernel.img /dev/ttyUSB0
./uart_flasher kernel.srec /dev/ttyUSB0
```

## License

The software is distributed under the MIT license. See attached LICENSE file for full licencing information.

|![University of West Bohemia](https://www.zcu.cz/en/assets/logo.svg)|![Department of Computer Science and Engineering](https://www.kiv.zcu.cz/site/documents/verejne/katedra/dokumenty/dcse-logo-barevne.png)|
|--|--|
