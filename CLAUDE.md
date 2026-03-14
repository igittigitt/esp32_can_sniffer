# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

**Framework:** ESP-IDF 5.5.2 | **Target:** ESP32-C6

```bash
idf.py build          # Compile firmware
idf.py flash          # Flash to device
idf.py monitor        # Serial monitor
idf.py flash monitor  # Flash and open monitor
idf.py menuconfig     # Project-level configuration (Kconfig)
```

Merge into a single flashable binary (for distribution):
```bash
esptool.py --chip esp32c6 merge_bin -o flash/esp32c6_esp32_can_sniffer_merged.bin \
  0x0000 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0xd000 build/ota_data_initial.bin \
  0x10000 build/esp32_can_sniffer.bin
```

There are no automated tests — validation is done manually via Telnet (`nc <IP> 23`) or browser (`http://<IP>`).

## Architecture

### Data Flow

CAN frames flow from hardware ISR → ring buffer → per-client readers → TCP/WebSocket output. The ring buffer decouples the ISR from all network I/O and supports simultaneous readers (each TCP/WS client has an independent reader position).

```
CAN TWAI ISR
     │
     ▼
 ring_buffer (128 entries × 160 bytes)
     │
     ├──► tcp_server_task (Telnet, port 23, ≤3 clients)
     └──► web_server (WebSocket, port 80, HTTP terminal UI)
                │
                └──► parse_command() — shared command handler
                                │
                                ├──► can_driver (TWAI + TJA1042 bus switching)
                                └──► NVS (WiFi config, CAN config)
```

### Key Components

| Component | Location | Responsibility |
|-----------|----------|----------------|
| CAN Driver | `components/can_driver/` | TWAI abstraction, multi-bus GPIO switching (MS/HS/MM via STB pins) |
| Ring Buffer | `main/ring_buffer.c` | Thread-safe circular buffer for CAN frames |
| Main App | `main/can_sniffer_main.c` | WiFi manager, TCP server, command parser, statistics |
| Web Server | `main/web_server.c` | HTTP + WebSocket server, embedded terminal UI, OTA updates |
| LED Indicator | `main/led_indicator.c` | WS2812B state machine (WiFi/CAN activity states) |

### Command Interface

Both Telnet (port 23) and WebSocket (port 80) share the same `parse_command()` function. Output uses candump format: `(timestamp) can0 ID#DATA`.

Key commands: `CANBUS <MS|HS|MM|OFF>`, `BITRATE <bus> <kbps>`, `MODE <LISTEN|NORMAL>`, `SEND <ID> [bytes]`, `WIFI <ssid> <pass>`, `WIFIMODE <AP|STA>`, `STATUS`, `PING`, `REBOOT`.

**`parse_command()` response buffer** is `char response[1024]` (stack-allocated). Every command response — including the full HELP text — must fit within 1024 bytes. When adding a new command, verify the HELP text still fits: count all bytes in the HELP `snprintf` format string and ensure the total stays below 1024. If it doesn't fit, increase the buffer size and update this note.

### WiFi Startup Logic

STA credentials are resolved in priority order: compile-time Kconfig → NVS → AP fallback (SSID: `CAN-Sniffer`, pass: `can12345`, IP: `192.168.4.1`).

### NVS Namespaces

- `wifi_config` — SSID, password
- `can_config` — bitrates per bus, active bus, listen-only flag

### Kconfig Options (`main/Kconfig.projbuild`)

Important runtime defaults configurable at build time: TCP port, max clients, WiFi credentials, GPIO pin assignments for TWAI TX/RX and STB pins per bus (HS/MS/MM), default bitrates, WebSocket reconnect history depth.

### CAN Bus Switching

Three TJA1042 transceivers share a single TWAI peripheral. Bus switching (`CANBUS` command) tears down the TWAI driver, switches STB GPIO output, and reinitializes TWAI — there is no hot-switching. All buses start in listen-only mode by default.

### OTA Updates

`POST /ota` on the HTTP server accepts a raw firmware binary. Progress is streamed back over the same connection. After success, device reboots automatically.
