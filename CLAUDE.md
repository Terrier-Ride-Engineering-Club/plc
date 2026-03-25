# Safety PLC — Project Context

## System Overview

Safety PLC for an amusement drop tower ride. Runs on an Arduino Nano. Independently verifies that the Ride Control Computer (RCC, a Raspberry Pi 5) is operating safely. The PLC does NOT command motors or control ride state — it only monitors and can cut power/assert E-stop if it detects a violation.

Power flows: 120VAC wall → fused IEC inlet → 24VDC supply → 12VDC supply → Safety PLC.
The PLC controls downstream power via two relays:
- **24V DC relay**: powers the RCC and Motor Controller (enabled first, on key ON)
- **120VAC relay**: powers the Ride Control Panel buttons (enabled after RCC handshake confirmed)

The Motor Controller's E-stop input is a series circuit gated by three independent sources: PLC, RCC, and physical E-stop pushbutton. Any one source can assert it independently.

## Hardware

**Platform:** Arduino Nano (ATmega328P)


**Outputs:**
| Pin | Signal | Active State | Notes |
|-----|--------|-------------|-------|
| D2  | Power relay enable | HIGH = relay on | Energize on key ON; controls 24V and 120VAC relays |
| D10 | E-stop control | LOW = E-stop asserted | Normally HIGH; pulled LOW to assert E-stop |

Both the 24V and 120VAC relays are controlled together by D2 (single control). Staged power-on is not possible in hardware; both relays energize simultaneously when the key turns ON. The E-stop is still held by D10 until handshake is confirmed, so panel buttons being powered early is safe.

**Inputs:**
| Pin | Signal | Active State | Notes |
|-----|--------|-------------|-------|
| D3  | ON key switch | HIGH = ON | Mutually exclusive with D4 |
| D4  | MAINTENANCE key switch | HIGH = MAINT | Mutually exclusive with D3 |
| D6  | Tower 1 bottom limit switch | HIGH = triggered | Normally open |
| D7  | Tower 1 top limit switch | HIGH = triggered | Normally open |
| D8  | Tower 2 bottom limit switch | HIGH = triggered | Normally open |
| D9  | Tower 2 top limit switch | HIGH = triggered | Normally open |

## Communication: PLC ↔ RCC Watchdog

**Physical layer:** UART over USB, 115200 baud, 10ms cycle

**RCC → PLC Packet (69 bytes):**
| Offset | Field | Type | Notes |
|--------|-------|------|-------|
| 0 | My Counter | uint16 | Wraps at 65535 |
| 2 | Your Counter | uint16 | Echo of last PLC counter |
| 4 | Status Bits | uint8 | Bit 1 = I'm OK; Bit 0/7 = E-stop status |
| 5 | M1 Speed | int32 | Signed QPPS (pos = fwd) |
| 9 | M1 Encoder | int32 | Signed encoder count |
| 13 | M1 Current | int16 | Raw 0.01A units |
| 15 | M2 Speed | int32 | Signed QPPS |
| 19 | M2 Encoder | int32 | Signed encoder count |
| 23 | M2 Current | int16 | Raw 0.01A units |
| 25 | MC Voltage | uint16 | 0.1V units |
| 27 | MC Status | uint32 | Raw RoboClaw status register |
| 31 | MC Time Since Update | uint16 | ms since last valid MC read; 0xFFFF = never |
| 33 | M1 Cmd Position | int32 | Target encoder count |
| 37 | M1 Cmd Speed | int32 | Target QPPS |
| 41 | M1 Cmd Accel | uint32 | QPPS/s |
| 45 | M1 Cmd Decel | uint32 | QPPS/s |
| 49 | M2 Cmd Position | int32 | Target encoder count |
| 53 | M2 Cmd Speed | int32 | Target QPPS |
| 57 | M2 Cmd Accel | uint32 | QPPS/s |
| 61 | M2 Cmd Decel | uint32 | QPPS/s |
| 65 | Ride State | uint8 | See RCC states below |
| 66 | Limit Switches | uint8 | Bits 0–3; RCC's own reading for cross-check |
| 67 | CRC16 | uint16 | ANSI CRC16, poly 0x8005 |

**PLC → RCC Packet (10 bytes):**
| Offset | Field | Type | Notes |
|--------|-------|------|-------|
| 0 | My Counter | uint16 | Wraps at 65535 |
| 2 | Your Counter | uint16 | Echo of last RCC counter |
| 4 | Status Bits | uint8 | Bit 1 = I'm OK; Bit 0/7 = E-stop status |
| 5 | Limit Switches | uint8 | PLC's own reading for cross-check |
| 6 | Reserved | uint16 | Future use |
| 8 | CRC16 | uint16 | ANSI CRC16, poly 0x8005 |

**Watchdog:** Both sides maintain a 50ms timeout. If no valid packet received in 50ms, the other side is marked unhealthy.

**CRC16 algorithm (ANSI, poly 0x8005):**
```
crc = 0x0000
for each byte:
    crc ^= (byte << 8)
    repeat 8 times:
        if crc & 0x8000: crc = (crc << 1) ^ 0x8005
        else: crc = crc << 1
    crc &= 0xFFFF
```

## RCC Ride States

| Value | State | Description |
|-------|-------|-------------|
| 0 | OFF | Key at OFF; awaiting power-on |
| 1 | IDLE | Powered and ready for operator input |
| 2 | RUNNING | Ride actively executing sequence |
| 3 | STOPPING | Controlled return to home (7s timeout → ESTOP) |
| 4 | RESETTING | 1-second fault-check window after E-stop reset |
| 5 | ESTOP | All motion halted; requires reset to clear |
| 6 | FAULT | Safety PLC has cut ride power (terminal until power cycle) |
| 7 | MAINTENANCE | Maintenance jog mode; dispatch unavailable |

## Stop Escalation Levels

| Level | State | Motors Powered | Electronics | Motion | Recovery |
|-------|-------|---------------|-------------|--------|----------|
| 0 | FAULT | No | Logic only | Inhibited | Full power cycle |
| 1 | E-STOP | Yes | Full | Inhibited | Reset button |
| 2 | STOPPING | Yes | Full | Active (homing) | Automatic → IDLE |

- **Level 2**: Operator presses STOP; ride decelerates and returns to loading position. If not homed within 7s, escalates to Level 1.
- **Level 1**: Physical E-stop, HIGH severity fault, or PLC-detected violation. Motor controller E-stop asserted. System stays energized for diagnostics.
- **Level 0**: PLC detects continued motion after Level 1 E-stop. PLC physically cuts motor power relay. Indicates potential hardware failure. Terminal — requires power cycle and inspection.

## Startup Sequence

1. Breaker ON → PLC powers up, begins monitoring. Everything else off.
2. Operator turns key to ON → PLC enables 24V relay (D2 HIGH). RCC cold-boots (30–60s).
3. RCC establishes watchdog handshake → PLC enables 120VAC relay (panel buttons).
4. System starts in E-STOP. All three sources (RCC, PLC, hardware circuit) must independently confirm OK before E-stop releases and system transitions to IDLE.
5. Motor controller may briefly show comm fault on first boot (expected; auto-clears).

## Safety Verification (PLC Responsibilities)

The PLC continuously:
- Verifies RCC watchdog is healthy (50ms timeout)
- Cross-checks its own limit switch readings against RCC-reported readings
- Verifies motor position/speed/acceleration are within the defined envelope
- Monitors for continued motion after E-stop assertion (→ Level 0)

Specific envelope thresholds and fault conditions: TBD (separate spec in progress).

## Maintenance Mode

- Accessed by turning key to MAINTENANCE from OFF (not from ON).
- Automated ride logic disabled; operator can jog gondolas manually at low speed.
- Gondolas stop individually on limit switch contact.
- E-stop and reset still function normally.
- Exiting maintenance mode or releasing jog input stops all motion immediately.

## Key Design Principles

- **Simple, readable code** — this is safety-critical; clarity over cleverness
- **Fail-safe defaults** — outputs de-energized on power loss, startup in safe state
- **No dynamic memory allocation** on the Arduino
- **Minimize global mutable state**
- The PLC is an independent verifier only — it does not command motors or set ride state
