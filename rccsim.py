import serial
import struct
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200
LOOP_INTERVAL = 0.01
WATCHDOG_TIMEOUT = 0.05

rcc_counter = 0
last_plc_counter = 0
last_valid_time = time.monotonic()
plc_healthy = False

def crc16(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

ser = serial.Serial(PORT, BAUD, timeout=0)

while True:
    loop_start = time.monotonic()

    # --------------------
    # RECEIVE PLC PACKET
    # --------------------
    if ser.in_waiting >= 10:
        rx = ser.read(10)

        received_crc = struct.unpack("<H", rx[8:10])[0]
        calc_crc = crc16(rx[:8])

        if received_crc == calc_crc:
            my_counter, your_counter, status, limit_sw, _ = struct.unpack("<HHBBH", rx[:8])

            last_plc_counter = my_counter
            last_valid_time = time.monotonic()
            plc_healthy = True

    if time.monotonic() - last_valid_time > WATCHDOG_TIMEOUT:
        plc_healthy = False

    # --------------------
    # BUILD RCC PACKET
    # --------------------

    status_bits = 0x02  # I'm OK

    packet_without_crc = struct.pack(
        "<HHB"      # Counters + Status
        "i i h"     # M1 speed, enc, current
        "i i h"     # M2 speed, enc, current
        "H I H"     # Voltage, status, time since update
        "i i I I"   # M1 cmd
        "i i I I"   # M2 cmd
        "B B",      # Ride state, limit switches
        rcc_counter,
        last_plc_counter,
        status_bits,

        0, 0, 0,        # M1 feedback
        0, 0, 0,        # M2 feedback
        480, 0, 0,      # Voltage, MC status, time since update

        0, 0, 0, 0,     # M1 command
        0, 0, 0, 0,     # M2 command

        0,              # Ride state
        0               # Limit switches
    )

    crc = crc16(packet_without_crc)
    packet = packet_without_crc + struct.pack("<H", crc)

    ser.write(packet)

    rcc_counter = (rcc_counter + 1) & 0xFFFF

    # --------------------
    # 10ms Discipline
    # --------------------
    elapsed = time.monotonic() - loop_start
    sleep_time = LOOP_INTERVAL - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)
