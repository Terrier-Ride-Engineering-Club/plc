import serial
import struct
import time

PORT = "/dev/tty.usbmodem48CA432F71742"
BAUD = 115200
LOOP_INTERVAL = 0.02
WATCHDOG_TIMEOUT = 0.05

RIDE_STATES = {0: "OFF", 1: "IDLE", 2: "RUNNING", 3: "STOPPING",
               4: "RESETTING", 5: "ESTOP", 6: "FAULT", 7: "MAINT"}

rcc_counter = 0
last_plc_counter = 0
last_valid_time = time.monotonic()
plc_healthy = False

# Last decoded PLC → RCC packet fields
plc_status_bits  = 0
plc_ctr          = 0
plc_echo_ctr     = 0

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
            my_counter, your_counter, status, _, __ = struct.unpack("<HHBBH", rx[:8])

            last_plc_counter = my_counter
            last_valid_time = time.monotonic()
            plc_healthy     = True
            plc_status_bits = status
            plc_ctr         = my_counter
            plc_echo_ctr    = your_counter

    if time.monotonic() - last_valid_time > WATCHDOG_TIMEOUT:
        plc_healthy = False

    # --------------------
    # BUILD RCC PACKET
    # --------------------

    status_bits  = 0x02  # I'm OK
    ride_state   = 1
    m1_speed     = 0
    m2_speed     = 0
    voltage_raw  = 240   # 0.1V units → 48.0V
    tx_limit_sw  = 0

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

        m1_speed, 0, 0,             # M1 feedback
        m2_speed, 0, 0,             # M2 feedback
        voltage_raw, 0, 0,          # Voltage, MC status, time since update

        0, 0, 0, 0,                 # M1 command
        0, 0, 0, 0,                 # M2 command

        ride_state,
        tx_limit_sw
    )

    crc = crc16(packet_without_crc)
    packet = packet_without_crc + struct.pack("<H", crc)

    ser.write(packet)


    rcc_counter = (rcc_counter + 1) & 0xFFFF

    # --------------------
    # Status Summary
    # --------------------
    plc_checks_ok  = bool(plc_status_bits & 0x02)
    plc_estop      = not bool(plc_status_bits & 0x01)
    plc_wdog_fault = bool(plc_status_bits & 0x04)
    plc_lim_fault  = bool(plc_status_bits & 0x08)
    plc_motion     = bool(plc_status_bits & 0x10)
    plc_fault      = bool(plc_status_bits & 0x20)

    # comm: serial link health in both directions
    #   OK       — sim hears PLC and PLC hears RCC
    #   RCC→PLC  — sim hears PLC but PLC reports it isn't hearing us
    #   PLC→RCC  — sim stopped hearing PLC
    if not plc_healthy:
        comm_str = "PLC→RCC:LOST"
    elif plc_wdog_fault:
        comm_str = "RCC→PLC:LOST"
    else:
        comm_str = "OK"

    faults = []
    if plc_lim_fault: faults.append("LIM_MISMATCH")
    if plc_motion:    faults.append("MOTION_POST_ESTOP")
    if plc_fault:     faults.append("LEVEL0_FAULT")
    fault_str = ",".join(faults) if faults else "none"

    print(
        f"TX  ctr={rcc_counter:5d}  state={RIDE_STATES.get(ride_state,'?'):9s}  M1={m1_speed:6d}  M2={m2_speed:6d}  lim={tx_limit_sw:04b}  V={voltage_raw/10:.1f}V"
        f"  |  "
        f"RX  ctr={plc_ctr:5d}  echo={plc_echo_ctr:5d}  comm={comm_str:12s}  checks={'OK' if plc_checks_ok else 'FAIL'}  estop={'Y' if plc_estop else 'N'}"
        f"  faults=[{fault_str}]"
        f"\033[K",
        end='\n', flush=True
    )

    # --------------------
    # 10ms Discipline
    # --------------------
    elapsed = time.monotonic() - loop_start
    sleep_time = LOOP_INTERVAL - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)
