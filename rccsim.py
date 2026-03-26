import serial
import struct
import time

PORT = "/dev/tty.usbmodem48CA432F71742"
BAUD = 115200
LOOP_INTERVAL = 0.02
WATCHDOG_TIMEOUT = 0.05

RIDE_STATES = {0: "OFF", 1: "IDLE", 2: "RUNNING", 3: "STOPPING",
               4: "RESETTING", 5: "ESTOP", 6: "FAULT", 7: "MAINT"}

# ---------------------------------------------------------------------------
# Test Scenario Framework
#
# Each scenario is a dict:
#   name        : short label shown in status line
#   desc        : longer description of what fault is being injected
#   duration_s  : seconds to hold this scenario before advancing (None = hold forever)
#   mutate      : callable(fields: dict) -> None   — modify fields in-place
#
# `fields` keys match the packet fields built in build_packet().
# A scenario need only touch the fields it wants to corrupt; defaults are normal.
# ---------------------------------------------------------------------------

def _scenario_normal(_f):
    """Baseline — no faults injected."""
    pass

def _scenario_bad_echo_counter(f):
    """Echo the wrong PLC counter (off by a large constant)."""
    f["echo_counter"] = (f["echo_counter"] + 1000) & 0xFFFF

def _scenario_stale_echo_counter(f):
    """Always echo counter 0 — simulates frozen mirror."""
    f["echo_counter"] = 0

def _scenario_bad_crc(f):
    """Force the CRC to be wrong — packet should be dropped by PLC."""
    f["corrupt_crc"] = True

def _scenario_watchdog_silence(f):
    """Stop sending packets entirely — PLC watchdog should fire."""
    f["drop_packet"] = True

def _scenario_limit_mismatch(f):
    """Report limit switches that disagree with PLC's own readings."""
    # Report all limit switches as triggered; PLC sees none triggered.
    f["tx_limit_sw"] = 0b1100

def _scenario_overspeed(f):
    """Report motor speed outside the safe envelope."""
    f["m1_speed"] = 50000   # large QPPS value

def _scenario_motion_in_estop(f):
    """Report non-zero motor speed while reporting ESTOP state."""
    f["ride_state"] = 5     # ESTOP
    f["m1_speed"] = 1000    # but motor is still moving


SCENARIOS = [
    # {"name": "NORMAL",            "desc": "Baseline — healthy operation",                "duration_s": 2,    "mutate": _scenario_normal},
    # {"name": "BAD_ECHO_CTR",      "desc": "Echo wrong PLC counter (+1000)",              "duration_s": 3,    "mutate": _scenario_bad_echo_counter},
    # {"name": "STALE_ECHO_CTR",    "desc": "Always echo counter 0 (frozen mirror)",       "duration_s": 3,    "mutate": _scenario_stale_echo_counter},
    # {"name": "NORMAL_RECOVERY", "desc": "Resume normal after silence", "duration_s": 1, "mutate": _scenario_normal},
    # {"name": "BAD_CRC",           "desc": "Corrupt the packet CRC",                      "duration_s": 3,    "mutate": _scenario_bad_crc},
    # {"name": "NORMAL_RECOVERY",   "desc": "Resume normal after silence",                 "duration_s": 1,    "mutate": _scenario_normal},
    # {"name": "WDOG_SILENCE",      "desc": "Stop sending packets (watchdog timeout)",     "duration_s": 3,    "mutate": _scenario_watchdog_silence},
    # {"name": "NORMAL_RECOVERY",   "desc": "Resume normal after silence",                 "duration_s": 1,    "mutate": _scenario_normal},
    # {"name": "LIM_MISMATCH",      "desc": "Report limit switches disagreeing with PLC",  "duration_s": 5,    "mutate": _scenario_limit_mismatch},
    {"name": "NORMAL_RECOVERY",   "desc": "Resume normal after silence",                 "duration_s": 1,    "mutate": _scenario_normal},
    {"name": "OVERSPEED",         "desc": "Report motor speed beyond safe envelope",     "duration_s": 5,    "mutate": _scenario_overspeed},
    {"name": "NORMAL_RECOVERY",   "desc": "Resume normal after silence",                 "duration_s": 1,    "mutate": _scenario_normal},
    {"name": "MOTION_IN_ESTOP",   "desc": "Report motion while in ESTOP state",          "duration_s": 5,    "mutate": _scenario_motion_in_estop},
    {"name": "NORMAL_FINAL",      "desc": "Return to healthy — end of test sequence",    "duration_s": None, "mutate": _scenario_normal},
]


class ScenarioRunner:
    def __init__(self, scenarios):
        self.scenarios = scenarios
        self.index = 0
        self._start = time.monotonic()

    def current(self):
        return self.scenarios[self.index]

    def tick(self):
        s = self.current()
        if s["duration_s"] is not None:
            if time.monotonic() - self._start >= s["duration_s"]:
                self.index = (self.index + 1) % len(self.scenarios)
                self._start = time.monotonic()

    def elapsed(self):
        s = self.current()
        e = time.monotonic() - self._start
        total = s["duration_s"]
        return e, total


def build_fields(tx_counter, echo_ctr):
    """Return default (healthy) packet field dict."""
    return {
        "counter":          tx_counter,
        "echo_counter":     echo_ctr,
        "status_bits":      0x02,   # I'm OK
        "m1_speed":         0,
        "m1_enc":           0,
        "m1_current":       0,
        "m2_speed":         0,
        "m2_enc":           0,
        "m2_current":       0,
        "voltage_raw":      240,    # 0.1V units → 24.0V
        "mc_status":        0,
        "mc_time_update":   0,
        "m1_cmd_pos":       0,
        "m1_cmd_speed":     0,
        "m1_cmd_accel":     0,
        "m1_cmd_decel":     0,
        "m2_cmd_pos":       0,
        "m2_cmd_speed":     0,
        "m2_cmd_accel":     0,
        "m2_cmd_decel":     0,
        "ride_state":       2,      # RUNNING
        "tx_limit_sw":      0b1111,
        # injection flags (not packed directly)
        "corrupt_crc":      False,
        "drop_packet":      False,
    }


def pack_packet(f):
    body = struct.pack(
        "<HHB"      # counter, echo_counter, status_bits
        "i i h"     # M1 speed, enc, current
        "i i h"     # M2 speed, enc, current
        "H I H"     # voltage, mc_status, mc_time_update
        "i i I I"   # M1 cmd pos, speed, accel, decel
        "i i I I"   # M2 cmd pos, speed, accel, decel
        "B B",      # ride_state, tx_limit_sw
        f["counter"],       f["echo_counter"],  f["status_bits"],
        f["m1_speed"],      f["m1_enc"],        f["m1_current"],
        f["m2_speed"],      f["m2_enc"],        f["m2_current"],
        f["voltage_raw"],   f["mc_status"],     f["mc_time_update"],
        f["m1_cmd_pos"],    f["m1_cmd_speed"],  f["m1_cmd_accel"],  f["m1_cmd_decel"],
        f["m2_cmd_pos"],    f["m2_cmd_speed"],  f["m2_cmd_accel"],  f["m2_cmd_decel"],
        f["ride_state"],    f["tx_limit_sw"],
    )
    crc = crc16(body)
    if f["corrupt_crc"]:
        crc ^= 0xFFFF
    return body + struct.pack("<H", crc)


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


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

ser = serial.Serial(PORT, BAUD, timeout=0)
runner = ScenarioRunner(SCENARIOS)

rcc_counter     = 0
last_plc_counter = 0
last_valid_time  = time.monotonic()
plc_healthy      = False
plc_status_bits  = 0
plc_ctr          = 0
plc_echo_ctr     = 0

while True:
    loop_start = time.monotonic()
    runner.tick()
    scenario = runner.current()

    # --------------------
    # RECEIVE PLC PACKET
    # --------------------
    if ser.in_waiting >= 10:
        rx = ser.read(10)
        received_crc = struct.unpack("<H", rx[8:10])[0]
        if received_crc == crc16(rx[:8]):
            my_counter, your_counter, status, _, __ = struct.unpack("<HHBBH", rx[:8])
            last_plc_counter = my_counter
            last_valid_time  = time.monotonic()
            plc_healthy      = True
            plc_status_bits  = status
            plc_ctr          = my_counter
            plc_echo_ctr     = your_counter

    if time.monotonic() - last_valid_time > WATCHDOG_TIMEOUT:
        plc_healthy = False

    # --------------------
    # BUILD + INJECT + SEND
    # --------------------
    fields = build_fields(rcc_counter, last_plc_counter)  # noqa: shadowing is intentional
    scenario["mutate"](fields)

    if not fields["drop_packet"]:
        ser.write(pack_packet(fields))

    rcc_counter = (rcc_counter + 1) & 0xFFFF

    # --------------------
    # Status line
    # --------------------
    plc_checks_ok  = bool(plc_status_bits & 0x02)
    plc_estop      = not bool(plc_status_bits & 0x01)
    plc_wdog_fault = bool(plc_status_bits & 0x04)
    plc_lim_fault  = bool(plc_status_bits & 0x08)
    plc_motion     = bool(plc_status_bits & 0x10)
    plc_fault      = bool(plc_status_bits & 0x20)
    plc_echo_fault = bool(plc_status_bits & 0x40)
    plc_motion_fault = bool(plc_status_bits & 0x80)


    if not plc_healthy:
        comm_str = "PLC→RCC:LOST"
    elif plc_wdog_fault:
        comm_str = "RCC→PLC:LOST"
    else:
        comm_str = "OK"

    faults = []
    if plc_lim_fault:  faults.append("LIM_MISMATCH")
    if plc_echo_fault: faults.append("ECHO_CTR_FAULT")
    if plc_motion:     faults.append("MOTION_POST_ESTOP")
    if plc_fault:      faults.append("LEVEL0_FAULT")
    if plc_motion_fault: faults.append("MOTION_FAULT")
    fault_str = ",".join(faults) if faults else "none"

    sc_elapsed, sc_total = runner.elapsed()
    timer_str = f"{sc_elapsed:.1f}/{sc_total:.0f}s" if sc_total else f"{sc_elapsed:.1f}s"

    print(
        f"[{scenario['name']:16s} {timer_str:8s}]  "
        f"TX ctr={fields['counter']:5d}  echo={fields['echo_counter']:5d}  state={RIDE_STATES.get(fields['ride_state'],'?'):9s}"
        f"  M1={fields['m1_speed']:6d}  lim={fields['tx_limit_sw']:04b}"
        f"  |  "
        f"RX ctr={plc_ctr:5d}  echo={plc_echo_ctr:5d}  comm={comm_str:12s}"
        f"  checks={'OK' if plc_checks_ok else 'FAIL'}  estop={'Y' if plc_estop else 'N'}"
        f"  faults=[{fault_str}]"
        f"\033[K",
        end='\r', flush=True
    )

    # --------------------
    # 20ms discipline
    # --------------------
    elapsed_loop = time.monotonic() - loop_start
    sleep_time = LOOP_INTERVAL - elapsed_loop
    if sleep_time > 0:
        time.sleep(sleep_time)
