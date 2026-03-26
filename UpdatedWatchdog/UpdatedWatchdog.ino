#include <Arduino.h>

// -- Pin Definitions ----------------------------------------------------------
#define PIN_RELAY       2   // Power relay, active HIGH
#define PIN_KEY_ON      3   // ON key switch, HIGH = ON (requires external pulldown)
#define PIN_KEY_MAINT   4   // MAINT key switch, HIGH = MAINT (requires external pulldown)
#define PIN_LIM_T1_BOT  6   // Tower 1 bottom limit (NC + INPUT_PULLUP; open wire = triggered)
#define PIN_LIM_T1_TOP  7   // Tower 1 top limit    (NC + INPUT_PULLUP)
#define PIN_LIM_T2_BOT  8   // Tower 2 bottom limit (NC + INPUT_PULLUP)
#define PIN_LIM_T2_TOP  9   // Tower 2 top limit    (NC + INPUT_PULLUP)
#define PIN_ESTOP       10  // E-stop control, active LOW (LOW = asserted)

// -- Protocol Constants -------------------------------------------------------
#define BAUD            115200
#define RCC_PACKET_SIZE 69
#define PLC_PACKET_SIZE 10
#define WATCHDOG_MS     50   // Max ms between valid RCC packets before unhealthy

// RCC ride states (rcc.rideState field)
#define RCC_STATE_OFF         0  // Key switch at OFF; awaiting power-on
#define RCC_STATE_IDLE        1  // Powered and ready for operator input
#define RCC_STATE_RUNNING     2  // Ride actively executing sequence
#define RCC_STATE_STOPPING    3  // Controlled return to home/loading position (7s timeout → ESTOP)
#define RCC_STATE_RESETTING   4  // 1-second fault-check window after E-stop reset is pressed
#define RCC_STATE_ESTOP       5  // All motion halted; requires reset to clear
#define RCC_STATE_FAULT       6  // Safety PLC has cut ride power
#define RCC_STATE_MAINTENANCE 7  // Maintenance jog mode; dispatch unavailable

// -- Safety Parameters --------------------------------------------------------
// After E-stop is asserted, motors must stop within this window.
// If still moving after this, escalate to Level 0 (FAULT).
// TODO: confirm acceptable deceleration time with hardware team.
#define MOTION_STOP_TIMEOUT_MS 2000
// Max cycles the RCC's echo of our counter may lag before flagging a fault.
// At 20ms/cycle this is 100ms — enough to absorb one dropped packet.
#define ECHO_LAG_LIMIT 5
// The speed a motor can be at during an E-Stop before a Cat 0 Stop is thrown
#define ESTOP_MOTOR_MOVE_THRESHOLD 5
#define MAX_MOTOR_SPEED 9500  // QPPS; triggers FAULT_MOTION if exceeded


// -- PLC State Machine --------------------------------------------------------
enum PlcState {
  STATE_OFF,      // Key at OFF — relay de-energized, everything dark
  STATE_STARTING, // Key ON — relay on, awaiting first valid RCC handshake
  STATE_OK,       // Watchdog healthy, no violations — PLC E-stop gate open
  STATE_ESTOP,    // PLC asserting Level 1 E-stop
  STATE_FAULT,    // Level 0 — relay cut, recoverable by key cycle to OFF
  STATE_MAINT     // Maintenance mode — automated ride logic disabled
};

// -- RCC Packet Data ----------------------------------------------------------
// Fields extracted from the most recent valid RCC → PLC packet.
struct RccPacket {
  uint16_t myCounter;          // offset 0  — RCC's own counter
  uint16_t echoOfPlc;          // offset 2  — RCC's echo of our counter
  uint8_t  statusBits;         // offset 4
  int32_t  m1Speed;            // offset 5
  int32_t  m1Encoder;          // offset 9
  int16_t  m1Current;          // offset 13
  int32_t  m2Speed;            // offset 15
  int32_t  m2Encoder;          // offset 19
  int16_t  m2Current;          // offset 23
  uint16_t mcVoltage;          // offset 25
  uint32_t mcStatus;           // offset 27
  uint16_t mcTimeSinceUpdate;  // offset 31
  int32_t  m1CmdPos;           // offset 33
  int32_t  m1CmdSpeed;         // offset 37
  uint32_t m1CmdAccel;         // offset 41
  uint32_t m1CmdDecel;         // offset 45
  int32_t  m2CmdPos;           // offset 49
  int32_t  m2CmdSpeed;         // offset 53
  uint32_t m2CmdAccel;         // offset 57
  uint32_t m2CmdDecel;         // offset 61
  uint8_t  rideState;          // offset 65
  uint8_t  limitSwitches;      // offset 66
};

// -- Global State -------------------------------------------------------------
static PlcState      plcState          = STATE_OFF;
static uint16_t      plcCounter        = 0;
static unsigned long lastValidPacketMs = 0;
static unsigned long estopAssertedMs   = 0;
static bool          rccHealthy        = false;
static bool          badCrcFlag        = false; // Set on CRC failure; cleared on good packet
static RccPacket     rcc               = {}; // Most recent valid RCC packet

// -- CRC16 — ANSI, poly 0x8005 ------------------------------------------------
static uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : crc << 1;
    }
  }
  return crc;
}

// -- I/O Helpers --------------------------------------------------------------
static void setRelay(bool on)       { digitalWrite(PIN_RELAY, on ? HIGH : LOW); }
static void setEstop(bool asserting) { digitalWrite(PIN_ESTOP, asserting ? LOW : HIGH); }

// Returns limit switch state packed into bits 0–3: T1_BOT, T1_TOP, T2_BOT, T2_TOP
static uint8_t readLimitSwitches() {
  uint8_t sw = 0;
  if (digitalRead(PIN_LIM_T1_BOT)) sw |= 0x01;
  if (digitalRead(PIN_LIM_T1_TOP)) sw |= 0x02;
  if (digitalRead(PIN_LIM_T2_BOT)) sw |= 0x04;
  if (digitalRead(PIN_LIM_T2_TOP)) sw |= 0x08;
  return sw;
}

// -- Packet Handling ----------------------------------------------------------
static bool parseRccPacket(const uint8_t *buf) {
  uint16_t rxCrc = (uint16_t)buf[67] | ((uint16_t)buf[68] << 8);
  if (crc16(buf, 67) != rxCrc) return false;

  rcc.myCounter        = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  rcc.echoOfPlc        = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
  rcc.statusBits       = buf[4];
  memcpy(&rcc.m1Speed,          buf + 5,  sizeof(rcc.m1Speed));
  memcpy(&rcc.m1Encoder,        buf + 9,  sizeof(rcc.m1Encoder));
  memcpy(&rcc.m1Current,        buf + 13, sizeof(rcc.m1Current));
  memcpy(&rcc.m2Speed,          buf + 15, sizeof(rcc.m2Speed));
  memcpy(&rcc.m2Encoder,        buf + 19, sizeof(rcc.m2Encoder));
  memcpy(&rcc.m2Current,        buf + 23, sizeof(rcc.m2Current));
  memcpy(&rcc.mcVoltage,        buf + 25, sizeof(rcc.mcVoltage));
  memcpy(&rcc.mcStatus,         buf + 27, sizeof(rcc.mcStatus));
  memcpy(&rcc.mcTimeSinceUpdate,buf + 31, sizeof(rcc.mcTimeSinceUpdate));
  memcpy(&rcc.m1CmdPos,         buf + 33, sizeof(rcc.m1CmdPos));
  memcpy(&rcc.m1CmdSpeed,       buf + 37, sizeof(rcc.m1CmdSpeed));
  memcpy(&rcc.m1CmdAccel,       buf + 41, sizeof(rcc.m1CmdAccel));
  memcpy(&rcc.m1CmdDecel,       buf + 45, sizeof(rcc.m1CmdDecel));
  memcpy(&rcc.m2CmdPos,         buf + 49, sizeof(rcc.m2CmdPos));
  memcpy(&rcc.m2CmdSpeed,       buf + 53, sizeof(rcc.m2CmdSpeed));
  memcpy(&rcc.m2CmdAccel,       buf + 57, sizeof(rcc.m2CmdAccel));
  memcpy(&rcc.m2CmdDecel,       buf + 61, sizeof(rcc.m2CmdDecel));
  rcc.rideState        = buf[65];
  rcc.limitSwitches    = buf[66];
  return true;
}

static void sendPlcPacket(uint8_t statusBits) {
  uint8_t tx[PLC_PACKET_SIZE];
  memcpy(tx + 0, &plcCounter,      2);
  memcpy(tx + 2, &rcc.myCounter,  2);
  tx[4] = statusBits;
  tx[5] = 0; tx[6] = 0; tx[7] = 0;  // reserved
  uint16_t crc = crc16(tx, 8);
  memcpy(tx + 8, &crc, 2);
  Serial.write(tx, PLC_PACKET_SIZE);
  plcCounter++;
}

// -- LED Status Indicator -----------------------------------------------------
//  OFF      — LED off            (system dark)
//  STARTING — 1 Hz blink         (waiting for RCC handshake)
//  OK       — solid on           (healthy, E-stop released)
//  ESTOP    — 2 Hz blink         (Level 1 E-stop asserted)
//  FAULT    — 5 Hz blink         (Level 0 fault, recoverable by key cycle)
//  MAINT    — double-pulse / sec (maintenance mode)
static void updateLed(unsigned long now) {
  bool on;
  switch (plcState) {
    case STATE_OFF:      on = false;                                              break;
    case STATE_STARTING: on = (now % 1000) < 500;                                break;
    case STATE_OK:       on = true;                                               break;
    case STATE_ESTOP:    on = (now % 500)  < 250;                                break;
    case STATE_FAULT:    on = (now % 200)  < 100;                                break;
    case STATE_MAINT:  { unsigned long t = now % 1000;
                         on = (t < 100) || (t >= 200 && t < 300);               break; }
    default:             on = false;
  }
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

// -- Status Byte (PLC → RCC, offset 4) ---------------------------------------
//
// Status bits — reflect current PLC state (live each cycle):
#define STATUS_ESTOP_RELEASED   0x01  // PLC E-stop gate open (not asserting)
#define STATUS_PLC_OK           0x02  // No active faults; in ESTOP this means safe to attempt reset
//
// Fault bits — active conditions only, computed live each cycle.
// Multiple faults are OR'd together:
#define FAULT_WATCHDOG          0x04  // RCC packets stopped
#define FAULT_LIMIT_MISMATCH    0x08  // PLC and RCC limit switch readings disagree
#define FAULT_BAD_CRC           0x10  // RCC packet received but CRC failed
#define FAULT_LEVEL0            0x20  // Relay cut — recoverable by key cycle to OFF
#define FAULT_ECHO_COUNTER      0x40  // RCC not mirroring PLC counter
#define FAULT_MOTION            0x80  // Motion envelope violation (overspeed; position/accel checks TBD)

// -- Safety Checks ------------------------------------------------------------
static bool motorsMoving() {
  return (rcc.m1Speed != 0) || (rcc.m2Speed != 0);
}

// Returns a bitmask of all active fault bits, or 0 if all checks pass.
// Shared by safetyViolation() and enterEstop() to avoid duplicating logic.
// TODO: add position/acceleration envelope checks once thresholds are defined; set FAULT_MOTION.
// TODO: confirm limit switch bit ordering matches RCC packet offset 66 before re-enabling.
static uint8_t checkSafetyFaults(uint8_t plcLimits) {
  uint8_t faults = 0;
  if (plcLimits != rcc.limitSwitches)                                                                                                           faults |= FAULT_LIMIT_MISMATCH;
  int16_t echoLag = (int16_t)((plcCounter - 1) - rcc.echoOfPlc);
  if (echoLag < 0 || echoLag > ECHO_LAG_LIMIT)                                                                                                faults |= FAULT_ECHO_COUNTER;
  if (abs(rcc.m1Speed) > MAX_MOTOR_SPEED || abs(rcc.m2Speed) > MAX_MOTOR_SPEED)                                                                faults |= FAULT_MOTION;
  return faults;
}

static bool safetyViolation(uint8_t plcLimits) {
  return checkSafetyFaults(plcLimits) != 0;
}

// Transitions into STATE_ESTOP.
static void enterEstop(unsigned long now) {
  estopAssertedMs = now;
  plcState = STATE_ESTOP;
}

static uint8_t buildStatusByte(uint8_t plcLimits) {
  switch (plcState) {
    case STATE_OK:
    case STATE_MAINT:
      return STATUS_ESTOP_RELEASED | STATUS_PLC_OK;
    case STATE_ESTOP: {
      uint8_t faults = checkSafetyFaults(plcLimits);
      if (!rccHealthy) faults |= FAULT_WATCHDOG;
      if (badCrcFlag)  faults |= FAULT_BAD_CRC;
      // No active faults — signal to RCC that reset is safe to attempt
      if (faults == 0) return STATUS_PLC_OK;
      return faults;
    }
    case STATE_FAULT:
      return FAULT_LEVEL0;
    default:
      return 0;
  }
}

// -- Setup --------------------------------------------------------------------
void setup() {
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_ESTOP,      OUTPUT);
  pinMode(LED_BUILTIN,    OUTPUT);
  pinMode(PIN_KEY_ON,     INPUT);
  pinMode(PIN_KEY_MAINT,  INPUT);
  pinMode(PIN_LIM_T1_BOT, INPUT_PULLUP);
  pinMode(PIN_LIM_T1_TOP, INPUT_PULLUP);
  pinMode(PIN_LIM_T2_BOT, INPUT_PULLUP);
  pinMode(PIN_LIM_T2_TOP, INPUT_PULLUP);

  setRelay(false);  // Everything off at boot
  setEstop(true);   // E-stop asserted until system is ready
  Serial.begin(BAUD);
}

// -- Main Loop ----------------------------------------------------------------
void loop() {
  static unsigned long lastTxMs = 0;
  unsigned long now = millis();
  if (now - lastTxMs < 20) return;
  lastTxMs = now;

  // Flush any backlog, then block-wait for one fresh packet this cycle.
  while (Serial.available()) Serial.read();
  uint8_t rxBuf[RCC_PACKET_SIZE];
  uint8_t rxLen = 0;
  unsigned long waitStart = millis();
  while (rxLen < RCC_PACKET_SIZE) {
    if (Serial.available() > 0) rxBuf[rxLen++] = Serial.read();
    if (millis() - waitStart > WATCHDOG_MS) break;
  }
  now = millis();
  if (rxLen == RCC_PACKET_SIZE) {
    if (parseRccPacket(rxBuf)) {
      lastValidPacketMs = now;
      rccHealthy = true;
      badCrcFlag = false;
    } else {
      badCrcFlag = true;
    }
  }

  uint8_t plcLimits = readLimitSwitches();
  bool    keyOn     = (digitalRead(PIN_KEY_ON)    == HIGH);
  bool    keyMaint  = (digitalRead(PIN_KEY_MAINT) == HIGH);

  if (now - lastValidPacketMs > WATCHDOG_MS) rccHealthy = false;

  // -- State machine --
  switch (plcState) {

    case STATE_OFF:
      setRelay(false);
      setEstop(true);
      if      (keyOn)    { setRelay(true); plcState = STATE_STARTING; }
      else if (keyMaint) { setRelay(true); plcState = STATE_MAINT;    }
      break;

    case STATE_STARTING:
      // Relay on, E-stop held until RCC establishes a healthy watchdog link.
      setEstop(true);
      if (!keyOn)     { plcState = STATE_OFF; break; }
      if (rccHealthy) { plcState = STATE_OK;  }
      break;

    case STATE_OK:
      if (!keyOn) { plcState = STATE_OFF; break; }
      if (!rccHealthy || safetyViolation(plcLimits)) { enterEstop(now); break; }
      setEstop(false);  // All checks pass — open PLC E-stop gate
      break;

    case STATE_ESTOP:
      setEstop(true);
      if (!keyOn && !keyMaint) { plcState = STATE_OFF; break; }
      // Level 0: motors still moving after deceleration window → cut power
      if (motorsMoving() && (now - estopAssertedMs > MOTION_STOP_TIMEOUT_MS)) {
        plcState = STATE_FAULT;
        break;
      }
      // Operator-acknowledged recovery: fault condition cleared AND reset button pressed
      if (rccHealthy && !safetyViolation(plcLimits) && rcc.rideState == RCC_STATE_RESETTING) {
        badCrcFlag = false;
        plcState = keyOn ? STATE_OK : STATE_MAINT;
      }
      break;

    case STATE_FAULT:
      // Relay cut, E-stop held. Recoverable by turning key to OFF.
      setEstop(true);
      setRelay(false);
      if (!keyOn && !keyMaint) { plcState = STATE_OFF; }
      break;

    case STATE_MAINT:
      if (!keyMaint) { setEstop(true); plcState = STATE_OFF; break; }
      if (!rccHealthy || safetyViolation(plcLimits)) { enterEstop(now); break; }
      setEstop(false);
      break;
  }

  sendPlcPacket(buildStatusByte(plcLimits));

  updateLed(now);
}
