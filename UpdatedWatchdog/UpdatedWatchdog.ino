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
// The max speed a motor can be at during any point in operation of the ride
#define MAX_MOTOR_SPEED 9500

// -- PLC State Machine --------------------------------------------------------
enum PlcState {
  STATE_OFF,      // Key at OFF — relay de-energized, everything dark
  STATE_STARTING, // Key ON — relay on, awaiting first valid RCC handshake
  STATE_OK,       // Watchdog healthy, no violations — PLC E-stop gate open
  STATE_ESTOP,    // PLC asserting Level 1 E-stop
  STATE_FAULT,    // Level 0 — relay cut, terminal until full power cycle
  STATE_MAINT     // Maintenance mode — automated ride logic disabled
};

// -- Global State -------------------------------------------------------------
static PlcState      plcState          = STATE_OFF;
static uint16_t      plcCounter        = 0;
static uint16_t      lastRccCounter    = 0;
static unsigned long lastValidPacketMs = 0;
static unsigned long estopAssertedMs   = 0;
static bool          rccHealthy        = false;
static uint8_t       latchedFaults     = 0;  // Latched at ESTOP entry; held until recovery

// Fields extracted from the most recent valid RCC packet
static uint8_t  rccStatusBits       = 0;  // offset 4
static int32_t  m1Speed             = 0;  // offset 5
static int32_t  m1Encoder           = 0;  // offset 9
static int16_t  m1Current           = 0;  // offset 13
static int32_t  m2Speed             = 0;  // offset 15
static int32_t  m2Encoder           = 0;  // offset 19
static int16_t  m2Current           = 0;  // offset 23
static uint16_t mcVoltage           = 0;  // offset 25
static uint32_t mcStatus            = 0;  // offset 27
static uint16_t mcTimeSinceUpdate   = 0;  // offset 31
static int32_t  m1CmdPos            = 0;  // offset 33
static int32_t  m1CmdSpeed          = 0;  // offset 37
static uint32_t m1CmdAccel          = 0;  // offset 41
static uint32_t m1CmdDecel          = 0;  // offset 45
static int32_t  m2CmdPos            = 0;  // offset 49
static int32_t  m2CmdSpeed          = 0;  // offset 53
static uint32_t m2CmdAccel          = 0;  // offset 57
static uint32_t m2CmdDecel          = 0;  // offset 61
static uint8_t  rideState           = 0;  // offset 65
static uint8_t  rccLimitSwitches    = 0;  // offset 66
static uint16_t rccEchoOfPlc        = 0;  // offset 2 — RCC's echo of our counter

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

  lastRccCounter      = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  rccEchoOfPlc        = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
  rccStatusBits       = buf[4];
  memcpy(&m1Speed,          buf + 5,  sizeof(m1Speed));
  memcpy(&m1Encoder,        buf + 9,  sizeof(m1Encoder));
  memcpy(&m1Current,        buf + 13, sizeof(m1Current));
  memcpy(&m2Speed,          buf + 15, sizeof(m2Speed));
  memcpy(&m2Encoder,        buf + 19, sizeof(m2Encoder));
  memcpy(&m2Current,        buf + 23, sizeof(m2Current));
  memcpy(&mcVoltage,        buf + 25, sizeof(mcVoltage));
  memcpy(&mcStatus,         buf + 27, sizeof(mcStatus));
  memcpy(&mcTimeSinceUpdate,buf + 31, sizeof(mcTimeSinceUpdate));
  memcpy(&m1CmdPos,         buf + 33, sizeof(m1CmdPos));
  memcpy(&m1CmdSpeed,       buf + 37, sizeof(m1CmdSpeed));
  memcpy(&m1CmdAccel,       buf + 41, sizeof(m1CmdAccel));
  memcpy(&m1CmdDecel,       buf + 45, sizeof(m1CmdDecel));
  memcpy(&m2CmdPos,         buf + 49, sizeof(m2CmdPos));
  memcpy(&m2CmdSpeed,       buf + 53, sizeof(m2CmdSpeed));
  memcpy(&m2CmdAccel,       buf + 57, sizeof(m2CmdAccel));
  memcpy(&m2CmdDecel,       buf + 61, sizeof(m2CmdDecel));
  rideState           = buf[65];
  rccLimitSwitches    = buf[66];
  return true;
}

static void sendPlcPacket(uint8_t statusBits) {
  uint8_t tx[PLC_PACKET_SIZE];
  memcpy(tx + 0, &plcCounter,     2);
  memcpy(tx + 2, &lastRccCounter, 2);
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
//  FAULT    — 5 Hz blink         (Level 0 fault, terminal)
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

// -- Safety Checks ------------------------------------------------------------
static bool motorsMoving() {
  return (m1Speed != 0) || (m2Speed != 0);
}

// Returns a bitmask of all active fault bits, or 0 if all checks pass.
// Shared by safetyViolation() and enterEstop() to avoid duplicating logic.
// TODO: add speed/position/acceleration envelope checks once thresholds are defined.
// TODO: confirm limit switch bit ordering matches RCC packet offset 66 before re-enabling.
static uint8_t checkSafetyFaults(uint8_t plcLimits) {
  uint8_t faults = 0;
  if (plcLimits != rccLimitSwitches)                                    faults |= 0x08;  // limit switch mismatch
  int16_t echoLag = (int16_t)((plcCounter - 1) - rccEchoOfPlc);
  if (echoLag < 0 || echoLag > ECHO_LAG_LIMIT)                          faults |= 0x40;  // echo counter fault
  if (rideState == 5 && (abs(m1Speed) > 5 || abs(m2Speed) > 5))         faults |= 0x10;  // motion while RCC in ESTOP
  if (abs(m1Speed) > MAX_MOTOR_SPEED || abs(m2Speed) > MAX_MOTOR_SPEED) faults |= 0x80;  // motion fault
  return faults;
}

static bool safetyViolation(uint8_t plcLimits) {
  return checkSafetyFaults(plcLimits) != 0;
}

// Transitions into STATE_ESTOP, latching the triggering fault bits for diagnostics.
static void enterEstop(unsigned long now, uint8_t plcLimits) {
  estopAssertedMs = now;
  latchedFaults   = checkSafetyFaults(plcLimits);
  if (!rccHealthy) latchedFaults |= 0x04;  // watchdog fault (checked separately from safetyViolation)
  plcState = STATE_ESTOP;
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
  if (rxLen == RCC_PACKET_SIZE && parseRccPacket(rxBuf)) {
    lastValidPacketMs = now;
    rccHealthy = true;
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
      if (!rccHealthy || safetyViolation(plcLimits)) { enterEstop(now, plcLimits); break; }
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
      // Auto-recovery: cause resolved — return to appropriate mode
      if (rccHealthy && !safetyViolation(plcLimits)) {
        latchedFaults = 0;
        plcState = keyOn ? STATE_OK : STATE_MAINT;
      }
      break;

    case STATE_FAULT:
      // Terminal. Cut power and hold. Requires a full physical power cycle.
      setEstop(true);
      setRelay(false);
      break;

    case STATE_MAINT:
      if (!keyMaint) { setEstop(true); plcState = STATE_OFF; break; }
      if (!rccHealthy || safetyViolation(plcLimits)) { enterEstop(now, plcLimits); break; }
      setEstop(false);
      break;
  }

  // -- Build status byte and transmit --
  // Bit 0: E-stop released  Bit 1: I'm OK
  // Bit 2: Watchdog fault   Bit 3: Limit switch mismatch
  // Bit 4: Motion post-stop Bit 5: Level 0 fault (terminal)  Bit 6: Echo counter fault
  uint8_t statusBits = 0;
  if (plcState == STATE_OK   || plcState == STATE_MAINT)  statusBits |= 0x03;
  if (plcState == STATE_ESTOP)                             statusBits |= latchedFaults;
  if (plcState == STATE_ESTOP && motorsMoving())           statusBits |= 0x10;
  if (plcState == STATE_FAULT)                             statusBits |= latchedFaults | 0x20;
  sendPlcPacket(statusBits);

  updateLed(now);
}
