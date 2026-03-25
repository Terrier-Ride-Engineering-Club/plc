#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
#define PIN_RELAY       2   // Power relay, active HIGH
#define PIN_KEY_ON      3   // ON key switch, HIGH = ON
#define PIN_KEY_MAINT   4   // MAINT key switch, HIGH = MAINT
#define PIN_LIM_T1_BOT  6   // Tower 1 bottom limit, HIGH = triggered (NO)
#define PIN_LIM_T1_TOP  7   // Tower 1 top limit
#define PIN_LIM_T2_BOT  8   // Tower 2 bottom limit
#define PIN_LIM_T2_TOP  9   // Tower 2 top limit
#define PIN_ESTOP       10  // E-stop control, active LOW (LOW = asserted)

// ---------------------------------------------------------------------------
// Protocol Constants
// ---------------------------------------------------------------------------
#define BAUD            115200
#define RCC_PACKET_SIZE 69
#define PLC_PACKET_SIZE 10
#define WATCHDOG_MS     50   // Max ms between valid RCC packets before unhealthy

// ---------------------------------------------------------------------------
// Safety Parameters
// ---------------------------------------------------------------------------
// After the PLC asserts E-stop, motors must come to rest within this window.
// If they are still moving after this, escalate to Level 0 (FAULT).
// TODO: confirm acceptable deceleration time with hardware team.
#define MOTION_STOP_TIMEOUT_MS 2000

// ---------------------------------------------------------------------------
// PLC State Machine
// ---------------------------------------------------------------------------
enum PlcState {
  STATE_OFF,      // Key at OFF — relay de-energized, everything dark
  STATE_STARTING, // Key ON — relay on, awaiting first valid RCC handshake
  STATE_OK,       // Watchdog healthy, no violations — PLC E-stop gate open
  STATE_ESTOP,    // PLC asserting Level 1 E-stop
  STATE_FAULT,    // Level 0 — relay cut, terminal until full power cycle
  STATE_MAINT     // Maintenance mode — automated ride logic disabled
};

// ---------------------------------------------------------------------------
// Global State (kept minimal)
// ---------------------------------------------------------------------------
static PlcState      plcState          = STATE_OFF;
static uint16_t      plcCounter        = 0;
static uint16_t      lastRccCounter    = 0;
static unsigned long lastValidPacketMs = 0;
static unsigned long estopAssertedMs   = 0;
static bool          rccHealthy        = false;

// Fields extracted from the most recent valid RCC packet
static int32_t  m1Speed          = 0;
static int32_t  m2Speed          = 0;
static uint8_t  rccLimitSwitches = 0;

// ---------------------------------------------------------------------------
// CRC16 — ANSI, poly 0x8005
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// I/O Helpers
// ---------------------------------------------------------------------------
static void setRelay(bool on) {
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
}

// asserting=true drives pin LOW, which asserts E-stop on the motor controller
static void setEstop(bool asserting) {
  digitalWrite(PIN_ESTOP, asserting ? LOW : HIGH);
}

// Returns limit switch state packed into bits 0–3: T1_BOT, T1_TOP, T2_BOT, T2_TOP
static uint8_t readLimitSwitches() {
  uint8_t sw = 0;
  if (digitalRead(PIN_LIM_T1_BOT)) sw |= 0x01;
  if (digitalRead(PIN_LIM_T1_TOP)) sw |= 0x02;
  if (digitalRead(PIN_LIM_T2_BOT)) sw |= 0x04;
  if (digitalRead(PIN_LIM_T2_TOP)) sw |= 0x08;
  return sw;
}

// ---------------------------------------------------------------------------
// Packet Handling
// ---------------------------------------------------------------------------
static bool parseRccPacket(const uint8_t *buf) {
  uint16_t rxCrc = (uint16_t)buf[67] | ((uint16_t)buf[68] << 8);
  if (crc16(buf, 67) != rxCrc) return false;

  lastRccCounter   = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  memcpy(&m1Speed,  buf + 5,  sizeof(m1Speed));
  memcpy(&m2Speed,  buf + 15, sizeof(m2Speed));
  rccLimitSwitches = buf[66];

  return true;
}

static void sendPlcPacket(uint8_t limitSwitches, bool plcOk, bool estopAsserted) {
  uint8_t tx[PLC_PACKET_SIZE];

  uint8_t statusBits = 0;
  if (plcOk)          statusBits |= 0x02;  // bit 1: I'm OK
  if (!estopAsserted) statusBits |= 0x01;  // bit 0: E-stop released

  memcpy(tx + 0, &plcCounter,     2);
  memcpy(tx + 2, &lastRccCounter, 2);
  tx[4] = statusBits;
  tx[5] = limitSwitches;
  tx[6] = 0; tx[7] = 0;  // reserved

  uint16_t crc = crc16(tx, 8);
  memcpy(tx + 8, &crc, 2);

  Serial.write(tx, PLC_PACKET_SIZE);
  plcCounter++;
}

// ---------------------------------------------------------------------------
// Safety Checks
// ---------------------------------------------------------------------------
static bool motorsMoving() {
  return (m1Speed != 0) || (m2Speed != 0);
}

// Compares PLC's direct limit switch reading against what the RCC reported.
// A mismatch suggests a wiring fault, sensor failure, or RCC software error.
// TODO: confirm bit ordering matches RCC packet definition before first run.
static bool limitSwitchMismatch(uint8_t plcLimits) {
  return plcLimits != rccLimitSwitches;
}

// Returns true if any condition requires the PLC to assert E-stop.
// TODO: add position / speed / acceleration envelope checks once spec is finalized.
static bool safetyViolation(uint8_t plcLimits) {
  if (limitSwitchMismatch(plcLimits)) return true;
  return false;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_ESTOP,      OUTPUT);
  pinMode(PIN_KEY_ON,     INPUT);
  pinMode(PIN_KEY_MAINT,  INPUT);
  pinMode(PIN_LIM_T1_BOT, INPUT);
  pinMode(PIN_LIM_T1_TOP, INPUT);
  pinMode(PIN_LIM_T2_BOT, INPUT);
  pinMode(PIN_LIM_T2_TOP, INPUT);

  setRelay(false);  // Everything off at boot
  setEstop(true);   // E-stop asserted until system is ready

  Serial.begin(BAUD);
  plcState = STATE_OFF;
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------
void loop() {
  unsigned long now       = millis();
  uint8_t       plcLimits = readLimitSwitches();
  bool          keyOn     = (digitalRead(PIN_KEY_ON)    == HIGH);
  bool          keyMaint  = (digitalRead(PIN_KEY_MAINT) == HIGH);

  // ---- Receive RCC packet ----
  if (Serial.available() >= RCC_PACKET_SIZE) {
    uint8_t buf[RCC_PACKET_SIZE];
    Serial.readBytes(buf, RCC_PACKET_SIZE);
    if (parseRccPacket(buf)) {
      lastValidPacketMs = now;
      rccHealthy = true;
    }
  }

  // ---- Watchdog ----
  if (now - lastValidPacketMs > WATCHDOG_MS) {
    rccHealthy = false;
  }

  // ---- State machine ----
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
      if (!rccHealthy || safetyViolation(plcLimits)) {
        estopAssertedMs = now;
        plcState = STATE_ESTOP;
        break;
      }
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
      // Auto-recovery: cause resolved — return to the appropriate mode
      if (rccHealthy && !safetyViolation(plcLimits)) {
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
      if (!rccHealthy || safetyViolation(plcLimits)) {
        estopAssertedMs = now;
        plcState = STATE_ESTOP;
        break;
      }
      setEstop(false);
      break;
  }

  // ---- Send PLC packet every cycle ----
  bool plcOk         = (plcState == STATE_OK || plcState == STATE_MAINT);
  bool estopAsserted = (plcState != STATE_OK  && plcState != STATE_MAINT);
  sendPlcPacket(plcLimits, plcOk, estopAsserted);

  delay(10);
}
