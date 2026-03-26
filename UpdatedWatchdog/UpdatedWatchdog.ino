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
static uint8_t       latchedFaults     = 0;  // Fault bits latched at ESTOP entry; held until recovery

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

static void sendPlcPacket(uint8_t limitSwitches, uint8_t statusBits) {
  uint8_t tx[PLC_PACKET_SIZE];

  memcpy(tx + 0, &plcCounter,     2);
  memcpy(tx + 2, &lastRccCounter, 2);
  tx[4] = statusBits;
  tx[5] = limitSwitches;
  tx[6] = 0; tx[7] = 0;  // reserved

  uint16_t crc = crc16(tx, 8);
  memcpy(tx + 8, &crc, 2);

  // Only write if the TX buffer has room — Serial.write() blocks if full,
  // which would freeze the entire loop including the safety-critical RX path.
  if (Serial.availableForWrite() >= PLC_PACKET_SIZE) {
    Serial.write(tx, PLC_PACKET_SIZE);
    plcCounter++;
  }
}

// ---------------------------------------------------------------------------
// LED Status Indicator
// ---------------------------------------------------------------------------
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
static bool safetyViolation(uint8_t plcLimits) {
  // TODO ADD SAFETY VIOLATIONS
  // if (limitSwitchMismatch(plcLimits)) return true;
  return false;
}

// Transitions into STATE_ESTOP, latching the triggering fault bits for diagnostics.
// Fault bits are held until the system fully recovers so the RCC always sees why.
static void enterEstop(unsigned long now, uint8_t plcLimits) {
  estopAssertedMs = now;
  latchedFaults   = 0;
  if (!rccHealthy)                    latchedFaults |= 0x04;  // watchdog fault
  if (limitSwitchMismatch(plcLimits)) latchedFaults |= 0x08;  // limit switch mismatch
  plcState = STATE_ESTOP;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_ESTOP,      OUTPUT);
  pinMode(LED_BUILTIN,        OUTPUT);
  pinMode(PIN_KEY_ON,     INPUT);
  pinMode(PIN_KEY_MAINT,  INPUT);
  pinMode(PIN_LIM_T1_BOT, INPUT_PULLUP);
  pinMode(PIN_LIM_T1_TOP, INPUT_PULLUP);
  pinMode(PIN_LIM_T2_BOT, INPUT_PULLUP);
  pinMode(PIN_LIM_T2_TOP, INPUT_PULLUP);

  setRelay(false);  // Everything off at boot
  setEstop(true);   // E-stop asserted until system is ready

  Serial.begin(BAUD);
  plcState = STATE_OFF;
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // ---- Run state machine every 20 ms ----
  static unsigned long lastTxMs = 0;
  if (now - lastTxMs < 20) return;
  lastTxMs = now;

  // ---- Flush stale bytes, then wait for one fresh packet ----
  // Any bytes already in the buffer are from a previous cycle or a boot-time
  // backlog.  Discarding them before receiving ensures the state machine always
  // acts on data from THIS cycle, keeping the echo counter current.
  // The RCC sends every 20 ms; after the flush the next packet arrives within
  // WATCHDOG_MS, so one missed alignment never trips the watchdog.
  while (Serial.available()) Serial.read();

  {
    uint8_t rxBuf[RCC_PACKET_SIZE];
    uint8_t rxLen = 0;
    unsigned long waitStart = millis();
    while (rxLen < RCC_PACKET_SIZE) {
      if (Serial.available() > 0) rxBuf[rxLen++] = Serial.read();
      if (millis() - waitStart > WATCHDOG_MS) break;
    }
    now = millis();  // refresh after blocking wait
    if (rxLen == RCC_PACKET_SIZE && parseRccPacket(rxBuf)) {
      lastValidPacketMs = now;
      rccHealthy = true;
    }
  }

  uint8_t plcLimits = readLimitSwitches();
  bool    keyOn     = (digitalRead(PIN_KEY_ON)    == HIGH);
  bool    keyMaint  = (digitalRead(PIN_KEY_MAINT) == HIGH);

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
        enterEstop(now, plcLimits);
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
      // Auto-recovery: cause resolved — clear latched faults and return to appropriate mode
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
      if (!rccHealthy || safetyViolation(plcLimits)) {
        enterEstop(now, plcLimits);
        break;
      }
      setEstop(false);
      break;
  }

  // ---- Build status byte and send PLC packet ----
  // Bit 0: E-stop released     Bit 1: I'm OK
  // Bit 2: Watchdog fault       Bit 3: Limit switch mismatch
  // Bit 4: Motion after E-stop  Bit 5: Level 0 fault (terminal)
  bool plcOk = (plcState == STATE_OK || plcState == STATE_MAINT);
  uint8_t statusBits = 0;
  if (plcOk)                                     statusBits |= 0x03;  // bits 0+1: OK and E-stop released
  if (plcState == STATE_ESTOP)                   statusBits |= latchedFaults;  // latched at ESTOP entry
  if (plcState == STATE_ESTOP && motorsMoving()) statusBits |= 0x10;
  if (plcState == STATE_FAULT)                   statusBits |= latchedFaults | 0x20;
  sendPlcPacket(plcLimits, statusBits);

  updateLed(now);
}
