#include <Arduino.h>

#define BAUD 115200
#define RCC_PACKET_SIZE 69
#define PLC_PACKET_SIZE 10
#define TIMEOUT_MS 50

uint16_t plcCounter = 0;
uint16_t lastRccCounter = 0;
unsigned long lastValidTime = 0;
bool rccHealthy = false;

uint16_t crc16(uint8_t *data, uint16_t length) {
  uint16_t crc = 0x0000;

  for (uint16_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x8005;
      else
        crc <<= 1;
    }
  }

  return crc & 0xFFFF;
}

void setup() {
  Serial.begin(BAUD);
}

void loop() {

  // -----------------------
  // RECEIVE RCC PACKET
  // -----------------------
  if (Serial.available() >= RCC_PACKET_SIZE) {

    uint8_t buffer[RCC_PACKET_SIZE];
    Serial.readBytes(buffer, RCC_PACKET_SIZE);

    uint16_t receivedCrc = *((uint16_t*)(buffer + 67));
    uint16_t calcCrc = crc16(buffer, 67);

    if (receivedCrc == calcCrc) {

      lastRccCounter = *((uint16_t*)(buffer + 0));
      lastValidTime = millis();
      rccHealthy = true;
    }
  }

  if (millis() - lastValidTime > TIMEOUT_MS) {
    rccHealthy = false;
  }

  // -----------------------
  // BUILD PLC PACKET
  // -----------------------

  uint8_t tx[PLC_PACKET_SIZE];

  uint8_t statusBits = 0x02;  // I'm OK

  *((uint16_t*)(tx + 0)) = plcCounter;
  *((uint16_t*)(tx + 2)) = lastRccCounter;
  tx[4] = statusBits;
  tx[5] = 0;  // limit switches
  *((uint16_t*)(tx + 6)) = 0;  // reserved

  uint16_t crc = crc16(tx, 8);
  *((uint16_t*)(tx + 8)) = crc;

  Serial.write(tx, PLC_PACKET_SIZE);

  plcCounter++;
  delay(10);
}