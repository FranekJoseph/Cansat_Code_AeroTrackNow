#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ===================== PINOUT (Waveshare Pico-LoRa-SX1262 on RPi Pico) =====================
static constexpr int PIN_LORA_CS    = 3;   // GP3 — NSS
static constexpr int PIN_LORA_DIO1  = 20;  // GP20 — DIO1 (IRQ)
static constexpr int PIN_LORA_RESET = 15;  // GP15 — RESET
static constexpr int PIN_LORA_BUSY  = 2;   // GP2 — BUSY
// SPI1 pins: SCK=GP10, MOSI=GP11, MISO=GP12

// ===================== LORA CONFIG (MATCH CURRENT CANSAT) =====================
static constexpr float LORA_FREQ_MHZ = 433.0f;
static constexpr int   LORA_TX_DBM   = 20;
static constexpr uint32_t TX_TIMEOUT_MS = 120;

// ===================== TDM / RTK SHAPING CONFIG =====================
static constexpr uint32_t TDM_TX_DELAY_MS   = 1;
static constexpr uint32_t TDM_TX_WINDOW_MS  = 2000;
static constexpr uint32_t TDM_FALLBACK_MS   = 10;

// v4: constant-rate shaping for real RTCM.
// Dummy data proved the link is healthy at a smooth 1200 B/s.
// Real RTCM fails because it arrives in bursts, so we intentionally smooth it.
static constexpr uint32_t FIXED_SEND_PERIOD_MS   = 100;   // one LoRa packet every 100 ms
static constexpr size_t   FIXED_RTK_PAYLOAD_BYTES = 120;  // 120 B payload => 1200 B/s shaped stream

static constexpr size_t   RTK_TX_CHUNK_MAX  = FIXED_RTK_PAYLOAD_BYTES;
static constexpr size_t   RTK_BUF_LEN       = 16384; // extra backlog headroom for bursty RTCM epochs
static constexpr size_t   LORA_BUF_LEN      = 255;

SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY, SPI1);

static volatile bool radioEvent = false;
static volatile uint32_t radioEventPending = 0; // count DIO1 edges under load
static bool txInProgress = false;
static uint32_t txStartMs = 0;
static uint32_t lastRxMs = 0;
static bool txWindowOpen = false;
static uint32_t lastRadioActivityMs = 0;        // ✓ FIXED v5.6: RX recovery watchdog
static constexpr uint32_t RX_RECOVERY_MS = 500;  // restart receiver if no DIO1 event for 500ms
static uint32_t rxRecoveryCount = 0;

static uint8_t loraBuf[LORA_BUF_LEN];
static uint8_t rtkBuf[RTK_BUF_LEN];
static size_t  rtkHead = 0;
static size_t  rtkTail = 0;
static size_t  rtkBufLen = 0;
static uint32_t rtk_overflow_bytes = 0;
static uint32_t rtk_serial_in_bytes = 0;
static uint32_t rtk_tx_bytes = 0;
static uint32_t txStartErrCount = 0;
static uint32_t rxPacketCount = 0;
static uint32_t txPacketCount = 0;
static uint16_t rtkSeqNum = 0;
static uint32_t lastStatusMs = 0;
static uint32_t prevStatusRtkInBytes = 0;
static uint32_t prevStatusRtkTxBytes = 0;
static uint32_t prevStatusOverflowBytes = 0;
static constexpr bool GS_VERBOSE_STATUS = false; // keep USB CDC quiet during RTCM ingest; enable only for debugging
static constexpr bool RTK_ACQUIRE_MODE = true;
static constexpr bool UPLINK_PRIORITY_MODE = false; // bidirectional: forward telemetry to USB + send RTK uplink
static constexpr uint32_t GS_HEARTBEAT_MS = 5000;   // ✓ FIXED v5.9: periodic heartbeat so Python knows GS is alive
static uint32_t lastHeartbeatMs = 0;
static uint32_t lastForcedTxMs = 0;
static constexpr uint8_t MSG_GS_PING = 0x04;        // ✓ FIXED v5.10: LoRa heartbeat so CanSat can verify link
static constexpr uint32_t GS_LORA_PING_MS = 3000;   // send ping every 3s when no RTCM to send
static uint32_t lastLoraPingMs = 0;
static uint32_t loraPingTxCount = 0;
static uint32_t lastTxDoneMs = 0;
static uint32_t nextTxDueMs = 0;

// ✓ FIXED v5.8: CRC16-CCITT for RTK packet integrity validation
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
  }
  return crc;
}

static void setRadioFlag() {
  radioEvent = true;
  if (radioEventPending < 1000000UL) radioEventPending++;
}

// ✓ FIXED v5.8: Checked startReceive wrapper with retry on failure
static uint32_t lora_start_rx_err_count = 0;
static void startReceive() {
  int err = radio.startReceive();
  if (err != RADIOLIB_ERR_NONE) {
    lora_start_rx_err_count++;
    delayMicroseconds(100);
    radio.startReceive();
  }
  txInProgress = false;
}

static inline bool timeReached(uint32_t now, uint32_t target) {
  return (int32_t)(now - target) >= 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);

  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  SPI1.begin();

  int st = radio.begin(LORA_FREQ_MHZ);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: LoRa init failed: %d\n", st);
    while (1) delay(1000);
  }

  // ✓ FIXED v5.6: Waveshare Pico-LoRa-SX1262 has onboard TCXO powered via DIO3.
  // Without this call the TCXO is unpowered and the SX1262 falls back to its
  // internal RC oscillator (~±15 ppm), degrading RX sensitivity by 10-20 dB.
  // This was the primary root cause of the 50m range cutoff.
  int err;
  err = radio.setTCXO(1.6);
  if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setTCXO failed: %d\n", err); while(1) delay(1000); }

  // ✓ FIXED v5.6: Explicitly enable DC-DC regulator for full +20 dBm TX power.
  // Without this, LDO mode may cause voltage sag under high-current PA operation.
  err = radio.setRegulatorDCDC();
  if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setRegulatorDCDC failed: %d\n", err); while(1) delay(1000); }

  // ✓ range fix: explicit OCP for +20 dBm (RadioLib default may trip at ~60 mA,
  // silently collapsing TX power to ~+5 dBm). +20 dBm PA needs ~140 mA.
  radio.setCurrentLimit(140.0);
  // ✓ range fix: +2 dB RX sensitivity at +2 mA cost — strictly helps RX range.
  radio.setRxBoostedGainMode(true);
  // ✓ range fix: band-specific image calibration for 433 MHz band, recovers
  // 1-3 dB of RX image rejection over the factory full-band calibration.
  radio.calibrateImage(433.0);

  radio.setDio2AsRfSwitch(true);
  err = radio.setSpreadingFactor(7);      if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setSF failed: %d\n", err); while(1) delay(1000); }
  err = radio.setBandwidth(500.0);        if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setBW failed: %d\n", err); while(1) delay(1000); }
  err = radio.setCodingRate(5);           if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setCR failed: %d\n", err); while(1) delay(1000); }
  err = radio.setOutputPower(LORA_TX_DBM);if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setPower failed: %d\n", err); while(1) delay(1000); }
  err = radio.setCRC(true);               if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setCRC failed: %d\n", err); while(1) delay(1000); }
  err = radio.setSyncWord(0xAE, 0x2B);    if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setSyncWord failed: %d\n", err); while(1) delay(1000); }
  err = radio.setPreambleLength(12);      if (err != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setPreamble failed: %d\n", err); while(1) delay(1000); }  // ✓ FIXED: was 8 — must match CanSat

  radio.setDio1Action(setRadioFlag);
  startReceive();

  lastRadioActivityMs = millis();  // seed RX recovery watchdog
  lastStatusMs = millis();
  prevStatusRtkInBytes = rtk_serial_in_bytes;
  prevStatusRtkTxBytes = rtk_tx_bytes;
  prevStatusOverflowBytes = rtk_overflow_bytes;
  nextTxDueMs = millis();

  Serial.println("GroundStation_v4 ready (constant-rate RTCM shaping)");
}

void loop() {
  uint32_t now = millis();

  // ✓ FIXED v5.6: Read volatile ISR flags with interrupts disabled to prevent
  // torn reads and missed events on RP2040 (32-bit but non-atomic for >1 variable).
  noInterrupts();
  bool hasEvent = radioEvent;
  uint32_t pending = radioEventPending;
  radioEvent = false;
  radioEventPending = 0;
  interrupts();

  if (hasEvent || pending > 0) {
    lastRadioActivityMs = now;  // update watchdog on any DIO1 event
    // Drain pending IRQ events to avoid missing TX-done / RX-ready transitions.
    while (hasEvent || pending > 0) {
      hasEvent = false;
      if (pending > 0) pending--;

      if (txInProgress) {
        radio.finishTransmit();
        startReceive();
        lastTxDoneMs = millis();
        continue;
      }

      int len = radio.getPacketLength();
      if (len > 0 && len <= (int)LORA_BUF_LEN) {
        int err = radio.readData(loraBuf, len);
        if (err == RADIOLIB_ERR_NONE) {
          if (!UPLINK_PRIORITY_MODE) {
            Serial.write(loraBuf, len);
          }
          rxPacketCount++;
          lastRxMs = now;
          // ✓ range fix: rate-limited link-margin diagnostics (500ms).
          // #GS_RX prefix matches existing #GS heartbeat convention and is
          // safely ignored by the Python relay's binary packet parser.
          {
            static uint32_t last_rssi_print_ms = 0;
            if (millis() - last_rssi_print_ms > 500) {
              last_rssi_print_ms = millis();
              Serial.printf("#GS_RX type=0x%02X len=%u rssi=%.1fdBm snr=%.1fdB\n",
                            loraBuf[0], (unsigned)len, radio.getRSSI(), radio.getSNR());
            }
          }
        }
      }
      startReceive();
    }
  }

  if (txInProgress && (now - txStartMs > TX_TIMEOUT_MS)) {
    radio.finishTransmit();
    startReceive();
    lastTxDoneMs = now;
  }

  // ✓ FIXED v5.6: RX recovery watchdog — if no DIO1 event for 500ms, the radio
  // may be stuck in an invalid state. Restart the receiver to recover.
  if (!txInProgress && (now - lastRadioActivityMs > RX_RECOVERY_MS)) {
    startReceive();
    lastRadioActivityMs = now;
    rxRecoveryCount++;
  }

  if (lastRxMs > 0) {
    uint32_t sinceRx = now - lastRxMs;
    txWindowOpen = (sinceRx >= TDM_TX_DELAY_MS && sinceRx <= (TDM_TX_DELAY_MS + TDM_TX_WINDOW_MS));
  } else {
    txWindowOpen = (now > TDM_FALLBACK_MS);
  }

  while (Serial.available() > 0 && rtkBufLen < RTK_BUF_LEN) {
    rtkBuf[rtkHead] = (uint8_t)Serial.read();
    rtkHead = (rtkHead + 1) % RTK_BUF_LEN;
    rtkBufLen++;
    rtk_serial_in_bytes++;
  }
  while (Serial.available() > 0 && rtkBufLen >= RTK_BUF_LEN) {
    Serial.read();
    rtk_serial_in_bytes++;
    rtk_overflow_bytes++;
  }

  // ✓ FIXED v5.8: RTK_ACQUIRE_MODE now respects a 2ms gap after last RX to avoid
  // colliding with consecutive CanSat telemetry TX (was unconditional true).
  bool txAllowed = UPLINK_PRIORITY_MODE ? true
                                        : (RTK_ACQUIRE_MODE ? (txWindowOpen || (lastRxMs == 0) || (now - lastRxMs >= 2))
                                                            : (txWindowOpen || (now - lastForcedTxMs >= 3)));

  if (!txInProgress && txAllowed && rtkBufLen > 0 && timeReached(now, nextTxDueMs)) {
    uint8_t txBuf[255];
    // Soft CRC removed — hardware LoRa CRC still active on SX1262 (setCRC true)
    // Frame: [0x02][seq_hi][seq_lo][RTCM payload] = chunk + 3 bytes total
    size_t chunk = (rtkBufLen > FIXED_RTK_PAYLOAD_BYTES) ? FIXED_RTK_PAYLOAD_BYTES : rtkBufLen;
    const uint16_t seqToSend = rtkSeqNum;
    txBuf[0] = 0x02; // MSG_RTK framing
    txBuf[1] = (uint8_t)(seqToSend >> 8);
    txBuf[2] = (uint8_t)(seqToSend & 0xFF);
    for (size_t i = 0; i < chunk; i++) {
      txBuf[3 + i] = rtkBuf[(rtkTail + i) % RTK_BUF_LEN];
    }

    // IMPORTANT: only advance the sequence number after startTransmit() succeeds.
    // If startTransmit() fails and we increment first, the rover sees fake "missed"
    // sequence gaps even though those packets were never actually on-air.
    int err = radio.startTransmit(txBuf, chunk + 3);  // no soft CRC — hardware LoRa CRC still active
    if (err == RADIOLIB_ERR_NONE) {
      rtkSeqNum = (uint16_t)(seqToSend + 1);
      txPacketCount++;
      rtk_tx_bytes += (uint32_t)chunk;
      lastForcedTxMs = now;
      txInProgress = true;
      txStartMs = now;
      nextTxDueMs = now + FIXED_SEND_PERIOD_MS;

      rtkTail = (rtkTail + chunk) % RTK_BUF_LEN;
      rtkBufLen -= chunk;
    } else {
      txStartErrCount++;
      startReceive();
    }
  }

  // ✓ FIXED v5.10: LoRa heartbeat ping — sends a small packet every 3s when the GS
  // has no RTCM to transmit, so the CanSat can verify the RF link is alive.
  // Without this, if no Python/RTCM is connected the GS is completely silent on LoRa
  // and the CanSat has no way to tell if the GS exists or if the link is dead.
  if (!txInProgress && rtkBufLen == 0 && (now - lastLoraPingMs >= GS_LORA_PING_MS)) {
    uint8_t ping[4];
    ping[0] = MSG_GS_PING;
    ping[1] = (uint8_t)(loraPingTxCount >> 8);
    ping[2] = (uint8_t)(loraPingTxCount & 0xFF);
    ping[3] = (uint8_t)(rxPacketCount & 0xFF);  // tell CanSat how many telem pkts GS received
    int err = radio.startTransmit(ping, 4);
    if (err == RADIOLIB_ERR_NONE) {
      txInProgress = true;
      txStartMs = now;
      loraPingTxCount++;
    } else {
      startReceive();
    }
    lastLoraPingMs = now;
  }

  if (GS_VERBOSE_STATUS && (now - lastStatusMs >= 1000)) {
    uint32_t dt_ms = now - lastStatusMs;
    lastStatusMs = now;
    uint32_t rx_age = (lastRxMs > 0) ? (now - lastRxMs) : 999999;

    uint32_t in_delta = rtk_serial_in_bytes - prevStatusRtkInBytes;
    uint32_t tx_delta = rtk_tx_bytes - prevStatusRtkTxBytes;
    uint32_t ovf_delta = rtk_overflow_bytes - prevStatusOverflowBytes;
    uint32_t in_bps = (dt_ms > 0) ? (uint32_t)((1000.0f * in_delta) / dt_ms) : 0;
    uint32_t tx_bps = (dt_ms > 0) ? (uint32_t)((1000.0f * tx_delta) / dt_ms) : 0;

    Serial.printf("GS_STATUS rx_pkts=%lu tx_pkts=%lu rtk_in=%luB rtk_tx=%luB in_Bps=%lu tx_Bps=%lu rtk_buf=%u ovf=%lu ovf_d=%lu tx_err=%lu rx_age_ms=%lu tx_window=%d seq=%u chunk=%u period_ms=%lu\n",
                  rxPacketCount, txPacketCount, rtk_serial_in_bytes, rtk_tx_bytes, in_bps, tx_bps, (unsigned)rtkBufLen, rtk_overflow_bytes, ovf_delta, txStartErrCount, rx_age, txWindowOpen ? 1 : 0, rtkSeqNum, (unsigned)FIXED_RTK_PAYLOAD_BYTES, (unsigned long)FIXED_SEND_PERIOD_MS);

    prevStatusRtkInBytes = rtk_serial_in_bytes;
    prevStatusRtkTxBytes = rtk_tx_bytes;
    prevStatusOverflowBytes = rtk_overflow_bytes;
  }

  // ✓ FIXED v5.9: Periodic heartbeat — lets Python know GS is alive and shows
  // LoRa RX health even when no CanSat telemetry is being received.
  // Format: #GS rx=<count> tx=<count> rx_age=<ms> rtk_buf=<bytes> rxerr=<count>\n
  // The '#' prefix ensures Python's binary telemetry parser won't match it.
  if (now - lastHeartbeatMs >= GS_HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    uint32_t rx_age = (lastRxMs > 0) ? (now - lastRxMs) : 999999;  // ✓ FIXED: was 0 when never received — misleading
    Serial.printf("#GS rx=%lu tx=%lu rx_age=%lu rtk_buf=%u rxerr=%lu recover=%lu up=%lu ping=%lu\n",
                  rxPacketCount, txPacketCount, rx_age, (unsigned)rtkBufLen,
                  lora_start_rx_err_count, rxRecoveryCount, now / 1000, loraPingTxCount);
  }

}
