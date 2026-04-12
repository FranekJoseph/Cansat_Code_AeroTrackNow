// =====================================================================
// AeroTrackNow CanSat — Drop Test Data Collection Firmware
// Version: 2.0 (aligned with flight firmware v5.5) — v4 pair keeps working Teensy receive path
// Purpose: Collect calibration data during drone drops.
//          NO GUIDANCE — only sensors, logging, and timed servo commands.
// =====================================================================
//
// BEFORE EACH DROP, CHANGE TEST_TYPE BELOW AND RE-UPLOAD.
//
// TEST_TYPE values:
//   0 = Free flight    — servos neutral, measures baseline sink rate & glide ratio
//   1 = Brake sweep UP — 0% → 25% → 50% → 75% → 100% symmetric brake (30s each)
//   2 = Brake sweep DN — 100% → 75% → 50% → 25% → 0% symmetric brake (30s each)
//   3 = Turn rate test — asymmetric brake: left100, right100, left50, right50
//   4 = Glide slope    — 0% for 60s, then 25% for 60s (long steady-state for glide ratio)
//   5 = Turn@brake — turn rate measurement at 0%, 25%, 50% symmetric brake levels
//   6 = Step response — sharp step commands for canopy tau measurement
//   7 = Flare test  — baseline 30s, then progressive brake: 0.3/0.6/0.8/1.0 (10s each), neutral
//
// CSV columns (29 total):
//   time_ms, baro_press_hPa, baro_temp_C, baro_alt_m,
//   gnss_lat, gnss_lon, gnss_alt_m, gnss_speed_mps, gnss_track_deg,
//   gnss_fix, gnss_sats, gnss_hacc_mm, gnss_vacc_mm, gnss_vert_vel_mps,
//   accel_x, accel_y, accel_z, accel_mag,
//   heading_rad, heading_conf, gyro_z_rads,
//   servo_cmd_L, servo_cmd_R, servo_us_L, servo_us_R,
//   test_phase, sink_rate_raw, sink_rate_filtered,
//   gnss_carr_soln
//
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
// Servo library REMOVED — uses direct PWM for full MG92B range (544-2400 us)
// #include <Servo.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO08x.h>
#include <RadioLib.h>

// ===================== CHANGE THIS BEFORE EACH DROP =====================
static constexpr int TEST_TYPE = 0;  // 0=free, 1=sweep_up, 2=sweep_down, 3=turn_rate, 4=glide_slope, 5=turn@brake, 6=step_response, 7=flare_test
static_assert(TEST_TYPE >= 0 && TEST_TYPE <= 7, "Invalid TEST_TYPE — must be 0-7");
// ========================================================================

// ===================== DROP DETECTION =====================
// Primary: accelerometer detects free fall (< 3 m/s²)
// Secondary: timer fallback (if accel detection fails)
static constexpr float FREEFALL_THRESHOLD = 3.0f;      // m/s² — below this = free fall
static constexpr int FREEFALL_COUNT_NEEDED = 2;         // 2 consecutive readings × 200ms = 400ms detection
static constexpr uint32_t TIMER_FALLBACK_MS = 0;         // DISABLED — rely on free-fall detection only
// Set to e.g. 120000 (2 min) as backup if accelerometer is unreliable.
// When 0, the ONLY way to trigger descent is free-fall detection (accel < 3 m/s²).
// NOTE: Flight firmware ALSO checks fabs(aMag - 9.81f) > DROP_ACCEL_SPIKE
// (detects acceleration events, e.g. hard mount release) AND altitude backup.
// Test firmware intentionally omits these: spike check may false-trigger on
// drone vibration, and altitude backup adds latency. Drop timing difference
// vs flight is <100ms and within deployment delay tolerance.
static constexpr float SINK_FILTER_ALPHA = 0.35f;  // MUST match flight firmware

// ===================== HEADING FILTER =====================
// All values MUST match flight firmware exactly
static constexpr float    HEADING_JUMP_MAX_RAD            = 1.2f;   // reject jumps > 1.2 rad
static constexpr uint32_t HEADING_JUMP_MAX_DT_MS          = 150;    // within 150 ms
static constexpr float    HEADING_CONFIDENCE_MAX_DELTA_RAD = 0.35f;  // full confidence if delta < 0.35 rad
static constexpr uint32_t HEADING_CONFIDENCE_MAX_DT_MS     = 300;    // full confidence if dt < 300 ms

// ===================== DEPLOYMENT DELAY =====================
// Wait this long after drop detection for parafoil to inflate
static constexpr uint32_t DEPLOY_DELAY_MS = 5000;       // 5 seconds — must stay synchronized with DEPLOYMENT_SKIP_S in process_drops.py. If one changes, the other must change too.

// ===================== HARDWARE PINOUT =====================
// Same as flight code — DO NOT CHANGE unless hardware changes
static constexpr int PIN_GNSS_RX      = 21;   // Serial5 RX
static constexpr int PIN_GNSS_TX      = 20;   // Serial5 TX
static constexpr int PIN_I2C_SDA      = 18;
static constexpr int PIN_I2C_SCL      = 19;
static constexpr int PIN_LORA_CS      = 10;
static constexpr int PIN_LORA_DIO1    = 7;
static constexpr int PIN_LORA_RESET   = 8;
static constexpr int PIN_LORA_BUSY    = 9;    // BUSY pin IS connected
static constexpr int PIN_LORA_RXEN    = 5;    // RF switch RX enable (CORE1262-LF)
static constexpr int PIN_LORA_TXEN    = 6;    // RF switch TX enable (CORE1262-LF)
static constexpr int PIN_SD_CS        = 23;
static constexpr int PIN_SERVO_LEFT   = 3;
static constexpr int PIN_SERVO_RIGHT  = 4;

// ===================== I2C BUS RECOVERY =====================
// (matches flight firmware)
static constexpr int      I2C_FAIL_THRESHOLD       = 3;
static constexpr int      I2C_SCL_TOGGLE_COUNT     = 16;
static constexpr uint32_t I2C_RECOVERY_COOLDOWN_MS = 3000;

// ===================== WATCHDOG =====================
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 5000;  // MUST match flight firmware
static constexpr bool ENABLE_WATCHDOG = false;   // v2.4: disabled for bench/drop-test stability

// ===================== SD FAILURE DETECTION =====================
static constexpr int SD_FAIL_WARN_THRESHOLD = 10;  // consecutive write failures before warning
static constexpr int SD_FAIL_DISABLE_THRESHOLD = 30; // disable SD after this many consecutive failures to protect SPI bus

// ===================== MAX DESCENT TIMEOUT =====================
// Safety limit: force LANDED if descent exceeds this duration (e.g. baro stuck)
static constexpr uint32_t MAX_DESCENT_MS = 600000;  // 10 minutes

// ===================== SERVO PARAMETERS =====================
static constexpr int SERVO_RELEASE_L_US = 544;   // Left 0% pull
static constexpr int SERVO_RELEASE_R_US = 2400;  // Right 0% pull (mirror-mounted)
static constexpr int SERVO_RANGE_US     = 1134;  // 0% to 100% brake: L 544→1678, R 2400→1266

// ===================== DIRECT PWM SERVO (MG92B) =====================
static constexpr int    PWM_SERVO_FREQ      = 50;
static constexpr int    PWM_SERVO_BITS      = 16;
static constexpr float  PWM_SERVO_PERIOD_US = 1000000.0f / PWM_SERVO_FREQ;
static constexpr float  PWM_SERVO_MAX_VAL   = (float)(1 << PWM_SERVO_BITS);

static void servoWriteUs(int pin, int us) {
  float duty = (float)us / PWM_SERVO_PERIOD_US * PWM_SERVO_MAX_VAL;
  analogWrite(pin, (int)(duty + 0.5f));
}

// ===================== LORA CONFIG =====================
static constexpr float LORA_FREQ_MHZ         = 433.0f;
static constexpr int LORA_TX_DBM             = 20;  // ✓ FIXED: was 14 — matched to GS power for symmetric link budget
static constexpr bool RTK_ACQUIRE_MODE = true;
static constexpr uint32_t TELEMETRY_PERIOD_MS = 5000;  // OLD value restored — reduces CanSat TX blackouts from 1% to 0.2%, protects RTK reception
static constexpr uint32_t LOOP_PERIOD_MS = RTK_ACQUIRE_MODE ? 5 : 40;  // ultra-aggressive LoRa polling during high-rate RTK ingest
static constexpr bool     FULL_STATUS_VERBOSE = true;                   // restore full serial telemetry/status block
static constexpr int PERF_STAGE_LEVEL = 5; // all subsystems: sensors, state, telemetry TX, SD logging
static constexpr uint32_t GNSS_READ_PERIOD_MS = 500; // stronger GNSS parser throttling for sustained RTK throughput
static constexpr uint32_t LORA_TX_TIMEOUT_MS  = 350;   // safety timeout for TX completion (more margin)
static constexpr uint32_t RTCM_UART_WRITE_TIMEOUT_MS = 400;  // prioritize draining RTK queue to GNSS
static constexpr size_t   RTCM_UART_CHUNK_BYTES = 512;      // larger write bursts to GNSS UART
static constexpr size_t   RTK_QUEUE_BYTES = 8192;           // decouple LoRa ingest from GNSS UART drain

// ===================== TELEMETRY PACKET =====================
// MUST match flight code exactly — ground station decodes this
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t  msgType;                 // 0x03
  uint32_t time_ms;
  int32_t  lat_e7;
  int32_t  lon_e7;
  int16_t  height_agl_dm;          // meters * 10
  uint16_t pressure_hPa_x10;
  int16_t  temp_c_x10;
  uint16_t ground_speed_cms;
  int16_t  servo_left_x1000;
  int16_t  servo_right_x1000;
  int32_t  pred_lat_e7;            // unused in test — set to 0
  int32_t  pred_lon_e7;            // unused in test — set to 0
  int16_t  wind_north_cms;         // repurposed: gnss_fix in test mode
  int16_t  wind_east_cms;          // repurposed: gnss_hacc_mm in test mode
  int16_t  heading_deg_x10;
  uint16_t heading_confidence_x1000;
  uint16_t wind_rejects;           // repurposed: test_type
  uint16_t wind_layer_rejects;     // repurposed: test_phase
  uint8_t  mission_state;          // 0=waiting, 1=descending, 2=landed
  uint8_t  gnss_carr_soln;         // v5.0: carrier solution type (0=none, 1=float, 2=fixed)
  uint8_t  gnss_num_sats;          // v5.0: number of satellites
  uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(TelemetryPacket) == 50, "TelemetryPacket size mismatch — ground station expects 50 bytes");

static constexpr uint8_t MSG_RTK       = 0x02;
static constexpr uint8_t MSG_TELEMETRY = 0x03;
static constexpr uint8_t MSG_GS_PING  = 0x04;  // ✓ v5.10: GS LoRa heartbeat ping

// ===================== STATE =====================
enum TestState : uint8_t {
  WAITING   = 0,  // On ground or ascending on drone — waiting for drop
  DESCENDING = 1, // Dropped — running test sequence
  LANDED     = 2  // On the ground after descent
};

// ===================== HARDWARE OBJECTS =====================
SFE_UBLOX_GNSS gnss;
Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t bnoValue;
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);
// Servo servoL, servoR;  // REMOVED — using direct PWM via servoWriteUs()
File logFile;

// ===================== SENSOR STATE =====================
// GNSS
double lat = 0.0, lon = 0.0;
double gnss_alt_msl = 0.0;
double ground_speed = 0.0;
double gnss_track_deg = 0.0;
uint8_t gnss_fix = 0;
uint8_t gnss_carrier_solution = 0;  // v5.0: 0=none, 1=float RTK, 2=fixed RTK
uint8_t gnss_sats = 0;
uint32_t gnss_hacc_mm = 9999;
uint32_t gnss_vacc_mm = 9999;
double gnss_vert_vel = 0.0;
uint32_t last_gnss_ms = 0;
bool gnss_diff_soln = false;
bool gnss_fix_ok = false;
uint16_t gnss_pdop_x100 = 0;
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
bool gnss_gga_seen = false;
uint8_t gnss_gga_quality = 0;
float gnss_gga_diff_age_s = -1.0f;
char gnss_gga_station_id[16] = "";
uint32_t gnss_last_gga_ms = 0;
#endif

// BMP280
double pressure_hPa = 0.0;
double temp_c = 0.0;
double baro_alt = 0.0;
double last_baro_alt = 0.0;
uint32_t last_baro_ms = 0;
double sink_rate_raw = 0.0;
double sink_rate_filtered = 0.0;
double baro_alt_raw = 0.0;       // unfiltered altitude (for debug display)
bool first_alt_read = true;       // flag for altitude EMA initialization
static int bmp_warmup_count = 0;  // counts BMP reads after init; FILTER_X2 settles in 2 cycles (datasheet Table 7), threshold set to 5 for margin

// Pressure moving-average filter — smooths out noise and I2C glitches
// ISSUE 3: At 40ms read rate (25Hz loop), 16 samples = 640ms window. FILTER_X2 with X4
// oversampling has ~0.5 Pa RMS noise → ±4cm altitude. The 640ms MA reduces this to
// ~±1cm, which is well below perceptible. 640ms window is acceptable for bench testing —
// fast enough to track hand lifts while suppressing HVAC/door transients.
static constexpr int PRESSURE_AVG_SIZE = 16;
double pressure_buffer[PRESSURE_AVG_SIZE];
int pressure_buf_idx = 0;
bool pressure_buf_full = false;

// Sea level pressure calibration (matches flight firmware: 10 samples, vacc<2000mm)
double sea_level_hpa = 1013.25;
bool sea_level_calibrated = false;
int sea_level_samples = 0;
double sea_level_accum = 0.0;

// Sensor init status
static bool bmp_ok = false;
static bool bno_ok = false;
static bool gnss_ok = false;

// BNO085
float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
float accel_mag = 0.0f;
float heading_rad = 0.0f;
float heading_conf = 0.0f;
bool heading_valid = false;
uint32_t last_heading_ms = 0;
float last_heading_rad = 0.0f;
float gyro_z = 0.0f;  // yaw rate from calibrated gyroscope (rad/s)

// Servo state
float servo_cmd_L = 0.0f;   // normalized [0, 1] for test
float servo_cmd_R = 0.0f;
int servo_us_L = SERVO_RELEASE_L_US;
int servo_us_R = SERVO_RELEASE_R_US;

// Test state
TestState test_state = WAITING;
int test_phase = -1;           // -1 = no phase, 0+ = current phase
uint32_t drop_detected_ms = 0;
uint32_t sequence_start_ms = 0;
int freefall_count = 0;
uint32_t power_on_ms = 0;

// Drop altitude (recorded at moment of drop detection)
double drop_alt_msl = 0.0;

// Logging
uint32_t last_log_ms = 0;
uint32_t last_telemetry_ms = 0;
uint32_t lora_tx_start_ms = 0;
bool     lora_tx_pending   = false;
static volatile bool lora_tx_done_flag = false;
static volatile bool lora_dio1_flag = false;
static volatile uint32_t lora_dio1_pending = 0;  // count IRQ edges to avoid missing events under load
bool     lora_ok           = false;
uint32_t last_flush_ms = 0;
uint32_t log_line_count = 0;
// RTK relay diagnostics
uint32_t rtk_lora_bytes_in = 0;       // bytes received over LoRa RTK frames
uint32_t rtk_uart_bytes_out = 0;      // bytes successfully written to GNSS UART
uint32_t rtk_packets_in = 0;
uint32_t rtk_queue_drop_bytes = 0;    // bytes dropped due to queue full
uint32_t rtk_uart_drop_bytes = 0;     // bytes dropped by UART timeout guard
uint32_t rtk_uart_short_writes = 0;
uint32_t prev_rtk_lora_bytes_in = 0;
uint32_t prev_rtk_uart_bytes_out = 0;
uint32_t prev_rtk_drop_bytes = 0;
uint32_t prev_rtk_status_ms = 0;
uint32_t last_rtk_rx_ms = 0;

uint8_t  rtk_queue[RTK_QUEUE_BYTES];
size_t   rtk_q_head = 0;
size_t   rtk_q_tail = 0;
size_t   rtk_q_len = 0;
uint32_t lora_rx_any_pkts = 0;
uint32_t lora_rx_any_bytes = 0;
static uint16_t rtk_expected_seq = 0;
static bool     rtk_seq_initialized = false;
static uint32_t rtk_seq_gaps = 0;
static uint32_t rtk_seq_missed_total = 0;
uint32_t lora_dio1_isr_count = 0;
uint32_t lora_rx_read_ok = 0;
uint32_t lora_rx_read_err = 0;
uint32_t lora_tx_start_count = 0;
uint32_t lora_tx_done_count = 0;
uint32_t lora_tx_timeout_count = 0;
uint32_t lora_rx_recovery_count = 0;  // v5.6: times RX was force-restarted due to idle radio
static uint32_t last_lora_activity_ms = 0;  // v5.6: tracks last DIO1 event or TX start for RX recovery

// I2C recovery state
uint32_t last_bmp_read_ms = 0;
uint32_t last_bno_read_ms = 0;
int bmp_fail_count = 0;
int bno_fail_count = 0;
uint32_t last_i2c_recovery_ms = 0;

// SD write failure tracking
int sd_write_fail_count = 0;
bool sd_disabled = false;  // ✓ FIXED v5.9: disable SD after too many failures to protect SPI bus for LoRa

// Landing detection (matches flight v5.5: baro + GNSS cross-check)
uint32_t landing_still_start = 0;

// ===================== CRC16-CCITT =====================
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

// ===================== HELPER =====================
static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// NaN/Inf guard — matches flight firmware isFiniteF()
static inline bool isFiniteF(float v) { return !isnan(v) && !isinf(v); }

// Feed watchdog safely (works whether WDOG is enabled or not)
static inline void serviceWatchdogIfRunning() {
  if (!ENABLE_WATCHDOG) return;
  noInterrupts();
  WDOG3_CNT = 0xB480A602;
  interrupts();
}

// Wrap angle to [-PI, PI] — matches flight firmware wrapAngle()
static float wrapAngle(float angle) {
  if (!isFiniteF(angle)) return 0.0f;
  angle = fmodf(angle, TWO_PI);
  if (angle > PI) angle -= TWO_PI;
  if (angle < -PI) angle += TWO_PI;
  return angle;
}

// ===================== SERVO CONTROL =====================
// Direct control — NO rate limiting (we want to measure real response time)
//
// ✓ FIXED: LEFT servo is mirror-mounted — its formula uses '-' to invert direction.
// RIGHT follows normal direction (higher µs = more brake), LEFT is inverted.
//
static void setServos(float left, float right) {
  servo_cmd_L = clampf(left, 0.0f, 1.0f);
  servo_cmd_R = clampf(right, 0.0f, 1.0f);
  servo_us_L = SERVO_RELEASE_L_US + (int)(servo_cmd_L * SERVO_RANGE_US);  // LEFT: 544 → 1678
  servo_us_R = SERVO_RELEASE_R_US - (int)(servo_cmd_R * SERVO_RANGE_US);  // RIGHT inverted (mirror-mounted): 2400 → 1266
  servoWriteUs(PIN_SERVO_LEFT, servo_us_L);
  servoWriteUs(PIN_SERVO_RIGHT, servo_us_R);
}

// ===================== SERVO DIRECTION SELF-TEST =====================
// Runs at boot before arming. Operator must visually confirm each servo
// pulls the correct trailing edge DOWN before proceeding.
static void servoDirectionSelfTest() {
  Serial.println("=============================================");
  Serial.println("  SERVO DIRECTION SELF-TEST");
  Serial.println("=============================================");
  Serial.println("LEFT servo is INVERTED (mirror-mounted).");
  Serial.println("Confirm trailing edge pulls DOWN when commanded.");
  Serial.println("");

  // Step 1: LEFT servo to 50% brake, RIGHT neutral
  Serial.println(">>> LEFT servo -> 50% brake, RIGHT -> neutral");
  Serial.println("    Verify: LEFT trailing edge pulls DOWN");
  setServos(0.5f, 0.0f);
  for (int i = 0; i < 40; i++) { serviceWatchdogIfRunning(); delay(50); }

  // Return to neutral
  setServos(0.0f, 0.0f);
  for (int i = 0; i < 20; i++) { serviceWatchdogIfRunning(); delay(50); }

  // Step 2: RIGHT servo to 50% brake, LEFT neutral
  Serial.println(">>> RIGHT servo -> 50% brake, LEFT -> neutral");
  Serial.println("    Verify: RIGHT trailing edge pulls DOWN");
  setServos(0.0f, 0.5f);
  for (int i = 0; i < 40; i++) { serviceWatchdogIfRunning(); delay(50); }

  // Return to neutral
  setServos(0.0f, 0.0f);

  Serial.println("");
  Serial.println("=============================================");
  Serial.println("  Self-test complete.");
  Serial.println("  Operator must confirm visually before proceeding.");
  Serial.println("  Send 'y' via Serial Monitor to continue.");
  Serial.println("=============================================");

  // Wait for operator confirmation (30s timeout for drone deployment without serial)
  uint32_t selftest_start = millis();
  constexpr uint32_t SELFTEST_TIMEOUT_MS = 30000;
  while (millis() - selftest_start < SELFTEST_TIMEOUT_MS) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'y' || c == 'Y') {
        Serial.println("Confirmed — proceeding to arm.");
        return;
      }
    }
    serviceWatchdogIfRunning();
    delay(50);
  }
  Serial.println("Self-test TIMEOUT (30s) — proceeding without confirmation.");
  Serial.println("Assumes servos are correctly configured from prior bench test.");
}

// ===================== TEST SEQUENCES =====================
// Returns the servo commands for the current time since sequence start.
// Updates test_phase as a side effect.

static void getTestCommands(uint32_t elapsed_ms, float &cmdL, float &cmdR) {
  float elapsed_s = elapsed_ms / 1000.0f;

  switch (TEST_TYPE) {
    case 0: // FREE FLIGHT — neutral the whole time
      test_phase = 0;
      cmdL = 0.0f;
      cmdR = 0.0f;
      break;

    case 1: // BRAKE SWEEP UP: 0% → 25% → 50% → 75% → 100% (30s each)
      if (elapsed_s < 30) {
        test_phase = 0; cmdL = cmdR = 0.0f;
      } else if (elapsed_s < 60) {
        test_phase = 1; cmdL = cmdR = 0.25f;
      } else if (elapsed_s < 90) {
        test_phase = 2; cmdL = cmdR = 0.50f;
      } else if (elapsed_s < 120) {
        test_phase = 3; cmdL = cmdR = 0.75f;
      } else if (elapsed_s < 150) {
        test_phase = 4; cmdL = cmdR = 1.0f;
      } else {
        test_phase = 5; cmdL = cmdR = 0.0f;  // safety neutral
      }
      break;

    case 2: // BRAKE SWEEP DOWN: 100% → 75% → 50% → 25% → 0% (30s each)
      if (elapsed_s < 30) {
        test_phase = 0; cmdL = cmdR = 1.0f;
      } else if (elapsed_s < 60) {
        test_phase = 1; cmdL = cmdR = 0.75f;
      } else if (elapsed_s < 90) {
        test_phase = 2; cmdL = cmdR = 0.50f;
      } else if (elapsed_s < 120) {
        test_phase = 3; cmdL = cmdR = 0.25f;
      } else if (elapsed_s < 150) {
        test_phase = 4; cmdL = cmdR = 0.0f;
      } else {
        test_phase = 5; cmdL = cmdR = 0.0f;  // safety neutral
      }
      break;

    case 3: // TURN RATE TEST
      if (elapsed_s < 30) {
        test_phase = 0; cmdL = 1.0f; cmdR = 0.0f;       // Full left turn
      } else if (elapsed_s < 45) {
        test_phase = 1; cmdL = 0.0f; cmdR = 0.0f;       // Neutral recovery
      } else if (elapsed_s < 75) {
        test_phase = 2; cmdL = 0.0f; cmdR = 1.0f;       // Full right turn
      } else if (elapsed_s < 90) {
        test_phase = 3; cmdL = 0.0f; cmdR = 0.0f;       // Neutral recovery
      } else if (elapsed_s < 120) {
        test_phase = 4; cmdL = 0.50f; cmdR = 0.0f;      // Half left turn
      } else if (elapsed_s < 135) {
        test_phase = 5; cmdL = 0.0f; cmdR = 0.0f;       // Neutral recovery
      } else if (elapsed_s < 165) {
        test_phase = 6; cmdL = 0.0f; cmdR = 0.50f;      // Half right turn
      } else {
        test_phase = 7; cmdL = 0.0f; cmdR = 0.0f;       // Safety neutral
      }
      break;

    case 4: // GLIDE SLOPE — long steady-state for precise glide ratio
      if (elapsed_s < 60) {
        test_phase = 0; cmdL = cmdR = 0.0f;              // No brake — baseline
      } else if (elapsed_s < 120) {
        test_phase = 1; cmdL = cmdR = 0.25f;             // 25% brake
      } else {
        test_phase = 2; cmdL = cmdR = 0.0f;              // Safety neutral
      }
      break;

    case 5: // TURN RATE AT MULTIPLE BRAKE LEVELS
      // Fixed L=1.0 (max brake one side), vary R to change overall brake level.
      // Measures turn rate per unit differential at different symmetric brake settings.
      // R=0.0: sym=0.5, diff=1.0 | R=0.25: sym=0.625, diff=0.75 | R=0.50: sym=0.75, diff=0.50
      if (elapsed_s < 30) {
        test_phase = 0; cmdL = 1.0f; cmdR = 0.0f;       // sym=0.5, diff=1.0
      } else if (elapsed_s < 45) {
        test_phase = 1; cmdL = 0.0f; cmdR = 0.0f;       // Recovery
      } else if (elapsed_s < 75) {
        test_phase = 2; cmdL = 1.0f; cmdR = 0.25f;      // sym=0.625, diff=0.75
      } else if (elapsed_s < 90) {
        test_phase = 3; cmdL = 0.0f; cmdR = 0.0f;       // Recovery
      } else if (elapsed_s < 120) {
        test_phase = 4; cmdL = 1.0f; cmdR = 0.50f;      // sym=0.75, diff=0.50
      } else if (elapsed_s < 135) {
        test_phase = 5; cmdL = 0.0f; cmdR = 0.0f;       // Recovery
      } else if (elapsed_s < 165) {
        test_phase = 6; cmdL = 0.75f; cmdR = 0.75f;     // Full symmetric brake, no turn (baseline)
      } else {
        test_phase = 7; cmdL = 0.0f; cmdR = 0.0f;       // Safety neutral
      }
      break;

    case 6: // STEP RESPONSE — canopy tau measurement
      // Sharp steps with neutral intervals to measure exponential response time
      if (elapsed_s < 10) {
        test_phase = 0; cmdL = 0.0f; cmdR = 0.0f;       // Baseline neutral
      } else if (elapsed_s < 20) {
        test_phase = 1; cmdL = 1.0f; cmdR = 0.0f;       // Step to full left
      } else if (elapsed_s < 30) {
        test_phase = 2; cmdL = 0.0f; cmdR = 0.0f;       // Release — measure decay
      } else if (elapsed_s < 40) {
        test_phase = 3; cmdL = 0.0f; cmdR = 1.0f;       // Step to full right
      } else if (elapsed_s < 50) {
        test_phase = 4; cmdL = 0.0f; cmdR = 0.0f;       // Release — measure decay
      } else if (elapsed_s < 60) {
        test_phase = 5; cmdL = 0.5f; cmdR = 0.0f;       // Half step left
      } else if (elapsed_s < 70) {
        test_phase = 6; cmdL = 0.0f; cmdR = 0.0f;       // Release
      } else if (elapsed_s < 80) {
        test_phase = 7; cmdL = 0.0f; cmdR = 0.5f;       // Half step right
      } else {
        test_phase = 8; cmdL = 0.0f; cmdR = 0.0f;       // Safety neutral
      }
      break;

    case 7: // FLARE TEST — progressive brake levels for deceleration profile
      // Measures sink rate reduction at each brake level for flare tuning
      if (elapsed_s < 30) {
        test_phase = 0; cmdL = cmdR = 0.0f;           // 30s baseline neutral
      } else if (elapsed_s < 40) {
        test_phase = 1; cmdL = cmdR = 0.3f;            // 10s at 30% brake
      } else if (elapsed_s < 50) {
        test_phase = 2; cmdL = cmdR = 0.6f;            // 10s at 60% brake
      } else if (elapsed_s < 60) {
        test_phase = 3; cmdL = cmdR = 0.8f;            // 10s at 80% brake
      } else if (elapsed_s < 70) {
        test_phase = 4; cmdL = cmdR = 1.0f;            // 10s at 100% brake
      } else {
        test_phase = 5; cmdL = cmdR = 0.0f;            // Safety neutral
      }
      break;

    default:
      test_phase = 0;
      cmdL = cmdR = 0.0f;
      break;
  }
}

// ===================== SENSOR SETUP =====================

static void setupBMP280() {
  bmp_ok = bmp.begin(0x76);
  if (!bmp_ok) {
    Serial.println("ERROR: BMP280 not found at 0x76!");
    return;
  }
  // ISSUE 1 FIX: Fast bench-test config — responsive to lift/lower/squish.
  // X2 temp / X4 pressure: measurement time ~13.5ms (datasheet formula: 1 + 2×2 + 2×4 + 0.5)
  // STANDBY_MS_1 = 0.5ms standby (BMP280 register value 0x00) → cycle ~14ms, ~71Hz
  // FILTER_X2 IIR: settles in 2 cycles (~28ms) vs FILTER_X16's 22 cycles (~2.6s)
  // Suppresses single-sample I2C noise while keeping step response under 30ms.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_X2,
                  Adafruit_BMP280::STANDBY_MS_1);
  last_baro_ms = millis();
  last_bmp_read_ms = millis();
  Serial.println("  BMP280 OK (X2/X4/F2/0.5ms — fast bench config, ~14ms cycle)");
  // Reset EMA warmup state so IIR settling is re-awaited after every init.
  bmp_warmup_count = 0;
  first_alt_read = true;
}

static void setupBNO085() {
  bno_ok = bno08x.begin_I2C(0x4A, &Wire);
  if (!bno_ok) {
    Serial.println("ERROR: BNO085 not found at 0x4A!");
    return;
  }
  // Match flight firmware exactly — same sensors, same rates
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);  // 50 Hz — primary heading
  // SH2_ROTATION_VECTOR intentionally NOT enabled — test firmware doesn't use
  // magnetometer blending (see heading comment at line ~715). Saves ~75 B/s I2C bandwidth.
  // Flight firmware enables it at the same rate (5Hz) for mag heading blending.
  bno08x.enableReport(SH2_ACCELEROMETER, 50000);           // 20 Hz
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);    // 50 Hz — yaw rate
  Serial.println("  BNO085 OK (GAME_ROTATION + GYRO + ACCEL)");
}

static void setupGNSS() {
  static uint8_t gnss_rx_buf[1024];
  static uint8_t gnss_tx_buf[4096];  // Teensy 4.x: buffer Serial5 writes so RTK forwarding doesn't block LoRa RX service
  Serial5.addMemoryForRead(gnss_rx_buf, sizeof(gnss_rx_buf));
  Serial5.addMemoryForWrite(gnss_tx_buf, sizeof(gnss_tx_buf));

  Serial5.begin(38400);
  delay(100);

  // Blind mode: attach library, ignore handshake result.
  // The library fully initializes internally even if begin() returns false.
  // Matches production flight firmware (flight.ino line 2814).
  gnss.begin(Serial5);
  Serial.println("  GNSS: Library attached (38400, blind mode)");

  // All config unconditional — matches production flight firmware
  gnss.setNavigationFrequency(5);  // 5Hz — safe for RTK with all constellations (25Hz exceeds F9P RTK limit)
  gnss.setAutoPVT(true);
  // Clean UART1-only setup for ZED-X20P on ArduSimple micro RX1/TX1.
  // Keep the correction port quiet: UBX only out, UBX+RTCM3 in.
  gnss.setUART1Output(COM_TYPE_UBX);
  gnss.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_RTCM3);
  gnss.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);
  gnss.setDynamicModel(DYN_MODEL_AIRBORNE4g);
  gnss_ok = true;
  Serial.println("  GNSS OK (38400, UART1 UBX out, UART1 UBX+RTCM3 in)");
}

static void setupLoRa() {
  // Flight firmware: setRfSwitchPins BEFORE radio.begin()
  radio.setRfSwitchPins(PIN_LORA_RXEN, PIN_LORA_TXEN);

  int st = radio.begin(LORA_FREQ_MHZ);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: LoRa init failed (code %d)\n", st);
    return;
  }
  delay(50);  // ✓ range fix: match flight.ino — SX1262 settling after reset
  // ✓ FIXED v5.7: Enable TCXO if CORE1262-LF has DIO3-powered TCXO.
  // Without this, SX1262 falls back to internal RC oscillator (~±15 ppm),
  // degrading RX sensitivity by 10-20 dB — primary cause of 50m range cutoff.
  st = radio.setTCXO(1.6);
  if (st != RADIOLIB_ERR_NONE) {
    // ✓ range fix: TCXO failure silently fell back to internal RC oscillator,
    // losing 10-20 dB of RX sensitivity. Match GS behavior — hard error.
    Serial.printf("FATAL: setTCXO failed: %d (CORE1262-LF needs DIO3 TCXO at 1.6V)\n", st);
    while (1) { delay(1000); }
  }
  // ✓ FIXED v5.6: Explicitly enable DC-DC regulator for full +20 dBm TX power.
  // CORE1262-LF module has external inductor for DC-DC. Without this call,
  // SX1262 may default to LDO mode, causing voltage sag at high TX power.
  st = radio.setRegulatorDCDC();
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setRegulatorDCDC failed: %d\n", st); return; }
  // ✓ range fix: explicit OCP for +20 dBm (RadioLib default may trip at ~60 mA,
  // silently collapsing TX power to ~+5 dBm). +20 dBm PA needs ~140 mA.
  radio.setCurrentLimit(140.0);
  // ✓ range fix: +2 dB RX sensitivity at +2 mA cost — strictly helps RX range.
  radio.setRxBoostedGainMode(true);
  // ✓ range fix: band-specific image calibration for 430-440 MHz, recovers
  // 1-3 dB of RX image rejection over the factory full-band calibration.
  radio.calibrateImage(430.0, 440.0);
  // Match flight firmware LoRa parameters exactly
  st = radio.setSpreadingFactor(7);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setSF failed: %d\n", st); return; }
  st = radio.setBandwidth(500.0);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setBW failed: %d\n", st); return; }
  st = radio.setCodingRate(5);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setCR failed: %d\n", st); return; }
  st = radio.setOutputPower(LORA_TX_DBM);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setPower failed: %d\n", st); return; }
  st = radio.setCRC(true);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setCRC failed: %d\n", st); return; }
  // v5.5: Sync word + preamble MUST match flight firmware
  st = radio.setSyncWord(0xAE, 0x2B);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setSyncWord failed: %d\n", st); return; }
  st = radio.setPreambleLength(12);  // ✓ FIXED: was 8 — longer preamble improves weak-signal detection at distance
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setPreamble failed: %d\n", st); return; }

  // Attach DIO1 ISR before entering RX (matches working RX test behavior)
  radio.setDio1Action(loraTxDoneISR);

  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: startRX failed: %d\n", st); return; }

  lora_ok = true;
  last_lora_activity_ms = millis();  // v5.6: seed RX recovery watchdog
  Serial.println("  LoRa OK (SF7/BW500/CR4-5, sync=0xAE2B, preamble=12) [high-rate mode]");
}

// ===================== I2C BUS RECOVERY =====================
static void i2cBusRecovery() {
  Serial.println("I2C: Toggling SCL to free bus...");
  Wire.end();
  pinMode(PIN_I2C_SCL, OUTPUT);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  for (int i = 0; i < I2C_SCL_TOGGLE_COUNT; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(5);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(200);
}

static void checkI2CRecovery() {
  // I2C recovery active in ALL states — bus lockup in WAITING
  // would prevent drop detection (accel stops updating, permanent hang)
  uint32_t now = millis();

  if (!bmp_ok || (last_bmp_read_ms > 0 && (now - last_bmp_read_ms) > 500)) bmp_fail_count++;
  if (!bno_ok || (last_bno_read_ms > 0 && (now - last_bno_read_ms) > 500)) bno_fail_count++;

  bmp_fail_count = min(bmp_fail_count, I2C_FAIL_THRESHOLD + 2);
  bno_fail_count = min(bno_fail_count, I2C_FAIL_THRESHOLD + 2);

  if (bmp_fail_count < I2C_FAIL_THRESHOLD && bno_fail_count < I2C_FAIL_THRESHOLD) return;
  if (now - last_i2c_recovery_ms < I2C_RECOVERY_COOLDOWN_MS) return;
  last_i2c_recovery_ms = now;

  Serial.printf("I2C RECOVERY: bmp_fails=%d bno_fails=%d\n", bmp_fail_count, bno_fail_count);
  i2cBusRecovery();

  // Reinit both sensors
  setupBMP280();
  bmp_fail_count = 0;
  setupBNO085();
  bno_fail_count = 0;
}

// ===================== SENSOR READING =====================

static void readBMP280() {
  if (!bmp_ok) return;

  // ISSUE 2 FIX: Rate-limit reads to 40ms (25Hz) — matches main loop rate.
  // Sensor cycle is ~14ms (13.5ms meas + 0.5ms standby), so every 40ms read gets fresh data.
  uint32_t now = millis();
  if (now - last_bmp_read_ms < 40) return;

  double raw_pressure = bmp.readPressure() / 100.0;
  double raw_temp = bmp.readTemperature();

  // Validate: pressure range [300, 1200] hPa, reject NaN
  if (raw_pressure < 300.0 || raw_pressure > 1200.0 ||
      isnan(raw_pressure) || isnan(raw_temp)) {
    bmp_fail_count++;
    return;  // skip update, keep stale data
  }

  // ISSUE 5 FIX: Outlier rejection threshold raised from 2 to 20 hPa.
  // At 40ms read rate, 20 hPa/40ms = 500 hPa/s ≈ 4166 m/s equivalent — far above any
  // flight scenario. Allows enclosure squish events (5–15 hPa instantaneous) through.
  // Genuine I2C glitches (0, NaN, out-of-range) are already caught by the range check above.
  // Do NOT increment bmp_fail_count — a pressure transient is not an I2C failure.
  if (pressure_buf_full && fabs(raw_pressure - pressure_hPa) > 20.0) {
    return;  // discard outlier; keep current moving average; not an I2C error
  }

  bmp_fail_count = 0;
  last_bmp_read_ms = now;
  temp_c = raw_temp;

  // Moving-average filter on pressure — smooths noise and I2C glitches
  if (!pressure_buf_full && pressure_buf_idx == 0) {
    // First valid read: fill entire buffer to avoid startup ramp
    for (int i = 0; i < PRESSURE_AVG_SIZE; i++) pressure_buffer[i] = raw_pressure;
    pressure_buf_full = true;
  }
  pressure_buffer[pressure_buf_idx] = raw_pressure;
  pressure_buf_idx = (pressure_buf_idx + 1) % PRESSURE_AVG_SIZE;
  if (pressure_buf_idx == 0) pressure_buf_full = true;

  // Compute average pressure from buffer
  int count = pressure_buf_full ? PRESSURE_AVG_SIZE : pressure_buf_idx;
  double sum = 0.0;
  for (int i = 0; i < count; i++) sum += pressure_buffer[i];
  pressure_hPa = sum / count;

  // Compute raw altitude from filtered pressure
  double raw_alt;
  if (sea_level_calibrated) {
    raw_alt = 44330.0 * (1.0 - pow(pressure_hPa / sea_level_hpa, 1.0 / 5.255));
  } else {
    raw_alt = 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 1.0 / 5.255));
  }
  baro_alt_raw = raw_alt;  // store for debug display

  // IIR warmup: Re-seed EMA during FILTER_X2 settling period.
  // FILTER_X2 settles in 2 cycles (datasheet Table 7); threshold set to 5 for margin.
  // Without this, the EMA chases the drifting IIR output after init, causing apparent
  // altitude oscillation even when stationary. Reset after every setupBMP280() call.
  if (first_alt_read || bmp_warmup_count < 5) {
    baro_alt = raw_alt;
    last_baro_alt = raw_alt;
    last_baro_ms = now;
    if (first_alt_read) first_alt_read = false;
    bmp_warmup_count++;
    return;  // no sink rate computation during warmup — IIR output not yet stable
  }

  // ISSUE 4 FIX: State-dependent altitude EMA.
  // WAITING:    alpha=0.4 → time constant ~80ms at 40ms reads (τ = -40/ln(0.6)).
  //             Tracks hand lifts and squish events within ~100ms.
  // DESCENDING: alpha=0.3 → NOT MODIFIED — calibrated for flight data quality.
  double alt_alpha = (test_state == DESCENDING) ? 0.3 : 0.4;
  baro_alt = baro_alt + alt_alpha * (raw_alt - baro_alt);

  // Compute sink rate (positive = descending)
  uint32_t dt_ms = now - last_baro_ms;
  if (dt_ms >= 100) {  // 10 Hz — aligned with fast print rate
    double dt_sec = dt_ms / 1000.0;
    double raw = (last_baro_alt - baro_alt) / dt_sec;
    // NaN guard — prevent corrupted baro from poisoning the filter permanently
    if (!isnan(raw) && !isinf(raw)) {
      sink_rate_raw = raw;
    }
    // NO CLAMPING — we want raw data for calibration
    sink_rate_filtered = sink_rate_filtered * (1.0 - SINK_FILTER_ALPHA) + sink_rate_raw * SINK_FILTER_ALPHA;
    last_baro_alt = baro_alt;
    last_baro_ms = now;
  }
}

static inline void serviceRTKHotPath();  // forward declaration for BNO085 loop

static void readBNO085() {
  if (!bno_ok) return;
  int event_count = 0;
  // ✓ FIXED v5.7: Cap at 5 events per call (was 50). BNO085 can buffer many events,
  // and draining all of them blocks the CPU for up to 250ms (50 × 5ms I2C).
  // At 5 events max (~25ms), the LoRa radio is not starved between serviceRTKHotPath() calls.
  while (bno08x.getSensorEvent(&bnoValue) && event_count < 5) {
    event_count++;
    // ✓ FIXED v5.7: Service LoRa between BNO085 events to prevent RTK packet loss
    // during I2C-heavy sensor reads.
    serviceRTKHotPath();
    last_bno_read_ms = millis();
    bno_fail_count = 0;

    // Accelerometer — with NaN/Inf guard (matches flight firmware)
    if (bnoValue.sensorId == SH2_ACCELEROMETER) {
      float ax = bnoValue.un.accelerometer.x;
      float ay = bnoValue.un.accelerometer.y;
      float az = bnoValue.un.accelerometer.z;
      if (isFiniteF(ax) && isFiniteF(ay) && isFiniteF(az)) {
        accel_x = ax; accel_y = ay; accel_z = az;
        accel_mag = sqrtf(ax * ax + ay * ay + az * az);
      }
    }

    // Primary heading: GAME_ROTATION_VECTOR (no mag)
    // NOTE: Flight firmware blends SH2_ROTATION_VECTOR (mag-corrected) at 5Hz
    // during straight flight (turn rate < 0.05 rad/s). Test firmware intentionally
    // omits this blending to capture raw gyro heading for calibration. The resulting
    // HEADING_OFFSET_RAD may have ~1-5° systematic bias vs flight heading.
    if (bnoValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = bnoValue.un.gameRotationVector.real;
      float qx = bnoValue.un.gameRotationVector.i;
      float qy = bnoValue.un.gameRotationVector.j;
      float qz = bnoValue.un.gameRotationVector.k;

      // NaN/Inf guard on quaternion (matches flight firmware)
      if (!isFiniteF(qw) || !isFiniteF(qx) || !isFiniteF(qy) || !isFiniteF(qz)) continue;

      float siny_cosp = 2.0f * (qw * qz + qx * qy);
      float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
      float new_heading = wrapAngle(atan2f(siny_cosp, cosy_cosp));  // [-PI, PI] matches flight firmware

      uint32_t now = millis();
      float delta = heading_valid ? wrapAngle(new_heading - heading_rad) : 0.0f;

      // Heading jump filter (matches flight firmware: reject >1.2 rad within 150ms)
      if (heading_valid && (now - last_heading_ms) < HEADING_JUMP_MAX_DT_MS) {
        if (fabsf(delta) > HEADING_JUMP_MAX_RAD) continue;
      }

      // Heading confidence (matches flight firmware caps)
      uint32_t dt_ms = (last_heading_ms > 0) ? (now - last_heading_ms) : 0;
      if (heading_valid && last_heading_ms > 0) {
        float delta_factor = 1.0f - clampf(fabsf(delta) / HEADING_CONFIDENCE_MAX_DELTA_RAD, 0.0f, 1.0f);
        float dt_factor = 1.0f - clampf((float)dt_ms / (float)HEADING_CONFIDENCE_MAX_DT_MS, 0.0f, 1.0f);
        heading_conf = clampf(delta_factor * dt_factor, 0.0f, 0.85f);  // cap at 0.85 matches flight firmware
      }

      heading_rad = new_heading;
      heading_valid = true;
      last_heading_ms = now;
      last_heading_rad = heading_rad;
    }

    // Calibrated gyroscope — with NaN/Inf guard
    if (bnoValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      float gz = bnoValue.un.gyroscope.z;
      if (isFiniteF(gz)) {
        gyro_z = gz;
      }
    }
  }
}

static inline bool rtkBridgeBusy();

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
static bool getNMEAField(const char *sentence, int targetField, char *out, size_t outLen) {
  if (outLen == 0) return false;
  out[0] = '\0';
  if (sentence == nullptr) return false;
  const char *p = sentence;
  if (*p == '$') p++;
  int field = 0;
  while (*p) {
    if (field == targetField) {
      size_t o = 0;
      while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') {
        if (o + 1 < outLen) out[o++] = *p;
        p++;
      }
      out[o] = '\0';
      return true;
    }
    if (*p == ',') field++;
    else if (*p == '*' || *p == '\r' || *p == '\n') break;
    p++;
  }
  return false;
}

static void updateGNSSGGADiagnostics() {
  NMEA_GGA_data_t gga;
  uint8_t freshness = 0;
  memset(&gga, 0, sizeof(gga));

  freshness = gnss.getLatestNMEAGNGGA(&gga);
  if (freshness == 0) {
    memset(&gga, 0, sizeof(gga));
    freshness = gnss.getLatestNMEAGPGGA(&gga);
  }
  if (freshness == 0 || gga.length == 0) return;

  char sentence[NMEA_GGA_MAX_LENGTH + 1];
  size_t n = (gga.length < NMEA_GGA_MAX_LENGTH) ? gga.length : NMEA_GGA_MAX_LENGTH;
  memcpy(sentence, gga.nmea, n);
  sentence[n] = '\0';

  char field[24];
  if (getNMEAField(sentence, 6, field, sizeof(field)) && field[0] != '\0') {
    gnss_gga_quality = (uint8_t)atoi(field);
  }
  if (getNMEAField(sentence, 13, field, sizeof(field)) && field[0] != '\0') {
    gnss_gga_diff_age_s = atof(field);
  } else {
    gnss_gga_diff_age_s = -1.0f;
  }
  if (getNMEAField(sentence, 14, field, sizeof(field)) && field[0] != '\0') {
    strncpy(gnss_gga_station_id, field, sizeof(gnss_gga_station_id) - 1);
    gnss_gga_station_id[sizeof(gnss_gga_station_id) - 1] = '\0';
  } else {
    gnss_gga_station_id[0] = '\0';
  }
  gnss_gga_seen = true;
  gnss_last_gga_ms = millis();
}
#endif

static void readGNSS() {
  if (!gnss_ok) return;
  if (rtkBridgeBusy()) return;
  gnss.checkUblox();
  if (gnss.getPVT()) {
    double new_lat  = gnss.getLatitude() * 1e-7;
    double new_lon  = gnss.getLongitude() * 1e-7;
    double new_alt  = gnss.getAltitudeMSL() / 1000.0;
    double new_gs   = gnss.getGroundSpeed() / 1000.0;
    double new_trk  = gnss.getHeading() * 1e-5;
    double new_vv   = gnss.getNedDownVel() / 1000.0;

    // NaN/Inf guard — matches flight firmware
    if (isnan(new_lat) || isnan(new_lon) || isnan(new_alt) ||
        isnan(new_gs)  || isnan(new_vv)  ||
        isinf(new_lat) || isinf(new_lon) || isinf(new_alt) ||
        isinf(new_gs)  || isinf(new_vv)) {
      return;  // Skip corrupted GNSS update
    }

    lat            = new_lat;
    lon            = new_lon;
    gnss_alt_msl   = new_alt;
    ground_speed   = new_gs;
    gnss_track_deg = new_trk;
    gnss_vert_vel  = new_vv;
    gnss_fix       = gnss.getFixType();
    gnss_carrier_solution = gnss.getCarrierSolutionType();
    gnss_sats      = gnss.getSIV();
    gnss_hacc_mm   = (uint32_t)gnss.getHorizontalAccEst();  // PVT-based (mm, non-blocking)
    gnss_vacc_mm   = (uint32_t)gnss.getVerticalAccEst();    // PVT-based (mm, non-blocking)
    gnss_fix_ok    = gnss.getGnssFixOk();
    gnss_diff_soln = gnss.getDiffSoln();
    gnss_pdop_x100 = gnss.getPDOP();
    last_gnss_ms   = millis();
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
    updateGNSSGGADiagnostics();
#endif

    // QNH calibration: match flight firmware (10 samples, vacc < 2000mm)
    if (!sea_level_calibrated && gnss_fix >= 3 && pressure_hPa > 800 && gnss_vacc_mm < 2000) {
      sea_level_accum += pressure_hPa / pow(1.0 - gnss_alt_msl / 44330.0, 5.255);
      sea_level_samples++;
      if (sea_level_samples >= 10) {
        sea_level_hpa = sea_level_accum / 10.0;
        sea_level_calibrated = true;
        Serial.printf("Baro calibrated: QNH = %.1f hPa (10-sample avg)\n", sea_level_hpa);
      }
    }
  }
}

// ===================== SD LOGGING (25 Hz) =====================

static void logToSD() {
  if (!logFile || sd_disabled) return;
  if (test_state == LANDED) return;  // stop logging after landing

  uint32_t now = millis();
  // 25 Hz = every 40 ms
  if (now - last_log_ms < 40) return;
  last_log_ms = now;

  int written = logFile.printf(
    "%lu,%.2f,%.2f,%.2f,%.7f,%.7f,%.2f,%.2f,%.1f,%d,%d,%lu,%lu,%.3f,"
    "%.3f,%.3f,%.3f,%.3f,%.4f,%.3f,%.4f,%.3f,%.3f,%d,%d,"
    "%d,%.2f,%.2f,%d\n",
    now,
    pressure_hPa, temp_c, baro_alt,
    lat, lon, gnss_alt_msl, ground_speed, gnss_track_deg,
    gnss_fix, gnss_sats, gnss_hacc_mm, gnss_vacc_mm, gnss_vert_vel,
    accel_x, accel_y, accel_z, accel_mag,
    heading_rad, heading_conf, gyro_z,
    servo_cmd_L, servo_cmd_R, servo_us_L, servo_us_R,
    test_phase, sink_rate_raw, sink_rate_filtered,
    gnss_carrier_solution
  );
  if (written > 0) {
    log_line_count++;
    sd_write_fail_count = 0;
  } else {
    sd_write_fail_count++;
    if (sd_write_fail_count == SD_FAIL_WARN_THRESHOLD) {
      Serial.println("WARNING: SD write failing — check card!");
    }
    // ✓ FIXED v5.9: Disable SD after persistent failures to protect SPI bus.
    // A stuck/ejected SD card can hold SPI0 busy, blocking LoRa operations
    // and killing telemetry. Close the file and stop all SD access.
    if (sd_write_fail_count >= SD_FAIL_DISABLE_THRESHOLD) {
      Serial.println("ERROR: SD disabled after 30 consecutive failures — LoRa protected");
      logFile.close();
      sd_disabled = true;
      return;
    }
  }

  // ✓ FIXED v5.7: Flush every 10 seconds (was 3). SD flush blocks SPI0 for 10-50ms,
  // which shares the bus with LoRa. Reducing frequency cuts SPI contention by ~3x
  // while still protecting against data loss on crash.
  // ✓ FIXED v5.8: Service RTK before/after flush to minimize LoRa blackout during SPI0 hold.
  if (now - last_flush_ms >= 10000) {
    serviceRTKHotPath();  // drain pending LoRa before blocking SPI0
    logFile.flush();
    last_flush_ms = now;
    serviceRTKHotPath();  // immediately resume LoRa after SPI0 released
  }
}

// ===================== RTK CORRECTION RECEIVE =====================

static void enqueueRTK(const uint8_t *data, size_t len) {
  if (len == 0) return;

  rtk_packets_in++;
  last_rtk_rx_ms = millis();
  rtk_lora_bytes_in += (uint32_t)len;

  for (size_t i = 0; i < len; i++) {
    if (rtk_q_len >= RTK_QUEUE_BYTES) {
      rtk_queue_drop_bytes += (uint32_t)(len - i);
      break;
    }
    rtk_queue[rtk_q_head] = data[i];
    rtk_q_head = (rtk_q_head + 1) % RTK_QUEUE_BYTES;
    rtk_q_len++;
  }
}

static inline bool rtkBridgeBusy() {
  if (rtk_q_len > 0) return true;
  if (last_rtk_rx_ms == 0) return false;
  return (millis() - last_rtk_rx_ms) < 120;  // keep GNSS parser / prints out of the way just after each RTK packet
}

static void serviceRTKToGNSS() {
  if (rtk_q_len == 0) return;

  int avail = Serial5.availableForWrite();
  if (avail <= 0) return;

  size_t chunk = rtk_q_len;
  if (chunk > RTCM_UART_CHUNK_BYTES) chunk = RTCM_UART_CHUNK_BYTES;
  if ((size_t)avail < chunk) chunk = (size_t)avail;
  if (chunk == 0) return;

  uint8_t temp[RTCM_UART_CHUNK_BYTES];
  for (size_t i = 0; i < chunk; i++) {
    temp[i] = rtk_queue[(rtk_q_tail + i) % RTK_QUEUE_BYTES];
  }

  size_t w = Serial5.write(temp, chunk);
  if (w == 0) {
    rtk_uart_short_writes++;
    return;
  }

  rtk_q_tail = (rtk_q_tail + w) % RTK_QUEUE_BYTES;
  rtk_q_len -= w;
  rtk_uart_bytes_out += (uint32_t)w;
  if (w < chunk) rtk_uart_short_writes++;
}

static inline void safeStartReceive();  // forward declaration
static uint32_t rtk_crc_fail_count = 0;  // ✓ FIXED v5.8: CRC validation counter (must be before pollLoRa)
static uint32_t gs_ping_rx_count = 0;    // ✓ v5.10: GS LoRa heartbeat ping counter
static uint8_t  gs_ping_last_rx_telem = 0; // last rx_telem byte from GS ping (how many telem pkts GS received)

static void pollLoRa() {
  if (!lora_ok) return;

  // ✓ FIXED: Atomically snapshot and clear ISR-shared flags to prevent race condition.
  // Without noInterrupts(), the ISR can fire between reading and clearing these flags,
  // causing lost DIO1 events that permanently stall RX polling.
  noInterrupts();
  bool has_event = lora_dio1_flag || (lora_dio1_pending > 0);
  interrupts();
  if (!has_event) return;

  // Drain all pending IRQ events to avoid dropping RX opportunities.
  // Each iteration atomically claims one pending event.
  while (true) {
    noInterrupts();
    bool got_flag = lora_dio1_flag;
    uint32_t pending = lora_dio1_pending;
    if (!got_flag && pending == 0) { interrupts(); break; }
    lora_dio1_flag = false;
    if (pending > 0) lora_dio1_pending = pending - 1;
    interrupts();

    if (lora_tx_pending) {
      // DIO1 can be TX-done when telemetry send completes
      radio.finishTransmit();
      safeStartReceive();
      lora_tx_pending = false;
      lora_tx_done_flag = false;
      lora_tx_done_count++;
      last_lora_activity_ms = millis();  // ✓ v5.8: moved from ISR
      continue;
    }

    uint8_t buf[255];
    int len = radio.getPacketLength();
    if (len <= 0 || len > (int)sizeof(buf)) {
      safeStartReceive();
      continue;
    }

    int err = radio.readData(buf, len);
    if (err == RADIOLIB_ERR_NONE && len > 0) {
      lora_rx_read_ok++;
      lora_rx_any_pkts++;
      lora_rx_any_bytes += (uint32_t)len;
      // ✓ range fix: rate-limited link-margin diagnostics (500ms)
      {
        static uint32_t last_rssi_print_ms = 0;
        if (millis() - last_rssi_print_ms > 500) {
          last_rssi_print_ms = millis();
          Serial.printf("LoRa RX: type=0x%02X len=%u rssi=%.1fdBm snr=%.1fdB\n",
                        buf[0], (unsigned)len, radio.getRSSI(), radio.getSNR());
        }
      }
      if (buf[0] == MSG_RTK && len > 3) {
        // Soft CRC removed — hardware LoRa CRC still active on SX1262 (setCRC true)
        // Frame: [MSG_RTK][seq_hi][seq_lo][RTCM payload]
        uint16_t seq = ((uint16_t)buf[1] << 8) | buf[2];
        if (rtk_seq_initialized) {
          if (seq != rtk_expected_seq) {
            uint16_t gap = (uint16_t)(seq - rtk_expected_seq);
            if (gap < 1000) { rtk_seq_gaps++; rtk_seq_missed_total += gap; }
          }
        }
        rtk_expected_seq = seq + 1;
        rtk_seq_initialized = true;
        enqueueRTK(&buf[3], (size_t)(len - 3));
      } else if (buf[0] == MSG_GS_PING && len >= 4) {
        // ✓ v5.10: GS LoRa heartbeat ping — confirms RF link is alive
        // Frame: [0x04][seq_hi][seq_lo][gs_rx_telem_count]
        gs_ping_rx_count++;
        gs_ping_last_rx_telem = buf[3];  // how many telem pkts the GS has received (low byte)
      }
    } else {
      lora_rx_read_err++;
    }
    safeStartReceive();
    last_lora_activity_ms = millis();  // ✓ v5.8: update activity timestamp on RX event
  }
}

static void checkLoRaTxDone();

// ✓ FIXED v5.8: Checked startReceive wrapper — a failed startReceive leaves
// the radio in standby, deaf to all packets. Retry once on failure.
static uint32_t lora_start_rx_err_count = 0;
static inline void safeStartReceive() {
  int err = radio.startReceive();
  if (err != RADIOLIB_ERR_NONE) {
    lora_start_rx_err_count++;
    delayMicroseconds(100);
    radio.startReceive();
  }
}

static inline void serviceRTKHotPath() {
  checkLoRaTxDone();
  if (!lora_tx_pending) pollLoRa();
  serviceRTKToGNSS();

  // ✓ FIXED v5.6: RX recovery watchdog — if no DIO1 activity for 500ms while not
  // transmitting, force radio back to RX mode. This recovers from stuck radio states
  // caused by missed DIO1 interrupts or SPI glitches that leave the radio idle.
  if (lora_ok && !lora_tx_pending && last_lora_activity_ms > 0) {
    if (millis() - last_lora_activity_ms > 500) {
      safeStartReceive();
      last_lora_activity_ms = millis();
      lora_rx_recovery_count++;
    }
  }
}

// ===================== LORA TX DONE ISR =====================

static void loraTxDoneISR() {
  lora_tx_done_flag = true;
  lora_dio1_flag = true;
  if (lora_dio1_pending < 1000000UL) lora_dio1_pending++;
  lora_dio1_isr_count++;
  // ✓ FIXED v5.8: Removed millis() from ISR — updated in checkLoRaTxDone() and pollLoRa() instead
}

// ===================== LORA TELEMETRY (1 Hz) =====================

static void sendTelemetry() {
  uint32_t now = millis();
  if (now - last_telemetry_ms < TELEMETRY_PERIOD_MS) return;
  last_telemetry_ms = now;

  TelemetryPacket p{};
  p.msgType = MSG_TELEMETRY;
  p.time_ms = now;
  p.lat_e7 = (int32_t)llround(lat * 1e7);
  p.lon_e7 = (int32_t)llround(lon * 1e7);
  p.height_agl_dm = (int16_t)clampf((float)(baro_alt * 10.0), -32768, 32767);  // NOTE: sends baro MSL, not AGL (no target alt in test mode)
  p.pressure_hPa_x10 = (uint16_t)clampf((float)(pressure_hPa * 10.0), 0, 65535);
  p.temp_c_x10 = (int16_t)clampf((float)(temp_c * 10.0), -32768, 32767);
  p.ground_speed_cms = (uint16_t)clampf((float)(ground_speed * 100.0), 0, 65535);
  p.servo_left_x1000 = (int16_t)clampf(servo_cmd_L * 1000.0f, -1000, 1000);
  p.servo_right_x1000 = (int16_t)clampf(servo_cmd_R * 1000.0f, -1000, 1000);
  // Quality flags in unused pred_lat_e7 field
  {
    uint32_t qf = 0;
    if (sea_level_calibrated) qf |= 0x01;
    if (heading_valid)        qf |= 0x02;
    if (gnss_fix >= 3)        qf |= 0x04;
    p.pred_lat_e7 = (int32_t)qf;
  }
  p.pred_lon_e7 = 0;
  p.wind_north_cms = (int16_t)gnss_fix;
  p.wind_east_cms = (int16_t)min(gnss_hacc_mm, (uint32_t)32767);
  p.heading_deg_x10 = (int16_t)clampf((float)(heading_rad * (180.0f / PI) * 10.0f), -32768, 32767);
  p.heading_confidence_x1000 = (uint16_t)clampf(heading_conf * 1000.0f, 0.0f, 1000.0f);
  p.wind_rejects = (uint16_t)TEST_TYPE;
  p.wind_layer_rejects = (uint16_t)max(test_phase, 0);  // clamp -1 to 0 for uint16
  p.mission_state = (uint8_t)test_state;
  p.gnss_carr_soln = gnss_carrier_solution;
  p.gnss_num_sats  = gnss_sats;
  p.crc16 = crc16_ccitt((uint8_t *)&p, sizeof(p) - 2);

  if (lora_ok) {
    // ✓ FIXED: guard against double-TX — don't start new TX if previous still pending
    if (lora_tx_pending) return;

    // ✓ FIXED: Atomically clear ISR-shared flags before starting TX
    noInterrupts();
    lora_tx_done_flag = false;
    lora_dio1_flag = false;
    interrupts();
    int err = radio.startTransmit((uint8_t *)&p, sizeof(p));
    if (err == RADIOLIB_ERR_NONE) {
      lora_tx_pending = true;
      lora_tx_start_ms = now;
      last_lora_activity_ms = now;  // v5.6: seed RX recovery watchdog
      lora_tx_start_count++;
    } else {
      // ✓ FIXED: recover to RX on startTransmit failure — prevents radio stuck in standby
      safeStartReceive();
      last_lora_activity_ms = now;  // v5.6: seed RX recovery watchdog
    }
  }
}

// v5.5: DIO1 interrupt TX completion with 350ms safety timeout
// v5.6: ISR race condition fix — atomically read/clear shared flags
static void checkLoRaTxDone() {
  if (!lora_tx_pending || !lora_ok) return;

  // ✓ FIXED: Atomically read ISR-shared flags to prevent race condition
  noInterrupts();
  bool tx_done = lora_tx_done_flag;
  bool dio1 = lora_dio1_flag;
  interrupts();

  // Normal path: TX done interrupt observed
  if (tx_done || dio1) {
    radio.finishTransmit();
    safeStartReceive();
    lora_tx_pending = false;
    noInterrupts();
    lora_tx_done_flag = false;
    lora_dio1_flag = false;
    interrupts();
    lora_tx_done_count++;
    last_lora_activity_ms = millis();  // ✓ v5.8: moved from ISR
    return;
  }

  // Safety timeout if DIO1 interrupt is missed
  if (millis() - lora_tx_start_ms > LORA_TX_TIMEOUT_MS) {
    radio.finishTransmit();
    safeStartReceive();
    lora_tx_pending = false;
    noInterrupts();
    lora_tx_done_flag = false;
    lora_dio1_flag = false;
    interrupts();
    lora_tx_timeout_count++;
    last_lora_activity_ms = millis();  // ✓ v5.8: update on timeout recovery
  }
}

// ===================== DROP DETECTION =====================

static void checkDropDetection() {
  uint32_t now = millis();

  // Method 1: Free-fall detection (accel near zero)
  if (accel_mag < FREEFALL_THRESHOLD && accel_mag > 0.1f) {
    freefall_count++;
    if (freefall_count >= FREEFALL_COUNT_NEEDED) {
      drop_detected_ms = now;
      drop_alt_msl = gnss_alt_msl;
      test_state = DESCENDING;
      sequence_start_ms = now + DEPLOY_DELAY_MS;
      Serial.printf(">>> DROP DETECTED (free-fall) at %.1f m MSL\n", drop_alt_msl);
      Serial.printf(">>> Sequence starts in %lu ms\n", DEPLOY_DELAY_MS);
      // BUG 3 FIX: Re-anchor altitude references at the moment of drop detection.
      // Without this, the first sink_rate_raw after the alpha transition (0.05 → 0.3)
      // is computed from a last_baro_alt accumulated under the slow EMA, producing a
      // spurious spike in the first seconds of descent data.
      last_baro_alt = baro_alt;
      sink_rate_raw = 0.0;
      sink_rate_filtered = 0.0;
      if (!sea_level_calibrated) {
        Serial.println(">>> WARNING: QNH NOT CALIBRATED — baro altitude using default 1013.25 hPa!");
        Serial.println(">>> Sink rate data will have absolute altitude offset.");
      }
      return;
    }
  } else {
    freefall_count = 0;
  }

  // Method 2: Timer fallback (disabled when TIMER_FALLBACK_MS == 0)
  if (TIMER_FALLBACK_MS > 0 && now - power_on_ms >= TIMER_FALLBACK_MS) {
    drop_detected_ms = now;
    drop_alt_msl = gnss_alt_msl;
    test_state = DESCENDING;
    sequence_start_ms = now + DEPLOY_DELAY_MS;
    Serial.printf(">>> TIMER FALLBACK triggered at %.1f m MSL\n", drop_alt_msl);
    Serial.printf(">>> Sequence starts in %lu ms\n", DEPLOY_DELAY_MS);
    // BUG 3 FIX: same anchor reset as free-fall branch (see above)
    last_baro_alt = baro_alt;
    sink_rate_raw = 0.0;
    sink_rate_filtered = 0.0;
    if (!sea_level_calibrated) {
      Serial.println(">>> WARNING: QNH NOT CALIBRATED — baro altitude using default 1013.25 hPa!");
      Serial.println(">>> Sink rate data will have absolute altitude offset.");
    }
  }
}

// ===================== LANDING DETECTION =====================
// Matches flight firmware v5.5: baro sink rate + GNSS ground speed cross-check
// ✓ FIXED v5.6: Added altitude-change guard to prevent false LANDED on bench.
// On bench, CanSat is stationary from boot → baro_still && gnss_still are trivially true
// after 30s, causing premature LANDED at ~50s. Now also requires actual descent
// (altitude must have dropped at least 5m from drop point) before landing can trigger.

static void checkLanding() {
  if (test_state != DESCENDING) return;
  if (millis() - sequence_start_ms < 30000) return;  // min 30s of descent

  // Guard: require actual altitude change from drop point before allowing landing detection.
  // This prevents false LANDED when CanSat is stationary on bench (no real descent occurred).
  double alt_dropped = drop_alt_msl - baro_alt;
  if (alt_dropped < 5.0) return;  // haven't descended at least 5m — not a real landing

  bool baro_still = (fabsf(sink_rate_filtered) < 0.3);
  bool gnss_still = (ground_speed < 0.5) || (gnss_fix < 3);
  // Require 2x longer hold when GNSS has no fix (reduce false landing risk)
  uint32_t required_hold_ms = (gnss_fix >= 3) ? 1000 : 2000;

  if (baro_still && gnss_still) {
    if (landing_still_start == 0) landing_still_start = millis();
    if (millis() - landing_still_start > required_hold_ms) {
      test_state = LANDED;
      setServos(0.0f, 0.0f);
      Serial.printf(">>> LANDED (dropped %.1fm) — servos neutral, logging stopped\n", alt_dropped);
      // Final flush
      if (logFile) {
        logFile.flush();
        logFile.close();
      }
    }
  } else {
    landing_still_start = 0;
  }
}

// ===================== SERIAL STATUS =====================

// ===================== HARDWARE WATCHDOG (matches flight firmware) =====================

#ifndef WDOG_CS_CMD32EN
#define WDOG_CS_CMD32EN ((uint32_t)(1 << 13))
#endif

static void setupWatchdog() {
  if (!ENABLE_WATCHDOG) {
    Serial.println("Watchdog disabled in v2.6 (test stability mode)");
    return;
  }
  __disable_irq();
  WDOG3_CNT = 0xD928C520;  // unlock
  WDOG3_TOVAL = (WATCHDOG_TIMEOUT_MS * 32768UL / 1000UL);
  WDOG3_CS = WDOG_CS_EN | WDOG_CS_CLK(1) | WDOG_CS_UPDATE | WDOG_CS_CMD32EN;
  __enable_irq();
  Serial.printf("Watchdog enabled: %lu ms timeout\n", WATCHDOG_TIMEOUT_MS);
}

static inline void kickWatchdog() {
  if (!ENABLE_WATCHDOG) return;
  noInterrupts();
  WDOG3_CNT = 0xB480A602;
  interrupts();
}

// ===================== SERIAL STATUS (LEGACY — commented out) =====================
// Renamed from printStatus(). Kept for easy revert if ANSI dashboard is not desired.
/*
static void printStatusLegacy() {
  uint32_t now = millis();

  // Fast 10Hz compact altitude print
  static uint32_t last_fast_print = 0;
  if (now - last_fast_print >= 100) {
    last_fast_print = now;
    Serial.printf("ALT %.2f (raw:%.2f) P:%.2f T:%.1f SR:%.2f\n",
                  baro_alt, baro_alt_raw, pressure_hPa, temp_c, sink_rate_filtered);
  }

  // Full status at 1 Hz
  static uint32_t last_print = 0;
  if (now - last_print < 1000) return;
  last_print = now;

  const char* state_names[] = {"WAITING", "DESCENDING", "LANDED"};
  const char* test_names[] = {"FREE_FLIGHT", "SWEEP_UP", "SWEEP_DOWN", "TURN_RATE", "GLIDE_SLOPE", "TURN_AT_BRAKE", "STEP_RESPONSE", "FLARE_TEST"};
  const char* test_name = (TEST_TYPE >= 0 && TEST_TYPE <= 7) ? test_names[TEST_TYPE] : "UNKNOWN";

  Serial.println("--------------------------------------------");
  Serial.printf("State: %s | Test: %s | Phase: %d\n",
                state_names[test_state], test_name, test_phase);
  const char* fix_names[] = {"No fix", "Dead reck", "2D", "3D", "3D+DGNSS", "Time-only"};
  const char* fix_name = (gnss_fix <= 5) ? fix_names[gnss_fix] : "Unknown";
  const char* carr_names[] = {"None", "Float", "Fixed"};
  const char* carr_name = (gnss_carrier_solution <= 2) ? carr_names[gnss_carrier_solution] : "?";
  Serial.printf("GNSS: %s sats=%d hacc=%lumm RTK=%s lat=%.6f lon=%.6f alt=%.1f\n",
                fix_name, gnss_sats, gnss_hacc_mm, carr_name, lat, lon, gnss_alt_msl);
  Serial.printf("Baro: %.1f hPa | Temp: %.1f C | Alt: %.1f m (raw:%.1f) | Sink: %.2f m/s\n",
                pressure_hPa, temp_c, baro_alt, baro_alt_raw, sink_rate_filtered);
  Serial.printf("IMU:  accel=%.2f m/s2 | heading=%.1f deg (conf=%.2f) | gyro_z=%.3f rad/s\n",
                accel_mag, heading_rad * 180.0f / PI, heading_conf, gyro_z);
  if (bmp_fail_count > 0) Serial.printf("  BMP fails: %d\n", bmp_fail_count);
  Serial.printf("Servo: L=%.2f (%d us) R=%.2f (%d us)\n",
                servo_cmd_L, servo_us_L, servo_cmd_R, servo_us_R);
  Serial.printf("SD: %lu lines | QNH: %s (%.1f hPa)\n",
                log_line_count,
                sea_level_calibrated ? "OK" : "PENDING",
                sea_level_hpa);
  if (test_state == WAITING) {
    if (TIMER_FALLBACK_MS > 0) {
      uint32_t remaining = 0;
      if (TIMER_FALLBACK_MS > (now - power_on_ms)) {
        remaining = (TIMER_FALLBACK_MS - (now - power_on_ms)) / 1000;
      }
      Serial.printf("Timer fallback in: %lu s\n", remaining);
    } else {
      Serial.println("Timer fallback: DISABLED (free-fall only)");
    }
  }
  Serial.println("--------------------------------------------");
}
*/

// ===================== SERIAL STATUS =====================
// Plain-text scrolling status block. Rate-limited to 2 Hz internally.

static void printStatus() {
  static uint32_t last_print_ms = 0;
  uint32_t now = millis();
  uint32_t min_period = rtkBridgeBusy() ? 1000 : (FULL_STATUS_VERBOSE ? 500 : 2000);
  if (now - last_print_ms < min_period) return;
  last_print_ms = now;

  if (!FULL_STATUS_VERBOSE) {
    uint32_t rtk_age = (last_rtk_rx_ms > 0) ? (now - last_rtk_rx_ms) : 999999;
    uint32_t rtk_dt_ms = (prev_rtk_status_ms > 0) ? (now - prev_rtk_status_ms) : 0;
    uint32_t lora_delta = rtk_lora_bytes_in - prev_rtk_lora_bytes_in;
    uint32_t uart_delta = rtk_uart_bytes_out - prev_rtk_uart_bytes_out;
    uint32_t rtk_drop_delta = (rtk_queue_drop_bytes + rtk_uart_drop_bytes) - prev_rtk_drop_bytes;
    uint32_t lora_bps = (rtk_dt_ms > 0) ? (uint32_t)((1000.0f * lora_delta) / rtk_dt_ms) : 0;
    uint32_t uart_bps = (rtk_dt_ms > 0) ? (uint32_t)((1000.0f * uart_delta) / rtk_dt_ms) : 0;

    Serial.printf("RTK lora_Bps=%lu uart_Bps=%lu q=%u age=%lums drop_d=%lu fix=%u rtk=%u sats=%u\n",
                  lora_bps, uart_bps, (unsigned)rtk_q_len, rtk_age, rtk_drop_delta,
                  (unsigned)gnss_fix, (unsigned)gnss_carrier_solution, (unsigned)gnss_sats);

    prev_rtk_status_ms = now;
    prev_rtk_lora_bytes_in = rtk_lora_bytes_in;
    prev_rtk_uart_bytes_out = rtk_uart_bytes_out;
    prev_rtk_drop_bytes = rtk_queue_drop_bytes + rtk_uart_drop_bytes;
    return;
  }

  // State/test name lookup
  const char* state_names[] = {"WAITING", "DESCENDING", "LANDED"};
  const char* state_name = (test_state <= 2) ? state_names[test_state] : "UNKNOWN";
  const char* test_names[] = {"FREE_FLIGHT", "SWEEP_UP", "SWEEP_DOWN", "TURN_RATE", "GLIDE_SLOPE", "TURN_AT_BRAKE", "STEP_RESPONSE", "FLARE_TEST"};
  const char* test_name = (TEST_TYPE >= 0 && TEST_TYPE <= 7) ? test_names[TEST_TYPE] : "UNKNOWN";
  const char* fix_names[] = {"No fix", "Dead reck", "2D", "3D", "3D+DGNSS", "Time-only"};
  const char* fix_name = (gnss_fix <= 5) ? fix_names[gnss_fix] : "Unknown";
  const char* carr_names[] = {"None", "Float", "Fixed"};
  const char* carr_name = (gnss_carrier_solution <= 2) ? carr_names[gnss_carrier_solution] : "?";

  char phase_str[16];
  if (test_phase == -1) snprintf(phase_str, sizeof(phase_str), "DEPLOY_WAIT");
  else snprintf(phase_str, sizeof(phase_str), "%d", test_phase);

  char qnh_str[24];
  if (sea_level_calibrated) snprintf(qnh_str, sizeof(qnh_str), "OK (%.1f hPa)", sea_level_hpa);
  else snprintf(qnh_str, sizeof(qnh_str), "PENDING");

  // ✓ FIXED v5.8: Interleave serviceRTKHotPath() every 2-3 prints to keep
  // max RTK blackout under ~10ms (was ~30ms between single mid-print service).
  Serial.println("========================================");
  Serial.printf(" T: %lu ms | State: %s | Phase: %s | Type: %s\n", now, state_name, phase_str, test_name);
  Serial.println("----------------------------------------");
  serviceRTKHotPath();
  Serial.printf(" BARO  Alt: %.2f m (raw: %.2f)  Sink: %+.3f m/s\n", baro_alt, baro_alt_raw, sink_rate_filtered);
  Serial.printf("       Press: %.2f hPa  Temp: %.1f C  QNH: %s\n", pressure_hPa, temp_c, qnh_str);
  Serial.printf(" GNSS  Alt: %.2f m  Spd: %.2f m/s  Trk: %.1f deg  PDOP: %.2f\n", gnss_alt_msl, ground_speed, gnss_track_deg, gnss_pdop_x100 / 100.0f);
  serviceRTKHotPath();
  Serial.printf("       Lat: %.7f  Lon: %.7f\n", lat, lon);
  Serial.printf("       Fix: %s  RTK: %s  Diff:%s  FixOK:%s  Sats: %d  hAcc: %lu mm\n",
                fix_name, carr_name, gnss_diff_soln ? "Y" : "N", gnss_fix_ok ? "Y" : "N", gnss_sats, gnss_hacc_mm);
  Serial.println("       UART1 clean mode: UBX out, RTCM in");
  serviceRTKHotPath();
  Serial.printf(" IMU   Accel: %.3f m/s2  Hdg: %.1f deg (conf: %.2f)  Gyro: %+.4f rad/s\n", accel_mag, heading_rad * 180.0f / PI, heading_conf, gyro_z);
  Serial.printf(" SERVO L: %+.3f (%d us)  R: %+.3f (%d us)\n", servo_cmd_L, servo_us_L, servo_cmd_R, servo_us_R);
  Serial.printf(" SD: %lu lines\n", log_line_count);
  serviceRTKHotPath();
  uint32_t rtk_age = (last_rtk_rx_ms > 0) ? (now - last_rtk_rx_ms) : 999999;
  uint32_t rtk_dt_ms = (prev_rtk_status_ms > 0) ? (now - prev_rtk_status_ms) : 0;
  uint32_t lora_delta = rtk_lora_bytes_in - prev_rtk_lora_bytes_in;
  uint32_t uart_delta = rtk_uart_bytes_out - prev_rtk_uart_bytes_out;
  uint32_t rtk_drop_delta = (rtk_queue_drop_bytes + rtk_uart_drop_bytes) - prev_rtk_drop_bytes;
  uint32_t lora_bps = (rtk_dt_ms > 0) ? (uint32_t)((1000.0f * lora_delta) / rtk_dt_ms) : 0;
  uint32_t uart_bps = (rtk_dt_ms > 0) ? (uint32_t)((1000.0f * uart_delta) / rtk_dt_ms) : 0;
  Serial.printf(" RTK lora_in=%luB uart_out=%luB pkts=%lu age=%lums lora_Bps=%lu uart_Bps=%lu q=%u drop_d=%lu drop_q=%lu drop_uart=%lu short=%lu\n",
                rtk_lora_bytes_in, rtk_uart_bytes_out, rtk_packets_in, rtk_age, lora_bps, uart_bps,
                (unsigned)rtk_q_len, rtk_drop_delta, rtk_queue_drop_bytes, rtk_uart_drop_bytes, rtk_uart_short_writes);
  prev_rtk_status_ms = now;
  prev_rtk_lora_bytes_in = rtk_lora_bytes_in;
  prev_rtk_uart_bytes_out = rtk_uart_bytes_out;
  prev_rtk_drop_bytes = rtk_queue_drop_bytes + rtk_uart_drop_bytes;
  Serial.printf(" LoRa RX(any): %lu pkts, %lu B\n", lora_rx_any_pkts, lora_rx_any_bytes);
  serviceRTKHotPath();
  Serial.printf(" LoRa flags: dio1=%d tx_pending=%d isr=%lu rxErr=%lu crcFail=%lu\n", lora_dio1_flag ? 1 : 0, lora_tx_pending ? 1 : 0, lora_dio1_isr_count, lora_start_rx_err_count, rtk_crc_fail_count);
  Serial.printf(" LoRa RX reads: ok=%lu err=%lu | TX: start=%lu done=%lu timeout=%lu rxRecovery=%lu\n", lora_rx_read_ok, lora_rx_read_err, lora_tx_start_count, lora_tx_done_count, lora_tx_timeout_count, lora_rx_recovery_count);
  Serial.printf(" RTK SEQ: gaps=%lu missed=%lu expected=%u\n", rtk_seq_gaps, rtk_seq_missed_total, rtk_expected_seq);
  Serial.printf(" GS PING: rx=%lu gs_saw_telem=%u\n", gs_ping_rx_count, gs_ping_last_rx_telem);
  Serial.println("========================================");
}

// ===================== SETUP =====================

void setup() {
  serviceWatchdogIfRunning();
  Serial.begin(921600);
  delay(500);
  power_on_ms = millis();

  Serial.println("=============================================");
  Serial.println("  AeroTrackNow Drop Test Firmware v2.6 + RTK v0.6.0 robust");
  Serial.println("  (aligned with flight firmware v5.5)");
  Serial.println("=============================================");

  const char* test_names[] = {"FREE_FLIGHT", "SWEEP_UP", "SWEEP_DOWN", "TURN_RATE", "GLIDE_SLOPE", "TURN_AT_BRAKE", "STEP_RESPONSE", "FLARE_TEST"};
  const char* test_name = (TEST_TYPE >= 0 && TEST_TYPE <= 7) ? test_names[TEST_TYPE] : "UNKNOWN";
  Serial.printf("  Test type: %d (%s)\n", TEST_TYPE, test_name);
  if (TIMER_FALLBACK_MS > 0) {
    Serial.printf("  Timer fallback: %lu s\n", TIMER_FALLBACK_MS / 1000);
  } else {
    Serial.println("  Timer fallback: DISABLED (free-fall only)");
  }
  Serial.printf("  Deploy delay: %lu ms\n", DEPLOY_DELAY_MS);
  Serial.println("---------------------------------------------");

  // I2C — match flight firmware: 400kHz, 200ms timeout
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(200);

  // Servos — direct PWM for MG92B (bypasses Servo library clamping)
  analogWriteFrequency(PIN_SERVO_LEFT, PWM_SERVO_FREQ);
  analogWriteFrequency(PIN_SERVO_RIGHT, PWM_SERVO_FREQ);
  analogWriteResolution(PWM_SERVO_BITS);
  setServos(0.0f, 0.0f);

  // Servo direction self-test — operator must confirm before proceeding
  servoDirectionSelfTest();

  // ✓ FIXED v5.6: Deselect all SPI0 peripherals before init to prevent bus contention.
  // LoRa (CS=10) and SD (CS=23) share SPI0; if either CS is floating LOW during
  // the other's init, SPI transactions corrupt both devices.
  pinMode(PIN_LORA_CS, OUTPUT);
  digitalWrite(PIN_LORA_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  SPI.begin();

  // Sensors
  Serial.println("Initializing sensors...");
  setupBMP280();
  setupBNO085();
  setupGNSS();
  setupLoRa();

  // SD Card
  Serial.println("Initializing SD card...");
  if (SD.begin(PIN_SD_CS)) {
    // Create unique filename with collision avoidance
    char filename[40];
    int file_idx = 0;
    uint32_t ts = millis();  // capture once — don't call millis() inside loop
    do {
      if (file_idx == 0)
        snprintf(filename, sizeof(filename), "drop_T%d_%lu.csv", TEST_TYPE, ts);
      else
        snprintf(filename, sizeof(filename), "drop_T%d_%lu_%d.csv", TEST_TYPE, ts, file_idx);
      file_idx++;
    } while (SD.exists(filename) && file_idx < 100);
    if (file_idx >= 100) {
      Serial.println("  ERROR: 100+ files on SD — clear old files to avoid collision!");
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      logFile.println(
        "time_ms,baro_press_hPa,baro_temp_C,baro_alt_m,"
        "gnss_lat,gnss_lon,gnss_alt_m,gnss_speed_mps,gnss_track_deg,"
        "gnss_fix,gnss_sats,gnss_hacc_mm,gnss_vacc_mm,gnss_vert_vel_mps,"
        "accel_x,accel_y,accel_z,accel_mag,"
        "heading_rad,heading_conf,gyro_z_rads,"
        "servo_cmd_L,servo_cmd_R,servo_us_L,servo_us_R,"
        "test_phase,sink_rate_raw,sink_rate_filtered,"
        "gnss_carr_soln"
      );
      logFile.flush();
      Serial.printf("  SD OK — logging to: %s\n", filename);
    } else {
      Serial.println("  ERROR: Could not create log file!");
    }
  } else {
    Serial.println("  ERROR: SD card not found!");
  }

  Serial.println("=============================================");
  Serial.println("READY — Waiting for drop detection...");
  if (RTK_ACQUIRE_MODE) Serial.println(" RTK acquire mode: 5ms loop, telemetry at 1Hz, aggressive LoRa polling");
  Serial.println("  (Free-fall accel < 3.0 m/s²)");
  Serial.println("=============================================");
  Serial.println("");

  last_flush_ms = millis();
  prev_rtk_status_ms = millis();
  prev_rtk_lora_bytes_in = rtk_lora_bytes_in;
  prev_rtk_uart_bytes_out = rtk_uart_bytes_out;
  prev_rtk_drop_bytes = rtk_queue_drop_bytes + rtk_uart_drop_bytes;

  // Hardware watchdog — matches flight firmware
  setupWatchdog();
}

// ===================== MAIN LOOP =====================

void loop() {
  uint32_t loop_start = millis();
  kickWatchdog();

  // 1. Sensor scheduling
  // In RTK acquire mode, prioritize LoRa/RTK transport over high-rate sensor polling.
  static uint32_t last_slow_sensor_ms = 0;
  uint32_t now_ms = millis();
  bool do_slow_sensors = !RTK_ACQUIRE_MODE || (now_ms - last_slow_sensor_ms >= 200);

  static uint32_t last_gnss_read_ms = 0;
  if (PERF_STAGE_LEVEL >= 2 && (now_ms - last_gnss_read_ms >= GNSS_READ_PERIOD_MS)) {
    serviceRTKHotPath();
    readGNSS();
    serviceRTKHotPath();
    last_gnss_read_ms = now_ms;
  }
  if (PERF_STAGE_LEVEL >= 3 && do_slow_sensors) {
    serviceRTKHotPath();
    readBNO085();
    readBMP280();
    checkI2CRecovery();
    serviceRTKHotPath();
    last_slow_sensor_ms = now_ms;
  }

  // 1b. High-priority RTK transport service
  serviceRTKHotPath();
  serviceRTKHotPath();

  // 2. State-dependent logic
  if (PERF_STAGE_LEVEL >= 3) switch (test_state) {
    case WAITING:
      setServos(0.0f, 0.0f);
      test_phase = -1;
      checkDropDetection();
      break;

    case DESCENDING: {
      uint32_t now = millis();
      // Safety: force landing if descent exceeds MAX_DESCENT_MS
      if (now - drop_detected_ms > MAX_DESCENT_MS) {
        Serial.println(">>> MAX DESCENT TIMEOUT — forcing LANDED state");
        test_state = LANDED;
        setServos(0.0f, 0.0f);
        if (logFile) { logFile.flush(); logFile.close(); }
        break;
      }
      if (now >= sequence_start_ms) {
        // Deployment delay has passed — run test sequence
        uint32_t elapsed = now - sequence_start_ms;
        float cmdL, cmdR;
        getTestCommands(elapsed, cmdL, cmdR);
        setServos(cmdL, cmdR);
      } else {
        // Still in deployment delay — servos neutral
        test_phase = -1;
        setServos(0.0f, 0.0f);
      }
      checkLanding();
      break;
    }

    case LANDED:
      // Do nothing — file already closed
      break;
  }

  // 3. Log to SD at 1 Hz (throttled to avoid stalling the 5ms RTK loop)
  if (PERF_STAGE_LEVEL >= 5 && (now_ms - last_log_ms >= 1000)) {
    last_log_ms = now_ms;
    logToSD();
  }

  // 4. LoRa telemetry at 1 Hz
  if (PERF_STAGE_LEVEL >= 4) sendTelemetry();

  // 5. Serial status at 2 Hz
  printStatus();

  // 5b. Extra high-priority RTK service after telemetry/state work
  serviceRTKHotPath();
  serviceRTKHotPath();

  // 6. Maintain loop cadence without going deaf to LoRa while idling.
  while ((millis() - loop_start) < LOOP_PERIOD_MS) {
    serviceRTKHotPath();
    yield();
  }
}
