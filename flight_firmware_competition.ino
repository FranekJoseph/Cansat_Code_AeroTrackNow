/*
 * AeroTrackNow CanSat — Production Flight Firmware v5.1
 * ======================================================
 * Autonomous parafoil guidance for precision landing.
 *
 * Hardware:
 *   Teensy 4.0, ZED-X20P GNSS (Serial5, 38400, 25Hz), BMP280 (I2C 0x76),
 *   BNO085 (I2C 0x4A), SX1262 LoRa (433MHz), SD card, 2x MG92B servos.
 *
 * Parafoil:
 *   Ram-air, DA2 airfoil, 8 cells, 0.141 m² active area, 0.550 m projected span.
 *   System mass 0.350 kg. Differential D-line brake steering.
 *
 * What this code does:
 *   1. Waits for drop detection (free-fall accelerometer spike or altitude change)
 *   2. Waits for stable parafoil descent
 *   3. Guides toward target using MPC (high alt), heading control (mid alt),
 *      and cross-track EKF controller (low alt)
 *   4. Flares at 3m, goes neutral at 1m, detects landing
 *
 * Before flight:
 *   1. Run 4 calibration drops with test_firmware/drop_test.ino
 *   2. Run: python analysis/process_drops.py <sd_card_path>
 *   3. Copy output values to CALIBRATION SECTION below
 *   4. Set target coordinates in TARGET SECTION below
 *   5. Upload to Teensy
 *   See COMPETITION_GUIDE.md for full instructions.
 */

#include <Arduino.h>
#include <Wire.h>
// Servo library REMOVED — uses direct PWM for full MG92B range (544-2400 us)
// #include <Servo.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO08x.h>
#include <RadioLib.h>

// =====================================================================
//  HARDWARE PINOUT — matches your Teensy 4.0 wiring, DO NOT CHANGE
// =====================================================================
static constexpr int PIN_GNSS_RX       = 21;   // Serial5 RX
static constexpr int PIN_GNSS_TX       = 20;   // Serial5 TX
static constexpr int PIN_I2C_SDA       = 18;
static constexpr int PIN_I2C_SCL       = 19;
static constexpr int PIN_LORA_CS       = 10;
static constexpr int PIN_LORA_DIO1     = 7;
static constexpr int PIN_LORA_RESET    = 8;
static constexpr int PIN_LORA_BUSY     = 9;    // ✓ FIXED: was -1, BUSY pin IS connected
static constexpr int PIN_LORA_RXEN     = 5;    // RF switch RX enable (CORE1262-LF)
static constexpr int PIN_LORA_TXEN     = 6;    // RF switch TX enable (CORE1262-LF)
static constexpr int PIN_SD_CS         = 23;
static constexpr int PIN_SERVO_LEFT    = 3;
static constexpr int PIN_SERVO_RIGHT   = 4;

// =====================================================================
//  TARGET COORDINATES — SET BEFORE EVERY FLIGHT
// =====================================================================
// Enter your landing target GPS coordinates here.
// How to find altitude: Google Earth, or read GNSS MSL at the target spot.
// Set USE_HARDCODED_TARGET = true after entering valid coordinates.
//
static constexpr double HARDCODED_TARGET_LAT = 0.0;    // decimal degrees, e.g. 52.229770
static constexpr double HARDCODED_TARGET_LON = 0.0;    // decimal degrees, e.g. 21.011780
static constexpr double HARDCODED_TARGET_ALT = 0.0;    // meters MSL at the landing spot
static constexpr bool   USE_HARDCODED_TARGET = false;   // !! SET true WHEN COORDS ARE ENTERED !!

// =====================================================================
//  CALIBRATION VALUES — from process_drops.py output
// =====================================================================
// After 4 calibration drops, run:
//   python analysis/process_drops.py /path/to/sd_card/
// Copy the printed values here, replacing the defaults below.
//
// BRAKE_TABLE is fixed (the brake input levels tested):
static constexpr float BRAKE_TABLE[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
static constexpr int   BRAKE_TABLE_SIZE = sizeof(BRAKE_TABLE) / sizeof(BRAKE_TABLE[0]);

// These are the values to REPLACE with process_drops.py output:
static constexpr float SINK_RATE_TABLE[]       = {5.5f, 5.9f, 6.4f, 6.9f, 7.2f};     // REPLACE
static constexpr float GLIDE_RATIO_TABLE[]     = {2.0f, 1.9f, 1.7f, 1.55f, 1.4f};     // REPLACE
static constexpr float TURN_RATE_SCALE_TABLE[] = {1.0f, 1.05f, 1.1f, 1.15f, 1.2f};    // REPLACE
// ✓ FIX R7: Compile-time safety — catch table size mismatches after calibration edits
static_assert(sizeof(SINK_RATE_TABLE) / sizeof(float) == BRAKE_TABLE_SIZE,
              "SINK_RATE_TABLE size must match BRAKE_TABLE_SIZE");
static_assert(sizeof(GLIDE_RATIO_TABLE) / sizeof(float) == BRAKE_TABLE_SIZE,
              "GLIDE_RATIO_TABLE size must match BRAKE_TABLE_SIZE");
static_assert(sizeof(TURN_RATE_SCALE_TABLE) / sizeof(float) == BRAKE_TABLE_SIZE,
              "TURN_RATE_SCALE_TABLE size must match BRAKE_TABLE_SIZE");
static constexpr float NOMINAL_SINK_RATE       = 5.5f;     // REPLACE (sink at 0% brake)
static constexpr float PARAFOIL_GLIDE_RATIO    = 2.0f;     // REPLACE (glide at 0% brake)
static constexpr float HEADING_OFFSET_RAD      = 0.0f;     // REPLACE (from heading calibration)

// Magnetic declination — converts BNO085 magnetic heading to true heading.
// ⚠️ CALIBRATE THIS! Current value (6.88°) is for Warsaw, Poland — MUST be
// recalculated for the competition location before drop tests.
// Use the NOAA calculator: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
// This MUST be set for the competition site BEFORE running drop tests, because
// HEADING_OFFSET_RAD calibrated from drops will be site-specific and won't
// transfer correctly if declination is wrong.
static constexpr float MAGNETIC_DECLINATION_RAD = 0.1201f;  // ⚠️ CALIBRATE THIS! (+6.88° for Warsaw)

// Stability detection thresholds (also from process_drops.py):
static constexpr float STABLE_SINK_MIN = 4.5f;   // REPLACE (nominal - 1.0)
static constexpr float STABLE_SINK_MAX = 6.5f;   // REPLACE (nominal + 1.0)

// =====================================================================
//  PHYSICAL PARAMETERS — from CDR and Surfplan data
// =====================================================================
static constexpr float PARAFOIL_AREA_M2       = 0.141f;  // active lift area
static constexpr float SYSTEM_MASS_KG         = 0.350f;
static constexpr float NOMINAL_AIRSPEED        = NOMINAL_SINK_RATE * PARAFOIL_GLIDE_RATIO;
static constexpr float SERVO_TIME_CONSTANT_S   = 0.60f;  // ✓ v5.3: was 0.10 — includes canopy aerodynamic response (0.3-1.2s), not just servo (0.10s)

// Wind profile (disabled by default; real-time estimation is preferred)
static constexpr bool  USE_SEEDED_WIND_PROFILE = false;
static constexpr float WARSAW_PROFILE_MAX_ALT_M = 2000.0f;
static constexpr float WARSAW_WIND_DIR_DEG[]   = {0.0f};
static constexpr float WARSAW_WIND_SPD_MPS[]   = {0.0f};
static constexpr int   WARSAW_WIND_BINS        = sizeof(WARSAW_WIND_SPD_MPS) / sizeof(WARSAW_WIND_SPD_MPS[0]);

// =====================================================================
//  CONTROL PARAMETERS
// =====================================================================
// Heading PID
static constexpr float KP_HEADING = 0.9f;
static constexpr float KI_HEADING = 0.05f;
static constexpr float KD_HEADING = 0.12f;
static constexpr float HEADING_D_FILTER_ALPHA  = 0.3f;    // low-pass on D-term (~0.15s tau at 20 Hz)
static constexpr float MAX_BANK_ANGLE_RAD      = 0.48f;   // ~27.5 degrees
static constexpr float MAX_TURN_RATE_RAD_S     = 0.60f;  // physics: g×tan(27.5°)/11.0 = 0.46 rad/s × 1.2 table max = 0.558, +7% margin

// Heading quality
static constexpr float HEADING_JUMP_MAX_RAD    = 1.2f;
static constexpr uint32_t HEADING_JUMP_MAX_DT_MS = 150;
static constexpr float MIN_GROUND_SPEED_FOR_TRACK = 2.0f;  // ✓ v5.4: raised from 0.5 — headMot accuracy ±30-60° below 2 m/s
static constexpr float HEADING_CONFIDENCE_MAX_DELTA_RAD = 0.35f;
static constexpr float HEADING_CONFIDENCE_MIN_FOR_WIND  = 0.4f;
static constexpr uint32_t HEADING_CONFIDENCE_MAX_DT_MS  = 300;

// Heading calibration mode (set true, upload, point north, read serial, compute offset)
static constexpr bool  HEADING_CALIBRATION_MODE = false;

// Servo
static constexpr float SERVO_TRIM       = 0.0f;
static constexpr int   SERVO_NEUTRAL_US = 1500;
static constexpr int   SERVO_RANGE_US   = 300;   // ±300 µs from neutral
static constexpr float SERVO_MAX_RATE    = 2.0f;  // max change per second
static constexpr float SERVO_DEADBAND    = 0.02f;
static constexpr float SERVO_FILTER_ALPHA = 0.4f;  // ✓ v5.1: was 0.3, reduced tau ~75ms for lower pipeline latency

// ✓ v5.6: Direct PWM for full MG92B range (replaces Servo library)
static constexpr int    PWM_SERVO_FREQ       = 50;        // Hz — standard servo
static constexpr int    PWM_SERVO_BITS       = 16;        // 16-bit resolution
static constexpr float  PWM_SERVO_PERIOD_US  = 1000000.0f / PWM_SERVO_FREQ;  // 20000 us
static constexpr float  PWM_SERVO_MAX_VAL    = (float)(1 << PWM_SERVO_BITS);  // 65536 — 0.305 us resolution

// State machine altitudes (descending order)
static constexpr float FINAL_APPROACH_HEIGHT_M   = 50.0f;  // GUIDED_DESCENT → FINAL_APPROACH
static constexpr float TERMINAL_HOMING_HEIGHT_M  = 30.0f;  // FINAL_APPROACH → TERMINAL_HOMING
static constexpr float FLARE_HEIGHT_M            = 4.0f;   // → FLARE (safety backup only — TTI is primary trigger)
static constexpr float TERMINAL_HEIGHT_M         = 1.0f;   // → TERMINAL (neutral)
static constexpr float CAPTURE_RADIUS_M          = 1.0f;   // RTK precision
static constexpr float CAPTURE_RADIUS_GNSS_M     = 5.0f;   // standard GNSS
static constexpr float CAPTURE_RADIUS_POOR_M     = 10.0f;  // poor GNSS

// Stable descent detection
static constexpr uint32_t STABLE_HOLD_MS             = 1500;
static constexpr uint32_t STABLE_DESCENT_TIMEOUT_MS  = 15000;  // 15 s relaxed
static constexpr uint32_t STABLE_DESCENT_HARD_MS     = 20000;  // 20 s forced
static constexpr float    STABLE_SINK_MIN_RELAXED    = 3.0f;
static constexpr float    STABLE_SINK_MAX_RELAXED    = 8.0f;

// Drop detection
static constexpr float    DROP_ACCEL_SPIKE     = 8.0f;   // m/s² deviation from 1g
static constexpr int      DROP_DEBOUNCE_COUNT  = 3;
static constexpr double   DROP_ALT_CHANGE_M    = 50.0;   // altitude backup threshold

// Sink rate filtering
static constexpr uint32_t BARO_UPDATE_INTERVAL_MS = 100;  // ✓ FIXED: 10Hz for lower lag
static constexpr float    SINK_FILTER_ALPHA = 0.35f;      // ✓ FIXED: faster response (~0.3s tau)
static constexpr float    SINK_RATE_MAX     = 9.0f;

// Wind estimation
static constexpr float    WIND_FILTER_ALPHA       = 0.03f;   // global wind EMA (slow, for gust baseline)
static constexpr float    WIND_LAYER_FILTER_ALPHA = 0.20f;   // per-layer EMA (fast, single-pass convergence)
static constexpr float    MAX_WIND_ESTIMATE       = 18.0f;
static constexpr float    MAX_WIND_UPDATE_DELTA   = 8.0f;
static constexpr float    MAX_WIND_LAYER_DELTA    = 10.0f;
static constexpr float    MIN_WIND_UPDATE_ALT_M   = 10.0f;
// Non-uniform wind layers: 5m steps below 100m (20 layers), 25m above (76 layers)
static constexpr float    WIND_LAYER_FINE_STEP_M  = 5.0f;    // step below 100m
static constexpr float    WIND_LAYER_COARSE_STEP_M= 25.0f;   // step above 100m
static constexpr float    WIND_LAYER_FINE_CEIL_M  = 100.0f;  // transition altitude
static constexpr int      WIND_LAYER_FINE_COUNT   = 20;      // 100m / 5m
static constexpr int      WIND_LAYER_COARSE_COUNT = 76;      // (2000m - 100m) / 25m
static constexpr int      WIND_LAYER_COUNT        = 96;      // 20 + 76
static constexpr float    WIND_LAYER_DECAY_PER_S  = 0.0002f; // ~5% loss over 6-min descent; each layer visited once
static constexpr float    MAX_WIND_FOR_GUIDANCE   = 6.0f;   // ✓ v5.2: was 8.0 — 55% of airspeed, literature shows severe degradation >70%
static constexpr float    WIND_EMERGENCY_MPS      = 12.0f;  // above this: neutral servos
static constexpr float    WIND_DAMPED_START_MPS   = 10.0f;  // above this: heavy damping
static constexpr float    WIND_GAIN_REDUCTION_START = 3.0f;  // ✓ v5.2: was 4.0 — earlier gain tapering for better marginal-wind accuracy
static constexpr float    WIND_GAIN_MIN_FACTOR    = 0.6f;
static constexpr float    WIND_LAYER_SMOOTH_ALPHA = 0.12f;
static constexpr int      WIND_LAYER_BLEND_RADIUS = 2;

// Gust detection
static constexpr float    GUST_SPEED_SPIKE_MPS = 3.0f;
static constexpr uint32_t GUST_HOLD_MS         = 1500;
static constexpr float    GUST_GAIN_FACTOR     = 0.7f;

// Glide-slope management
static constexpr float GLIDESLOPE_TARGET_SINK_HIGH  = 5.6f;
// ✓ v5.4: 0.3 m/s margin above minimum achievable sink (SINK_RATE_TABLE[0]).
// Gives controller room for downward correction in updrafts/ground effect.
static constexpr float GLIDESLOPE_TARGET_SINK_LOW   = SINK_RATE_TABLE[0] + 0.3f;  // margin above min sink for control authority
static constexpr float GLIDESLOPE_TARGET_HIGH_ALT_M = 200.0f;
static constexpr float GLIDESLOPE_TARGET_LOW_ALT_M  = 20.0f;
static constexpr float GLIDESLOPE_MAX_BRAKE         = 0.35f;
static constexpr float GLIDESLOPE_KP                = 0.12f;
static constexpr float GLIDESLOPE_KI                = 0.02f;

// Lateral damping
static constexpr float LATERAL_DAMPING_GAIN = 0.15f;

// Safety
static constexpr float    MAX_EKF_SPEED_MPS          = 25.0f;
static constexpr int      MIN_GNSS_SATS_FOR_GUIDANCE = 6;
static constexpr uint32_t MAX_GNSS_AGE_MS            = 1500;

// I2C bus recovery
static constexpr int      I2C_FAIL_THRESHOLD         = 3;     // consecutive failures before recovery (must trigger before watchdog)
static constexpr int      I2C_SCL_TOGGLE_COUNT       = 16;    // clock toggles to free stuck SDA
static constexpr uint32_t I2C_RECOVERY_COOLDOWN_MS   = 3000;  // min time between recovery attempts
static constexpr uint32_t I2C_SENSOR_TIMEOUT_MS      = 500;   // sensor data age before counting failure

// Hardware watchdog (RTWDOG on IMXRT1062, 32kHz LPO clock)
static constexpr uint32_t WATCHDOG_TIMEOUT_MS        = 5000;

// MPC — two-phase command sequences (captures turn-then-straighten maneuvers)
static constexpr int   MPC_HORIZON         = 12;
static constexpr int   MPC_PHASE1_STEPS    = 4;    // first phase: variable command
static constexpr int   MPC_PHASE2_STEPS    = 8;    // second phase: different command (hold)
static constexpr int   MPC_CMD_LEVELS      = 15;   // command levels per phase (15^2=225 evals)
static constexpr float MPC_CONTROL_WEIGHT  = 0.05f;  // ✓ v5.0: reduced from 0.15 — allow aggressive commands for precision
static constexpr float MPC_SMOOTH_WEIGHT   = 0.05f; // penalize command change between phases
static constexpr float APPROACH_GAIN      = 1.8f;

// Final Approach Line (FAL) trajectory planning
static constexpr float FAL_MIN_WIND_MPS          = 1.0f;   // below this, no FAL (fly direct)
static constexpr float FAL_FREEZE_HEIGHT_M       = 20.0f;  // ✓ v5.1: was 40m, allows wind-driven FAL updates through FINAL_APPROACH
static constexpr uint32_t FAL_UPDATE_INTERVAL_MS  = 10000;  // ✓ FIXED: uint32_t for ms count

// Turn rate feedback (inner rate loop)
static constexpr float TURN_RATE_FB_GAIN         = 0.7f;   // ✓ v5.3: was 0.4 — increased to maintain damping ratio with reduced MAX_TURN_RATE_RAD_S
static constexpr float TURN_RATE_FILTER_ALPHA    = 0.25f;  // low-pass on measured yaw rate

// Feed-forward wind compensation
// ✓ v5.1: FF gain is now adaptive (see getWindFFGain()) — these are the bounds
static constexpr float WIND_FF_GAIN_MIN          = 0.65f;  // low confidence: conservative
static constexpr float WIND_FF_GAIN_MAX          = 0.95f;  // high confidence: aggressive

// Dynamic flare
static constexpr float FLARE_TRIGGER_TIME_S      = 0.9f;   // ✓ v5.2: was 1.2 — tighter for 350g system where ground effect is minimal
// ⚠️ CALIBRATE: Brake mechanical delay is ~0.3-0.5s (line travel + ram-air compression).
// With 0.9s TTI, only ~0.4-0.6s of actual deceleration occurs before impact.
// If drop tests show hard landings, increase to 1.1-1.2s. If flaring too high, decrease.
// Adaptive flare brake level: scales with ground speed / wind
static constexpr float FLARE_BRAKE_MIN           = 0.6f;   // minimum brake in low wind
static constexpr float FLARE_BRAKE_MAX           = 1.0f;   // full brake in high wind
static constexpr float FLARE_WIND_LOW_MPS        = 2.0f;   // below this: use min brake
static constexpr float FLARE_WIND_HIGH_MPS       = 6.0f;   // above this: use max brake
// Asymmetric flare: crosswind correction during flare
static constexpr float FLARE_CROSSWIND_GAIN      = 0.08f;  // differential per m/s crosswind
static constexpr float FLARE_MAX_ASYMMETRY       = 0.25f;  // ✓ v5.1: increased from 0.20 — stall margin >40% at 0.875 avg brake

// Flare wind compensation: pre-offset to compensate drift during flare
static constexpr float FLARE_COMP_HEIGHT_M       = 15.0f;  // start compensating below this
static constexpr float FLARE_COMP_TIME_S          = 1.4f;   // ✓ v5.1: accounts for full flare duration incl. ground effect (~1.5s)
static constexpr float FLARE_COMP_ALONG_GAIN      = 0.85f;  // ✓ v5.1: 15% margin for ground effect + estimate error (was 0.8)

// Terminal cross-track controller
static constexpr float KP_CROSSTRACK              = 0.09f;  // ✓ v5.1: was 0.07 — 23% faster convergence, safe with turn rate feedback
static constexpr float KD_CROSSTRACK              = 0.22f;  // ✓ v5.1: was 0.18 — matched damping ratio for new KP
static constexpr float MAX_CROSSTRACK_CMD          = 1.0f;
static constexpr float HEADING_GAIN_HIGH_ALT       = 0.7f;
static constexpr float HEADING_GAIN_LOW_ALT        = 1.25f;
static constexpr float HEADING_GAIN_HIGH_ALT_M     = 150.0f;
static constexpr float HEADING_GAIN_LOW_ALT_M      = 20.0f;
static constexpr float CROSSTRACK_GAIN_HIGH_ALT    = 0.8f;
static constexpr float CROSSTRACK_GAIN_LOW_ALT     = 1.5f;
static constexpr float CROSSTRACK_GAIN_HIGH_ALT_M  = 40.0f;
static constexpr float CROSSTRACK_GAIN_LOW_ALT_M   = 8.0f;

// ---- 9-state EKF tuning ----
// State: [N, E, D, vN, vE, vD, baro_bias, wN, wE]
// Process noise (per second, applied as Q*dt each prediction step)
static constexpr float EKF9_Q_POS       = 0.01f;   // position (m²/s) — small, driven by velocity
static constexpr float EKF9_Q_VEL_H     = 2.0f;    // horizontal velocity (m²/s³) — gusts/model error
static constexpr float EKF9_Q_VEL_V     = 0.5f;    // vertical velocity (m²/s³) — sink rate variation
static constexpr float EKF9_Q_BARO_BIAS = 0.005f;   // baro bias drift (m²/s) — thermal during descent
static constexpr float EKF9_Q_WIND      = 0.25f;    // wind turbulence (m²/s³)
// Measurement noise (R matrices)
static constexpr float EKF9_R_POS_H_FLOOR  = 0.003f;  // ✓ v5.1: was 0.01 — faster transient recovery, chi² gate protects outliers
static constexpr float EKF9_R_POS_V_FLOOR  = 0.02f;   // ✓ v5.1: was 0.1 — trusts RTK vertical (vacc ~30mm) more closely
static constexpr float EKF9_R_VEL_H        = 0.0625f;  // GNSS horizontal velocity (m²/s²) — 0.25 m/s
static constexpr float EKF9_R_VEL_V        = 0.25f;    // GNSS vertical velocity (m²/s²) — 0.5 m/s
// FIX M1: R_BARO reduced from 1.0 to 0.1 m² — BMP280 with IIR x16 has ~0.04 m² noise.
// 0.1 m² adds margin for dynamic pressure transients while properly weighting baro data.
// r_baro is further inflated dynamically by brake_diff*3.0 during maneuvering (line 1774).
// Found by sensor-auditor, verified by ekf-auditor.
static constexpr float EKF9_R_BARO         = 0.1f;     // barometric altitude (m²) — BMP280 IIR x16
static constexpr float EKF9_R_MODEL        = 4.0f;     // parafoil model pseudo-measurement (m²/s²)
static constexpr float EKF9_R_MODEL_UNCAL  = 16.0f;    // uncalibrated model (m²/s²)
// Innovation gating thresholds (chi-squared, 1 DOF)
static constexpr float EKF9_NIS_POS   = 9.21f;   // GNSS position — 99% confidence
static constexpr float EKF9_NIS_VEL   = 6.63f;   // GNSS velocity — 99%
static constexpr float EKF9_NIS_BARO  = 12.0f;   // barometric — lenient (drift is gradual)
static constexpr float EKF9_NIS_MODEL = 15.0f;   // model — very lenient (approximate)

// Auto heading calibration (runtime backup for HEADING_OFFSET_RAD)
static constexpr float    AUTO_HEADING_MIN_SPEED      = 2.0f;
static constexpr float    AUTO_HEADING_MAX_BRAKE_DIFF = 0.05f;
static constexpr int      AUTO_HEADING_MIN_SAMPLES    = 50;   // 2s at 25Hz — robust against turn contamination
static constexpr float    AUTO_HEADING_MAX_CORRECTION  = 0.8f;   // ✓ FIXED: max ~46 deg (covers most BNO085 offsets)

// =====================================================================
//  RADIO SETTINGS
// =====================================================================
static constexpr float    LORA_FREQ_MHZ       = 433.0f;
static constexpr int      LORA_TX_DBM         = 20;  // ✓ v5.6: matched to ground station (was 14)
static constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;
static constexpr uint32_t LORA_TX_TIMEOUT_MS  = 350;   // ✓ v5.9: safety timeout for TX completion (was hardcoded 200ms)

// =====================================================================
//  PACKET STRUCTURES
// =====================================================================
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t  msgType;                // 0x03
  uint32_t time_ms;
  int32_t  lat_e7;
  int32_t  lon_e7;
  int16_t  height_agl_dm;
  uint16_t pressure_hPa_x10;
  int16_t  temp_c_x10;
  uint16_t ground_speed_cms;
  int16_t  servo_left_x1000;
  int16_t  servo_right_x1000;
  int32_t  pred_lat_e7;
  int32_t  pred_lon_e7;
  int16_t  wind_north_cms;
  int16_t  wind_east_cms;
  int16_t  heading_deg_x10;
  uint16_t heading_confidence_x1000;
  uint16_t wind_rejects;
  uint16_t wind_layer_rejects;
  uint8_t  mission_state;
  uint8_t  gnss_carr_soln;    // ✓ v5.0: 0=none, 1=RTK float, 2=RTK fixed
  uint8_t  gnss_num_sats;     // ✓ v5.0: number of satellites
  uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(TelemetryPacket) == 50, "TelemetryPacket size mismatch — ground station expects 50 bytes");

#pragma pack(push, 1)
struct TargetPacket {
  uint8_t  msgType;       // 0x01
  int32_t  tgt_lat_e7;
  int32_t  tgt_lon_e7;
  int32_t  tgt_alt_cm;    // meters MSL * 100
  uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(TargetPacket) == 15, "TargetPacket size mismatch — ground station expects 15 bytes");

// ✓ v5.3: Added MSG_GROUND_WIND for ground-level wind uplink
#pragma pack(push, 1)
struct GroundWindPacket {
  uint8_t  msgType;        // 0x04
  int16_t  wind_n_cms;     // ground wind north component, cm/s
  int16_t  wind_e_cms;     // ground wind east component, cm/s
  uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(GroundWindPacket) == 7, "GroundWindPacket size mismatch — ground station expects 7 bytes");

static constexpr uint8_t MSG_TARGET      = 0x01;
static constexpr uint8_t MSG_RTK         = 0x02;
static constexpr uint8_t MSG_TELEMETRY   = 0x03;
// NOTE: 0x04 is dual-purpose and dispatched by packet length in pollLoRa():
//   - 7 bytes (sizeof GroundWindPacket) → ground wind uplink (legacy GS)
//   - 4 bytes                           → heartbeat ping from Cansat_tests
//                                         GroundStation_transport_seqfix
// Do NOT add a second constant for the ping; length disambiguation keeps
// both GS variants compatible with a single flight firmware build.
static constexpr uint8_t MSG_GROUND_WIND = 0x04;  // ✓ v5.3: ground wind uplink / GS ping

// Forward declaration so RTK hot-path service can be called from sensor and
// MPC loops that are defined earlier in the file than the LoRa section.
// Body lives after checkLoRaTxDone() (see LoRa section below).
static void serviceRTKHotPath();

// =====================================================================
//  STATE MACHINE
// =====================================================================
// Flight sequence:
//   BOOT → WAIT_FOR_DROP → WAIT_FOR_STABLE_DESCENT → GUIDED_DESCENT (>50m)
//   → FINAL_APPROACH (50m) → TERMINAL_HOMING (30m) → FLARE (3m)
//   → TERMINAL (1m) → LANDED
enum MissionState : uint8_t {
  BOOT = 0,
  WAIT_FOR_DROP,
  WAIT_FOR_STABLE_DESCENT,
  GUIDED_DESCENT,
  FINAL_APPROACH,
  TERMINAL_HOMING,
  FLARE,
  TERMINAL,
  LANDED
};
MissionState state = BOOT;
MissionState last_state = BOOT;

// =====================================================================
//  GLOBAL OBJECTS
// =====================================================================
SFE_UBLOX_GNSS gnss;
Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t bnoValue;
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);
// Servo servoL, servoR;  // REMOVED — using direct PWM via servoWriteUs()
File logFile;

// =====================================================================
//  STATE VARIABLES
// =====================================================================
// Target
bool   targetReceived = false;
double target_lat     = 0.0;
double target_lon     = 0.0;
double target_alt_msl = 0.0;

// GNSS
double   lat = 0.0, lon = 0.0;
double   gnss_alt_msl        = 0.0;
double   ground_speed        = 0.0;
double   ground_track_rad    = 0.0;
bool     ground_track_valid  = false;
double   gnss_vertical_velocity = 0.0;
uint8_t  gnss_fix_type       = 0;
uint8_t  gnss_carrier_solution = 0;  // ✓ FIXED: 0=none, 1=RTK float, 2=RTK fixed (from carrSoln)
uint8_t  gnss_satellites     = 0;
uint32_t gnss_h_accuracy_mm  = 9999;
uint32_t gnss_v_accuracy_mm  = 9999;
uint32_t last_gnss_update    = 0;
bool gnss_stream_fallback = false; // true when UBX begin handshake fails but stream is alive
// NMEA fallback parser state
static char nmea_line[128];
static uint8_t nmea_len = 0;
static uint32_t nmea_last_update_ms = 0;
static bool nmea_seen = false;
// ✓ FIXED v5.0: Moved declarations before first use in updateGNSS()
static uint8_t  rtk_best_fix_type     = 0;    // highest carrier solution achieved
static uint32_t rtk_best_hacc_mm      = 9999; // lowest hAcc achieved

// Heading (BNO085)
double   heading_rad       = 0.0;
bool     heading_valid     = false;
uint32_t last_heading_ms   = 0;
double   last_heading_rad  = 0.0;
float    heading_confidence = 0.0f;

// IMU quaternion (BNO085 rotation vector — body to ENU frame)
float    imu_qw = 1.0f, imu_qx = 0.0f, imu_qy = 0.0f, imu_qz = 0.0f;
bool     imu_quat_valid = false;
uint32_t last_quat_ms   = 0;

// IMU accel
float    imu_ax = 0.0f, imu_ay = 0.0f, imu_az = 0.0f;
bool     imu_accel_valid = false;
uint32_t last_imu_ms     = 0;

// Barometric
double   pressure_hPa     = 0.0;
double   temp_c           = 0.0;
double   baro_alt_msl     = 0.0;
double   last_baro_alt    = 0.0;
uint32_t last_baro_ms     = 0;

// Fused altitude
double fused_alt_msl      = 0.0;
double fused_alt_velocity = 0.0;

// Sink rate
double sink_rate_raw      = 0.0;
double sink_rate_filtered = 0.0;  // ✓ v5.1: was NOMINAL — 0.0 prevents false "stable descent" before first baro reading
double sink_rate_gnss     = 0.0;

// Wind estimation
double   wind_north = 0.0, wind_east = 0.0;
double   wind_speed = 0.0;
float    wind_layer_north[WIND_LAYER_COUNT]{};  // ✓ FIXED: float (was double — no precision benefit, saves 768B RAM)
float    wind_layer_east[WIND_LAYER_COUNT]{};
uint32_t wind_layer_ms[WIND_LAYER_COUNT]{};
bool     gust_active  = false;
uint32_t gust_until_ms = 0;
double   last_wind_speed = 0.0;
uint16_t wind_update_rejects = 0;
uint16_t wind_layer_rejects  = 0;

// Glide-slope
float glide_integral = 0.0f;

// Prediction
double pred_lat = 0.0, pred_lon = 0.0;
double predicted_error_m = 999.0;

// ✓ v5.1: Unreachable target detection
static double prev_predicted_error_m = 999.0;
static int    error_growing_count    = 0;
static bool   target_unreachable     = false;

// Servo state
float    cmdL_current = 0.0f, cmdR_current = 0.0f;
float    cmdL_target  = 0.0f, cmdR_target  = 0.0f;
float    cmdL_filtered = 0.0f, cmdR_filtered = 0.0f;
uint32_t last_servo_update_ms = 0;

// IMU-derived turn rate (for inner rate loop)
float    measured_turn_rate     = 0.0f;
double   prev_heading_for_rate  = 0.0;
uint32_t prev_heading_rate_ms   = 0;

// Final Approach Line (trajectory planning)
struct ApproachPlan {
  double fal_bearing_rad;      // bearing from upwind toward target (direction to fly)
  double fal_dn, fal_de;       // unit vector along FAL (toward target)
  double upwind_n, upwind_e;   // unit vector pointing upwind
  bool valid;
  uint32_t last_update_ms;
};
static ApproachPlan approach_plan{};

// Servo failure detection
static constexpr float    SERVO_FAIL_RATE_THRESHOLD  = 0.3f;  // rad/s discrepancy threshold
static constexpr int      SERVO_FAIL_WINDOW          = 25;    // consecutive samples (~1.0s at 25Hz)
static constexpr float    SERVO_FAIL_MIN_CMD         = 0.15f; // minimum command to test against
static int    servo_fail_count    = 0;
static bool   servo_failure_flag  = false;
// ✓ v5.1: Proportional servo gain estimate — tracks ratio of actual/expected turn rate
// Compensates for partial servo failures (e.g., backlash, stall, reduced travel)
static float  servo_gain_estimate = 1.0f;

// ✓ v5.5 AUDIT FIX: Anti-spiral protection
static uint32_t spiral_detect_start_ms   = 0;
static bool     spiral_recovery_active   = false;
static uint32_t spiral_recovery_start_ms = 0;
static constexpr float    SPIRAL_RATE_THRESHOLD = 0.3f;    // rad/s (~17 deg/s sustained)
static constexpr uint32_t SPIRAL_DETECT_MS      = 3000;    // 3s sustained uncommanded turn to trigger
static constexpr uint32_t SPIRAL_RECOVERY_MS    = 2000;    // 2s neutral servos to recover

// ✓ v5.3: Online canopy time constant identification
static float  canopy_tau_estimate    = SERVO_TIME_CONSTANT_S;  // initialized from config
static float  canopy_tau_cmd_start   = 0.0f;    // command delta that triggered measurement
static float  canopy_tau_rate_start  = 0.0f;    // turn rate at command change
static float  canopy_tau_expected    = 0.0f;    // expected final turn rate
static uint32_t canopy_tau_start_ms  = 0;       // when command change was detected
static bool   canopy_tau_measuring   = false;    // currently measuring response

// Flare state
bool     flare_active   = false;

// Drop detection
int    drop_spike_count   = 0;
double drop_check_alt     = 0.0;
bool   drop_check_alt_set = false;

// Stability detection
uint32_t stableStartMs        = 0;
uint32_t stable_wait_start_ms = 0;

// Landing detection (global, not static, so we can reset on state change)
uint32_t landing_still_start = 0;

// Timers
uint32_t lastTelemetryMs = 0;
uint32_t lora_tx_start_ms = 0;
uint32_t loop_start_ms   = 0;
// ✓ FIX: Must be volatile — read in DIO1 ISR (loraTxDoneISR) and set in main loop
volatile bool lora_tx_pending  = false;
static volatile bool lora_tx_done_flag = false;
bool     lora_ok          = false;  // set true only if setupLoRa() fully succeeds
bool     lora_link_weak   = false;  // ✓ v5.4: set by pollLoRa() when RSSI/SNR indicates imminent RTK dropout
// ✓ v5.9: RX recovery watchdog — tracks last LoRa activity to detect stuck radio
static uint32_t last_lora_activity_ms  = 0;
static uint32_t lora_rx_recovery_count = 0;
static uint32_t lora_tx_timeout_count  = 0;
bool     sd_ok            = false;  // M12 FIX: tracks SD card health for flight logic

// Sea level pressure
double sea_level_pressure_hpa = 1013.25;
bool   sea_level_calibrated   = false;
int    qnh_sample_count       = 0;
double qnh_accum              = 0.0;

// Heading controller state
double   heading_error_integral   = 0.0;
double   last_heading_error       = 0.0;
float    last_heading_measurement = 0.0f;  // H3 FIX: for D-on-measurement
float    heading_d_filtered       = 0.0f;  // low-pass filtered D-term
uint32_t last_heading_update_ms   = 0;

// Navigation frame
double origin_lat = 0.0, origin_lon = 0.0;
double origin_alt_msl = 0.0;
bool   origin_set = false;

// 9-state EKF
static constexpr int EKF9_N = 9;
// State indices
static constexpr int EKF9_N_IDX  = 0, EKF9_E_IDX  = 1, EKF9_D_IDX  = 2;
static constexpr int EKF9_VN_IDX = 3, EKF9_VE_IDX = 4, EKF9_VD_IDX = 5;
static constexpr int EKF9_BIAS_IDX = 6;
static constexpr int EKF9_WN_IDX = 7, EKF9_WE_IDX = 8;

struct EKF9State {
  double   x[EKF9_N];                // state vector
  float    P[EKF9_N][EKF9_N];        // covariance matrix
  bool     initialized;
  uint32_t last_predict_ms;
  uint32_t last_gnss_ms;
  uint32_t gnss_dropout_duration_ms;  // saved for R inflation
  uint8_t  gnss_reacq_countdown;      // updates remaining with inflated R
  float    last_nis[EKF9_N];          // last NIS per channel (diagnostic)
  float    nis_ema[EKF9_N];           // exponential moving average of NIS (α=0.05)
  uint32_t model_mismatch_since_ms;   // millis() when model NIS EMA first exceeded 3.0
  bool     model_mismatch;            // true if model NIS high for >5s
  uint16_t gnss_reject_count;
  uint16_t baro_reject_count;
  bool     healthy;
};
static EKF9State ekf9{};

// I2C bus recovery state
static int      bmp_fail_count       = 0;
static int      bno_fail_count       = 0;
static uint32_t last_i2c_recovery_ms = 0;
static uint32_t last_bmp_success_ms  = 0;
static uint32_t last_bno_success_ms  = 0;
static bool     bmp_operational      = true;
static bool     bno_operational      = true;
static int      bno_reinit_failures  = 0;

// Auto heading calibration (runtime)
int    auto_heading_count   = 0;
double auto_heading_accum   = 0.0;
bool   auto_heading_done    = false;
float  heading_offset_runtime = 0.0f;

// =====================================================================
//  UTILITY FUNCTIONS
// =====================================================================
// ✓ v5.6: Direct PWM servo control (replaces Servo library writeMicroseconds)
static void servoWriteUs(int pin, int us) {
  float duty = (float)us / PWM_SERVO_PERIOD_US * PWM_SERVO_MAX_VAL;
  analogWrite(pin, (int)(duty + 0.5f));
}

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

// ✓ FIXED: WGS84-accurate meridional arc length (was flat 111111.0, ~2m error over 2km at 52°N)
static inline double metersPerDegLat(double lat_deg) {
  double lat_rad = lat_deg * DEG_TO_RAD;
  return 111132.92 - 559.82 * cos(2.0 * lat_rad) + 1.175 * cos(4.0 * lat_rad);
}
static inline double metersPerDegLon(double lat_deg) {
  double lat_rad = lat_deg * DEG_TO_RAD;
  // WGS84 transverse radius of curvature * cos(lat) * deg_to_rad
  // ✓ v5.6: Guard against near-zero at extreme latitudes (>~89°)
  double m = 111412.84 * cos(lat_rad) - 93.5 * cos(3.0 * lat_rad);
  if (fabs(m) < 1.0) return 1.0;  // safe fallback — prevents division by zero in NE conversion
  return m;
}

static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
             sin(dLon / 2) * sin(dLon / 2);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

// Bearing convention: atan2(dE, dN) = CW from north (0=N, π/2=E, ±π=S, -π/2=W).
// This matches aviation/navigation convention and is used consistently throughout:
// heading control, wind estimation, MPC prediction, FAL cross-track, and auto-heading.
static double bearingRad(double fromLat, double fromLon, double toLat, double toLon) {
  double dLat = toLat - fromLat;
  double dLon = (toLon - fromLon) * cos(fromLat * DEG_TO_RAD);
  return atan2(dLon, dLat);
}

// C1 FIX: NaN returns midpoint (safe neutral) instead of silently hitting hi bound
static float clampf(float v, float lo, float hi) {
  if (isnan(v)) return 0.5f * (lo + hi);
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static double clampd(double v, double lo, double hi) {
  if (isnan(v)) return 0.5 * (lo + hi);
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
// L5 FIX: Clamp t to [0,1] and NaN-guard
static float lerpf(float a, float b, float t) {
  if (isnan(t)) t = 0.5f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return a + (b - a) * t;
}
static inline bool isFiniteF(float v) { return !isnan(v) && !isinf(v); }
static inline bool isFiniteD(double v) { return !isnan(v) && !isinf(v); }

static float lookupTable(const float *x, const float *y, int size, float value) {
  if (size <= 0) return 0.0f;
  if (value <= x[0]) return y[0];
  if (value >= x[size - 1]) return y[size - 1];
  for (int i = 0; i < size - 1; i++) {
    if (value >= x[i] && value <= x[i + 1]) {
      float denom = x[i + 1] - x[i];
      if (fabsf(denom) < 1e-6f) return y[i];  // guard against duplicate x entries
      float t = (value - x[i]) / denom;
      return lerpf(y[i], y[i + 1], t);
    }
  }
  return y[size - 1];
}

static float altitudeGain(double height_agl, float high_gain, float low_gain,
                          float high_m, float low_m) {
  if (height_agl >= high_m) return high_gain;
  if (height_agl <= low_m)  return low_gain;
  if (fabsf(high_m - low_m) < 0.001f) return low_gain;  // ✓ FIXED: fabsf for float args
  float t = (float)((height_agl - low_m) / (high_m - low_m));
  return lerpf(low_gain, high_gain, t);
}

static float wrapAngle(float angle) {
  if (!isFiniteF(angle)) return 0.0f;
  angle = fmodf(angle, TWO_PI);
  if (angle > PI) angle -= TWO_PI;
  if (angle < -PI) angle += TWO_PI;
  return angle;
}
static float wrapAngleDeg(float angle) {
  if (!isFiniteF(angle)) return 0.0f;
  angle = fmodf(angle, 360.0f);
  if (angle < 0.0f) angle += 360.0f;
  return angle;
}

// =====================================================================
//  HARDWARE WATCHDOG (Teensy 4.0 RTWDOG)
// =====================================================================
// Resets the MCU if the main loop hangs (I2C bus hang, deadlock, etc.).
// Uses the IMXRT1062 Real-Time Watchdog with 32kHz LPO clock.
#ifndef WDOG_CS_CMD32EN
#define WDOG_CS_CMD32EN ((uint32_t)(1 << 13))
#endif

static void setupWatchdog() {
  // RTWDOG unlock sequence
  __disable_irq();
  WDOG3_CNT = 0xD928C520;  // unlock
  WDOG3_TOVAL = (WATCHDOG_TIMEOUT_MS * 32768UL / 1000UL);  // 32.768 kHz LPO: exact tick count
  WDOG3_CS = WDOG_CS_EN | WDOG_CS_CLK(1) | WDOG_CS_UPDATE | WDOG_CS_CMD32EN;
  __enable_irq();
  // Immediately refresh counter — if disableWatchdog() left the counter running,
  // it could be near overflow and trigger an instant reset on re-enable.
  noInterrupts();
  WDOG3_CNT = 0xB480A602;
  interrupts();
  Serial.printf("Watchdog enabled: %lu ms timeout\n", WATCHDOG_TIMEOUT_MS);
}

static inline void kickWatchdog() {
  // RTWDOG refresh sequence (must write in correct order)
  noInterrupts();
  WDOG3_CNT = 0xB480A602;
  interrupts();
}

static void disableWatchdog() {
  // Disable RTWDOG — used at start of setup() to prevent a watchdog inherited
  // from the previous firmware run from firing during bootloader/init.
  // The deferred arming in loop() re-enables it after 5 stable iterations.
  __disable_irq();
  WDOG3_CNT = 0xD928C520;  // unlock
  WDOG3_CS = WDOG_CS_CLK(1) | WDOG_CS_UPDATE | WDOG_CS_CMD32EN;  // same config minus EN bit
  __enable_irq();
}

// =====================================================================
//  SNVS STATE RECOVERY (warm-reboot persistence)
// =====================================================================
// SNVS_LPGPR0-3: 4×32-bit registers that survive warm resets (watchdog,
// software reset) but clear on power-off. Used to preserve critical
// flight state across watchdog reboots so recovery takes ~2-3s instead
// of 15-20s.
//
// Layout:
//   LPGPR0: [magic:16][state:4][flags:4][reboot_count:8]
//   LPGPR1: sea_level_pressure_hpa as float
//   LPGPR2: heading_offset_runtime as float
//   LPGPR3: reserved
//
static constexpr uint16_t SNVS_MAGIC = 0xAE70;

// Ensure SNVS register macros are available (Teensy 4.x imxrt.h defines these,
// but provide fallback definitions for safety).
#ifndef SNVS_LPGPR0
// IMXRT1062 SNVS LP General Purpose Registers (NXP Ref Manual Rev 3, offset 0x100-0x10C)
#define SNVS_LPGPR0 (*(volatile uint32_t *)0x400D4100)
#define SNVS_LPGPR1 (*(volatile uint32_t *)0x400D4104)
#define SNVS_LPGPR2 (*(volatile uint32_t *)0x400D4108)
#define SNVS_LPGPR3 (*(volatile uint32_t *)0x400D410C)
#endif

static void saveStateToSNVS() {
  uint8_t flags = 0;
  if (sea_level_calibrated) flags |= 0x01;
  if (auto_heading_done)    flags |= 0x02;
  if (targetReceived)       flags |= 0x04;

  uint8_t reboot_count = SNVS_LPGPR0 & 0xFF;  // preserve existing count

  SNVS_LPGPR0 = ((uint32_t)SNVS_MAGIC << 16) |
                 ((uint32_t)((uint8_t)state & 0x0F) << 12) |
                 ((uint32_t)(flags & 0x0F) << 8) |
                 reboot_count;

  float qnh = (float)sea_level_pressure_hpa;
  uint32_t tmp;
  memcpy(&tmp, &qnh, 4);
  SNVS_LPGPR1 = tmp;
  memcpy(&tmp, &heading_offset_runtime, 4);
  SNVS_LPGPR2 = tmp;

  // C3 FIX: Persist target coordinates across warm reboot.
  // Pack lat/lon as int16 with 1e-4 degree resolution (~11m precision).
  // Coarse but prevents loss of target on watchdog reset.
  if (targetReceived) {
    int16_t lat_e4 = (int16_t)constrain((int32_t)round(target_lat * 1e4), -32768, 32767);
    int16_t lon_e4 = (int16_t)constrain((int32_t)round(target_lon * 1e4), -32768, 32767);
    SNVS_LPGPR3 = ((uint32_t)(uint16_t)lat_e4 << 16) | (uint16_t)lon_e4;
  }
}

static bool restoreStateFromSNVS() {
  uint32_t reg0 = SNVS_LPGPR0;
  uint16_t magic = (reg0 >> 16) & 0xFFFF;
  if (magic != SNVS_MAGIC) return false;  // cold boot — no saved state

  uint8_t saved_state  = (reg0 >> 12) & 0x0F;
  // L4 FIX: Validate SNVS data isn't stale garbage from uninitialized memory
  if (saved_state > (uint8_t)LANDED) {
    Serial.println("SNVS: Invalid saved state — treating as cold boot");
    return false;
  }
  uint8_t flags        = (reg0 >> 8) & 0x0F;
  uint8_t reboot_count = reg0 & 0xFF;

  // Increment reboot counter (saturate at 255)
  reboot_count = (reboot_count < 255) ? reboot_count + 1 : 255;
  SNVS_LPGPR0 = (reg0 & 0xFFFFFF00) | reboot_count;

  // L10 FIX: Safe mode after 5 consecutive reboots — likely persistent crash
  if (reboot_count >= 5) {
    Serial.printf("SNVS: %u consecutive reboots — entering safe mode (servos neutral)\n", reboot_count);
    return false;
  }

  Serial.printf("SNVS: Warm reboot #%u detected (was state %u)\n", reboot_count, saved_state);

  // Restore QNH (most impactful — saves ~15s recalibration)
  uint32_t raw1 = SNVS_LPGPR1;
  float qnh;
  memcpy(&qnh, &raw1, 4);
  if (qnh > 900.0f && qnh < 1100.0f) {
    sea_level_pressure_hpa = qnh;
    sea_level_calibrated = (flags & 0x01);
    Serial.printf("SNVS: Restored QNH = %.1f hPa\n", qnh);
  }

  // Restore heading offset
  uint32_t raw2 = SNVS_LPGPR2;
  float hoff;
  memcpy(&hoff, &raw2, 4);
  if (fabsf(hoff) < 3.15f) {  // sanity: within +/-pi
    heading_offset_runtime = hoff;
    auto_heading_done = (flags & 0x02);
    Serial.printf("SNVS: Restored heading offset = %.3f rad\n", hoff);
  }

  // Restore state — cap at WAIT_FOR_STABLE_DESCENT since EKF/wind profile
  // can't be saved. System re-detects stable descent quickly (~2s).
  if (saved_state >= (uint8_t)GUIDED_DESCENT && saved_state < (uint8_t)LANDED) {
    state = WAIT_FOR_STABLE_DESCENT;
    Serial.println("SNVS: Resuming at WAIT_FOR_STABLE_DESCENT (fast recovery)");
  }

  // C3 FIX: Restore target coordinates from SNVS
  if ((flags & 0x04) && !USE_HARDCODED_TARGET) {
    uint32_t reg3 = SNVS_LPGPR3;
    int16_t lat_e4 = (int16_t)(reg3 >> 16);
    int16_t lon_e4 = (int16_t)(reg3 & 0xFFFF);
    double restored_lat = lat_e4 / 1e4;
    double restored_lon = lon_e4 / 1e4;
    // Sanity check: valid GPS coordinates
    if (fabs(restored_lat) > 1.0 && fabs(restored_lat) < 90.0 &&
        fabs(restored_lon) > 1.0 && fabs(restored_lon) < 180.0) {
      target_lat = restored_lat;
      target_lon = restored_lon;
      // FIX HIGH-1: Do NOT set targetReceived — target_alt_msl is 0.0 (not saved in SNVS).
      // Setting targetReceived=true with wrong altitude causes FLARE at wrong height.
      // Ground station re-sends full target within ~1-2s on state regression.
      // Lat/lon are pre-loaded for faster convergence when target packet arrives.
      targetReceived = false;
      Serial.printf("SNVS: Pre-loaded target lat/lon: %.4f, %.4f (awaiting altitude via LoRa)\n",
                    target_lat, target_lon);
    } else {
      Serial.println("SNVS: WARNING — target coords invalid after restore!");
    }
  }

  return true;
}

// =====================================================================
//  I2C BUS RECOVERY
// =====================================================================
// BNO085 can hold SDA low, killing the entire I2C bus (both BNO085 and
// BMP280). This section detects stuck sensors via consecutive read
// failures, toggles SCL to free SDA, reinitializes Wire, and
// reinitializes the failed sensor(s). Non-blocking.

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
  // Generate STOP condition: SDA low→high while SCL is high
  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(5);
  // Reinitialize Wire
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(200);  // re-apply timeout after bus recovery
}

static void reinitBMP280() {
  if (bmp.begin(0x76)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1);
    bmp_fail_count = 0;
    bmp_operational = true;
    Serial.println("I2C: BMP280 recovered");
  } else {
    Serial.println("I2C: BMP280 reinit FAILED");
  }
}

static void reinitBNO085() {
  if (bno_reinit_failures >= 3) {
    bno_operational = false;
    return;
  }
  Wire.setTimeout(100);
  if (bno08x.begin_I2C(0x4A, &Wire)) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 200000);
    bno08x.enableReport(SH2_ACCELEROMETER, 50000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);
    bno_fail_count = 0;
    bno_operational = true;
    bno_reinit_failures = 0;
    heading_valid = false;
    Serial.println("I2C: BNO085 recovered");
  } else {
    bno_reinit_failures++;
    Serial.printf("I2C: BNO085 reinit FAILED (%d/3)\n", bno_reinit_failures);
    if (bno_reinit_failures >= 3) {
      bno_operational = false;
      Serial.println("I2C: BNO085 PERMANENTLY DISABLED — will use EKF/GNSS heading");
    }
  }
  Wire.setTimeout(200);
}

static void checkI2CRecovery() {
  // C2 FIX: Enable I2C recovery from WAIT_FOR_DROP onward (was GUIDED_DESCENT,
  // blocking recovery during pre-guidance when sensors are most needed for init)
  if (state < WAIT_FOR_DROP) return;

  uint32_t now = millis();

  // Count failures based on data staleness
  if (bmp_operational && last_bmp_success_ms > 0 &&
      (now - last_bmp_success_ms) > I2C_SENSOR_TIMEOUT_MS) {
    bmp_fail_count++;
    last_bmp_success_ms = now;  // reset timer for next check interval
  }
  if (bno_operational && last_bno_success_ms > 0 &&
      (now - last_bno_success_ms) > I2C_SENSOR_TIMEOUT_MS) {
    bno_fail_count++;
    last_bno_success_ms = now;
  }

  bmp_fail_count = min(bmp_fail_count, I2C_FAIL_THRESHOLD + 2);
  bno_fail_count = min(bno_fail_count, I2C_FAIL_THRESHOLD + 2);

  bool bmp_hung = (bmp_fail_count >= I2C_FAIL_THRESHOLD);
  bool bno_hung = (bno_fail_count >= I2C_FAIL_THRESHOLD) && (bno_reinit_failures < 3);
  if (!bmp_hung && !bno_hung) return;

  // Cooldown between recovery attempts
  if (now - last_i2c_recovery_ms < I2C_RECOVERY_COOLDOWN_MS) return;
  last_i2c_recovery_ms = now;

  Serial.printf("I2C RECOVERY: bmp_fails=%d bno_fails=%d\n",
                bmp_fail_count, bno_fail_count);

  // Step 1: Free the bus
  i2cBusRecovery();
  kickWatchdog();  // bus recovery (SCL toggle + reinit Wire) takes time

  // Step 2: Reinitialize ALL I2C sensors — Wire.end()/begin() in bus recovery
  // disrupts every device on the bus, not just the failed one. If we only reinit
  // the failed sensor, the other sensor's next I2C call can hang on the reset bus.
  bmp_operational = false;
  reinitBMP280();
  kickWatchdog();
  if (bno_reinit_failures < 3) {
    bno_operational = false;
    reinitBNO085();
    kickWatchdog();
  }
}

// =====================================================================
//  HEADING HELPERS
// =====================================================================
// EKF-derived airspeed heading: ground velocity minus wind = airspeed vector
static constexpr float EKF_HEADING_MIN_AIRSPEED = 3.0f;  // m/s minimum to trust direction
static double getEKFAirspeedHeading(bool &valid) {
  valid = false;
  if (!ekf9.initialized || !ekf9.healthy) return 0.0;
  double air_n = ekf9.x[EKF9_VN_IDX] - ekf9.x[EKF9_WN_IDX];
  double air_e = ekf9.x[EKF9_VE_IDX] - ekf9.x[EKF9_WE_IDX];
  double air_speed = sqrt(air_n * air_n + air_e * air_e);
  if (air_speed < EKF_HEADING_MIN_AIRSPEED) return 0.0;
  valid = true;
  return atan2(air_e, air_n);
}

static double getHeadingRad() {
  bool bno_ok = heading_valid && (millis() - last_heading_ms) < 1000;
  // FIX M2: Hysteresis on heading confidence threshold — prevents mode chattering.
  // Enter BNO085 mode at 0.35, stay until drops below 0.25. Found by heading-auditor.
  static bool using_bno_heading = false;
  if (bno_ok) {
    if (using_bno_heading)
      using_bno_heading = heading_confidence >= 0.25f;
    else
      using_bno_heading = heading_confidence >= 0.35f;
  } else {
    using_bno_heading = false;
  }
  if (using_bno_heading) {
    return heading_rad;  // BNO085 is good — use it
  }
  // BNO085 degraded — try EKF airspeed heading
  bool ekf_heading_ok = false;
  double ekf_hdg = getEKFAirspeedHeading(ekf_heading_ok);
  if (ekf_heading_ok) {
    if (bno_ok) {
      // Blend: low BNO confidence → weight toward EKF
      float w = clampf(heading_confidence / 0.3f, 0.0f, 1.0f);  // 1.0 at conf=0.3, 0.0 at conf=0
      double delta = wrapAngle(heading_rad - ekf_hdg);
      return wrapAngle(ekf_hdg + delta * w);
    }
    return ekf_hdg;
  }
  // Fallbacks
  if (bno_ok) return heading_rad;
  if (ground_track_valid) return ground_track_rad;
  return last_heading_rad;
}
static bool isHeadingFresh() {
  bool bno_ok = heading_valid && (millis() - last_heading_ms) < 1000;
  if (bno_ok) return true;
  // EKF heading counts as fresh if available
  bool ekf_ok = false;
  getEKFAirspeedHeading(ekf_ok);
  return ekf_ok;
}
static float getHeadingConfidence() {
  if (!heading_valid) {
    // No BNO085 — check if EKF heading available
    bool ekf_ok = false;
    getEKFAirspeedHeading(ekf_ok);
    return ekf_ok ? 0.5f : 0.0f;  // EKF heading has moderate confidence
  }
  uint32_t age_ms = millis() - last_heading_ms;
  if (age_ms >= HEADING_CONFIDENCE_MAX_DT_MS) return 0.0f;
  float age_factor = 1.0f - (float)age_ms / (float)HEADING_CONFIDENCE_MAX_DT_MS;
  return clampf(heading_confidence * age_factor, 0.0f, 1.0f);
}

// =====================================================================
//  BRAKE / AERODYNAMIC LOOKUPS
// =====================================================================
static float normalizeBrake(float left, float right) {
  // Only count positive brake (negative = slack line, no physical effect)
  return clampf(0.5f * (fmaxf(0.0f, left) + fmaxf(0.0f, right)), 0.0f, 1.0f);
}
static float brakeToSinkRate(float brake) {
  return clampf(lookupTable(BRAKE_TABLE, SINK_RATE_TABLE, BRAKE_TABLE_SIZE, brake),
                1.0f, SINK_RATE_MAX);
}
static float brakeToGlideRatio(float brake) {
  return clampf(lookupTable(BRAKE_TABLE, GLIDE_RATIO_TABLE, BRAKE_TABLE_SIZE, brake),
                0.8f, 3.5f);
}
static float brakeToTurnRateScale(float brake) {
  return clampf(lookupTable(BRAKE_TABLE, TURN_RATE_SCALE_TABLE, BRAKE_TABLE_SIZE, brake),
                0.5f, 2.0f);
}
// Air density correction: airspeed and sink rate increase at altitude due to lower air density.
// At 2000m MSL, ρ = 0.822 * ρ₀ → airspeed/sink increase by 1/sqrt(0.822) = +10.3%.
// Linear approximation of 1/sqrt(ρ/ρ₀), max error 0.25% at 1000m. Returns 1.0 at sea level.
static inline float airDensityFactor(float alt_msl_m) {
  return 1.0f + fmaxf(alt_msl_m, 0.0f) * (1.0f / 19400.0f);
}

static float turnRateFromBank(float bank_rad, float airspeed) {
  if (airspeed < 0.1f) return 0.0f;
  return (9.81f * tanf(bank_rad)) / airspeed;
}

// =====================================================================
//  WIND LAYER HELPERS
// =====================================================================
// Fine layers: indices 0-19 (0-99.99m, 5m steps).
// Coarse layers: indices 20-95 (100-2000m, 25m steps).
// Boundary at exactly 100.0m maps to coarse layer 20.
static int windLayerIndex(double height_agl) {
  if (height_agl < 0) return 0;
  if (height_agl < WIND_LAYER_FINE_CEIL_M) {
    // Fine layers: 5m steps below 100m → indices 0-19
    int idx = (int)floor(height_agl / WIND_LAYER_FINE_STEP_M);
    return min(idx, WIND_LAYER_FINE_COUNT - 1);
  }
  // Coarse layers: 25m steps above 100m → indices 20-95
  int idx = WIND_LAYER_FINE_COUNT +
            (int)floor((height_agl - WIND_LAYER_FINE_CEIL_M) / WIND_LAYER_COARSE_STEP_M);
  if (idx >= WIND_LAYER_COUNT) idx = WIND_LAYER_COUNT - 1;
  return idx;
}

// Convert wind layer index back to representative altitude (layer center)
static float windLayerAltitude(int idx) {
  if (idx < WIND_LAYER_FINE_COUNT) {
    return idx * WIND_LAYER_FINE_STEP_M + WIND_LAYER_FINE_STEP_M * 0.5f;
  }
  return WIND_LAYER_FINE_CEIL_M +
         (idx - WIND_LAYER_FINE_COUNT) * WIND_LAYER_COARSE_STEP_M +
         WIND_LAYER_COARSE_STEP_M * 0.5f;
}

// Step size for a given wind layer (used in landing prediction descent simulation)
static float windLayerStepSize(int idx) {
  return (idx < WIND_LAYER_FINE_COUNT) ? WIND_LAYER_FINE_STEP_M : WIND_LAYER_COARSE_STEP_M;
}

// ---- Ground wind extrapolation using log boundary layer ----
// For unvisited layers near the ground, extrapolate from the lowest
// observed wind layer using: V(z) = V(z_ref) * ln(z/z0) / ln(z_ref/z0)
// z0 = 0.03m (open grass/farmland — typical for CanSat competition fields)
static constexpr float WIND_Z0_M = 0.03f;

// M5 FIX: Cached result — updated after wind writes, avoids O(n) scan in MPC inner loop
static int cached_lowest_observed_layer = -1;

static int findLowestObservedLayer() {
  for (int i = 0; i < WIND_LAYER_COUNT; i++) {
    if (wind_layer_ms[i] > 0) return i;
  }
  return -1;  // no observations yet
}

static void updateLowestObservedLayerCache() {
  cached_lowest_observed_layer = findLowestObservedLayer();
}

// Get wind at a layer index, with log-law extrapolation for unvisited ground layers
static void getWindAtLayer(int idx, double &wn, double &we) {
  if (wind_layer_ms[idx] > 0) {
    wn = wind_layer_north[idx];
    we = wind_layer_east[idx];
    return;
  }
  // Unvisited: try log-law extrapolation from lowest observed layer
  // M5 FIX: Use cached value (updated after wind writes) to avoid O(n) scan per call
  int ref_idx = cached_lowest_observed_layer;
  if (ref_idx < 0) {
    // No observations at all — use global EKF/EMA wind
    wn = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WN_IDX] : wind_north;
    we = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WE_IDX] : wind_east;
    return;
  }
  float z_ref = fmaxf(windLayerAltitude(ref_idx), 1.0f);
  float z_query = fmaxf(windLayerAltitude(idx), 0.5f);
  float log_ratio = logf(z_query / WIND_Z0_M) / logf(z_ref / WIND_Z0_M);
  log_ratio = clampf(log_ratio, 0.3f, 1.5f);  // bound extrapolation
  wn = wind_layer_north[ref_idx] * log_ratio;
  we = wind_layer_east[ref_idx] * log_ratio;
}

static void smoothWindLayers() {
  // Use temp buffers so smoothing reads only from original values (no directional bias)
  // Only include VISITED neighbors to avoid biasing toward zero
  static float tmp_n[WIND_LAYER_COUNT], tmp_e[WIND_LAYER_COUNT];
  for (int i = 0; i < WIND_LAYER_COUNT; i++) {
    if (wind_layer_ms[i] == 0) {
      // Unvisited layer: leave as-is (will use fallback wind at query time)
      tmp_n[i] = wind_layer_north[i];
      tmp_e[i] = wind_layer_east[i];
      continue;
    }
    int lo = max(0, i - (int)WIND_LAYER_BLEND_RADIUS);
    int hi = min(WIND_LAYER_COUNT - 1, i + (int)WIND_LAYER_BLEND_RADIUS);
    float sum_n = 0.0f, sum_e = 0.0f;
    int count = 0;
    for (int j = lo; j <= hi; j++) {
      if (wind_layer_ms[j] == 0) continue;  // skip unvisited neighbors
      sum_n += wind_layer_north[j];
      sum_e += wind_layer_east[j];
      count++;
    }
    if (count > 0) {
      float avg_n = sum_n / count;
      float avg_e = sum_e / count;
      tmp_n[i] = wind_layer_north[i] * (1.0f - WIND_LAYER_SMOOTH_ALPHA) +
                 avg_n * WIND_LAYER_SMOOTH_ALPHA;
      tmp_e[i] = wind_layer_east[i] * (1.0f - WIND_LAYER_SMOOTH_ALPHA) +
                 avg_e * WIND_LAYER_SMOOTH_ALPHA;
    } else {
      tmp_n[i] = wind_layer_north[i];
      tmp_e[i] = wind_layer_east[i];
    }
  }
  memcpy(wind_layer_north, tmp_n, sizeof(wind_layer_north));
  memcpy(wind_layer_east,  tmp_e, sizeof(wind_layer_east));
}

static void seedWindProfile() {
  if (!USE_SEEDED_WIND_PROFILE) {
    for (int i = 0; i < WIND_LAYER_COUNT; i++) {
      wind_layer_north[i] = 0.0;
      wind_layer_east[i]  = 0.0;
      wind_layer_ms[i]    = 0;  // 0 = never observed
    }
    return;
  }
  for (int i = 0; i < WIND_LAYER_COUNT; i++) {
    float altitude = windLayerAltitude(i);
    float t = altitude / WARSAW_PROFILE_MAX_ALT_M;
    float idx_f = t * (WARSAW_WIND_BINS - 1);
    int idx0 = (int)floor(idx_f);
    int idx1 = idx0 + 1;
    if (idx0 < 0) idx0 = 0;
    if (idx1 >= WARSAW_WIND_BINS) idx1 = WARSAW_WIND_BINS - 1;
    float mix = idx_f - idx0;
    float dir_deg = wrapAngleDeg(WARSAW_WIND_DIR_DEG[idx0] * (1.0f - mix) +
                                 WARSAW_WIND_DIR_DEG[idx1] * mix);
    float spd = WARSAW_WIND_SPD_MPS[idx0] * (1.0f - mix) +
                WARSAW_WIND_SPD_MPS[idx1] * mix;
    float dir_rad = dir_deg * DEG_TO_RAD;
    wind_layer_north[i] = -spd * cosf(dir_rad);
    wind_layer_east[i]  = -spd * sinf(dir_rad);
    wind_layer_ms[i]    = millis();
  }
}

// =====================================================================
//  SAFETY & GUIDANCE HELPERS
// =====================================================================
static bool isGNSSQualityGood();  // forward declaration

// Returns a gain factor 0.0–1.0 for graduated wind degradation.
// - Below MAX_WIND_FOR_GUIDANCE (6 m/s): full control (1.0)
// - 6–10 m/s: linear reduction to 0.3
// - 10–12 m/s: heavy damping (0.15)
// - Above 12 m/s: emergency — neutral servos (0.0)
// ✓ v5.6: Added serial warnings at wind thresholds for operator awareness.
static uint32_t last_wind_warn_ms = 0;
static float guidanceWindGain() {
  if (wind_speed >= WIND_EMERGENCY_MPS) {
    if (millis() - last_wind_warn_ms > 2000) {
      Serial.printf("!! WIND EMERGENCY: %.1f m/s >= %.1f — GUIDANCE SUSPENDED, servos neutral !!\n",
                    wind_speed, WIND_EMERGENCY_MPS);
      last_wind_warn_ms = millis();
    }
    return 0.0f;
  }
  if (wind_speed >= WIND_DAMPED_START_MPS) {
    if (millis() - last_wind_warn_ms > 3000) {
      Serial.printf("! WIND WARNING: %.1f m/s — guidance heavily damped (15%%) !\n", wind_speed);
      last_wind_warn_ms = millis();
    }
    return 0.15f;
  }
  if (wind_speed <= MAX_WIND_FOR_GUIDANCE) return 1.0f;
  // Linear ramp from 1.0 at threshold to 0.3 at 10 m/s
  if (millis() - last_wind_warn_ms > 5000) {
    Serial.printf("WIND: %.1f m/s — guidance degraded\n", wind_speed);
    last_wind_warn_ms = millis();
  }
  float t = (float)((wind_speed - MAX_WIND_FOR_GUIDANCE) /
                    (WIND_DAMPED_START_MPS - MAX_WIND_FOR_GUIDANCE));
  return lerpf(1.0f, 0.3f, clampf(t, 0.0f, 1.0f));
}

static bool guidanceSafe() {
  // Wind: graduated degradation instead of hard cutoff.
  // guidanceWindGain() returns 0.0 only above WIND_EMERGENCY_MPS (12 m/s).
  if (guidanceWindGain() <= 0.0f) return false;
  if (gnss_satellites < MIN_GNSS_SATS_FOR_GUIDANCE) return false;
  if (gnss_fix_type < 3) return false;  // ✓ v5.6: require 3D fix for guidance (was sat-count only)
  if (millis() - last_gnss_update > MAX_GNSS_AGE_MS) return false;
  if (ekf9.initialized && ekf9.healthy) {
    double speed = sqrt(ekf9.x[EKF9_VN_IDX] * ekf9.x[EKF9_VN_IDX] +
                        ekf9.x[EKF9_VE_IDX] * ekf9.x[EKF9_VE_IDX]);
    if (speed > MAX_EKF_SPEED_MPS) return false;
  }
  return true;
}

// ✓ v5.1: Adaptive feed-forward crab gain based on data quality.
// High confidence (good heading + many wind layers observed) → aggressive gain (0.95).
// Low confidence → conservative gain (0.65) to avoid oscillation from noisy estimates.
static float getWindFFGain() {
  float heading_conf = getHeadingConfidence();
  // Count observed wind layers in the lower 100m (fine layers, most relevant for approach)
  int observed = 0;
  for (int i = 0; i < WIND_LAYER_FINE_COUNT; i++) {
    if (wind_layer_ms[i] > 0) observed++;
  }
  float layer_fill = (float)observed / (float)WIND_LAYER_FINE_COUNT;  // 0-1
  // Confidence = heading quality × wind data availability
  float confidence = clampf(heading_conf * (0.5f + 0.5f * layer_fill), 0.0f, 1.0f);
  return lerpf(WIND_FF_GAIN_MIN, WIND_FF_GAIN_MAX, confidence);
}

static float applyGlideSlope(double height_agl) {
  // ✓ FIXED: Use actual elapsed time instead of BARO_UPDATE_INTERVAL_MS
  static uint32_t last_glide_ms = 0;
  uint32_t now = millis();
  float dt = 0.04f;  // default to 25Hz loop period
  if (last_glide_ms > 0) {
    dt = clampf((now - last_glide_ms) / 1000.0f, 0.01f, 0.5f);
  }
  last_glide_ms = now;

  float target_sink = altitudeGain(height_agl,
                                   GLIDESLOPE_TARGET_SINK_HIGH,
                                   GLIDESLOPE_TARGET_SINK_LOW,
                                   GLIDESLOPE_TARGET_HIGH_ALT_M,
                                   GLIDESLOPE_TARGET_LOW_ALT_M);

  // Terminal descent rate optimization: adjust target sink based on horizontal error.
  // Below 50m: if far from target, slow down to extend glide time.
  //            if right above target, speed up to minimize wind drift time.
  if (height_agl < FINAL_APPROACH_HEIGHT_M && predicted_error_m < 900.0) {
    // Optimal glide angle: descent per horizontal = 1/glide_ratio
    // Ideal sink for current error: error * sink / (error + height * GR)
    float glide_ratio = PARAFOIL_GLIDE_RATIO;
    float ideal_range = (float)height_agl * glide_ratio;
    float range_ratio = (float)predicted_error_m / fmaxf(ideal_range, 1.0f);

    // range_ratio < 1: we have excess range → speed up (higher sink)
    // range_ratio > 1: we're short → slow down (lower sink)
    // Clamp adjustment to ±1.0 m/s
    float sink_adjust = clampf((1.0f - range_ratio) * 0.8f, -1.0f, 1.0f);
    target_sink += sink_adjust;
    // ✓ FIX: Lower clamp was 3.5 m/s — below minimum achievable sink rate (5.5 m/s).
    // The PI controller would wind up trying to achieve an impossible target, driving
    // glide_cmd negative, reducing servo authority, and causing delayed recovery.
    target_sink = clampf(target_sink, NOMINAL_SINK_RATE, SINK_RATE_MAX - 1.0f);
  }

  // ✓ v5.1 CRITICAL FIX: Sign was inverted. Old: (filtered - target) → positive when sinking
  // too fast → adds brake → increases sink further (positive feedback). Correct sign:
  // (target - filtered) → positive when sinking too slow → adds brake to increase sink.
  float sink_err = (float)(target_sink - sink_rate_filtered);
  glide_integral += sink_err * dt;
  glide_integral = clampf(glide_integral, -1.0f, 1.0f);
  float cmd = GLIDESLOPE_KP * sink_err + GLIDESLOPE_KI * glide_integral;
  return clampf(cmd, -GLIDESLOPE_MAX_BRAKE, GLIDESLOPE_MAX_BRAKE);
}

static void updateGustDetector() {
  double delta = wind_speed - last_wind_speed;
  last_wind_speed = wind_speed;
  // BUG #12 FIX: detect both wind increases AND decreases
  if (fabs(delta) > GUST_SPEED_SPIKE_MPS) {
    gust_active = true;
    gust_until_ms = millis() + GUST_HOLD_MS;
  }
  if (gust_active && millis() > gust_until_ms) {
    gust_active = false;
  }
}

// =====================================================================
//  NAVIGATION FRAME
// =====================================================================
static void setOriginIfNeeded() {
  if (!origin_set && gnss_fix_type >= 3) {
    origin_lat = lat;
    origin_lon = lon;
    origin_alt_msl = gnss_alt_msl;
    origin_set = true;
  }
}

static void latLonToNE(double in_lat, double in_lon, double &n, double &e) {
  n = (in_lat - origin_lat) * metersPerDegLat(origin_lat);
  e = (in_lon - origin_lon) * metersPerDegLon(origin_lat);
}
static void neToLatLon(double n, double e, double &out_lat, double &out_lon) {
  out_lat = origin_lat + n / metersPerDegLat(origin_lat);
  out_lon = origin_lon + e / metersPerDegLon(origin_lat);
}
static void getEstimatedLatLon(double &out_lat, double &out_lon) {
  if (origin_set && ekf9.initialized) {
    neToLatLon(ekf9.x[EKF9_N_IDX], ekf9.x[EKF9_E_IDX], out_lat, out_lon);
    return;
  }
  out_lat = lat;
  out_lon = lon;
}

// =====================================================================
//  9-STATE EKF IMPLEMENTATION
// =====================================================================
// State: x = [N, E, D, vN, vE, vD, baro_bias, wN, wE]
// Prediction: constant-velocity position propagation, Q scaled by dt.
// Measurements: GNSS pos/vel, barometric altitude (with bias), parafoil model.
// Features: Joseph-form covariance, chi-squared gating, adaptive GNSS dropout.
// =====================================================================

// --- General scalar measurement update with arbitrary H vector ---
// H[EKF9_N]: measurement Jacobian row. z: measurement. r: noise variance.
// Returns true if accepted, false if chi-squared rejected or degenerate.
static bool ekf9UpdateGeneral(const float H[EKF9_N], float z, float r,
                               float nis_threshold, float *nis_out) {
  // Innovation: y = z - H*x
  double hx = 0.0;
  for (int i = 0; i < EKF9_N; i++) hx += (double)H[i] * ekf9.x[i];
  float y = (float)((double)z - hx);

  // P*H^T vector
  float PHt[EKF9_N];
  for (int i = 0; i < EKF9_N; i++) {
    float s = 0.0f;
    for (int j = 0; j < EKF9_N; j++) s += ekf9.P[i][j] * H[j];
    PHt[i] = s;
  }

  // Innovation covariance: S = H*P*H^T + r
  float S = r;
  for (int i = 0; i < EKF9_N; i++) S += H[i] * PHt[i];
  if (S < 1e-8f) return false;

  // Chi-squared innovation gate
  float nis = y * y / S;
  if (nis_out) *nis_out = nis;
  if (nis > nis_threshold) return false;

  // Kalman gain: K = P*H^T / S
  float K[EKF9_N];
  for (int i = 0; i < EKF9_N; i++) K[i] = PHt[i] / S;

  // State update
  for (int i = 0; i < EKF9_N; i++) ekf9.x[i] += (double)K[i] * (double)y;

  // Joseph form: P = (I-KH)*P*(I-KH)^T + K*r*K^T
  // Step 1: A = (I-KH)*P  where A[i][j] = P[i][j] - K[i]*HP[j]
  float HP[EKF9_N];  // H*P: HP[j] = sum_k H[k]*P[k][j]
  for (int j = 0; j < EKF9_N; j++) {
    float s = 0.0f;
    for (int k = 0; k < EKF9_N; k++) s += H[k] * ekf9.P[k][j];
    HP[j] = s;
  }
  static float A[EKF9_N][EKF9_N];
  for (int i = 0; i < EKF9_N; i++)
    for (int j = 0; j < EKF9_N; j++)
      A[i][j] = ekf9.P[i][j] - K[i] * HP[j];

  // Step 2: P = A*(I-KH)^T + K*r*K^T = A - AHt*K^T + K*r*K^T
  float AHt[EKF9_N];  // A*H^T
  for (int i = 0; i < EKF9_N; i++) {
    float s = 0.0f;
    for (int j = 0; j < EKF9_N; j++) s += A[i][j] * H[j];
    AHt[i] = s;
  }
  for (int i = 0; i < EKF9_N; i++)
    for (int j = 0; j < EKF9_N; j++)
      ekf9.P[i][j] = A[i][j] - AHt[i] * K[j] + K[i] * r * K[j];

  // Enforce symmetry
  for (int i = 0; i < EKF9_N; i++)
    for (int j = i + 1; j < EKF9_N; j++) {
      float avg = 0.5f * (ekf9.P[i][j] + ekf9.P[j][i]);
      ekf9.P[i][j] = avg;
      ekf9.P[j][i] = avg;
    }

  // Diagonal floor (prevent covariance collapse)
  // ✓ v5.6: Velocity and wind states use higher floor (1e-4) to prevent
  // over-confidence lockup after prolonged good data, which rejects valid
  // measurements after a brief GNSS dropout.
  for (int i = 0; i < EKF9_N; i++) {
    float floor_val = (i >= EKF9_VN_IDX) ? 1e-4f : 1e-6f;  // velocity/wind states: higher floor
    if (ekf9.P[i][i] < floor_val) ekf9.P[i][i] = floor_val;
  }

  return true;
}

// Identity-H wrapper: measurement directly observes state[idx]
static bool ekf9UpdateScalar(int idx, float z, float r,
                              float nis_threshold, float *nis_out) {
  float H[EKF9_N] = {};
  H[idx] = 1.0f;
  return ekf9UpdateGeneral(H, z, r, nis_threshold, nis_out);
}

// Update NIS exponential moving average for a channel (α=0.05)
static inline void updateNisEma(int channel, float nis) {
  static constexpr float NIS_EMA_ALPHA = 0.05f;
  ekf9.nis_ema[channel] = (1.0f - NIS_EMA_ALPHA) * ekf9.nis_ema[channel]
                         + NIS_EMA_ALPHA * nis;
}

// --- Initialization (requires QNH calibration + 3D GNSS + origin) ---
static uint32_t ekf9_recovery_start_ms = 0;

static void ekf9Init() {
  // Allow reinit if health failed
  if (ekf9.initialized && ekf9.healthy) return;
  if (!sea_level_calibrated || gnss_fix_type < 3 || !origin_set) return;

  // For reinit after health failure, require 3s of continuous good GNSS
  if (ekf9.initialized && !ekf9.healthy) {
    if (millis() - last_gnss_update > 1000) {
      ekf9_recovery_start_ms = 0;  // reset timer if GNSS stale
      return;
    }
    if (ekf9_recovery_start_ms == 0) { ekf9_recovery_start_ms = millis(); return; }
    if (millis() - ekf9_recovery_start_ms < 3000) return;  // wait 3s good GNSS
    ekf9_recovery_start_ms = 0;
    Serial.println("EKF9 REINIT after health failure");
  }

  memset(ekf9.x, 0, sizeof(ekf9.x));
  // Position: [0,0,0] since origin set from same fix
  // Velocity from GNSS — only use ground track if it's valid and speed is sufficient
  if (ground_track_valid && ground_speed >= MIN_GROUND_SPEED_FOR_TRACK) {
    ekf9.x[EKF9_VN_IDX] = ground_speed * cos(ground_track_rad);
    ekf9.x[EKF9_VE_IDX] = ground_speed * sin(ground_track_rad);
  } else {
    ekf9.x[EKF9_VN_IDX] = 0.0;
    ekf9.x[EKF9_VE_IDX] = 0.0;
  }
  ekf9.x[EKF9_VD_IDX] = -gnss_vertical_velocity;  // NED positive-down
  // Baro bias: initial estimate of barometer error
  ekf9.x[EKF9_BIAS_IDX] = baro_alt_msl - gnss_alt_msl;
  // Wind from current estimate
  ekf9.x[EKF9_WN_IDX] = wind_north;
  ekf9.x[EKF9_WE_IDX] = wind_east;

  // Initial covariance
  memset(ekf9.P, 0, sizeof(ekf9.P));
  ekf9.P[0][0] = 4.0f;    // N  — 2m 1-sigma
  ekf9.P[1][1] = 4.0f;    // E
  ekf9.P[2][2] = 9.0f;    // D  — 3m (vertical less certain)
  ekf9.P[3][3] = 4.0f;    // vN — 2 m/s
  ekf9.P[4][4] = 4.0f;    // vE
  ekf9.P[5][5] = 2.0f;    // vD — 1.4 m/s
  ekf9.P[6][6] = 25.0f;   // baro_bias — 5m (QNH may be imperfect)
  ekf9.P[7][7] = 12.0f;   // wN — 3.5 m/s
  ekf9.P[8][8] = 12.0f;   // wE

  ekf9.last_predict_ms = millis();
  ekf9.last_gnss_ms = millis();
  ekf9.gnss_dropout_duration_ms = 0;
  ekf9.gnss_reacq_countdown = 0;
  ekf9.gnss_reject_count = 0;
  ekf9.baro_reject_count = 0;
  ekf9.healthy = true;
  memset(ekf9.last_nis, 0, sizeof(ekf9.last_nis));
  for (int i = 0; i < EKF9_N; i++) ekf9.nis_ema[i] = 1.0f;  // expected NIS = 1.0
  ekf9.model_mismatch_since_ms = 0;
  ekf9.model_mismatch = false;
  ekf9.initialized = true;
  Serial.printf("EKF9 init: bias=%.1f wN=%.1f wE=%.1f\n",
                ekf9.x[EKF9_BIAS_IDX], ekf9.x[EKF9_WN_IDX], ekf9.x[EKF9_WE_IDX]);
}

// --- Prediction: constant-velocity with adaptive Q ---
static void ekf9Predict(float dt) {
  if (!ekf9.initialized || dt <= 0.0f || dt > 1.0f) return;

  // State propagation
  ekf9.x[EKF9_N_IDX]  += ekf9.x[EKF9_VN_IDX] * dt;
  ekf9.x[EKF9_E_IDX]  += ekf9.x[EKF9_VE_IDX] * dt;
  ekf9.x[EKF9_D_IDX]  += ekf9.x[EKF9_VD_IDX] * dt;
  // velocity, bias, wind: random walk (no change in prediction)

  // F = I + position-velocity coupling
  static float F[EKF9_N][EKF9_N];
  memset(F, 0, sizeof(F));
  for (int i = 0; i < EKF9_N; i++) F[i][i] = 1.0f;
  F[0][3] = dt;  // dN/dvN
  F[1][4] = dt;  // dE/dvE
  F[2][5] = dt;  // dD/dvD

  // P = F*P*F^T
  static float FP[EKF9_N][EKF9_N];
  for (int i = 0; i < EKF9_N; i++)
    for (int j = 0; j < EKF9_N; j++) {
      float s = 0.0f;
      for (int k = 0; k < EKF9_N; k++) s += F[i][k] * ekf9.P[k][j];
      FP[i][j] = s;
    }
  for (int i = 0; i < EKF9_N; i++)
    for (int j = 0; j < EKF9_N; j++) {
      float s = 0.0f;
      for (int k = 0; k < EKF9_N; k++) s += FP[i][k] * F[j][k];
      ekf9.P[i][j] = s;
    }

  // Adaptive Q during GNSS dropout (>2s without GNSS) and gusts
  float q_vel_scale = 1.0f, q_wind_scale = 1.0f, q_bias_scale = 1.0f;
  uint32_t gnss_age = millis() - ekf9.last_gnss_ms;
  if (gnss_age > 2000) {
    q_vel_scale = 2.0f;    // velocity less certain without GNSS
    q_wind_scale = 3.0f;   // wind highly uncertain
    q_bias_scale = 0.1f;   // ✓ FIXED v4.9: slow adaptation — allows bias correction when GNSS returns
  }
  // Gust: velocity and wind change faster than steady-state model expects
  if (gust_active) {
    q_vel_scale  = fmaxf(q_vel_scale, 1.5f);
    q_wind_scale = fmaxf(q_wind_scale, 2.0f);
  }
  // ✓ v5.3: Calm conditions — trust model more (tighter Q) for smoother state estimation
  float wind_mag_ekf = sqrtf((float)(ekf9.x[EKF9_WN_IDX] * ekf9.x[EKF9_WN_IDX] +
                                      ekf9.x[EKF9_WE_IDX] * ekf9.x[EKF9_WE_IDX]));
  if (wind_mag_ekf < 2.0f && !gust_active && gnss_age < 2000) {
    q_vel_scale  *= 0.5f;
    q_wind_scale *= 0.5f;
  }
  // ✓ v5.1: Turns violate the constant-velocity model (centripetal acceleration).
  // At max turn (diff=1.0): centripetal ~8.8 m/s² → 0.35 m/s per 40ms step unmodeled.
  // Inflate Q_VEL to admit this: scale = 1 + 2*|diff|, so max turn gives 3x Q.
  float servo_diff = fabsf(cmdR_current - cmdL_current);
  q_vel_scale = fmaxf(q_vel_scale, 1.0f + 2.0f * servo_diff);

  // Add process noise Q*dt
  ekf9.P[0][0] += EKF9_Q_POS * dt;
  ekf9.P[1][1] += EKF9_Q_POS * dt;
  ekf9.P[2][2] += EKF9_Q_POS * dt;
  ekf9.P[3][3] += EKF9_Q_VEL_H * q_vel_scale * dt;
  ekf9.P[4][4] += EKF9_Q_VEL_H * q_vel_scale * dt;
  ekf9.P[5][5] += EKF9_Q_VEL_V * q_vel_scale * dt;
  ekf9.P[6][6] += EKF9_Q_BARO_BIAS * q_bias_scale * dt;
  ekf9.P[7][7] += EKF9_Q_WIND * q_wind_scale * dt;
  ekf9.P[8][8] += EKF9_Q_WIND * q_wind_scale * dt;

  // M3 FIX: Clamp P diagonals to prevent runaway growth before health check triggers.
  // Health check fires at 1e6, but values above 1e4 are already meaningless for guidance.
  for (int i = 0; i < EKF9_N; i++) {
    if (ekf9.P[i][i] > 1e5f) ekf9.P[i][i] = 1e5f;
  }

  // Enforce symmetry
  for (int i = 0; i < EKF9_N; i++)
    for (int j = i + 1; j < EKF9_N; j++) {
      float avg = 0.5f * (ekf9.P[i][j] + ekf9.P[j][i]);
      ekf9.P[i][j] = avg;
      ekf9.P[j][i] = avg;
    }
}

// --- GNSS measurement update (position + velocity, adaptive R) ---
static void ekf9UpdateGNSS() {
  if (!ekf9.initialized || gnss_fix_type < 3 || !origin_set) return;

  uint32_t now = millis();
  uint32_t gnss_gap = now - ekf9.last_gnss_ms;

  // Detect re-acquisition after dropout
  if (gnss_gap > 2000 && ekf9.gnss_reacq_countdown == 0) {
    ekf9.gnss_reacq_countdown = 5;
    ekf9.gnss_dropout_duration_ms = gnss_gap;
  }
  ekf9.last_gnss_ms = now;

  // Adaptive R from GNSS accuracy reports
  float hacc_m = gnss_h_accuracy_mm * 0.001f;
  float vacc_m = gnss_v_accuracy_mm * 0.001f;
  float r_h = fmaxf(hacc_m * hacc_m, EKF9_R_POS_H_FLOOR);
  float r_v = fmaxf(vacc_m * vacc_m, EKF9_R_POS_V_FLOOR);

  // ✓ v5.4: pDOP gating — satellite geometry quality. hAcc alone misses poor geometry.
  // 6 sats in a narrow band (pDOP>5) give 3-5x worse position than hAcc suggests.
  uint16_t pdop_raw = gnss_stream_fallback ? 0 : gnss.getPDOP();  // returns pDOP × 100; skip in NMEA-only mode
  float pdop_f = pdop_raw * 0.01f;
  if (pdop_f > 8.0f) return;  // reject entirely — geometry too poor
  if (pdop_f > 3.0f) {
    float pdop_scale = pdop_f / 2.0f;
    r_h *= pdop_scale;
    r_v *= pdop_scale;
  }

  // ✓ v5.4: Pre-inflate R when LoRa link is weak — RTK corrections about to drop out.
  // Prepares EKF for degraded position accuracy before hAcc actually increases.
  if (lora_link_weak && gnss_carrier_solution >= 1) {
    r_h *= 2.0f;
    r_v *= 2.0f;
  }

  // ✓ FIXED v4.9: quadratic R inflation — position uncertainty grows quadratically with gap time.
  // For 2s dropout at 11 m/s: ~22m drift. Linear gave inflate=1.0 (no inflation!) for <2s gaps.
  if (ekf9.gnss_reacq_countdown > 0) {
    float dropout_s = ekf9.gnss_dropout_duration_ms * 0.001f;
    float inflate = clampf(dropout_s * dropout_s * 0.5f, 1.0f, 100.0f);
    r_h *= inflate;
    r_v *= inflate;
    ekf9.gnss_reacq_countdown--;
  }

  // Adaptive R from NIS EMA: scale R by running average NIS, clamped [0.5, 4.0]
  // When NIS EMA ≈ 1.0 (well-tuned), no change. >1 = measurement noisier than modeled.
  float nis_scale_pos_h = clampf(0.5f * (ekf9.nis_ema[0] + ekf9.nis_ema[1]), 0.5f, 4.0f);
  float nis_scale_pos_v = clampf(ekf9.nis_ema[2], 0.5f, 4.0f);
  r_h *= nis_scale_pos_h;
  r_v *= nis_scale_pos_v;

  // Position measurements
  double meas_n = 0.0, meas_e = 0.0;
  latLonToNE(lat, lon, meas_n, meas_e);
  double meas_d = -(gnss_alt_msl - origin_alt_msl);  // NED down

  float nis;
  if (!ekf9UpdateScalar(EKF9_N_IDX, (float)meas_n, r_h, EKF9_NIS_POS, &nis))
    ekf9.gnss_reject_count++;
  ekf9.last_nis[0] = nis;
  updateNisEma(0, nis);

  if (!ekf9UpdateScalar(EKF9_E_IDX, (float)meas_e, r_h, EKF9_NIS_POS, &nis))
    ekf9.gnss_reject_count++;
  ekf9.last_nis[1] = nis;
  updateNisEma(1, nis);

  if (!ekf9UpdateScalar(EKF9_D_IDX, (float)meas_d, r_v, EKF9_NIS_POS, &nis))
    ekf9.gnss_reject_count++;
  ekf9.last_nis[2] = nis;
  updateNisEma(2, nis);

  // Velocity measurements
  // ✓ FIXED: only decompose velocity when ground track is valid (speed >= 0.5 m/s),
  // otherwise use zero horizontal velocity to avoid feeding stale heading direction
  double meas_vd = -gnss_vertical_velocity;  // positive down

  // ✓ v5.1: Use GNSS speed accuracy (sAcc) as velocity R floor when available.
  // sAcc directly measures velocity noise — adapts faster than NIS EMA.
  float r_vel_h_base = EKF9_R_VEL_H;
  uint32_t sacc_mmps = gnss_stream_fallback ? 0 : gnss.getSpeedAccEst();  // mm/s; skip in NMEA-only mode
  if (sacc_mmps > 0 && sacc_mmps < 10000) {
    float sacc_mps = sacc_mmps * 0.001f;
    r_vel_h_base = fmaxf(sacc_mps * sacc_mps, EKF9_R_VEL_H);
  }
  // Adaptive R for velocity from NIS EMA
  float nis_scale_vel = clampf(0.5f * (ekf9.nis_ema[3] + ekf9.nis_ema[4]), 0.5f, 4.0f);

  if (ground_track_valid) {
    double meas_vn = ground_speed * cos(ground_track_rad);
    double meas_ve = ground_speed * sin(ground_track_rad);
    float r_vel_h_adapted = r_vel_h_base * nis_scale_vel;
    if (!ekf9UpdateScalar(EKF9_VN_IDX, (float)meas_vn, r_vel_h_adapted, EKF9_NIS_VEL, &nis))
      ekf9.gnss_reject_count++;
    ekf9.last_nis[3] = nis;
    updateNisEma(3, nis);
    if (!ekf9UpdateScalar(EKF9_VE_IDX, (float)meas_ve, r_vel_h_adapted, EKF9_NIS_VEL, &nis))
      ekf9.gnss_reject_count++;
    ekf9.last_nis[4] = nis;
    updateNisEma(4, nis);
  } else {
    // Low speed — update with zero velocity and inflated R
    float r_low_speed = r_vel_h_base * 4.0f * nis_scale_vel;
    if (!ekf9UpdateScalar(EKF9_VN_IDX, 0.0f, r_low_speed, EKF9_NIS_VEL, &nis))
      ekf9.gnss_reject_count++;
    ekf9.last_nis[3] = nis;
    updateNisEma(3, nis);
    if (!ekf9UpdateScalar(EKF9_VE_IDX, 0.0f, r_low_speed, EKF9_NIS_VEL, &nis))
      ekf9.gnss_reject_count++;
    ekf9.last_nis[4] = nis;
    updateNisEma(4, nis);
  }

  // FIX M8: Skip vD update in NMEA-only mode — NMEA provides no vertical velocity.
  // Feeding stale gnss_vertical_velocity (0.0 or last UBX value) corrupts EKF vD.
  // Baro update (ekf9UpdateBaro) still constrains altitude. Found by sensor-auditor.
  if (!gnss_stream_fallback) {
    float nis_scale_vd = clampf(ekf9.nis_ema[5], 0.5f, 4.0f);
    if (!ekf9UpdateScalar(EKF9_VD_IDX, (float)meas_vd, EKF9_R_VEL_V * nis_scale_vd, EKF9_NIS_VEL, &nis))
      ekf9.gnss_reject_count++;
    ekf9.last_nis[5] = nis;
    updateNisEma(5, nis);
  }
}

// --- Barometric altitude update (non-identity H, estimates bias) ---
static void ekf9UpdateBaro() {
  if (!ekf9.initialized || !origin_set) return;

  // Measurement model: baro_alt_msl = origin_alt_msl - D + bias (D positive down,
  // bias captures sensor offset). z - Hx = (baro - origin) - (-D + bias) → zero when correct.
  // H = [0, 0, -1, 0, 0, 0, +1, 0, 0]
  // Adjusted z: z_adj = baro_alt_msl - origin_alt_msl, h(x) = -D + bias
  float H[EKF9_N] = {};
  H[EKF9_D_IDX] = -1.0f;
  H[EKF9_BIAS_IDX] = 1.0f;

  float z_adj = (float)(baro_alt_msl - origin_alt_msl);
  // Dynamic R: inflate during maneuvering (differential brake causes dynamic pressure transients)
  float brake_diff = fabsf(cmdL_current - cmdR_current);
  float r_baro = EKF9_R_BARO + brake_diff * 3.0f;  // up to 4.0 m² during full differential
  float nis;
  // Adaptive R from NIS EMA
  float nis_scale_baro = clampf(ekf9.nis_ema[6], 0.5f, 4.0f);
  r_baro *= nis_scale_baro;

  if (!ekf9UpdateGeneral(H, z_adj, r_baro, EKF9_NIS_BARO, &nis))
    ekf9.baro_reject_count++;
  ekf9.last_nis[6] = nis;
  updateNisEma(6, nis);
}

// --- Parafoil model pseudo-measurement (velocity-wind constraint) ---
static void ekf9UpdateModel() {
  if (!ekf9.initialized) return;
  if (!isHeadingFresh() || getHeadingConfidence() < 0.6f) return;
  if (state != GUIDED_DESCENT && state != FINAL_APPROACH &&
      state != TERMINAL_HOMING) return;

  // R depends on calibration state (detect placeholder values)
  float r = (SINK_RATE_TABLE[0] == 5.5f && GLIDE_RATIO_TABLE[0] == 2.0f)
            ? EKF9_R_MODEL_UNCAL : EKF9_R_MODEL;

  // Model mismatch detection: if NIS EMA for model channels exceeds 3.0
  // for >5 seconds, the parafoil model doesn't match reality. Inflate R
  // to 64 m²/s² so model constraint doesn't corrupt the EKF state.
  float model_nis_avg = 0.5f * (ekf9.nis_ema[7] + ekf9.nis_ema[8]);
  if (model_nis_avg > 3.0f) {
    if (ekf9.model_mismatch_since_ms == 0)
      ekf9.model_mismatch_since_ms = millis();
    if (millis() - ekf9.model_mismatch_since_ms > 5000) {
      ekf9.model_mismatch = true;
      r = 64.0f;
    }
  } else {
    ekf9.model_mismatch_since_ms = 0;
    ekf9.model_mismatch = false;
  }

  float brake = normalizeBrake(cmdL_current, cmdR_current);
  float rho_f = airDensityFactor((float)fused_alt_msl);
  double airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake) * rho_f;
  double heading = getHeadingRad();
  double as_n = airspeed * cos(heading);
  double as_e = airspeed * sin(heading);

  // Constraint: vN - wN ≈ airspeed * cos(heading)
  // z = as_n, h(x) = x[vN] - x[wN], H = [0,0,0,1,0,0,0,-1,0]
  {
    float H[EKF9_N] = {};
    H[EKF9_VN_IDX] = 1.0f;
    H[EKF9_WN_IDX] = -1.0f;
    float nis;
    ekf9UpdateGeneral(H, (float)as_n, r, EKF9_NIS_MODEL, &nis);
    ekf9.last_nis[7] = nis;
    updateNisEma(7, nis);
  }
  // Constraint: vE - wE ≈ airspeed * sin(heading)
  {
    float H[EKF9_N] = {};
    H[EKF9_VE_IDX] = 1.0f;
    H[EKF9_WE_IDX] = -1.0f;
    float nis;
    ekf9UpdateGeneral(H, (float)as_e, r, EKF9_NIS_MODEL, &nis);
    ekf9.last_nis[8] = nis;
    updateNisEma(8, nis);
  }
}

// --- Health monitoring ---
static bool ekf9Healthy() {
  if (!ekf9.initialized) return false;
  for (int i = 0; i < EKF9_N; i++) {
    if (isnan(ekf9.x[i]) || isinf(ekf9.x[i])) {
      ekf9.healthy = false;
      return false;
    }
  }
  for (int i = 0; i < EKF9_N; i++) {
    if (ekf9.P[i][i] < 0.0f || ekf9.P[i][i] > 1e6f || isnan(ekf9.P[i][i])) {
      ekf9.healthy = false;
      return false;
    }
  }
  // ✓ v5.1 AUDIT FIX: Full upper-triangle check for NaN/Inf propagation
  // (was adjacent-only P[i][i+1] — NaN in P[0][8] could go undetected)
  for (int i = 0; i < EKF9_N; i++)
    for (int j = i + 1; j < EKF9_N; j++) {
      if (isnan(ekf9.P[i][j]) || isinf(ekf9.P[i][j])) {
        ekf9.healthy = false;
        return false;
      }
    }
  ekf9.healthy = true;
  return true;
}

// --- Convenience: get EKF9 estimated position as lat/lon ---
static void ekf9GetLatLon(double &out_lat, double &out_lon) {
  if (ekf9.initialized && origin_set) {
    neToLatLon(ekf9.x[EKF9_N_IDX], ekf9.x[EKF9_E_IDX], out_lat, out_lon);
  } else {
    out_lat = lat;
    out_lon = lon;
  }
}

// --- Convenience: get EKF9 altitude MSL ---
static double ekf9GetAltMSL() {
  if (ekf9.initialized && origin_set)
    return origin_alt_msl - ekf9.x[EKF9_D_IDX];
  return fused_alt_msl;
}

// =====================================================================
//  WIND ESTIMATION (sole authority for wind_north/wind_east)
// =====================================================================
static void estimateWind() {
  if (state != GUIDED_DESCENT && state != FINAL_APPROACH &&
      state != TERMINAL_HOMING && state != WAIT_FOR_STABLE_DESCENT)
    return;

  double height_agl = fused_alt_msl - target_alt_msl;
  if (height_agl < MIN_WIND_UPDATE_ALT_M) return;

  float heading_conf = getHeadingConfidence();
  if (ground_speed < 0.5 || sink_rate_filtered < 0.3 || !isHeadingFresh() ||
      !ground_track_valid || !isGNSSQualityGood() ||
      heading_conf < HEADING_CONFIDENCE_MIN_FOR_WIND)
    return;
  // ✓ v5.1: Skip wind estimation during turns. Wind = ground_vel - airspeed_vec assumes
  // straight flight. During turns, heading changes faster than ground track (parafoil inertia),
  // producing transient wind errors that contaminate layers.
  if (fabsf(measured_turn_rate) > 0.15f) return;

  float brake = normalizeBrake(cmdL_current, cmdR_current);
  float rho_f = airDensityFactor((float)fused_alt_msl);
  double airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake) * rho_f;
  double heading_used = getHeadingRad();
  double airspeed_north  = airspeed * cos(heading_used);
  double airspeed_east   = airspeed * sin(heading_used);
  double ground_vel_north = ground_speed * cos(ground_track_rad);
  double ground_vel_east  = ground_speed * sin(ground_track_rad);

  double wind_north_measured = ground_vel_north - airspeed_north;
  double wind_east_measured  = ground_vel_east  - airspeed_east;

  // Reject outliers
  double delta_n = wind_north_measured - wind_north;
  double delta_e = wind_east_measured  - wind_east;
  if (fabs(delta_n) > MAX_WIND_UPDATE_DELTA || fabs(delta_e) > MAX_WIND_UPDATE_DELTA) {
    wind_update_rejects++;
    return;
  }

  // Update global wind
  // ✓ FIXED: First measurement sets wind directly for fast initialization
  static bool wind_first_sample = true;
  if (wind_first_sample) {
    wind_north = wind_north_measured;
    wind_east  = wind_east_measured;
    wind_first_sample = false;
  } else {
    wind_north = wind_north * (1.0 - WIND_FILTER_ALPHA) + wind_north_measured * WIND_FILTER_ALPHA;
    wind_east  = wind_east  * (1.0 - WIND_FILTER_ALPHA) + wind_east_measured  * WIND_FILTER_ALPHA;
  }

  // M1 FIX: Decay BEFORE update — apply decay to old values before blending new observation.
  // Previous order (update→decay) would immediately decay freshly-written data.
  static uint32_t last_decay_ms = 0;
  uint32_t now_decay = millis();
  if (last_decay_ms > 0) {
    float dt_s = (now_decay - last_decay_ms) / 1000.0f;
    if (dt_s > 0.001f && dt_s < 2.0f) {
      float decay = expf(-WIND_LAYER_DECAY_PER_S * dt_s);
      for (int i = 0; i < WIND_LAYER_COUNT; i++) {
        if (wind_layer_ms[i] > 0) {
          wind_layer_north[i] *= decay;
          wind_layer_east[i]  *= decay;
        }
      }
    }
  }
  last_decay_ms = now_decay;

  // Update wind layer — fast alpha for single-pass convergence
  int idx = windLayerIndex(height_agl);
  if (wind_layer_ms[idx] == 0) {
    // First measurement for this layer: initialize directly (no alpha lag)
    wind_layer_north[idx] = wind_north_measured;
    wind_layer_east[idx]  = wind_east_measured;
  } else if (fabsf((float)wind_north_measured - wind_layer_north[idx]) <= MAX_WIND_LAYER_DELTA &&
             fabsf((float)wind_east_measured  - wind_layer_east[idx])  <= MAX_WIND_LAYER_DELTA) {
    wind_layer_north[idx] = wind_layer_north[idx] * (1.0f - WIND_LAYER_FILTER_ALPHA) +
                            (float)wind_north_measured * WIND_LAYER_FILTER_ALPHA;
    wind_layer_east[idx]  = wind_layer_east[idx]  * (1.0f - WIND_LAYER_FILTER_ALPHA) +
                            (float)wind_east_measured  * WIND_LAYER_FILTER_ALPHA;
  } else {
    wind_layer_rejects++;
  }
  wind_layer_ms[idx] = millis();
  smoothWindLayers();
  updateLowestObservedLayerCache();  // M5 FIX

  // NaN protection on wind estimate
  if (!isFiniteD(wind_north) || !isFiniteD(wind_east)) {
    wind_north = 0.0;
    wind_east = 0.0;
  }
  // Clamp
  wind_speed = sqrt(wind_north * wind_north + wind_east * wind_east);
  if (wind_speed > MAX_WIND_ESTIMATE) {
    double scale = MAX_WIND_ESTIMATE / wind_speed;
    wind_north *= scale;
    wind_east  *= scale;
    wind_speed  = MAX_WIND_ESTIMATE;
  }
  updateGustDetector();
}

// =====================================================================
//  ASCENT WIND PROFILING
// =====================================================================
// During rocket ascent and free-fall after ejection, the CanSat has no
// aerodynamic control. Its horizontal GNSS velocity ≈ wind velocity.
// Pre-fill wind layers so guidance has a wind profile from the first second.
static void estimateAscentWind() {
  // Only run before guidance starts
  if (state != WAIT_FOR_DROP && state != WAIT_FOR_STABLE_DESCENT) return;
  if (gnss_fix_type < 3 || !ground_track_valid) return;
  if (gnss_h_accuracy_mm > 3000) return;  // need decent fix quality

  double height_agl = fused_alt_msl - target_alt_msl;
  if (height_agl < MIN_WIND_UPDATE_ALT_M) return;

  // During ascent/free-fall, assume zero airspeed: wind = GNSS ground velocity
  double wind_n = ground_speed * cos(ground_track_rad);
  double wind_e = ground_speed * sin(ground_track_rad);

  // Reject unreasonable values
  double wmag = sqrt(wind_n * wind_n + wind_e * wind_e);
  if (wmag > MAX_WIND_ESTIMATE) return;

  // Fill the wind layer
  int idx = windLayerIndex(height_agl);
  if (wind_layer_ms[idx] == 0) {
    // First measurement: initialize directly
    wind_layer_north[idx] = wind_n;
    wind_layer_east[idx]  = wind_e;
  } else {
    // Slow alpha — ascent data is noisier than descent (rocket vibration, tumble)
    const float ASCENT_ALPHA = 0.10f;
    wind_layer_north[idx] = wind_layer_north[idx] * (1.0f - ASCENT_ALPHA) + (float)wind_n * ASCENT_ALPHA;
    wind_layer_east[idx]  = wind_layer_east[idx]  * (1.0f - ASCENT_ALPHA) + (float)wind_e * ASCENT_ALPHA;
  }
  wind_layer_ms[idx] = millis();

  // Also seed the global wind estimate
  static bool ascent_wind_initialized = false;
  if (!ascent_wind_initialized) {
    wind_north = wind_n;
    wind_east  = wind_e;
    wind_speed = wmag;
    ascent_wind_initialized = true;
  } else {
    wind_north = wind_north * 0.95 + wind_n * 0.05;
    wind_east  = wind_east  * 0.95 + wind_e * 0.05;
    wind_speed = sqrt(wind_north * wind_north + wind_east * wind_east);
  }
}

// =====================================================================
//  ALTITUDE FUSION (✓ FIXED: proper complementary filter)
// =====================================================================
// Predict with baro change (high-frequency), correct toward GNSS (low-frequency).
// This eliminates baro drift while keeping baro's fast response.
static void updateAltitudeFusion() {
  static double last_baro_for_fusion = 0.0;
  static bool fusion_initialized = false;
  static double last_fused = 0.0;
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  float dt = 0.05f;
  if (last_ms > 0) {
    dt = clampf((now - last_ms) / 1000.0f, 0.001f, 0.5f);
  }
  last_ms = now;

  if (!fusion_initialized) {
    fused_alt_msl = (gnss_fix_type >= 3) ? gnss_alt_msl : baro_alt_msl;
    last_baro_for_fusion = baro_alt_msl;
    last_fused = fused_alt_msl;
    fusion_initialized = true;
    return;
  }

  // Predict: integrate baro change (tracks fast altitude changes with zero lag)
  double baro_change = baro_alt_msl - last_baro_for_fusion;
  last_baro_for_fusion = baro_alt_msl;
  fused_alt_msl += baro_change;

  // Correct: slowly pull toward GNSS to remove baro drift
  if (gnss_fix_type >= 3) {
    float alpha;
    if (gnss_carrier_solution >= 1 && gnss_v_accuracy_mm < 150) {
      alpha = 0.20f;  // ✓ FIXED: RTK float/fixed: fast correction, trust GNSS vertical
    } else {
      float acc_m = gnss_v_accuracy_mm * 0.001f;
      alpha = clampf(0.05f / max(acc_m, 0.5f), 0.005f, 0.10f);
    }
    double gnss_error = gnss_alt_msl - fused_alt_msl;
    fused_alt_msl += alpha * gnss_error;
  }
  // No GNSS: coast on baro (baro_change already applied)

  // M2 FIX: Blend EKF altitude in over ~1 second to prevent step discontinuity
  // when EKF health toggles (could trigger premature FLARE or skip it)
  static float ekf_blend = 0.0f;
  float ekf_blend_target = (ekf9.initialized && ekf9.healthy) ? 1.0f : 0.0f;
  float blend_rate = 1.0f;  // full transition in ~1s at 25Hz
  // FIX IMP-3: Slower EKF blend decay below 10m AGL — halves worst-case altitude
  // step rate near ground where FLARE timing is critical. Found by ekf-auditor.
  double height_agl_blend = fused_alt_msl - target_alt_msl;
  if (ekf_blend_target < ekf_blend && height_agl_blend < 10.0 && targetReceived)
    blend_rate = 0.5f;
  if (ekf_blend_target > ekf_blend)
    ekf_blend = fminf(ekf_blend + blend_rate * dt, 1.0f);
  else
    ekf_blend = fmaxf(ekf_blend - blend_rate * dt, 0.0f);

  if (ekf_blend > 0.01f) {
    double ekf_alt = origin_alt_msl - ekf9.x[EKF9_D_IDX];
    double pre_blend_alt = fused_alt_msl;  // ✓ FIXED: save before blend to avoid velocity transient
    fused_alt_msl = fused_alt_msl * (1.0 - ekf_blend) + ekf_alt * ekf_blend;
    fused_alt_velocity = -ekf9.x[EKF9_VD_IDX] * ekf_blend +
                         ((pre_blend_alt - last_fused) / fmaxf(dt, 0.01f)) * (1.0f - ekf_blend);
  } else if (dt > 0.01) {
    fused_alt_velocity = (fused_alt_msl - last_fused) / dt;
  }
  last_fused = fused_alt_msl;
}

// =====================================================================
//  LANDING PREDICTION (BUG #1 FIX: no wind double-counting)
// =====================================================================
static void predictLandingWithPhysics() {
  double height_agl = fused_alt_msl - target_alt_msl;
  if (height_agl < 0.5) {
    getEstimatedLatLon(pred_lat, pred_lon);
    predicted_error_m = distanceMeters(lat, lon, target_lat, target_lon);
    return;
  }
  double remaining = height_agl;
  double sim_lat = 0.0, sim_lon = 0.0;
  getEstimatedLatLon(sim_lat, sim_lon);
  double heading_used = getHeadingRad();

  // Estimate current turn rate from servo differential for heading propagation
  float brake = normalizeBrake(cmdL_current, cmdR_current);
  float diff_cmd = cmdR_current - cmdL_current;  // positive = right turn
  float bank_rad = diff_cmd * 0.5f * MAX_BANK_ANGLE_RAD;
  float rho_f0 = airDensityFactor((float)fused_alt_msl);
  float airspeed0 = brakeToSinkRate(brake) * brakeToGlideRatio(brake) * rho_f0;
  float current_turn_rate = turnRateFromBank(bank_rad, airspeed0) *
                            brakeToTurnRateScale(brake);
  current_turn_rate = clampf(current_turn_rate, -MAX_TURN_RATE_RAD_S, MAX_TURN_RATE_RAD_S);
  // Propagate turn for first 10s (after that, assume heading converges)
  double turn_budget_s = 10.0;

  while (remaining > 0.5) {
    int idx = windLayerIndex(remaining);
    double layer_step = windLayerStepSize(idx);
    double step = (remaining > layer_step) ? layer_step : remaining;
    // Air density correction at simulated altitude
    float sim_alt_msl = (float)(target_alt_msl + remaining);
    float rho_f = airDensityFactor(sim_alt_msl);
    double sink = brakeToSinkRate(brake) * rho_f;
    if (sink < 0.1) break;  // guard against divide-by-zero at extreme altitude
    double glide = brakeToGlideRatio(brake);
    double airspeed = sink * glide;
    double dt = step / sink;

    // Propagate heading using current turn rate while budget remains
    if (turn_budget_s > 0.0) {
      double turn_dt = fmin(dt, turn_budget_s);
      heading_used = wrapAngle(heading_used + current_turn_rate * turn_dt);
      turn_budget_s -= turn_dt;
    }

    double airspeed_north = airspeed * cos(heading_used);
    double airspeed_east  = airspeed * sin(heading_used);

    // Use layer wind with log-law extrapolation for unvisited ground layers
    double layer_wind_n, layer_wind_e;
    getWindAtLayer(idx, layer_wind_n, layer_wind_e);

    double ground_vel_n = airspeed_north + layer_wind_n;
    double ground_vel_e = airspeed_east  + layer_wind_e;
    sim_lat += (ground_vel_n * dt) / metersPerDegLat(sim_lat);
    sim_lon += (ground_vel_e * dt) / metersPerDegLon(sim_lat);
    remaining -= step;
  }
  pred_lat = sim_lat;
  pred_lon = sim_lon;
  predicted_error_m = distanceMeters(pred_lat, pred_lon, target_lat, target_lon);
}

// =====================================================================
//  FINAL APPROACH LINE (FAL) — TRAJECTORY PLANNING
// =====================================================================
// The FAL is the into-wind line passing through the target.
// Above 50m: predictive/MPC guidance positions the vehicle upwind of target.
// Below 50m: the FAL controller tracks this line inbound for precision.

static void updateApproachPlan() {
  double height_agl = fused_alt_msl - target_alt_msl;

  // Freeze FAL direction below transition altitude for stability
  if (approach_plan.valid && height_agl < FAL_FREEZE_HEIGHT_M) return;

  // Rate-limit updates
  if (approach_plan.valid && millis() - approach_plan.last_update_ms < FAL_UPDATE_INTERVAL_MS) return;

  double w_n = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WN_IDX] : wind_north;
  double w_e = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WE_IDX] : wind_east;
  double w_mag = sqrt(w_n * w_n + w_e * w_e);

  if (w_mag < FAL_MIN_WIND_MPS) {
    approach_plan.valid = false;
    return;
  }

  // Upwind unit vector (into the wind)
  approach_plan.upwind_n = -w_n / w_mag;
  approach_plan.upwind_e = -w_e / w_mag;

  // FAL direction: from upwind toward target (downwind)
  approach_plan.fal_dn = -approach_plan.upwind_n;
  approach_plan.fal_de = -approach_plan.upwind_e;

  approach_plan.fal_bearing_rad = atan2(approach_plan.fal_de, approach_plan.fal_dn);

  approach_plan.valid = true;
  approach_plan.last_update_ms = millis();
}

// Compute cross-track and along-track error relative to FAL line through target.
// cross_track: positive = vehicle is right of FAL (looking toward target)
// along_track: positive = vehicle is upwind of target (still approaching)
static void getFALErrors(double veh_lat, double veh_lon,
                          double &cross_track_m, double &along_track_m) {
  if (!approach_plan.valid) {
    cross_track_m = 0.0;
    double dn = (target_lat - veh_lat) * metersPerDegLat(target_lat);
    double de = (target_lon - veh_lon) * metersPerDegLon(target_lat);
    along_track_m = sqrt(dn * dn + de * de);
    return;
  }
  // Vector from target to vehicle
  double vn = (veh_lat - target_lat) * metersPerDegLat(target_lat);
  double ve = (veh_lon - target_lon) * metersPerDegLon(target_lat);

  // Cross-track: 2D cross product of FAL direction and vehicle vector
  // Positive = vehicle is to the right of FAL when looking toward target
  cross_track_m = approach_plan.fal_dn * ve - approach_plan.fal_de * vn;

  // Along-track: negative projection (positive = vehicle is before target)
  along_track_m = -(vn * approach_plan.fal_dn + ve * approach_plan.fal_de);
}

// =====================================================================
//  TURN RATE FEEDBACK (inner rate loop)
// =====================================================================
// Damps actual turn rate toward the commanded turn rate.
// Suppresses parafoil yaw oscillations and improves heading stability.
static float applyTurnRateFeedback(float turn_cmd) {
  if (!heading_valid || !isHeadingFresh()) return turn_cmd;
  // ✓ v5.1: Compensate for estimated servo gain loss (backlash, partial failure)
  float gain_comp = (servo_gain_estimate > 0.4f) ? (1.0f / servo_gain_estimate) : 2.5f;
  gain_comp = clampf(gain_comp, 1.0f, 2.5f);  // only amplify, never reduce
  float desired_rate = turn_cmd * MAX_TURN_RATE_RAD_S;
  float rate_error = measured_turn_rate - desired_rate;
  turn_cmd -= TURN_RATE_FB_GAIN * rate_error / MAX_TURN_RATE_RAD_S;
  turn_cmd *= gain_comp;
  return clampf(turn_cmd, -1.0f, 1.0f);
}

// =====================================================================
//  SERVO FAILURE DETECTION
// =====================================================================
// Compares commanded differential to measured turn rate. If the parafoil
// consistently does not respond to commands, flag a potential servo failure.
static void checkServoFailure() {
  if (!heading_valid || !isHeadingFresh()) return;
  if (state < GUIDED_DESCENT || state > TERMINAL_HOMING) return;

  float diff = cmdR_current - cmdL_current;
  // Only check when we're actively commanding a turn
  if (fabsf(diff) < SERVO_FAIL_MIN_CMD) {
    servo_fail_count = max(servo_fail_count - 1, 0);
    return;
  }

  // Expected turn direction from servo differential
  float expected_sign = (diff > 0.0f) ? 1.0f : -1.0f;
  float actual_sign   = (measured_turn_rate > 0.0f) ? 1.0f : -1.0f;

  // Check if turn rate magnitude is too low OR wrong direction
  bool mismatch = (fabsf(measured_turn_rate) < SERVO_FAIL_RATE_THRESHOLD &&
                   fabsf(diff) > 0.3f) ||
                  (expected_sign != actual_sign &&
                   fabsf(measured_turn_rate) > SERVO_FAIL_RATE_THRESHOLD);

  if (mismatch) {
    servo_fail_count++;
    if (servo_fail_count >= SERVO_FAIL_WINDOW && !servo_failure_flag) {
      servo_failure_flag = true;
      Serial.println("WARNING: Possible servo failure detected!");
    }
  } else {
    servo_fail_count = max(servo_fail_count - 2, 0);
    if (servo_fail_count == 0) servo_failure_flag = false;
  }

  // ✓ v5.1: Estimate proportional servo gain from expected vs actual turn rate.
  // Used to compensate for partial failures (backlash, reduced travel).
  float expected_rate = diff * MAX_TURN_RATE_RAD_S * 0.5f;  // simplified expected rate
  if (fabsf(expected_rate) > 0.05f) {
    float gain_sample = fabsf(measured_turn_rate) / fabsf(expected_rate);
    gain_sample = clampf(gain_sample, 0.2f, 2.0f);  // bound to reasonable range
    servo_gain_estimate += 0.02f * (gain_sample - servo_gain_estimate);  // slow EMA
    servo_gain_estimate = clampf(servo_gain_estimate, 0.3f, 1.5f);
  }
}

// =====================================================================
//  CANOPY TIME CONSTANT IDENTIFICATION (v5.3)
// =====================================================================
// During GUIDED_DESCENT above 100m, measure canopy response time by tracking
// how quickly measured_turn_rate responds to command changes. Updates the
// MPC servo_alpha for more accurate trajectory prediction.
static void identifyCanopyTimeConstant(double height_agl) {
  if (state != GUIDED_DESCENT || height_agl < 100.0f) return;
  if (!heading_valid || !isHeadingFresh()) return;

  float diff = cmdR_current - cmdL_current;

  if (!canopy_tau_measuring) {
    // Look for significant command change to start measurement
    static float prev_diff_tau = 0.0f;
    float cmd_delta = fabsf(diff - prev_diff_tau);
    prev_diff_tau = diff;
    if (cmd_delta > 0.3f) {
      canopy_tau_measuring = true;
      canopy_tau_start_ms = millis();
      canopy_tau_rate_start = measured_turn_rate;
      canopy_tau_expected = diff * MAX_TURN_RATE_RAD_S * 0.5f;  // expected steady-state rate
      canopy_tau_cmd_start = diff;
    }
  } else {
    uint32_t elapsed = millis() - canopy_tau_start_ms;
    // Abort if command changed again (not a clean step response)
    if (fabsf(diff - canopy_tau_cmd_start) > 0.15f) {
      canopy_tau_measuring = false;
      return;
    }
    // Check if we've reached 63% of the expected change (= 1 time constant)
    float rate_change = measured_turn_rate - canopy_tau_rate_start;
    float expected_change = canopy_tau_expected - canopy_tau_rate_start;
    if (fabsf(expected_change) < 0.05f) {
      canopy_tau_measuring = false;
      return;
    }
    float fraction = rate_change / expected_change;
    if (fraction >= 0.63f && elapsed > 50) {
      // Reached 63% — elapsed time is the time constant
      float tau_sample = elapsed / 1000.0f;
      tau_sample = clampf(tau_sample, 0.10f, 2.0f);  // bound to reasonable range
      canopy_tau_estimate += 0.3f * (tau_sample - canopy_tau_estimate);
      canopy_tau_estimate = clampf(canopy_tau_estimate, 0.10f, 2.0f);
      Serial.printf("CANOPY TAU: sample=%.2fs, estimate=%.2fs\n", tau_sample, canopy_tau_estimate);
      canopy_tau_measuring = false;
    }
    // Timeout: abort after 3s (response should be well within this)
    if (elapsed > 3000) {
      canopy_tau_measuring = false;
    }
  }
}

// =====================================================================
//  FAL GUIDANCE (cross-track + heading + feed-forward + rate loop)
// =====================================================================
// Tracks the into-wind Final Approach Line for precision terminal guidance.
// Used by FINAL_APPROACH and TERMINAL_HOMING. Returns true if active.
static bool computeFALGuidance(double height_agl) {
  if (!approach_plan.valid || !origin_set || !ekf9.initialized || !ekf9.healthy)
    return false;

  double veh_lat, veh_lon;
  ekf9GetLatLon(veh_lat, veh_lon);

  // FAL errors
  double cross_track_m, along_track_m;
  getFALErrors(veh_lat, veh_lon, cross_track_m, along_track_m);

  // Direct distance for telemetry
  double vn = (veh_lat - target_lat) * metersPerDegLat(target_lat);
  double ve = (veh_lon - target_lon) * metersPerDegLon(target_lat);
  predicted_error_m = sqrt(vn * vn + ve * ve);

  // ✓ v5.3: Pre-compensate for drift during flare using ground-layer wind.
  // Uses layer 0 (0-5m AGL) to account for wind shear near the surface.
  if (height_agl < FLARE_COMP_HEIGHT_M) {
    // Use ground-layer wind (0-5m AGL) for drift compensation, not current-altitude
    // EKF velocity — wind shear near ground is common (e.g. 3 m/s at 15m → 1 m/s at surface).
    double comp_wn, comp_we;
    getWindAtLayer(0, comp_wn, comp_we);
    float vn_comp = (float)comp_wn;
    float ve_comp = (float)comp_we;
    // Crosswind velocity component relative to FAL (positive = moving right of FAL)
    float cw = (float)approach_plan.fal_dn * ve_comp - (float)approach_plan.fal_de * vn_comp;
    // Bias: shift target against drift direction.
    float taper = 1.0f - clampf((float)height_agl / FLARE_COMP_HEIGHT_M, 0.0f, 1.0f);
    cross_track_m += cw * FLARE_COMP_TIME_S * taper;
    // Along-track: positive aw = moving toward target → will overshoot → fly shorter approach.
    float aw = (float)approach_plan.fal_dn * vn_comp + (float)approach_plan.fal_de * ve_comp;
    along_track_m -= aw * FLARE_COMP_TIME_S * FLARE_COMP_ALONG_GAIN * taper;
  }

  // Cross-track velocity (positive = moving right of FAL)
  double cross_track_rate = approach_plan.fal_dn * ekf9.x[EKF9_VE_IDX]
                          - approach_plan.fal_de * ekf9.x[EKF9_VN_IDX];

  // Altitude-dependent gains
  float cross_gain = altitudeGain(height_agl,
      CROSSTRACK_GAIN_HIGH_ALT, CROSSTRACK_GAIN_LOW_ALT,
      CROSSTRACK_GAIN_HIGH_ALT_M, CROSSTRACK_GAIN_LOW_ALT_M);

  // Cross-track PD: negative feedback for error and rate (damping)
  float cross_cmd = -KP_CROSSTRACK * cross_gain * (float)cross_track_m
                    - KD_CROSSTRACK * cross_gain * (float)cross_track_rate;
  cross_cmd = clampf(cross_cmd, -MAX_CROSSTRACK_CMD, MAX_CROSSTRACK_CMD);

  // Heading: blend between FAL bearing (far upwind) and direct-to-target (near/past target)
  float along_blend = clampf((float)(along_track_m / 30.0), 0.0f, 1.0f);
  double direct_bearing = bearingRad(veh_lat, veh_lon, target_lat, target_lon);
  double angle_diff = wrapAngle(approach_plan.fal_bearing_rad - direct_bearing);
  double base_heading = wrapAngle(direct_bearing + along_blend * angle_diff);

  // Feed-forward: crab angle to compensate crosswind
  float brake = normalizeBrake(cmdL_current, cmdR_current);
  float airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake) * airDensityFactor((float)fused_alt_msl);
  float wn_est = (float)(ekf9.initialized ? ekf9.x[EKF9_WN_IDX] : wind_north);
  float we_est = (float)(ekf9.initialized ? ekf9.x[EKF9_WE_IDX] : wind_east);
  float crosswind = (float)(approach_plan.fal_dn * we_est - approach_plan.fal_de * wn_est);
  float crab_angle = 0.0f;
  if (airspeed > 1.0f) {
    // Negative: if wind pushes right, crab left
    crab_angle = -asinf(clampf(crosswind * getWindFFGain() / airspeed, -0.7f, 0.7f));
  }

  double desired_heading = wrapAngle(base_heading + crab_angle);

  // Heading controller
  float heading_gain = altitudeGain(height_agl,
      HEADING_GAIN_HIGH_ALT, HEADING_GAIN_LOW_ALT,
      HEADING_GAIN_HIGH_ALT_M, HEADING_GAIN_LOW_ALT_M);
  float heading_cmd = computeHeadingControl(desired_heading, heading_gain);

  // Combine heading and cross-track
  float turn_cmd = clampf(heading_cmd + cross_cmd, -1.0f, 1.0f);

  // Turn rate feedback (inner loop)
  turn_cmd = applyTurnRateFeedback(turn_cmd);

  // Glide-slope
  float glide_cmd = applyGlideSlope(height_agl);

  cmdL_target = clampf(-turn_cmd + glide_cmd, 0.0f, 1.0f);
  cmdR_target = clampf( turn_cmd + glide_cmd, 0.0f, 1.0f);
  return true;
}

// =====================================================================
//  MPC GUIDANCE
// =====================================================================
static float computeMPCControl(float glide_cmd_current = 0.0f) {
  uint32_t mpc_start_us = micros();  // ✓ v5.4: MPC timing instrumentation
  float best_cmd = 0.0f;
  float best_cost = 1e9f;
  double current_lat = 0.0, current_lon = 0.0;
  getEstimatedLatLon(current_lat, current_lon);
  float current_heading = (float)getHeadingRad();
  float current_alt = (float)fused_alt_msl;
  float current_cmd = clampf(0.5f * (cmdR_current - cmdL_current), -1.0f, 1.0f);
  // ✓ v5.1: Altitude-proportional horizon — ensures MPC always sees the ground.
  // At 100m: dt=100/(5.5*12)=1.52→clamped to 1.2. At 60m: dt=0.91. At 30m: dt=0.45.
  float height_agl_mpc = current_alt - (float)target_alt_msl;
  float dt = clampf(height_agl_mpc / (NOMINAL_SINK_RATE * (float)MPC_HORIZON), 0.3f, 2.5f);
  // ✓ v5.3: Use online canopy time constant estimate if available
  float servo_alpha = dt / (canopy_tau_estimate + dt);

  // ✓ FIXED: Simulate in NE (meters) space using float trig — eliminates ~16k double
  // trig calls per MPC cycle. At <2km range, flat-earth is sub-cm accurate.
  float start_n = 0.0f, start_e = 0.0f;
  float target_n = 0.0f, target_e = 0.0f;
  if (origin_set) {
    double dn, de;
    latLonToNE(current_lat, current_lon, dn, de);
    start_n = (float)dn; start_e = (float)de;
    latLonToNE(target_lat, target_lon, dn, de);
    target_n = (float)dn; target_e = (float)de;
  } else {
    // Fallback: approximate NE from lat/lon difference
    float m_lat = (float)metersPerDegLat(current_lat);
    float m_lon = (float)metersPerDegLon(current_lat);
    target_n = (float)(target_lat - current_lat) * m_lat;
    target_e = (float)(target_lon - current_lon) * m_lon;
    // start_n/e stay at 0
  }

  // Two-phase MPC: test MPC_CMD_LEVELS^2 = 225 command sequences
  // Phase 1 (first 4 steps): one command. Phase 2 (remaining): another command.
  // This captures turn-then-straighten and other two-phase maneuvers.
  // ✓ v5.2: Increased horizon at low altitude — captures full terminal maneuver.
  // At <100m: 16 steps (33% more computation, Teensy 4.0 handles easily).
  int horizon = (height_agl_mpc > 150.0f) ? 20 : (height_agl_mpc < 100.0f) ? 16 : MPC_HORIZON;
  // ✓ v5.6: Cap horizon to 80% of time-to-impact — prevents MPC from simulating
  // past ground impact where physics model is invalid (no wind below ground).
  if (sink_rate_filtered > 0.5f) {
    float tti_s = height_agl_mpc / (float)sink_rate_filtered;
    int max_steps = (int)(0.8f * tti_s / dt);
    if (max_steps < 4) max_steps = 4;  // minimum useful horizon
    if (horizon > max_steps) horizon = max_steps;
  }
  // ✓ v5.1: Non-linear command spacing — denser near zero for precision,
  // sparser at extremes. Pre-compute once outside inner loop.
  // L6: t^1.5 curve gives ~3x resolution near zero vs extremes — critical for
  // precision landing where small differential inputs dominate final corrections.
  static float mpc_cmd_table[MPC_CMD_LEVELS];
  static bool mpc_cmd_table_init = false;
  if (!mpc_cmd_table_init) {
    const int half = MPC_CMD_LEVELS / 2;  // 7
    mpc_cmd_table[half] = 0.0f;
    for (int k = 1; k <= half; k++) {
      float t = (float)k / (float)half;  // 0..1
      float v = t * sqrtf(t);            // power-1.5 spacing: dense near zero
      mpc_cmd_table[half + k] = v;
      mpc_cmd_table[half - k] = -v;
    }
    mpc_cmd_table_init = true;
  }
  for (int p1 = 0; p1 < MPC_CMD_LEVELS; p1++) {
    // H1 FIX: Timeout guard — abort MPC if exceeding 30ms to protect 40ms loop budget
    if (micros() - mpc_start_us > 30000) {
      Serial.println("MPC: TIMEOUT — partial result used");
      break;
    }
    // ✓ v6.2: MPC outer-loop RTK service. 15 iterations × ~1.5-2 ms per iter means
    // without this the LoRa radio can go 20-30 ms without a drain, causing the
    // seqfix RTK stream (100 ms cadence) to miss packets during guided descent.
    serviceRTKHotPath();
    float cmd1 = mpc_cmd_table[p1];
    for (int p2 = 0; p2 < MPC_CMD_LEVELS; p2++) {
      float cmd2 = mpc_cmd_table[p2];

      float sim_n = start_n, sim_e = start_e;
      float sim_heading = current_heading;
      float sim_alt = current_alt;
      float cmd_state = current_cmd;

      for (int step = 0; step < horizon; step++) {
        float cmd_target = (step < MPC_PHASE1_STEPS) ? cmd1 : cmd2;
        cmd_state += servo_alpha * (cmd_target - cmd_state);
        // ✓ FIXED: Model physical [0,1] brake range — negative = slack, no effect
        // ✓ AUDIT FIX: Include glide-slope symmetric brake in MPC model
        float sim_L = fmaxf(-cmd_state + glide_cmd_current, 0.0f);
        float sim_R = fmaxf( cmd_state + glide_cmd_current, 0.0f);
        float brake_sim = 0.5f * (sim_L + sim_R);  // average brake level
        float glide_sim = brakeToGlideRatio(brake_sim);
        float rho_sim = airDensityFactor(sim_alt);
        float sink_sim  = brakeToSinkRate(brake_sim) * rho_sim;
        float airspeed  = sink_sim * glide_sim;
        float diff = sim_R - sim_L;  // turn differential
        float bank = diff * 0.5f * MAX_BANK_ANGLE_RAD;
        float turn_rate = turnRateFromBank(bank, airspeed) * brakeToTurnRateScale(brake_sim);
        turn_rate = clampf(turn_rate, -MAX_TURN_RATE_RAD_S, MAX_TURN_RATE_RAD_S);
        sim_heading = wrapAngle(sim_heading + turn_rate * dt);

        float sim_height_agl = sim_alt - (float)target_alt_msl;
        int layer_idx = windLayerIndex((double)sim_height_agl);
        double sim_wind_n, sim_wind_e;
        getWindAtLayer(layer_idx, sim_wind_n, sim_wind_e);

        // ✓ FIXED: cosf/sinf (hardware FPU) instead of cos/sin (software FP64)
        sim_n += (airspeed * cosf(sim_heading) + (float)sim_wind_n) * dt;
        sim_e += (airspeed * sinf(sim_heading) + (float)sim_wind_e) * dt;
        sim_alt -= sink_sim * dt;
        if (sim_alt < (float)target_alt_msl) break;
      }
      // ✓ FIXED: flat-earth Euclidean distance instead of haversine (~100x faster)
      float dn = sim_n - target_n;
      float de = sim_e - target_e;
      float final_error = sqrtf(dn * dn + de * de);
      // ✓ v5.2: Heading alignment penalty — encourage arriving aligned with target or into-wind.
      // Literature shows MPC without heading cost produces trajectories that arrive
      // with wrong heading, requiring terminal corrections that can't be completed in time.
      float heading_to_target = atan2f(-de, -dn);  // bearing FROM sim position TO target
      float arrival_align = fabsf(wrapAngle(sim_heading - heading_to_target));
      // Also penalize not being aligned into the wind (preferred landing direction)
      float into_wind_heading_cost = 0.0f;
      {
        float sim_height_for_wind = fmaxf(sim_alt - (float)target_alt_msl, 0.0f);
        int wind_idx = windLayerIndex((double)sim_height_for_wind);
        double wn_cost, we_cost;
        getWindAtLayer(wind_idx, wn_cost, we_cost);
        float wind_mag = sqrtf((float)(wn_cost * wn_cost + we_cost * we_cost));
        if (wind_mag > 1.0f) {
          float into_wind = atan2f(-(float)we_cost, -(float)wn_cost);
          into_wind_heading_cost = fabsf(wrapAngle(sim_heading - into_wind));
        }
      }
      float heading_cost = 0.3f * fminf(arrival_align, arrival_align * 0.5f + into_wind_heading_cost * 0.5f);
      // ✓ v5.3: FAL handoff quality cost — penalize cross-track error from the FAL at handoff altitude
      float fal_handoff_cost = 0.0f;
      if (approach_plan.valid) {
        float sim_dn = sim_n - target_n;
        float sim_de = sim_e - target_e;
        float fal_cross = fabsf((float)approach_plan.fal_dn * sim_de - (float)approach_plan.fal_de * sim_dn);
        fal_handoff_cost = 0.4f * fal_cross;
      }
      float total_cost = final_error + heading_cost + fal_handoff_cost
                       + (fabsf(cmd1) + fabsf(cmd2)) * 0.5f * MPC_CONTROL_WEIGHT  // ✓ v5.0: penalize both phases
                       + fabsf(cmd1 - cmd2) * MPC_SMOOTH_WEIGHT;
      if (total_cost < best_cost) {
        best_cost = total_cost;
        best_cmd  = cmd1;  // return the phase-1 command (what to do NOW)
      }
    }
  }
  // ✓ v5.4: Log MPC computation time — verify within 40ms loop budget
  uint32_t mpc_us = micros() - mpc_start_us;
  static uint32_t mpc_max_us = 0;
  if (mpc_us > mpc_max_us) {
    mpc_max_us = mpc_us;
    Serial.printf("MPC: new max %lu us\n", mpc_us);
  }
  return best_cmd;
}

// =====================================================================
//  HEADING CONTROLLER
// =====================================================================
static void resetHeadingController() {
  heading_error_integral   = 0.0;
  last_heading_error       = 0.0;
  last_heading_measurement = 0.0f;  // H3 FIX
  heading_d_filtered       = 0.0f;
  last_heading_update_ms   = 0;
}
static void resetGuidanceIntegrators() {
  resetHeadingController();
  glide_integral = 0.0f;
}

static float computeHeadingControl(double desired_bearing, float gain_multiplier) {
  // ✓ FIX W7: Cache heading once — previous code called getHeadingRad() twice, risking
  // inconsistency if a new BNO085 reading arrives between the P-term and D-term calls.
  double current_heading_d = getHeadingRad();
  double heading_error = wrapAngle(desired_bearing - current_heading_d);
  uint32_t now = millis();
  float dt = 0.02f;
  if (last_heading_update_ms > 0) {
    dt = (now - last_heading_update_ms) / 1000.0f;
    dt = clampf(dt, 0.005f, 0.2f);
  }
  last_heading_update_ms = now;

  heading_error_integral += heading_error * dt;
  heading_error_integral = clampf(heading_error_integral, -2.5f, 2.5f);
  // ✓ v5.6: Conditional integral reset — clear when error crosses zero while saturated.
  // Prevents hunting after sustained crosswind correction.
  if (fabsf(heading_error_integral) >= 2.4f &&
      ((heading_error > 0.0 && last_heading_error < 0.0) ||
       (heading_error < 0.0 && last_heading_error > 0.0))) {
    heading_error_integral = 0.0;
  }
  // H3 FIX: Differentiate measurement (heading), not error.
  // Prevents D-term kick when MPC changes target heading suddenly.
  float current_hdg = (float)current_heading_d;
  float raw_rate = (float)(wrapAngle(current_hdg - last_heading_measurement) / dt);
  // Low-pass filter on D-term to suppress magnetometer noise spikes
  heading_d_filtered += HEADING_D_FILTER_ALPHA * (raw_rate - heading_d_filtered);
  float heading_error_rate = heading_d_filtered;
  last_heading_measurement = current_hdg;
  last_heading_error = heading_error;

  float wind_factor = 1.0f;
  if (wind_speed > WIND_GAIN_REDUCTION_START) {
    float t = (float)((wind_speed - WIND_GAIN_REDUCTION_START) /
                      (MAX_WIND_FOR_GUIDANCE - WIND_GAIN_REDUCTION_START));
    wind_factor = 1.0f - clampf(t, 0.0f, 1.0f) * (1.0f - WIND_GAIN_MIN_FACTOR);
  }
  float gust_factor = gust_active ? GUST_GAIN_FACTOR : 1.0f;
  float kp = KP_HEADING * gain_multiplier * wind_factor * gust_factor;
  float ki = KI_HEADING * gain_multiplier * wind_factor * gust_factor;
  float kd = KD_HEADING * gain_multiplier * wind_factor * gust_factor;

  // FIX CRIT-1: D-term sign correction — was +kd, must be -kd to damp (not amplify) heading rate.
  // Found by heading-auditor, verified by mpc-auditor and red-team.
  return clampf(kp * heading_error + ki * heading_error_integral
                - kd * heading_error_rate, -1.0f, 1.0f);
}

// =====================================================================
//  PREDICTIVE GUIDANCE
// =====================================================================
static void runPredictiveGuidance() {
  estimateWind();
  predictLandingWithPhysics();
  double height_agl = fused_alt_msl - target_alt_msl;
  if (height_agl < 0.5) {
    cmdL_target = 0.0f;
    cmdR_target = 0.0f;
    return;
  }

  // ✓ v5.1: Unreachable target detection. If predicted error is growing for >5s
  // AND wind exceeds 70% of airspeed, the target is likely unreachable.
  // Switch to cross-wind heading to minimize closest-approach distance.
  float current_airspeed = brakeToSinkRate(normalizeBrake(cmdL_current, cmdR_current))
                         * brakeToGlideRatio(normalizeBrake(cmdL_current, cmdR_current))
                         * airDensityFactor((float)fused_alt_msl);
  if (predicted_error_m > prev_predicted_error_m + 0.5) {
    error_growing_count++;
  } else {
    error_growing_count = max(error_growing_count - 2, 0);
  }
  prev_predicted_error_m = predicted_error_m;
  target_unreachable = (error_growing_count > 75 &&   // ~3s at 25Hz
                        wind_speed > 0.7 * current_airspeed &&
                        predicted_error_m > 30.0);

  // Steer toward target (FAL handles into-wind approach at lower altitudes)
  double desired_bearing = bearingRad(pred_lat, pred_lon, target_lat, target_lon);
  // If target unreachable, fly perpendicular to wind for closest approach
  if (target_unreachable) {
    double wind_bearing = atan2(wind_east, wind_north);
    // Choose the cross-wind direction that moves us toward the target
    double cross1 = wrapAngle((float)(wind_bearing + M_PI / 2.0));
    double cross2 = wrapAngle((float)(wind_bearing - M_PI / 2.0));
    double err1 = fabs(wrapAngle((float)(cross1 - desired_bearing)));
    double err2 = fabs(wrapAngle((float)(cross2 - desired_bearing)));
    desired_bearing = (err1 < err2) ? cross1 : cross2;
  }
  float altitude_gain_val = altitudeGain(height_agl,
                                         HEADING_GAIN_HIGH_ALT, HEADING_GAIN_LOW_ALT,
                                         HEADING_GAIN_HIGH_ALT_M, HEADING_GAIN_LOW_ALT_M);
  // ✓ AUDIT FIX: Compute glide_cmd BEFORE MPC so simulation includes symmetric brake.
  // applyGlideSlope has internal state — call once, pass value to MPC.
  float glide_cmd = applyGlideSlope(height_agl);
  float turn_cmd;
  if (height_agl > 60.0 && state == GUIDED_DESCENT && !target_unreachable) {
    // ✓ FIX C1: MPC already computes optimal commands via forward simulation —
    // post-scaling by altitude_gain_val corrupts the optimization result.
    // altitude_gain_val is for PID reactive control only, not MPC predictive control.
    turn_cmd = computeMPCControl(glide_cmd);
  } else {
    float gain = (state == FINAL_APPROACH) ? APPROACH_GAIN : 1.0f;
    turn_cmd = computeHeadingControl(desired_bearing, gain * altitude_gain_val);
  }

  // Lateral velocity damping — damp lateral AIRSPEED, not ground velocity.
  // ✓ FIX: Previous code used ground velocity, which includes wind drift. In crosswind,
  // the constant drift component fought the crab angle computed by MPC/heading controller,
  // adding 1-3m lateral error. Subtracting EKF wind gives lateral airspeed (slip), which
  // is the oscillation we actually want to damp.
  if (ekf9.initialized) {
    double heading = getHeadingRad();
    double air_n = ekf9.x[EKF9_VN_IDX] - ekf9.x[EKF9_WN_IDX];
    double air_e = ekf9.x[EKF9_VE_IDX] - ekf9.x[EKF9_WE_IDX];
    double vel_lat = -air_n * sin(heading) + air_e * cos(heading);
    turn_cmd -= (float)(LATERAL_DAMPING_GAIN * vel_lat);
  }
  turn_cmd = clampf(turn_cmd, -1.0f, 1.0f);

  // Turn rate feedback (inner loop) — damps yaw oscillations
  turn_cmd = applyTurnRateFeedback(turn_cmd);

  cmdL_target = clampf(-turn_cmd + glide_cmd, 0.0f, 1.0f);
  cmdR_target = clampf( turn_cmd + glide_cmd, 0.0f, 1.0f);
}

// =====================================================================
//  NMEA FALLBACK PARSER (ported from v6.0 — backup when UBX handshake fails)
// =====================================================================
static bool nmeaChecksumOk(const char *s) {
  if (!s || s[0] != '$') return false;
  const char *star = strchr(s, '*');
  if (!star || (star - s) < 1) return false;
  uint8_t cs = 0;
  for (const char *p = s + 1; p < star; p++) cs ^= (uint8_t)(*p);
  if (!isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2])) return false;
  auto hexv = [](char c) -> uint8_t {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    c = (char)toupper((unsigned char)c);
    return (uint8_t)(10 + c - 'A');
  };
  uint8_t rx = (uint8_t)((hexv(star[1]) << 4) | hexv(star[2]));
  return cs == rx;
}

static double nmeaToDegrees(const char *v, char hemi, bool is_lat) {
  if (!v || !*v) return 0.0;
  double x = atof(v);
  if (x <= 0.0) return 0.0;
  int deg = (int)(x / 100.0);
  double minutes = x - (double)deg * 100.0;
  double out = (double)deg + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') out = -out;
  return out;
}

static void applyNMEALine(const char *line) {
  if (!line || line[0] != '$') return;
  if (!nmeaChecksumOk(line)) return;

  static char buf[128];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char *star = strchr(buf, '*');
  if (star) *star = '\0';

  static char *tok[24];
  int n = 0;
  char *save = nullptr;
  char *p = strtok_r(buf, ",", &save);
  while (p && n < 24) {
    tok[n++] = p;
    p = strtok_r(nullptr, ",", &save);
  }
  if (n < 2) return;

  // $GNRMC / $GPRMC
  if (strstr(tok[0], "RMC") != nullptr && n >= 7) {
    if (tok[2] && tok[2][0] == 'A') {
      if (tok[3] && tok[4] && tok[5] && tok[6]) {
        double new_lat = nmeaToDegrees(tok[3], tok[4][0], true);
        double new_lon = nmeaToDegrees(tok[5], tok[6][0], false);
        if (fabs(new_lat) > 0.0001 && fabs(new_lon) > 0.0001) {
          lat = new_lat;
          lon = new_lon;
        }
      }
      if (tok[7] && *tok[7]) {
        ground_speed = atof(tok[7]) * 0.514444;  // knots -> m/s
      }
      // ✓ v6.1: Extract course over ground from RMC field 8 (degrees true)
      // Enables ground_track_valid in NMEA-only mode for wind estimation and EKF velocity
      if (n >= 9 && tok[8] && *tok[8] && ground_speed >= MIN_GROUND_SPEED_FOR_TRACK) {
        double cog_deg = atof(tok[8]);
        if (cog_deg >= 0.0 && cog_deg <= 360.0) {
          ground_track_rad = cog_deg * DEG_TO_RAD;
          ground_track_valid = true;
        }
      }
    }
  }

  // $GNGGA / $GPGGA
  if (strstr(tok[0], "GGA") != nullptr && n >= 10) {
    int fixq = (tok[6] && *tok[6]) ? atoi(tok[6]) : 0;
    int sats = (tok[7] && *tok[7]) ? atoi(tok[7]) : 0;

    if (fixq >= 1 && tok[2] && tok[3] && tok[4] && tok[5]) {
      double new_lat = nmeaToDegrees(tok[2], tok[3][0], true);
      double new_lon = nmeaToDegrees(tok[4], tok[5][0], false);
      if (fabs(new_lat) > 0.0001 && fabs(new_lon) > 0.0001) {
        lat = new_lat;
        lon = new_lon;
      }
    }

    if (tok[9] && *tok[9]) {
      gnss_alt_msl = atof(tok[9]);
    }

    gnss_satellites = (uint8_t)clampf((float)sats, 0.0f, 255.0f);

    if (fixq <= 0) {
      gnss_fix_type = 0;
      gnss_carrier_solution = 0;
    } else {
      gnss_fix_type = 3;  // treat SPS/DGPS/RTK as 3D fix
      // ✓ v6.1 FIX: Extract RTK status from GGA fix quality field.
      // GGA fixq: 0=invalid, 1=GPS SPS, 2=DGPS, 4=RTK fixed, 5=RTK float
      // Without this, firmware never detects RTK even when RTCM corrections are working.
      if (fixq == 4) {
        gnss_carrier_solution = 2;  // RTK fixed
      } else if (fixq == 5) {
        gnss_carrier_solution = 1;  // RTK float
      } else {
        gnss_carrier_solution = 0;  // standalone or DGPS
      }
    }

    // Accuracy estimates based on fix quality
    if (gnss_carrier_solution == 2) {
      gnss_h_accuracy_mm = 20;    // RTK fixed: ~20mm typical
      gnss_v_accuracy_mm = 40;    // RTK fixed: ~40mm typical
    } else if (gnss_carrier_solution == 1) {
      gnss_h_accuracy_mm = 500;   // RTK float: ~0.5m typical
      gnss_v_accuracy_mm = 1000;  // RTK float: ~1m typical
    } else {
      gnss_h_accuracy_mm = 10000; // Standalone: ~10m typical
      gnss_v_accuracy_mm = 10000;
    }

    nmea_last_update_ms = millis();
    nmea_seen = true;
  }
}

static void pollNMEAFromSerial5() {
  while (Serial5.available()) {
    char c = (char)Serial5.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (nmea_len > 6) {
        nmea_line[nmea_len] = '\0';
        applyNMEALine(nmea_line);
      }
      nmea_len = 0;
      continue;
    }
    if (nmea_len < sizeof(nmea_line) - 1) {
      nmea_line[nmea_len++] = c;
    } else {
      nmea_len = 0;  // overflow guard
    }
  }
}

// =====================================================================
//  GNSS SETUP (v6.1 architecture: NMEA-primary with UBX opportunistic)
// =====================================================================
// ZED-X20P: running at 38400 baud, UBX binary mode.
// Blind mode — we attach the library without checking the handshake result.
static void setupGNSS() {
  // ✓ v6.2: Enlarged Serial5 RX buffer to 4096 B for RTCM bursts from LoRa relay.
  // The Cansat_tests/GroundStation_transport_seqfix GS sends 120-byte RTCM payloads
  // every 100 ms; the u-blox takes ~30 ms per RTCM frame to parse. Burst headroom
  // matters when the RTK queue drains in 512-byte chunks via serviceRTKToGNSS().
  static uint8_t gnss_rx_buf[4096];
  Serial5.addMemoryForRead(gnss_rx_buf, sizeof(gnss_rx_buf));
  // ✓ v6.2: TX-side memory so Serial5.availableForWrite() returns meaningful chunks
  // for the chunked RTK drain. Without this the drain stalls at ~63-byte chunks.
  static uint8_t gnss_tx_buf[2048];
  Serial5.addMemoryForWrite(gnss_tx_buf, sizeof(gnss_tx_buf));

  // ✓ v5.9: Use 38400 baud throughout — matches ZED-X20P default after battery loss.
  // No baud probing or reconfiguration needed at 38400.
  Serial5.begin(38400);
  delay(100);
  kickWatchdog();
  bool gnss_found = gnss.begin(Serial5);
  kickWatchdog();
  if (gnss_found) {
    Serial.println("GNSS: UBX connection established at 38400");
  } else {
    Serial.println("GNSS: No UBX response at 38400 — entering NMEA stream fallback");
    gnss_stream_fallback = true;
  }

  // Configure module (only if UBX handshake succeeded)
  if (!gnss_stream_fallback) {
    gnss.setNavigationFrequency(25);
    kickWatchdog();
    gnss.setAutoPVT(true);
    kickWatchdog();
    gnss.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);
    kickWatchdog();
    gnss.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_RTCM3);
    kickWatchdog();
    gnss.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);
    kickWatchdog();
    gnss.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    kickWatchdog();
    Serial.println("GNSS: UBX mode active (38400, 25Hz, Airborne4G)");
  }
}

static bool isGNSSQualityGood() {
  if (gnss_fix_type < 3) return false;
  if (gnss_satellites < 6) return false;
  if (millis() - last_gnss_update > 2000) return false;
  if (gnss_carrier_solution >= 1) {
    // ✓ FIXED: RTK detected via carrSoln, not fixType
    if (gnss_h_accuracy_mm > 100) return false;
    if (gnss_v_accuracy_mm > 150) return false;
  }
  return true;
}

static float getDynamicCaptureRadius() {
  if (gnss_carrier_solution >= 1 && gnss_h_accuracy_mm < 100) return CAPTURE_RADIUS_M;  // ✓ FIXED
  if (gnss_fix_type >= 3 && gnss_satellites >= 8) return CAPTURE_RADIUS_GNSS_M;
  return CAPTURE_RADIUS_POOR_M;
}

static void updateGNSS() {
  // v6.0: non-blocking fallback path. If UBX begin/handshake failed, avoid
  // SparkFun UBX polling in loop (can stall ~2s) and use NMEA-only updates.
  if (gnss_stream_fallback) {
    // NMEA-only mode: poll NMEA sentences, RTK detected from GGA fix quality.
    // No UBX retry — if receiver is NMEA-only, retries will never succeed.
    pollNMEAFromSerial5();
    if (nmea_seen && (millis() - nmea_last_update_ms) < MAX_GNSS_AGE_MS) {
      last_gnss_update = millis();
      // ✓ v6.1: ground_track_valid now set by NMEA RMC parser (course over ground)
      // gnss_carrier_solution preserved from NMEA GGA parser (fixq 4/5 = RTK)
      setOriginIfNeeded();

      // RTK status monitoring — track best achieved carrier solution
      if (gnss_carrier_solution > rtk_best_fix_type) {
        rtk_best_fix_type = gnss_carrier_solution;
        Serial.printf("RTK: New best carrier solution: %d (hAcc=%lu mm)\n",
                      gnss_carrier_solution, gnss_h_accuracy_mm);
      }
      if (gnss_h_accuracy_mm < rtk_best_hacc_mm) {
        rtk_best_hacc_mm = gnss_h_accuracy_mm;
      }

      // 9-state EKF: init if ready, then GNSS measurement update
      ekf9Init();
      if (ekf9.initialized) {
        ekf9UpdateGNSS();
        ekf9UpdateModel();
        ekf9Healthy();
      }

      // QNH calibration from GNSS altitude
      if (!sea_level_calibrated && gnss_fix_type >= 3 && pressure_hPa > 800.0 &&
          gnss_v_accuracy_mm < 2000) {
        qnh_accum += pressure_hPa / pow(1.0 - gnss_alt_msl / 44330.0, 5.255);
        qnh_sample_count++;
        if (qnh_sample_count >= 10) {
          sea_level_pressure_hpa = qnh_accum / qnh_sample_count;
          sea_level_calibrated = true;
          Serial.printf("Baro calibrated: QNH = %.1f hPa (10-sample avg)\n",
                        sea_level_pressure_hpa);
        }
      }
    } else if (millis() - last_gnss_update > MAX_GNSS_AGE_MS) {
      ground_track_valid = false;
    }
    return;
  }

  // Always poll raw NMEA stream as fallback path
  pollNMEAFromSerial5();

  // ✓ v6.2: drain RTK queue before/after blocking UBX parse so RTCM keeps flowing.
  serviceRTKHotPath();
  gnss.checkUblox();
  serviceRTKHotPath();
  if (gnss.getPVT()) {
    double new_lat = gnss.getLatitude() * 1e-7;
    double new_lon = gnss.getLongitude() * 1e-7;
    double new_alt = gnss.getAltitudeMSL() / 1000.0;
    double new_gs  = gnss.getGroundSpeed() / 1000.0;
    double new_vv  = -gnss.getNedDownVel() / 1000.0;  // getNedDownVel() is positive-DOWN; negate for positive-UP
    // NaN protection: skip update if GNSS returns garbage
    if (!isFiniteD(new_lat) || !isFiniteD(new_lon) || !isFiniteD(new_alt) ||
        !isFiniteD(new_gs) || !isFiniteD(new_vv)) {
      return;
    }
    lat = new_lat;
    lon = new_lon;
    gnss_alt_msl = new_alt;
    ground_speed = new_gs;
    if (ground_speed >= MIN_GROUND_SPEED_FOR_TRACK) {
      // ✓ v5.4: Gate by headAcc — reject headMot when GNSS heading accuracy is poor
      uint32_t head_acc_raw = gnss.getHeadingAccEst();  // deg × 1e-5
      float head_acc_deg = head_acc_raw * 1e-5f;
      if (head_acc_raw == 0) {
        // headAcc unavailable — only trust headMot at higher speed (±30-60° error at 2 m/s)
        ground_track_valid = (ground_speed >= 4.0f);
      } else {
        ground_track_valid = (head_acc_deg < 15.0f);
      }
      if (ground_track_valid) {
        ground_track_rad = gnss.getHeading() * 1e-5 * DEG_TO_RAD;
      }
    } else {
      ground_track_valid = false;
    }
    gnss_vertical_velocity = new_vv;
    gnss_fix_type          = gnss.getFixType();
    gnss_carrier_solution  = gnss.getCarrierSolutionType();  // ✓ FIXED: 0=none,1=float,2=fixed
    gnss_satellites        = gnss.getSIV();
    // L9: getHorizontalAccuracy()/getVerticalAccuracy() are v2.x compatible
    // (SparkFun_u-blox_GNSS_Arduino_Library). Do NOT replace with v3.x-only methods.
    gnss_h_accuracy_mm     = gnss.getHorizontalAccuracy();
    gnss_v_accuracy_mm     = gnss.getVerticalAccuracy();
    last_gnss_update       = millis();

    // RTK status monitoring — track best achieved carrier solution
    if (gnss_carrier_solution > rtk_best_fix_type) {
      rtk_best_fix_type = gnss_carrier_solution;
      Serial.printf("RTK: New best carrier solution: %d (hAcc=%lu mm)\n",
                    gnss_carrier_solution, gnss_h_accuracy_mm);
    }
    if (gnss_h_accuracy_mm < rtk_best_hacc_mm) {
      rtk_best_hacc_mm = gnss_h_accuracy_mm;
    }

    setOriginIfNeeded();

    // 9-state EKF: init if ready, then GNSS measurement update
    // (predict runs every loop in loop(), not here)
    ekf9Init();
    if (ekf9.initialized) {
      ekf9UpdateGNSS();
      ekf9UpdateModel();
      ekf9Healthy();
    }

    // BUG #8 FIX: Average 5 GNSS samples for QNH calibration
    // ✓ FIXED: Require reasonable vertical accuracy to prevent 10m+ altitude bias
    // ✓ v5.1: Tightened QNH calibration: 2m accuracy threshold (was 5m), 10 samples (was 5)
    if (!sea_level_calibrated && gnss_fix_type >= 3 && pressure_hPa > 800.0 &&
        gnss_v_accuracy_mm < 2000) {  // <2m vertical accuracy required
      qnh_accum += pressure_hPa / pow(1.0 - gnss_alt_msl / 44330.0, 5.255);
      qnh_sample_count++;
      if (qnh_sample_count >= 10) {
        sea_level_pressure_hpa = qnh_accum / qnh_sample_count;
        sea_level_calibrated = true;
        Serial.printf("Baro calibrated: QNH = %.1f hPa (10-sample avg)\n",
                      sea_level_pressure_hpa);
      }
    }
  } else {
    // UBX PVT not available this cycle. If NMEA fallback is active and fresh,
    // keep GNSS data alive from NMEA fields.
    if (nmea_seen && (millis() - nmea_last_update_ms) < MAX_GNSS_AGE_MS) {
      last_gnss_update = millis();
      ground_track_valid = false;  // no reliable heading in minimal NMEA fallback
      // ✓ v6.1: gnss_carrier_solution preserved from NMEA GGA parser (fixq 4/5 = RTK)
      setOriginIfNeeded();
    } else if (millis() - last_gnss_update > MAX_GNSS_AGE_MS) {
      ground_track_valid = false;
    }
  }
}

// RTK status monitoring
static uint32_t rtcm_packets_received = 0;
static uint32_t rtcm_bytes_received   = 0;
static uint32_t last_rtcm_rx_ms       = 0;

// =====================================================================
//  RTK HOT-PATH QUEUE (ported from Cansat_tests/CanSat_transport_final)
// =====================================================================
// Previously feedRTKToGNSS() wrote RTCM directly to Serial5 inline inside
// pollLoRa(), which blocked the LoRa RX loop during UART TX and starved
// subsequent RTK packets during MPC/I2C/SD flushes. The test firmware uses
// an 8 KB ring buffer drained in 512-byte chunks by serviceRTKToGNSS(),
// with serviceRTKHotPath() invoked at ~20 points around blocking sections.
static constexpr size_t RTK_QUEUE_BYTES       = 8192;
static constexpr size_t RTCM_UART_CHUNK_BYTES = 512;
static uint8_t           rtk_queue[RTK_QUEUE_BYTES];
static volatile size_t   rtk_q_head = 0;
static volatile size_t   rtk_q_tail = 0;
static volatile size_t   rtk_q_len  = 0;
static uint32_t          rtk_queue_drop_bytes  = 0;
static uint32_t          rtk_uart_bytes_out    = 0;
static uint32_t          rtk_uart_short_writes = 0;

// RTK sequence tracking (from transport GS [0x02][seq_hi][seq_lo][RTCM][CRC])
static uint16_t rtk_expected_seq    = 0;
static bool     rtk_seq_initialized = false;
static uint32_t rtk_seq_gaps        = 0;
static uint32_t rtk_seq_missed_total = 0;
static uint32_t rtk_crc_fail_count  = 0;

// GS heartbeat ping counters (4-byte packets from GroundStation_transport_seqfix,
// disambiguated from 7-byte GroundWindPacket by length — both share msgType 0x04)
static uint32_t gs_ping_rx_count      = 0;
static uint8_t  gs_ping_last_rx_telem = 0;

// Enqueue RTCM bytes into the ring buffer; drops excess on overflow.
static void enqueueRTK(const uint8_t *data, size_t len) {
  if (len == 0) return;
  rtcm_packets_received++;
  rtcm_bytes_received += (uint32_t)len;
  last_rtcm_rx_ms = millis();
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

// Drain a bounded chunk from the RTK queue to the GNSS UART. Non-blocking —
// respects Serial5.availableForWrite() so it never stalls the caller.
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

// =====================================================================
//  BMP280 (BUG #4 FIX: no raw sink rate clamping, BUG #11 FIX: GNSS fallback)
// =====================================================================
static void setupBMP280() {
  if (!bmp.begin(0x76)) {
    Serial.println("ERROR: BMP280 not found!");
    bmp_operational = false;  // M13 FIX: explicit flag for flight logic
    return;
  }
  bmp_operational = true;  // M13 FIX
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
  last_baro_ms  = millis();
  last_baro_alt = 0.0;
  Serial.println("BMP280 initialized");
}

static void updateBMP280() {
  // ✓ v5.5 AUDIT FIX: Removed SDA stuck-low pre-check. digitalRead(SDA) during
  // normal I2C traffic reads LOW ~50% of the time (data/ACK bits), causing false
  // skips of valid BMP280 reads. Wire.setTimeout(200ms) handles actual bus hangs,
  // and checkI2CRecovery() handles persistent stuck-bus conditions.
  // ✓ v6.2: drain RTK queue around blocking I2C reads so LoRa RX isn't starved.
  serviceRTKHotPath();
  float raw_pressure = bmp.readPressure() / 100.0;
  float raw_temp     = bmp.readTemperature();
  serviceRTKHotPath();

  // Detect I2C failure: pressure=0 or NaN means bus hung or sensor dead
  if (raw_pressure < 300.0f || raw_pressure > 1200.0f ||
      isnan(raw_pressure) || isnan(raw_temp)) {
    bmp_fail_count++;
    return;  // skip this update, use stale data
  }
  bmp_fail_count = 0;
  last_bmp_success_ms = millis();
  pressure_hPa = raw_pressure;
  temp_c       = raw_temp;

  // BUG #11 FIX: Use GNSS altitude if QNH not yet calibrated
  if (sea_level_calibrated) {
    baro_alt_msl = 44330.0 * (1.0 - pow(pressure_hPa / sea_level_pressure_hpa, 1.0 / 5.255));
  } else if (gnss_fix_type >= 3) {
    baro_alt_msl = gnss_alt_msl;
  } else {
    baro_alt_msl = bmp.readAltitude(1013.25);
  }

  uint32_t now = millis();
  uint32_t dt_ms = now - last_baro_ms;
  if (dt_ms >= BARO_UPDATE_INTERVAL_MS) {
    double dt_sec = dt_ms / 1000.0;
    // BUG #4 FIX: Do NOT clamp raw sink rate — let the filter handle it
    sink_rate_raw = (last_baro_alt - baro_alt_msl) / dt_sec;
    sink_rate_filtered = sink_rate_filtered * (1.0 - SINK_FILTER_ALPHA) +
                         sink_rate_raw * SINK_FILTER_ALPHA;
    // ✓ FIXED: Allow small negative values (updrafts) so glide slope integral
    // doesn't saturate. Landing detection (< 0.3) still works with negative values.
    sink_rate_filtered = clampd(sink_rate_filtered, -2.0, SINK_RATE_MAX);
    last_baro_alt = baro_alt_msl;
    last_baro_ms  = now;
  }

  if (gnss_fix_type >= 3 && fabs(gnss_vertical_velocity) < 20.0) {
    sink_rate_gnss = -gnss_vertical_velocity;
  }
  updateAltitudeFusion();

  // 9-state EKF: barometric altitude update
  if (ekf9.initialized) {
    ekf9UpdateBaro();
  }
}

// =====================================================================
//  BNO085 + AUTO HEADING CALIBRATION (BUG #9)
// =====================================================================
static void setupBNO085() {
  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("ERROR: BNO085 not found!");
    bno_operational = false;  // M13 FIX: explicit flag for flight logic
    return;
  }
  bno_operational = true;  // M13 FIX
  // ✓ v5.4: Primary heading from GAME_ROTATION_VECTOR (gyro+accel, NO magnetometer).
  // Servo motors at 3-8cm produce 10-50µT EMI that corrupts magnetometer heading by ±5-8°
  // during turns. GAME_ROTATION_VECTOR is immune to magnetic interference.
  // Drift (~1°/min) is corrected by slow blending from ROTATION_VECTOR during straight flight.
  // ✓ v5.4.1 FIX: enableReport() interval parameter is in MICROSECONDS, not milliseconds.
  // FIX HIGH-4: Verify enableReport() return values — I2C glitch can silently
  // prevent report activation. Found by sensor-auditor-v2.
  bool r1 = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);  // 50 Hz — primary heading
  bool r2 = bno08x.enableReport(SH2_ROTATION_VECTOR, 200000);       // 5 Hz — mag drift correction
  bool r3 = bno08x.enableReport(SH2_ACCELEROMETER, 50000);           // 20 Hz
  bool r4 = bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);    // 50 Hz — yaw rate
  if (!r1 || !r3 || !r4) {
    // Critical reports failed — mark sensor as non-operational
    Serial.printf("BNO085: enableReport FAILED (game=%d rot=%d accel=%d gyro=%d)\n", r1, r2, r3, r4);
    bno_operational = false;
    return;
  }
  if (!r2) {
    Serial.println("BNO085: WARNING — ROTATION_VECTOR failed (mag drift correction disabled)");
    // Non-fatal: GAME_ROTATION_VECTOR is primary, mag correction is supplementary
  }
  Serial.println("BNO085 initialized (GAME_ROTATION + MAG_DRIFT_CORR + GYRO + ACCEL)");
}

static void updateAutoHeadingCalibration() {
  if (auto_heading_done) return;
  if (ground_speed < AUTO_HEADING_MIN_SPEED) return;
  if (!ground_track_valid || !heading_valid) return;
  if (fabs(cmdL_current - cmdR_current) > AUTO_HEADING_MAX_BRAKE_DIFF) return;
  // ✓ AUDIT FIX: Skip when wind estimate is unreliable — crab angle uses wind data.
  if (wind_speed < 0.1 && !ekf9.initialized) return;

  // ✓ v5.1: Subtract expected wind crab angle. In crosswind, ground_track = heading + crab.
  // Without correction, crab angle (~15° at 3 m/s crosswind) corrupts the heading offset.
  double delta = wrapAngle(ground_track_rad - heading_rad);
  // Estimate and subtract crab angle using current wind estimate
  double wn = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WN_IDX] : wind_north;
  double we = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WE_IDX] : wind_east;
  float brake = normalizeBrake(cmdL_current, cmdR_current);
  float rho_f = airDensityFactor((float)fused_alt_msl);
  double airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake) * rho_f;
  if (airspeed > 2.0) {
    // Crosswind component perpendicular to current heading
    double crosswind = -wn * sin(heading_rad) + we * cos(heading_rad);
    double crab_angle = asin(clampf((float)(crosswind / airspeed), -0.7f, 0.7f));
    delta -= crab_angle;
  }
  auto_heading_accum += delta;
  auto_heading_count++;

  if (auto_heading_count >= AUTO_HEADING_MIN_SAMPLES) {
    float correction = (float)(auto_heading_accum / auto_heading_count);
    correction = clampf(correction, -AUTO_HEADING_MAX_CORRECTION,
                        AUTO_HEADING_MAX_CORRECTION);
    heading_offset_runtime = correction;
    auto_heading_done = true;
    Serial.printf("Auto-heading offset: %.3f rad (%.1f deg)\n",
                  correction, correction * RAD_TO_DEG);
  }
}

// ✓ v5.2: Continuous slow heading correction below 100m.
// After initial auto-cal, allow very slow drift correction during guided descent.
// Only during straight flight (low turn rate), good GNSS, with crab angle subtraction.
static void updateContinuousHeadingCorrection() {
  if (!auto_heading_done) return;  // wait for initial cal
  if (ground_speed < 3.0f) return;
  if (!ground_track_valid || !heading_valid) return;
  if (fabs(cmdL_current - cmdR_current) > 0.05f) return;  // not turning

  // Compute delta = ground_track - heading, then subtract crab angle
  double delta = wrapAngle(ground_track_rad - heading_rad);
  double wn = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WN_IDX] : wind_north;
  double we = (ekf9.initialized && ekf9.healthy) ? ekf9.x[EKF9_WE_IDX] : wind_east;
  float brake = normalizeBrake(cmdL_current, cmdR_current);
  float rho_f = airDensityFactor((float)fused_alt_msl);
  double airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake) * rho_f;
  if (airspeed > 2.0) {
    double crosswind = -wn * sin(heading_rad) + we * cos(heading_rad);
    double crab_angle = asin(clampf((float)(crosswind / airspeed), -0.7f, 0.7f));
    delta -= crab_angle;
  }
  // Only apply small corrections to avoid large jumps
  float correction = (float)delta;
  if (fabsf(correction) < 0.3f) {
    heading_offset_runtime += 0.005f * correction;  // very slow alpha
    heading_offset_runtime = clampf(heading_offset_runtime,
                                    -AUTO_HEADING_MAX_CORRECTION,
                                     AUTO_HEADING_MAX_CORRECTION);
  }
}

static void updateIMU() {
  // ✓ v5.5 AUDIT FIX: Removed SDA stuck-low pre-check. digitalRead(SDA) during
  // normal I2C traffic reads LOW ~50% of the time (data/ACK bits), causing false
  // skips of valid IMU updates. Wire.setTimeout(200ms) handles actual bus hangs,
  // and checkI2CRecovery() handles persistent stuck-bus conditions.
  bool got_event = false;
  int event_count = 0;
  // ✓ v6.2: drain RTK queue before the BNO085 event loop begins.
  serviceRTKHotPath();
  while (bno08x.getSensorEvent(&bnoValue) && event_count < 50) {
    event_count++;
    got_event = true;
    // ✓ v6.2: every 5 events (~25 ms at typical rates), service RTK so the LoRa
    // radio isn't starved while we're hammering the shared I2C bus.
    if ((event_count % 5) == 0) serviceRTKHotPath();
    if (bnoValue.sensorId == SH2_ACCELEROMETER) {
      float ax = bnoValue.un.accelerometer.x;
      float ay = bnoValue.un.accelerometer.y;
      float az = bnoValue.un.accelerometer.z;
      // NaN protection on accelerometer
      if (isFiniteF(ax) && isFiniteF(ay) && isFiniteF(az)) {
        imu_ax = ax; imu_ay = ay; imu_az = az;
        imu_accel_valid = true;
        last_imu_ms = millis();
      }
    }
    // ✓ v5.1: Gyroscope for direct yaw rate — much lower latency than heading-derived rate
    if (bnoValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      float gz = bnoValue.un.gyroscope.z;  // rad/s, body-frame yaw rate
      if (isFiniteF(gz)) {
        // Low-pass filter to match heading-derived rate smoothness
        measured_turn_rate += TURN_RATE_FILTER_ALPHA * (gz - measured_turn_rate);
      }
    }
    // ✓ v5.4: PRIMARY heading from SH2_GAME_ROTATION_VECTOR (gyro+accel, NO mag).
    // Immune to servo motor EMI that corrupts magnetometer heading by ±5-8° during turns.
    if (bnoValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = bnoValue.un.gameRotationVector.real;
      float qx = bnoValue.un.gameRotationVector.i;
      float qy = bnoValue.un.gameRotationVector.j;
      float qz = bnoValue.un.gameRotationVector.k;
      // NaN protection on quaternion
      if (!isFiniteF(qw) || !isFiniteF(qx) || !isFiniteF(qy) || !isFiniteF(qz)) continue;
      // Store full quaternion for body-to-NED rotation (used by EKF9)
      imu_qw = qw; imu_qx = qx; imu_qy = qy; imu_qz = qz;
      imu_quat_valid = true;
      last_quat_ms = millis();
      float siny_cosp = 2.0f * (qw * qz + qx * qy);
      float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
      // GAME_ROTATION_VECTOR: no magnetometer → MAGNETIC_DECLINATION_RAD NOT applied here.
      // Declination is only for ROTATION_VECTOR (mag-fused heading, line ~3218).
      // Heading reference comes from initial alignment + gyro integration.
      // Apply static calibration offset and runtime auto-correction only.
      // ✓ v5.5 AUDIT NOTE: GAME_ROTATION_VECTOR has no magnetic reference, so
      // MAGNETIC_DECLINATION_RAD is NOT applied here. HEADING_OFFSET_RAD corrects
      // only for sensor mounting offset. If HEADING_OFFSET_RAD was calibrated
      // using the HEADING_CALIBRATION_MODE (which also uses GAME_ROTATION_VECTOR),
      // the offset is self-consistent. If calibrated against a mag-fused reference,
      // it may include a declination component — updateAutoHeadingCalibration()
      // corrects for this in flight via ground-track comparison.
      float new_heading = wrapAngle(atan2f(siny_cosp, cosy_cosp) +
                                    HEADING_OFFSET_RAD + heading_offset_runtime);

      uint32_t now = millis();
      float delta = heading_valid ? wrapAngle(new_heading - heading_rad) : 0.0f;
      if (heading_valid && (now - last_heading_ms) < HEADING_JUMP_MAX_DT_MS) {
        if (fabs(delta) > HEADING_JUMP_MAX_RAD) continue;
      }
      // Compute dt BEFORE updating last_heading_ms (otherwise dt is always 0)
      uint32_t dt_ms = (last_heading_ms > 0) ? (now - last_heading_ms) : 0;

      heading_rad   = new_heading;
      heading_valid = true;
      last_heading_ms  = now;
      last_heading_rad = heading_rad;

      float delta_factor = 1.0f;
      if (HEADING_CONFIDENCE_MAX_DELTA_RAD > 0.0f)
        delta_factor = 1.0f - clampf(fabs(delta) / HEADING_CONFIDENCE_MAX_DELTA_RAD, 0.0f, 1.0f);
      float dt_factor = 1.0f;
      if (HEADING_CONFIDENCE_MAX_DT_MS > 0)
        dt_factor = 1.0f - clampf((float)dt_ms / (float)HEADING_CONFIDENCE_MAX_DT_MS, 0.0f, 1.0f);
      // GAME_ROTATION_VECTOR doesn't provide accuracy field — use gyro quality as proxy.
      // Confidence based on stability (small delta) and freshness (small dt) only.
      // ✓ AUDIT FIX: Cap at 0.85 — GAME_ROTATION_VECTOR has no accuracy field,
      // gyro drift (~1 deg/min) can accumulate while confidence appears perfect.
      heading_confidence = clampf(delta_factor * dt_factor, 0.0f, 0.85f);
    }
    // ✓ v5.4: SECONDARY — SH2_ROTATION_VECTOR (mag-fused) at 5Hz for slow drift correction.
    // During straight flight (low turn rate → low servo EMI), slowly pull heading toward
    // the mag-referenced north. This corrects gyro drift (~1°/min) without corrupting
    // heading during turns when servo EMI is strongest.
    if (bnoValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = bnoValue.un.rotationVector.real;
      float qx = bnoValue.un.rotationVector.i;
      float qy = bnoValue.un.rotationVector.j;
      float qz = bnoValue.un.rotationVector.k;
      float bno_accuracy_rad = bnoValue.un.rotationVector.accuracy;
      if (!isFiniteF(qw) || !isFiniteF(qx) || !isFiniteF(qy) || !isFiniteF(qz)) continue;
      float siny_cosp = 2.0f * (qw * qz + qx * qy);
      float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
      // ✓ v5.5 AUDIT NOTE: ROTATION_VECTOR includes magnetometer → apply declination.
      // Both MAGNETIC_DECLINATION_RAD and HEADING_OFFSET_RAD are applied so that
      // the mag-fused heading aligns with the GAME_ROTATION_VECTOR primary heading.
      float mag_heading = wrapAngle(atan2f(siny_cosp, cosy_cosp) +
                                    MAGNETIC_DECLINATION_RAD + HEADING_OFFSET_RAD +
                                    heading_offset_runtime);
      // Only blend in during straight flight (servos near-neutral → low EMI)
      // and when mag accuracy is good (low bno_accuracy_rad)
      bool straight_flight = fabsf(measured_turn_rate) < 0.05f;
      bool mag_good = isFiniteF(bno_accuracy_rad) && bno_accuracy_rad < 0.3f;
      if (heading_valid && straight_flight && mag_good) {
        float correction = wrapAngle(mag_heading - heading_rad);
        // Very slow blend: alpha=0.01 at 5Hz → time constant ~20s
        heading_rad = wrapAngle(heading_rad + 0.01f * correction);
        last_heading_rad = heading_rad;
      }
    }
  }
  if (got_event) {
    bno_fail_count = 0;
    last_bno_success_ms = millis();
  }
}

// =====================================================================
//  LORA
// =====================================================================

// ✓ v5.5 AUDIT FIX: DIO1 interrupt for TX completion — recovers ~51ms RX window.
// FIX M7: Gate ISR on tx_pending to disambiguate TX-done from RX-done on DIO1.
// SX1262 fires DIO1 for both events. Without this gate, a stale tx_done flag
// could be set by an RX event. Found by red-team, verified by protocol-auditor.
static void loraTxDoneISR() {
  if (lora_tx_pending) {
    lora_tx_done_flag = true;
  }
}

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

static void setupLoRa() {
  // Guard against double-init (can happen on warm reboot if watchdog fires during setup)
  static bool lora_init_done = false;
  if (lora_init_done) return;
  lora_init_done = true;

  lora_ok = false;

  Serial.println("  Setting RF switch pins...");
  // ✓ FIXED v5.1: Configure RF switch pins BEFORE radio.begin().
  // The CORE1262-LF module uses an external RF switch controlled by RXEN/TXEN.
  radio.setRfSwitchPins(PIN_LORA_RXEN, PIN_LORA_TXEN);

  Serial.println("  Calling radio.begin()...");
  Serial.flush();
  kickWatchdog();
  int st = radio.begin(LORA_FREQ_MHZ);
  kickWatchdog();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: LoRa init failed: %d\n", st);
    Serial.println("  Check: LoRa module wired? CS=10, DIO1=7, RST=8, BUSY=9");
    Serial.println("  Check: SPI wired? MOSI=11, MISO=12, SCK=13");
    Serial.println("  Check: LoRa antenna connected?");
    return;
  }

  // Give SX1262 time to settle after hardware reset performed by begin()
  delay(50);

  Serial.println("  LoRa radio found, configuring...");
  Serial.flush();

  // ✓ FIXED v5.7: Enable TCXO if CORE1262-LF has DIO3-powered TCXO.
  // Without this, SX1262 falls back to internal RC oscillator (~±15 ppm),
  // degrading RX sensitivity by 10-20 dB — primary cause of 50m range cutoff.
  st = radio.setTCXO(1.6);
  if (st != RADIOLIB_ERR_NONE) {
    // CORE1262-LF may use an external crystal instead of DIO3-powered TCXO.
    // If setTCXO fails, log warning but continue — the crystal is already running.
    Serial.printf("WARN: setTCXO returned %d (module may use external crystal — OK to continue)\n", st);
  }

  // ✓ FIXED v5.7: Enable DC-DC regulator for full +20 dBm TX power.
  // CORE1262-LF has external inductor for DC-DC. Without this call,
  // SX1262 defaults to LDO mode, causing voltage sag at high TX power.
  st = radio.setRegulatorDCDC();
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setRegulatorDCDC failed: %d\n", st); return; }

  // ✓ v6.3 range fix: explicit OCP for +20 dBm — RadioLib default may trip at ~60 mA
  // and silently collapse TX power to ~+5 dBm. +20 dBm PA needs ~140 mA.
  radio.setCurrentLimit(140.0);
  // ✓ v6.3 range fix: +2 dB RX sensitivity at +2 mA cost — strictly helps range.
  radio.setRxBoostedGainMode(true);
  // ✓ v6.3 range fix: band-specific image calibration for 430-440 MHz, recovers
  // 1-3 dB of RX image rejection over the factory full-band calibration.
  radio.calibrateImage(430.0, 440.0);

  kickWatchdog();

  // ✓ FIXED v4.9: Explicit LoRa parameters — MUST match ground station exactly.
  // SF7/BW500/CR4-5 gives ~22ms air time for 50-byte telemetry.
  // Check every return value — a failed config call means the SPI link is broken.
  st = radio.setSpreadingFactor(7);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setSF failed: %d\n", st); return; }

  // BW=500 to match transport ground station (GroundStation_transport_seqfix.ino)
  st = radio.setBandwidth(500.0);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setBW failed: %d\n", st); return; }

  st = radio.setCodingRate(5);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setCR failed: %d\n", st); return; }

  kickWatchdog();

  st = radio.setOutputPower(LORA_TX_DBM);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setPower failed: %d\n", st); return; }

  st = radio.setCRC(true);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setCRC failed: %d\n", st); return; }

  // ✓ v5.5 AUDIT FIX: Explicit sync word + preamble — prevents cross-talk with other
  // SX1262 devices at competition. MUST match ground station exactly.
  st = radio.setSyncWord(0xAE, 0x2B);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setSyncWord failed: %d\n", st); return; }

  // ✓ FIXED v5.7: Preamble=12 to match all ground stations (was 8).
  // Longer preamble improves weak-signal synchronization at distance.
  st = radio.setPreambleLength(12);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: setPreamble failed: %d\n", st); return; }

  // ✓ FIXED v5.8: Register DIO1 handler BEFORE starting RX to prevent race condition.
  // If a packet arrives between startReceive and setDio1Action, the DIO1 interrupt
  // fires with no handler registered and the packet is silently lost.
  radio.setDio1Action(loraTxDoneISR);

  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERROR: startRX failed: %d\n", st); return; }

  lora_ok = true;
  last_lora_activity_ms = millis();  // ✓ v5.9: seed RX recovery watchdog
  Serial.println("LoRa initialized (SF7/BW500/CR4-5, CRC, sync=0xAE2B, preamble=12)");
}

static void pollLoRa() {
  if (!lora_ok) return;
  if (!radio.available()) return;
  static uint8_t buf[255];
  int len = radio.getPacketLength();
  if (len <= 0 || len > (int)sizeof(buf)) {
    safeStartReceive();
    return;
  }
  int err = radio.readData(buf, len);
  if (err == RADIOLIB_ERR_NONE) {
    // ✓ v5.4: LoRa RSSI/SNR monitoring — predict RTK dropout before it happens
    static float lora_rssi_ema = -70.0f;
    static float lora_snr_ema = 10.0f;
    float rssi = radio.getRSSI();
    float snr  = radio.getSNR();
    lora_rssi_ema += 0.3f * (rssi - lora_rssi_ema);
    lora_snr_ema  += 0.3f * (snr  - lora_snr_ema);
    // Flag weak link for EKF R pre-inflation (checked in ekf9UpdateGNSS via global)
    lora_link_weak = (lora_rssi_ema < -110.0f || lora_snr_ema < -5.0f);

    uint8_t type = buf[0];
    if (type == MSG_TARGET && len == (int)sizeof(TargetPacket)) {
      TargetPacket tp;
      memcpy(&tp, buf, sizeof(tp));
      uint16_t crcCalc = crc16_ccitt((uint8_t *)&tp, sizeof(tp) - 2);
      if (crcCalc == tp.crc16) {
        double t_lat = tp.tgt_lat_e7 * 1e-7;
        double t_lon = tp.tgt_lon_e7 * 1e-7;
        double t_alt = tp.tgt_alt_cm / 100.0;
        // H4 FIX: Tighter coordinate validation — reject implausible locations
        if (fabs(t_lat) > 1.0 && fabs(t_lat) < 90.0 &&
            fabs(t_lon) > 1.0 && fabs(t_lon) < 180.0 &&
            t_alt > -500.0 && t_alt < 5000.0) {
          target_lat     = t_lat;
          target_lon     = t_lon;
          target_alt_msl = t_alt;
          targetReceived = true;
          Serial.printf("TARGET via LoRa: %.7f, %.7f, %.1f m\n",
                        target_lat, target_lon, target_alt_msl);
        } else {
          Serial.printf("WARN: Rejected invalid target: %.7f, %.7f, %.1f m\n",
                        t_lat, t_lon, t_alt);
        }
      }
    }
    // ✓ v6.3: Soft CRC16 removed to match Cansat_tests/GroundStation_transport_seqfix.
    // Transport GS sends: [0x02][seq_hi][seq_lo][RTCM payload] — hardware LoRa CRC
    // (setCRC true) handles bit-error detection; soft CRC was double-checking and
    // made every RTK packet fail silently when the GS reverted to the no-CRC format.
    // ✓ v6.2: Enqueue to RTK ring buffer instead of synchronous Serial5.write().
    // Sequence tracking mirrors Cansat_tests/CanSat_transport_final for diagnostics.
    if (type == MSG_RTK && len > 3) {
      uint16_t seq = ((uint16_t)buf[1] << 8) | buf[2];
      if (rtk_seq_initialized && seq != rtk_expected_seq) {
        uint16_t gap = (uint16_t)(seq - rtk_expected_seq);
        if (gap < 1000) {
          rtk_seq_gaps++;
          rtk_seq_missed_total += gap;
        }
      }
      rtk_expected_seq    = (uint16_t)(seq + 1);
      rtk_seq_initialized = true;
      enqueueRTK(&buf[3], (size_t)(len - 3));  // strip [type][seq_hi][seq_lo]
    }
    // ✓ v6.2: 0x04 is dispatched by packet length to serve two GS variants:
    //   len == 4 → heartbeat ping from GroundStation_transport_seqfix
    //              Frame: [0x04][ping_hi][ping_lo][gs_rx_telem_count]
    //   len == 7 → legacy GroundWindPacket (ground wind uplink)
    if (type == MSG_GROUND_WIND && len == 4) {
      gs_ping_rx_count++;
      gs_ping_last_rx_telem = buf[3];
    } else if (type == MSG_GROUND_WIND && len == (int)sizeof(GroundWindPacket)) {
      GroundWindPacket gw;
      memcpy(&gw, buf, sizeof(gw));
      uint16_t crcCalc = crc16_ccitt((uint8_t *)&gw, sizeof(gw) - 2);
      if (crcCalc == gw.crc16) {
        float wn = gw.wind_n_cms / 100.0f;
        float we = gw.wind_e_cms / 100.0f;
        // H7 FIX: Use <= to accept exactly 20.0 m/s (edge case at clamp boundary)
        if (fabsf(wn) <= 20.0f && fabsf(we) <= 20.0f) {
          // Write to layers 0-3 (0-20m AGL, each 5m step)
          uint32_t now = millis();
          for (int i = 0; i < 4 && i < WIND_LAYER_COUNT; i++) {
            wind_layer_north[i] = wn;
            wind_layer_east[i]  = we;
            wind_layer_ms[i]    = now;  // marks as observed — prevents log-law extrapolation
          }
          updateLowestObservedLayerCache();  // M5 FIX
          Serial.printf("GROUND WIND via LoRa: N=%.2f E=%.2f m/s\n", wn, we);
        }
      }
    }
  }
  safeStartReceive();
  last_lora_activity_ms = millis();  // ✓ v5.9: update activity timestamp on RX event
}

// =====================================================================
//  SERVOS
// =====================================================================
static void setServosNormalized(float left, float right) {
  // NaN protection: if either command is NaN/Inf, go neutral (safe default)
  if (!isFiniteF(left) || !isFiniteF(right)) {
    left = 0.0f;
    right = 0.0f;
  }
  // ✓ FIXED: Servo failure response — reduce turn authority to prevent spiral
  // if one servo is stuck. Keeps symmetric (glide slope) brake intact.
  if (servo_failure_flag && !flare_active) {
    float sym  = 0.5f * (left + right);
    float diff = 0.5f * (right - left);
    diff *= 0.5f;  // halve turn authority
    left  = sym - diff;
    right = sym + diff;
  }
  if (flare_active) {
    // Flare bypass: direct command, no filter or rate limit.
    // The transient lift from a STEP brake input is what makes a flare work.
    cmdL_current = clampf(left, 0.0f, 1.0f);
    cmdR_current = clampf(right, 0.0f, 1.0f);
    // Also snap the filter state so transition out of flare is smooth
    cmdL_filtered = cmdL_current;
    cmdR_filtered = cmdR_current;
  } else {
    // Normal: low-pass filter + trim + deadband + rate limit
    // ✓ v5.2: Altitude-adaptive filter — faster response at low altitude for precision
    float current_height_agl = (float)(fused_alt_msl - target_alt_msl);
    float alpha, max_rate;
    if (current_height_agl > 50.0f) {
      alpha = SERVO_FILTER_ALPHA;       // 0.4 — standard tau ~75ms
      max_rate = SERVO_MAX_RATE;         // 2.0/s
    } else if (current_height_agl > 10.0f) {
      // Lerp from responsive to standard between 10-50m
      float t = (current_height_agl - 10.0f) / 40.0f;
      alpha = 0.7f * (1.0f - t) + SERVO_FILTER_ALPHA * t;
      max_rate = 3.5f * (1.0f - t) + SERVO_MAX_RATE * t;
    } else {
      alpha = 0.85f;                     // Near-direct below 10m (~20ms tau)
      max_rate = 3.5f;                   // 3.5/s — faster terminal corrections
    }
    cmdL_filtered += alpha * (left - cmdL_filtered);
    cmdR_filtered += alpha * (right - cmdR_filtered);
    left  = cmdL_filtered;
    right = cmdR_filtered;

    left  += SERVO_TRIM;
    right -= SERVO_TRIM;
    // ✓ FIXED: Clamp to [0,1] — negative brake is slack cable, no physical effect
    left  = clampf(left,  0.0f, 1.0f);
    right = clampf(right, 0.0f, 1.0f);
    if (left  < SERVO_DEADBAND) left  = 0.0f;
    if (right < SERVO_DEADBAND) right = 0.0f;

    uint32_t now = millis();
    if (last_servo_update_ms > 0) {
      float dt = (now - last_servo_update_ms) / 1000.0f;
      float max_change = max_rate * dt;
      float dL = clampf(left  - cmdL_current, -max_change, max_change);
      float dR = clampf(right - cmdR_current, -max_change, max_change);
      cmdL_current += dL;
      cmdR_current += dR;
    } else {
      cmdL_current = left;
      cmdR_current = right;
    }
    // Final safety clamp
    cmdL_current = clampf(cmdL_current, 0.0f, 1.0f);
    cmdR_current = clampf(cmdR_current, 0.0f, 1.0f);
  }
  last_servo_update_ms = millis();

  // ✓ FIXED: LEFT servo is mirror-mounted — invert its direction to match CasNat.
  // RIGHT follows normal direction (higher µs = more brake).
  // LEFT is inverted (higher µs = LESS brake), so subtract to compensate.
  int usL = SERVO_NEUTRAL_US - (int)(cmdL_current * SERVO_RANGE_US);
  int usR = SERVO_NEUTRAL_US + (int)(cmdR_current * SERVO_RANGE_US);
  servoWriteUs(PIN_SERVO_LEFT, usL);
  servoWriteUs(PIN_SERVO_RIGHT, usR);
}

// ✓ v5.5 AUDIT FIX: Anti-spiral protection — detect sustained uncommanded turning.
// Trigger: |measured_turn_rate| > 0.3 rad/s while commanding near-straight for >3s.
// Recovery: neutral servos for 2s, then resume guidance with cleared integrators.
static void checkAntiSpiral() {
  // Only active during guided phases
  if (state < GUIDED_DESCENT || state > TERMINAL_HOMING) {
    spiral_detect_start_ms = 0;
    spiral_recovery_active = false;
    return;
  }
  // If in recovery, enforce neutral until timeout
  if (spiral_recovery_active) {
    if (millis() - spiral_recovery_start_ms < SPIRAL_RECOVERY_MS) {
      cmdL_target = 0.0f;
      cmdR_target = 0.0f;
      return;
    }
    spiral_recovery_active = false;
    Serial.println("ANTI-SPIRAL: Recovery complete, resuming guidance");
  }
  // Detect: high turn rate while commanding near-straight flight
  float cmd_diff = fabsf(cmdL_current - cmdR_current);
  bool commanding_straight = (cmd_diff < 0.1f);
  bool turning_fast = (fabsf(measured_turn_rate) > SPIRAL_RATE_THRESHOLD);
  if (commanding_straight && turning_fast) {
    if (spiral_detect_start_ms == 0) spiral_detect_start_ms = millis();
    if (millis() - spiral_detect_start_ms > SPIRAL_DETECT_MS) {
      Serial.printf("ANTI-SPIRAL: Triggered! rate=%.2f rad/s, entering 2s recovery\n",
                     measured_turn_rate);
      spiral_recovery_active = true;
      spiral_recovery_start_ms = millis();
      spiral_detect_start_ms = 0;
      resetGuidanceIntegrators();
    }
  } else {
    spiral_detect_start_ms = 0;
  }
}

// =====================================================================
//  EVENT DETECTION (BUG #6 FIX: altitude backup, BUG #10 FIX: landing timer)
// =====================================================================
static bool dropDetected() {
  // ✓ FIXED: Track peak altitude so backup triggers on descent from drone altitude
  if (gnss_fix_type >= 3) {
    if (!drop_check_alt_set) {
      drop_check_alt     = fused_alt_msl;
      drop_check_alt_set = true;
    } else if (fused_alt_msl > drop_check_alt) {
      drop_check_alt = fused_alt_msl;  // continuously track peak altitude
    }
  }

  // H2 FIX: Altitude gate — don't trigger drop detection below 100m AGL.
  // Prevents false triggers from ground-level bumps, vehicle handling, or
  // vibration during pre-launch. 100m is well below any expected drop altitude.
  if (drop_check_alt_set) {
    double height_agl = drop_check_alt - target_alt_msl;
    if (height_agl < 100.0) return false;
  }

  // Method 1: Accelerometer free-fall detection (primary)
  // In free-fall, accel magnitude drops near 0 (not 9.81)
  if (imu_accel_valid && (millis() - last_imu_ms) < 500) {
    float aMag = sqrt(imu_ax * imu_ax + imu_ay * imu_ay + imu_az * imu_az);
    if (aMag < 3.0f || fabs(aMag - 9.81f) > DROP_ACCEL_SPIKE) {
      drop_spike_count++;
      if (drop_spike_count >= DROP_DEBOUNCE_COUNT) {
        Serial.println("DROP DETECTED (accel)!");
        return true;
      }
    } else {
      drop_spike_count = 0;
    }
  }

  // Method 2: Altitude change detection (backup for drone drop)
  if (drop_check_alt_set && gnss_fix_type >= 3) {
    double alt_change = drop_check_alt - fused_alt_msl;
    if (alt_change > DROP_ALT_CHANGE_M) {
      Serial.println("DROP DETECTED (altitude)!");
      return true;
    }
  }

  return false;
}

// BUG #5 FIX: Stable descent with timeout
static bool descentStable() {
  if (stable_wait_start_ms == 0) stable_wait_start_ms = millis();
  uint32_t elapsed = millis() - stable_wait_start_ms;

  // Normal threshold
  if (sink_rate_filtered >= STABLE_SINK_MIN && sink_rate_filtered <= STABLE_SINK_MAX) {
    if (stableStartMs == 0) stableStartMs = millis();
    if (millis() - stableStartMs >= STABLE_HOLD_MS) {
      Serial.println("DESCENT STABLE!");
      return true;
    }
  } else {
    stableStartMs = 0;
  }

  // 15s timeout with relaxed threshold
  if (elapsed > STABLE_DESCENT_TIMEOUT_MS) {
    if (sink_rate_filtered >= STABLE_SINK_MIN_RELAXED &&
        sink_rate_filtered <= STABLE_SINK_MAX_RELAXED) {
      Serial.printf("DESCENT STABLE (timeout, relaxed, sink=%.1f)\n", sink_rate_filtered);
      return true;
    }
  }

  // 20s hard timeout — enter guidance regardless
  if (elapsed > STABLE_DESCENT_HARD_MS) {
    Serial.printf("DESCENT STABLE (hard timeout, sink=%.1f)\n", sink_rate_filtered);
    // ✓ v5.1: Clear wind profile on hard timeout ONLY if layers were filled
    // during the tumble phase (last 5s). If descent took >15s to stabilize,
    // early ascent wind data is likely valid and should be preserved.
    uint32_t cutoff = millis() - 5000;
    bool recent_fills = false;
    for (int i = 0; i < WIND_LAYER_COUNT; i++) {
      if (wind_layer_ms[i] > cutoff) { recent_fills = true; break; }
    }
    if (recent_fills) {
      for (int i = 0; i < WIND_LAYER_COUNT; i++) wind_layer_ms[i] = 0;
      wind_north = 0.0;
      wind_east  = 0.0;
      wind_speed = 0.0;
      Serial.println("Wind profile cleared (hard timeout — recent tumble contamination)");
    } else {
      Serial.println("Wind profile preserved (hard timeout — no recent layer updates)");
    }
    return true;
  }

  return false;
}

// BUG #10 FIX: landing timer is global, reset on state changes
static bool landingDetected() {
  // ✓ v5.5 AUDIT FIX: Cross-check baro sink rate with GNSS ground speed.
  // Baro alone can false-trigger in gusty ground-level conditions.
  // If GNSS has no fix, bypass the ground speed check (don't block landing detection).
  // ✓ v5.6: Tightened thresholds — was 0.3/0.5/1000ms, now 0.2/0.4/1500ms
  // Reduces false landing triggers from gusts and ground-effect pressure transients.
  bool baro_still = (sink_rate_filtered < 0.2);
  bool gnss_still = (ground_speed < 0.4) || (gnss_fix_type < 3);
  if (baro_still && gnss_still) {
    if (landing_still_start == 0) landing_still_start = millis();
    if (millis() - landing_still_start > 1500) {
      Serial.println("LANDED!");
      return true;
    }
  } else {
    landing_still_start = 0;
  }
  return false;
}

// Called on every state transition to reset timers and integrators
static void onStateTransition(MissionState from, MissionState to) {
  Serial.printf("STATE: %d -> %d\n", from, to);

  // Only reset guidance integrators when the control mode truly changes.
  // Transitions between guided phases (GUIDED→FINAL_APP→TERMINAL_HOMING) share
  // the heading/crosstrack controller — resetting kills accumulated integral and
  // causes 2-4s of degraded control at critical altitudes.
  bool from_guided = (from == GUIDED_DESCENT || from == FINAL_APPROACH ||
                      from == TERMINAL_HOMING);
  bool to_guided   = (to == GUIDED_DESCENT || to == FINAL_APPROACH ||
                      to == TERMINAL_HOMING);
  if (!(from_guided && to_guided)) {
    resetGuidanceIntegrators();
  }

  landing_still_start   = 0;
  stableStartMs         = 0;
  stable_wait_start_ms  = 0;
  drop_spike_count      = 0;

  // ✓ v5.5 AUDIT FIX: Flush SD at critical phase entries so data survives power loss.
  // Only two flushes during entire flight — each takes ~30ms on slow cards.
  // kickWatchdog() brackets prevent watchdog timeout during flush.
  // ✓ v6.2: bracket with serviceRTKHotPath() so RTK doesn't starve across the
  // ~30 ms SD flush (SPI0 is shared with the LoRa radio).
  if ((to == GUIDED_DESCENT || to == FLARE) && logFile) {
    serviceRTKHotPath();
    kickWatchdog();
    logFile.flush();
    kickWatchdog();
    serviceRTKHotPath();
  }
}

// =====================================================================
//  TELEMETRY + SD LOGGING
// =====================================================================
static void sendTelemetry1Hz() {
  uint32_t now = millis();
  // ✓ v5.5 AUDIT FIX: 2Hz telemetry during terminal phases for better ground awareness.
  // At SF7/BW250/CR4-5, 50-byte TX takes ~49ms. 500ms period still leaves 451ms for RX.
  uint32_t telem_interval = (state >= TERMINAL_HOMING && state <= TERMINAL)
                            ? (TELEMETRY_PERIOD_MS / 2) : TELEMETRY_PERIOD_MS;
  if (now - lastTelemetryMs < telem_interval) return;
  lastTelemetryMs = now;

  TelemetryPacket p{};
  p.msgType          = MSG_TELEMETRY;
  p.time_ms          = now;
  // ✓ v5.1: Send EKF-filtered position (what guidance actually uses),
  // not raw GNSS (which can differ by up to 2m from EKF estimate).
  double tx_lat, tx_lon;
  getEstimatedLatLon(tx_lat, tx_lon);
  p.lat_e7           = (int32_t)llround(tx_lat * 1e7);
  p.lon_e7           = (int32_t)llround(tx_lon * 1e7);
  double height_agl  = fused_alt_msl - target_alt_msl;
  p.height_agl_dm    = (int16_t)clampf((float)(height_agl * 10.0), -32768, 32767);
  p.pressure_hPa_x10 = (uint16_t)clampf((float)(pressure_hPa * 10.0), 0, 65535);
  p.temp_c_x10       = (int16_t)clampf((float)(temp_c * 10.0), -32768, 32767);
  p.ground_speed_cms = (uint16_t)clampf((float)(ground_speed * 100.0), 0, 65535);
  p.servo_left_x1000  = (int16_t)clampf(cmdL_current * 1000.0f, -1000, 1000);
  p.servo_right_x1000 = (int16_t)clampf(cmdR_current * 1000.0f, -1000, 1000);
  p.pred_lat_e7 = (int32_t)llround(pred_lat * 1e7);
  p.pred_lon_e7 = (int32_t)llround(pred_lon * 1e7);
  p.wind_north_cms = (int16_t)clampf((float)(wind_north * 100.0), -32768, 32767);
  p.wind_east_cms  = (int16_t)clampf((float)(wind_east  * 100.0), -32768, 32767);
  p.heading_deg_x10 = (int16_t)clampf((float)(getHeadingRad() * RAD_TO_DEG * 10.0f), -32768, 32767);
  p.heading_confidence_x1000 = (uint16_t)clampf(getHeadingConfidence() * 1000.0f, 0.0f, 1000.0f);
  p.wind_rejects       = wind_update_rejects;
  p.wind_layer_rejects = wind_layer_rejects;
  p.mission_state      = (uint8_t)state;
  p.gnss_carr_soln     = gnss_carrier_solution;
  p.gnss_num_sats      = gnss_satellites;
  p.crc16 = crc16_ccitt((uint8_t *)&p, sizeof(p) - 2);

  // Non-blocking LoRa transmit to avoid stalling the control loop
  if (lora_ok && !lora_tx_pending) {  // ✓ v5.9 M1: guard against double-TX
    // ✓ v5.9 M2: Atomically clear ISR-shared flag before starting TX
    noInterrupts();
    lora_tx_done_flag = false;
    interrupts();
    int err = radio.startTransmit((uint8_t *)&p, sizeof(p));
    if (err == RADIOLIB_ERR_NONE) {
      lora_tx_pending = true;
      lora_tx_start_ms = millis();
      last_lora_activity_ms = millis();  // ✓ v5.9 M4: seed RX recovery watchdog
    } else {
      // ✓ v5.9 M3: recover to RX on startTransmit failure — prevents radio stuck in standby
      safeStartReceive();
      last_lora_activity_ms = millis();
    }
  }

  // SD logging
  static uint32_t last_flush_ms = 0;
  if (logFile) {
    // ✓ v5.1: Added ekf_N, ekf_E, ekf_vN, ekf_vE, ekf_wN, ekf_wE for full EKF state reconstruction
    logFile.printf(
      "%lu,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.7f,%.7f,"
      "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u,%d,%lu,"
      "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%u,%u,%d,%u,"
      "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      p.time_ms, lat, lon, height_agl, pressure_hPa, temp_c, ground_speed,
      cmdL_current, cmdR_current, pred_lat, pred_lon, (int)state,
      gnss_fix_type, gnss_satellites, wind_speed, predicted_error_m,
      wind_north, wind_east, getHeadingConfidence(),
      wind_update_rejects, wind_layer_rejects,
      (int)(heading_offset_runtime * 1000), gnss_h_accuracy_mm,
      ekf9.initialized ? (float)ekf9.x[EKF9_D_IDX] : 0.0f,
      ekf9.initialized ? (float)ekf9.x[EKF9_VD_IDX] : 0.0f,
      ekf9.initialized ? (float)ekf9.x[EKF9_BIAS_IDX] : 0.0f,
      ekf9.initialized ? ekf9.P[EKF9_N_IDX][EKF9_N_IDX] : 0.0f,
      ekf9.initialized ? ekf9.P[EKF9_E_IDX][EKF9_E_IDX] : 0.0f,
      ekf9.initialized ? ekf9.P[EKF9_D_IDX][EKF9_D_IDX] : 0.0f,
      ekf9.gnss_reject_count, ekf9.baro_reject_count,
      (int)ekf9.healthy, gnss_carrier_solution,
      ekf9.nis_ema[0], ekf9.nis_ema[1], ekf9.nis_ema[2],
      ekf9.nis_ema[3], ekf9.nis_ema[4], ekf9.nis_ema[5],
      ekf9.nis_ema[6], ekf9.nis_ema[7], ekf9.nis_ema[8],
      // v5.1: EKF horizontal state (N, E, vN, vE, wN, wE)
      ekf9.initialized ? (float)ekf9.x[EKF9_N_IDX] : 0.0f,
      ekf9.initialized ? (float)ekf9.x[EKF9_E_IDX] : 0.0f,
      ekf9.initialized ? (float)ekf9.x[EKF9_VN_IDX] : 0.0f,
      ekf9.initialized ? (float)ekf9.x[EKF9_VE_IDX] : 0.0f,
      ekf9.initialized ? (float)ekf9.x[EKF9_WN_IDX] : 0.0f,
      ekf9.initialized ? (float)ekf9.x[EKF9_WE_IDX] : 0.0f);
    // ✓ v5.1 AUDIT FIX: No flush during ANY guided phase (GUIDED_DESCENT and beyond).
    // SD flush blocks for 10-50ms on slow cards, stalling the control loop.
    // Flush only during pre-guidance phases (BOOT, WAIT_DROP, WAIT_STABLE) and at landing.
    // Data is protected by SD write-caching; final landing flush captures everything.
    if (state < GUIDED_DESCENT) {
      uint32_t flush_interval = 30000;
      if (now - last_flush_ms > flush_interval) {
        serviceRTKHotPath();  // ✓ v6.2: drain RTK before SPI0 blocks
        kickWatchdog();
        logFile.flush();
        kickWatchdog();
        serviceRTKHotPath();  // ✓ v6.2: resume RTK drain after flush
        last_flush_ms = now;
      }
    }
  }
}

// Check if LoRa TX is done and return to receive mode.
// ✓ v5.9: Atomic flag reads, named timeout constant, diagnostic counters, activity watchdog seed
static void checkLoRaTxDone() {
  if (!lora_tx_pending || !lora_ok) return;

  // ✓ v5.9 M2: Atomically read ISR-shared flag to prevent race condition
  noInterrupts();
  bool tx_done = lora_tx_done_flag;
  interrupts();

  if (tx_done) {
    noInterrupts();
    lora_tx_done_flag = false;
    interrupts();
    radio.finishTransmit();
    safeStartReceive();
    lora_tx_pending = false;
    last_lora_activity_ms = millis();  // ✓ v5.9 M4: seed RX recovery watchdog
  } else if (millis() - lora_tx_start_ms > LORA_TX_TIMEOUT_MS) {
    // ✓ v5.9 M5: 350ms named constant (was hardcoded 200ms)
    radio.finishTransmit();
    safeStartReceive();
    lora_tx_pending = false;
    lora_tx_timeout_count++;
    last_lora_activity_ms = millis();
  }
}

// ✓ v6.2: RTK hot-path service — ported from Cansat_tests/CanSat_transport_final.
// Must be called from as many loop-body hot spots as possible (sensor reads,
// SD flushes, MPC iterations, status prints) to prevent the LoRa RX chain from
// being starved while the main loop is stuck in a blocking operation. Safe to
// call at any time: the inner helpers early-return when idle, so the cost is
// a handful of flag reads + one millis() check when there is nothing to do.
static void serviceRTKHotPath() {
  checkLoRaTxDone();
  if (!lora_tx_pending) pollLoRa();
  serviceRTKToGNSS();

  // RX recovery watchdog — if no DIO1 activity for 500ms while not transmitting,
  // force radio back to RX. Recovers from stuck radio states caused by missed
  // DIO1 interrupts or SPI glitches that leave the radio idle.
  if (lora_ok && !lora_tx_pending && last_lora_activity_ms > 0) {
    if (millis() - last_lora_activity_ms > 500) {
      safeStartReceive();
      last_lora_activity_ms = millis();
      lora_rx_recovery_count++;
    }
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  // ✓ FIX: Disable watchdog IMMEDIATELY — it survives soft resets on IMXRT1062.
  // Just kicking it isn't enough: the bootloader doesn't kick the watchdog, so
  // if the previous firmware had the watchdog armed, it fires during the
  // bootloader→setup transition, preventing USB soft-reboot uploads.
  // Disabling it here ensures clean init. Re-armed after 5 stable loop iterations.
  disableWatchdog();

  Serial.begin(115200);
  delay(500);

  // ✓ v5.5 AUDIT FIX: Peek at SNVS to detect warm reboot BEFORE CrashReport delays.
  // On warm reboot (mid-flight watchdog recovery), skip the 4-second CrashReport
  // pause and reduce boot delays — saves ~22m of unguided descent at 5.5 m/s.
  bool early_warm_detect = false;
  {
    uint32_t reg0 = SNVS_LPGPR0;
    uint16_t magic = (reg0 >> 16) & 0xFFFF;
    if (magic == SNVS_MAGIC) early_warm_detect = true;
  }

  // Teensy 4.0 CrashReport — prints hard fault address, type, and registers
  // if a crash occurred before this reboot. Read-only, only prints if crash happened.
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println("------ END CRASH REPORT ------");
    if (!early_warm_detect) {
      delay(2000);
      kickWatchdog();
      delay(2000);
      kickWatchdog();
    } else {
      delay(100);
      kickWatchdog();
      Serial.println("(warm reboot — reduced CrashReport delay for fast recovery)");
    }
  }

  if (!early_warm_detect) {
    delay(1000);
    kickWatchdog();  // keep watchdog happy during setup delays on cold boot
    delay(1000);
  } else {
    delay(200);
    kickWatchdog();
  }
  Serial.println("=============================================");
  Serial.println("  AeroTrackNow CanSat — Flight v5.1");
  Serial.println("  Autonomous Parafoil Guidance");
  Serial.println("  FAL trajectory, rate loop, dynamic flare");
  Serial.println("=============================================");

  // Hardware init
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(200);  // 200ms I2C timeout — prevents indefinite hang on stuck bus

  // Check for warm reboot (watchdog recovery) — restore saved state from SNVS
  bool warm_reboot = restoreStateFromSNVS();

  // L7 FIX: Reset static locals that assume cold-boot initial state.
  if (warm_reboot) {
    resetGuidanceIntegrators();
    Serial.println("SNVS: Guidance integrators reset for warm reboot");
  }

  // C4 FIX: Warn if calibration tables are still at placeholder defaults
  if (SINK_RATE_TABLE[0] == 5.5f && SINK_RATE_TABLE[4] == 7.2f &&
      GLIDE_RATIO_TABLE[0] == 2.0f && HEADING_OFFSET_RAD == 0.0f) {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!! WARNING: Calibration tables are DEFAULTS  !!");
    Serial.println("!! Run process_drops.py and update values    !!");
    Serial.println("!! before competition flight!                !!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  // C5 FIX: Warn if magnetic declination hasn't been updated for competition site
  if (fabsf(MAGNETIC_DECLINATION_RAD - 0.1201f) < 0.001f) {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!! WARNING: MAGNETIC_DECLINATION_RAD = Warsaw !!");
    Serial.println("!! Update for competition site using NOAA calc!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  // Initialize I2C recovery timestamps
  last_bmp_success_ms = millis();
  last_bno_success_ms = millis();

  // ✓ v5.6: Direct PWM setup (replaces Servo library attach)
  analogWriteFrequency(PIN_SERVO_LEFT, PWM_SERVO_FREQ);
  analogWriteFrequency(PIN_SERVO_RIGHT, PWM_SERVO_FREQ);
  analogWriteResolution(PWM_SERVO_BITS);
  setServosNormalized(0, 0);

  Serial.println("Initializing sensors...");
  i2cBusRecovery();  // free I2C bus if stuck from previous watchdog reboot
  setupBMP280();
  kickWatchdog();  // prevent watchdog timeout during long init
  setupBNO085();
  kickWatchdog();

  // ✓ AUDIT FIX: Moved AFTER Wire.begin()+setupBNO085() — was previously before sensor init,
  // causing getSensorEvent() to fail silently (I2C not initialized).
  if (HEADING_CALIBRATION_MODE) {
    Serial.println("*** HEADING CALIBRATION MODE ***");
    Serial.println("Point CanSat exactly TRUE NORTH and read IMU heading below.");
    Serial.println("Calculate: HEADING_OFFSET_RAD = (0.0 - measured_value)");
    Serial.println("Note: MAGNETIC_DECLINATION_RAD is applied separately.");
    Serial.println("Update code, set HEADING_CALIBRATION_MODE=false, re-upload.");
    Serial.println("HALTED — calibration mode prevents flight. Reading IMU heading...");
    while (true) {
      if (bno08x.getSensorEvent(&bnoValue) &&
          bnoValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float qw = bnoValue.un.gameRotationVector.real;
        float qx = bnoValue.un.gameRotationVector.i;
        float qy = bnoValue.un.gameRotationVector.j;
        float qz = bnoValue.un.gameRotationVector.k;
        float siny = 2.0f * (qw * qz + qx * qy);
        float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
        float raw_heading = atan2f(siny, cosy);
        Serial.printf("Raw heading: %.4f rad (%.1f deg)  ->  HEADING_OFFSET_RAD = %.4f\n",
                       raw_heading, raw_heading * RAD_TO_DEG, -raw_heading);
      }
      delay(200);
    }
  }

  setupGNSS();
  kickWatchdog();
  seedWindProfile();

  // Initialize SPI bus and deselect ALL SPI devices BEFORE using any of them.
  // SD (CS=23) and LoRa (CS=10) share the SPI bus. If any CS pin is floating
  // when another device is being initialized, it will respond to SPI traffic
  // meant for the other device and corrupt communication.
  SPI.begin();
  pinMode(PIN_LORA_CS, OUTPUT);
  digitalWrite(PIN_LORA_CS, HIGH);   // deselect LoRa
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);     // deselect SD
  delay(10);

  kickWatchdog();

  Serial.println("Initializing SD card...");
  if (SD.begin(PIN_SD_CS)) {
    // L3 FIX: Unique filename using reboot count to prevent overwriting previous data
    char fname[24];
    uint8_t boot_num = SNVS_LPGPR0 & 0xFF;
    snprintf(fname, sizeof(fname), "flight_%03u.csv", boot_num);
    logFile = SD.open(fname, FILE_WRITE);
    if (logFile) {
      sd_ok = true;  // M12 FIX: flag for flight logic
      logFile.println("time_ms,lat,lon,height_agl,press_hPa,temp_C,gs_mps,"
                      "cmdL,cmdR,pred_lat,pred_lon,state,fix_type,sats,"
                      "wind_ms,pred_error_m,wind_n,wind_e,heading_conf,"
                      "wind_rejects,wind_layer_rejects,heading_offset_mrad,hacc_mm,"
                      "ekf_D,ekf_vD,ekf_baro_bias,P_N,P_E,P_D,"
                      "gnss_rej,baro_rej,ekf_healthy,carr_soln,"
                      "nis_N,nis_E,nis_D,nis_vN,nis_vE,nis_vD,"
                      "nis_baro,nis_modelN,nis_modelE,"
                      "ekf_N,ekf_E,ekf_vN,ekf_vE,ekf_wN,ekf_wE");
      logFile.flush();
      Serial.printf("SD logging enabled: %s\n", fname);
    }
  } else {
    Serial.println("WARNING: SD card failed");
  }

  kickWatchdog();

  Serial.println("Initializing LoRa...");
  Serial.flush();
  setupLoRa();
  kickWatchdog();
  if (!lora_ok) {
    Serial.println("WARNING: LoRa failed — telemetry disabled, firmware continues without radio");
  }
  Serial.flush();

  // TARGET HANDLING
  if (USE_HARDCODED_TARGET) {
    // ✓ FIXED v5.0: Validate hardcoded target coordinates
    if (fabs(HARDCODED_TARGET_LAT) > 1.0 && fabs(HARDCODED_TARGET_LON) > 1.0
        && HARDCODED_TARGET_ALT < 5000.0) {
      target_lat     = HARDCODED_TARGET_LAT;
      target_lon     = HARDCODED_TARGET_LON;
      target_alt_msl = HARDCODED_TARGET_ALT;
      targetReceived = true;
      if (HARDCODED_TARGET_ALT == 0.0) {
        Serial.println("WARNING: Target alt is EXACTLY 0.0 m MSL — verify this is correct!");
        Serial.println("         Most landing sites are NOT at sea level.");
      }
      Serial.printf("TARGET (hardcoded): %.7f, %.7f, %.1f m MSL\n",
                    target_lat, target_lon, target_alt_msl);
    } else {
      Serial.println("ERROR: Hardcoded target coordinates invalid!");
    }
  } else {
    Serial.println("WARNING: USE_HARDCODED_TARGET=false — target coords will be");
    Serial.println("         lost on watchdog reboot! Set true before flight.");
  }

  // ✓ FIXED v5.0: Check each calibration table independently (was single AND chain)
  bool sink_cal  = !(SINK_RATE_TABLE[0] == 5.5f && SINK_RATE_TABLE[4] == 7.2f);
  bool glide_cal = !(GLIDE_RATIO_TABLE[0] == 2.0f && GLIDE_RATIO_TABLE[4] == 1.4f);
  bool turn_cal  = !(TURN_RATE_SCALE_TABLE[0] == 1.0f && TURN_RATE_SCALE_TABLE[4] == 1.2f);
  bool heading_cal = (HEADING_OFFSET_RAD != 0.0f);
  bool cal_ok = sink_cal && glide_cal && turn_cal && heading_cal;
  if (!cal_ok) {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("  WARNING: PLACEHOLDER CALIBRATION VALUES!");
    if (!sink_cal)    Serial.println("  - Sink rate table: UNCALIBRATED");
    if (!glide_cal)   Serial.println("  - Glide ratio table: UNCALIBRATED");
    if (!turn_cal)    Serial.println("  - Turn rate table: UNCALIBRATED");
    if (!heading_cal) Serial.println("  - Heading offset: UNCALIBRATED (0.0)");
    Serial.println("  Run drop tests and process_drops.py first!");
    Serial.println("  Guidance will be INACCURATE with defaults.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  Serial.println("=============================================");
  Serial.printf("  Parafoil: %.3f m^2, %.0f g, sink=%.1f m/s\n",
                PARAFOIL_AREA_M2, SYSTEM_MASS_KG * 1000, NOMINAL_SINK_RATE);
  Serial.printf("  Heading offset: %.3f rad (%.1f deg)\n",
                HEADING_OFFSET_RAD, HEADING_OFFSET_RAD * RAD_TO_DEG);
  if (cal_ok)
    Serial.println("  Calibration: OK");
  else
    Serial.println("  Calibration: *** DEFAULTS — NOT CALIBRATED ***");
  if (targetReceived)
    Serial.println("  Target: SET — ready for drop");
  else
    Serial.println("  Target: WAITING for LoRa TargetPacket...");
  Serial.println("=============================================");

  // Reset I2C state so long GNSS init doesn't count as sensor failure.
  // Also reset fail counts — BMP280 first conversion may not be ready yet
  // (MODE_NORMAL with 63ms standby + x16 oversampling), causing 1-2 bad reads.
  last_bmp_success_ms = millis();
  last_bno_success_ms = millis();
  bmp_fail_count = 0;
  bno_fail_count = 0;
  last_i2c_recovery_ms = millis();  // prevent recovery during first 3 seconds

  // Watchdog is NOT enabled here — deferred to loop() after first iterations
  // prove stable. Prevents infinite boot loop if first loop() takes >timeout.
  Serial.flush();  // ensure all init output is visible before loop starts

  // On warm reboot, restoreStateFromSNVS() already set state to
  // WAIT_FOR_STABLE_DESCENT. Only reset to BOOT on cold boot.
  if (!warm_reboot) {
    state = BOOT;
  }
}

// =====================================================================
//  MAIN LOOP
// =====================================================================
void loop() {
  kickWatchdog();
  loop_start_ms = millis();

  // Deferred watchdog arm: enable RTWDOG only after 5 stable loop iterations.
  // This prevents infinite boot loops if the first iteration takes too long
  // (IMU buffer drain + SD card first-write can exceed the watchdog timeout).
  {
    static uint8_t stable_loops = 0;
    if (stable_loops < 5) {
      stable_loops++;
      if (stable_loops == 5) {
        setupWatchdog();
        Serial.println("Watchdog armed after 5 stable loops");
      }
    }
  }

  // EKF prediction — runs every loop for fresh state between sensor updates
  if (ekf9.initialized) {
    uint32_t now_ekf = millis();
    float dt_ekf = (ekf9.last_predict_ms > 0)
                   ? clampf((now_ekf - ekf9.last_predict_ms) / 1000.0f, 0.001f, 0.2f)
                   : 0.05f;
    if (dt_ekf >= 0.005f) {  // min 5ms between predicts (~200 Hz cap)
      ekf9.last_predict_ms = now_ekf;
      ekf9Predict(dt_ekf);
    }
  }

  // Complete any pending LoRa transmission before reading
  checkLoRaTxDone();

  // Read all sensors (kick watchdog between reads to prevent timeout).
  // ✓ v6.2: serviceRTKHotPath() interleaved with each sensor read so the LoRa
  // RX chain is drained between blocking I2C/SPI calls. Mirrors the call pattern
  // in Cansat_tests/CanSat_transport_final, which is the reference implementation
  // validated end-to-end with the seqfix ground station.
  serviceRTKHotPath();
  updateIMU();
  serviceRTKHotPath();
  kickWatchdog();       // IMU can drain many buffered events on first iteration
  updateBMP280();
  serviceRTKHotPath();
  updateGNSS();
  serviceRTKHotPath();
  kickWatchdog();       // GNSS + baro reads complete
  // RX recovery watchdog is now integrated inside serviceRTKHotPath() — no need
  // to duplicate it here. The helper is called every iteration above.

  // I2C bus recovery — detect stuck sensors, toggle SCL, reinitialize
  checkI2CRecovery();

  // Ascent wind profiling — pre-fill wind layers before guidance starts
  estimateAscentWind();

  // Servo failure detection — compare commands to measured turn rate
  checkServoFailure();

  // ✓ v5.5: Anti-spiral protection — detect sustained uncommanded turning
  checkAntiSpiral();

  // ✓ v5.3: Online canopy time constant identification
  identifyCanopyTimeConstant(fused_alt_msl - target_alt_msl);

  // Auto heading calibration — only during early guided descent (above 100m)
  if (state == GUIDED_DESCENT && (fused_alt_msl - target_alt_msl) > 100.0) {
    updateAutoHeadingCalibration();
  }
  // ✓ v5.2: Continuous slow heading correction below 100m (above 20m)
  double heading_height = fused_alt_msl - target_alt_msl;
  if (state >= GUIDED_DESCENT && state <= TERMINAL_HOMING &&
      heading_height < 100.0 && heading_height > 20.0) {
    updateContinuousHeadingCorrection();
  }

  // Detect state transitions for cleanup
  if (state != last_state) {
    onStateTransition(last_state, state);
    last_state = state;
  }

  // State machine
  switch (state) {

    case BOOT: {
      setServosNormalized(0, 0);
      static uint32_t boot_start_ms = 0;
      if (boot_start_ms == 0) boot_start_ms = millis();
      if (targetReceived) {
        state = WAIT_FOR_DROP;
      } else if (millis() - boot_start_ms > 300000) {
        // ✓ v5.5 AUDIT FIX: Do NOT transition without target. Previous code entered
        // WAIT_FOR_DROP without target coords, causing completely unguided flight.
        // Stay in BOOT — operator must send target via LoRa or use hardcoded coords.
        static uint32_t last_boot_warn_ms = 0;
        if (millis() - last_boot_warn_ms > 10000) {
          Serial.println("WARNING: 5+ min without target. STAYING IN BOOT.");
          Serial.println("  Send target via Mission Control or set USE_HARDCODED_TARGET=true");
          last_boot_warn_ms = millis();
        }
      }
      break;
    }

    case WAIT_FOR_DROP: {
      // ✓ FIX C5: Altitude-based fallback if accelerometer drop detection fails.
      // Tracks altitude at state entry; 30m descent is unmistakable free-fall.
      static uint32_t wait_drop_entry_ms = 0;
      static double   wait_drop_entry_alt = 0.0;
      if (wait_drop_entry_ms == 0) {
        wait_drop_entry_ms  = millis();
        wait_drop_entry_alt = fused_alt_msl;
      }
      setServosNormalized(0, 0);
      if (dropDetected()) {
        state = WAIT_FOR_STABLE_DESCENT;
        wait_drop_entry_ms = 0;
      }
      else if (wait_drop_entry_alt > 10.0 &&
               fused_alt_msl < wait_drop_entry_alt - 30.0) {
        Serial.println("WARN: Drop detected by altitude fallback (30m descent)");
        state = WAIT_FOR_STABLE_DESCENT;
        wait_drop_entry_ms = 0;
      }
      break;
    }

    case WAIT_FOR_STABLE_DESCENT:
      setServosNormalized(0, 0);
      if (descentStable()) {
        state = GUIDED_DESCENT;
      }
      break;

    // GUIDED_DESCENT (>50m) → FINAL_APPROACH (50m) → TERMINAL_HOMING (30m) → FLARE → TERMINAL
    case GUIDED_DESCENT: {
      // ✓ v5.3.1 AUDIT FIX: Altitude safety transitions MUST run regardless of guidance status.
      // Previously, !guidanceSafe() caused break before FLARE check — parafoil could hit ground
      // without flaring during GNSS dropout.
      double height_agl = fused_alt_msl - target_alt_msl;
      // Check lowest altitude first to prevent skipping states on fast descent
      if (height_agl < TERMINAL_HEIGHT_M) {
        state = TERMINAL;
        break;
      }
      // Dynamic flare trigger: time-to-impact
      // ✓ v5.2: Triple-blended sink rate for faster response: 50% baro + 25% EKF + 25% GNSS
      // ✓ v5.1 AUDIT FIX: Guard EKF access — use baro sink as fallback if EKF uninitialized
      {
        float ekf_vd = (ekf9.initialized && ekf9.healthy)
                       ? fmaxf((float)ekf9.x[EKF9_VD_IDX], 0.0f)
                       : (float)sink_rate_filtered;
        float gnss_vd = (gnss_fix_type >= 3 && fabs(gnss_vertical_velocity) > 0.01)
                        ? fmaxf((float)(-gnss_vertical_velocity), 0.0f)
                        : (float)sink_rate_filtered;
        float blended_sink = 0.5f * fmaxf((float)sink_rate_filtered, 0.0f) + 0.25f * ekf_vd + 0.25f * gnss_vd;
        float tti = (blended_sink > 0.5f) ? (float)(height_agl / blended_sink) : 99.0f;
        if (tti < FLARE_TRIGGER_TIME_S || height_agl < FLARE_HEIGHT_M) {
          flare_active = true;
          state = FLARE;
          break;
        }
      }
      if (height_agl < TERMINAL_HOMING_HEIGHT_M) {
        state = TERMINAL_HOMING;
        break;
      }
      if (height_agl < FINAL_APPROACH_HEIGHT_M) {
        state = FINAL_APPROACH;
        break;
      }
      // Guidance safety check — only affects steering, not altitude transitions
      if (!targetReceived || !guidanceSafe()) {
        resetGuidanceIntegrators();
        setServosNormalized(0, 0);
        break;
      }
      updateApproachPlan();
      runPredictiveGuidance();
      // Apply graduated wind gain to TURN component only (not glide slope)
      // ✓ FIXED: Glide slope (symmetric brake for sink rate) must remain at full authority
      float wg = guidanceWindGain();
      if (wg < 1.0f) {
        float sym = 0.5f * (cmdL_target + cmdR_target);  // glide slope component
        float diff = 0.5f * (cmdR_target - cmdL_target);  // turn component
        diff *= wg;  // only scale the turn
        cmdL_target = clampf(sym - diff, 0.0f, 1.0f);
        cmdR_target = clampf(sym + diff, 0.0f, 1.0f);
      }
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }

    // FINAL_APPROACH (50→30m): FAL cross-track + heading + feed-forward + rate loop
    case FINAL_APPROACH: {
      // ✓ v5.3.1 AUDIT FIX: Altitude safety transitions MUST run regardless of guidance status.
      double height_agl = fused_alt_msl - target_alt_msl;
      // ✓ FIX C4: At TERMINAL_HEIGHT_M (1m AGL), ALWAYS transition to TERMINAL.
      // Previous code required predicted_error_m < capture_r — could keep steering
      // at ground level if far from target, causing servo thrashing at touchdown.
      if (height_agl < TERMINAL_HEIGHT_M) {
        cmdL_target = 0.0f; cmdR_target = 0.0f;
        state = TERMINAL;
        break;
      }
      // ✓ v5.5 AUDIT FIX: Normalized triple-blend sink rate (same as GUIDED_DESCENT)
      // 50% baro + 25% EKF vD + 25% GNSS vD — consistent flare trigger across all states.
      {
        float ekf_vd = (ekf9.initialized && ekf9.healthy)
                       ? fmaxf((float)ekf9.x[EKF9_VD_IDX], 0.0f)
                       : (float)sink_rate_filtered;
        float gnss_vd = (gnss_fix_type >= 3 && fabs(gnss_vertical_velocity) > 0.01)
                        ? fmaxf((float)(-gnss_vertical_velocity), 0.0f)
                        : (float)sink_rate_filtered;
        float blended_sink = 0.5f * fmaxf((float)sink_rate_filtered, 0.0f) + 0.25f * ekf_vd + 0.25f * gnss_vd;
        float tti = (blended_sink > 0.5f) ? (float)(height_agl / blended_sink) : 99.0f;
        if (tti < FLARE_TRIGGER_TIME_S || height_agl < FLARE_HEIGHT_M) {
          flare_active = true;
          state = FLARE;
          break;
        }
      }
      if (height_agl < TERMINAL_HOMING_HEIGHT_M) {
        state = TERMINAL_HOMING;
        break;
      }
      // Guidance safety check — only affects steering, not altitude transitions
      if (!targetReceived || !guidanceSafe()) {
        resetGuidanceIntegrators();
        setServosNormalized(0, 0);
        break;
      }
      estimateWind();
      updateApproachPlan();
      predictLandingWithPhysics();
      // FAL guidance with inline fallback (avoids double wind estimation)
      if (!computeFALGuidance(height_agl)) {
        double desired_bearing = bearingRad(pred_lat, pred_lon, target_lat, target_lon);
        float gain = APPROACH_GAIN;
        float altitude_gain_val = altitudeGain(height_agl,
            HEADING_GAIN_HIGH_ALT, HEADING_GAIN_LOW_ALT,
            HEADING_GAIN_HIGH_ALT_M, HEADING_GAIN_LOW_ALT_M);
        float turn_cmd = computeHeadingControl(desired_bearing, gain * altitude_gain_val);
        // ✓ FIX: Damp lateral airspeed (slip), not ground velocity — same as predictive guidance fix
        if (ekf9.initialized) {
          double hdg = getHeadingRad();
          double air_n = ekf9.x[EKF9_VN_IDX] - ekf9.x[EKF9_WN_IDX];
          double air_e = ekf9.x[EKF9_VE_IDX] - ekf9.x[EKF9_WE_IDX];
          double vel_lat = -air_n * sin(hdg) + air_e * cos(hdg);
          turn_cmd -= (float)(LATERAL_DAMPING_GAIN * vel_lat);
        }
        turn_cmd = clampf(turn_cmd, -1.0f, 1.0f);
        turn_cmd = applyTurnRateFeedback(turn_cmd);
        float glide_cmd_val = applyGlideSlope(height_agl);
        cmdL_target = clampf(-turn_cmd + glide_cmd_val, 0.0f, 1.0f);
        cmdR_target = clampf( turn_cmd + glide_cmd_val, 0.0f, 1.0f);
      }
      { float wg_fa = guidanceWindGain();
        if (wg_fa < 1.0f) {
          float sym_fa = 0.5f * (cmdL_target + cmdR_target);
          float diff_fa = 0.5f * (cmdR_target - cmdL_target);
          diff_fa *= wg_fa;
          cmdL_target = clampf(sym_fa - diff_fa, 0.0f, 1.0f);
          cmdR_target = clampf(sym_fa + diff_fa, 0.0f, 1.0f);
        }
      }
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }

    // TERMINAL_HOMING (30m→flare): FAL cross-track + heading + feed-forward + rate loop
    case TERMINAL_HOMING: {
      // ✓ v5.3.1 AUDIT FIX: Altitude safety transitions MUST run regardless of guidance status.
      double height_agl = fused_alt_msl - target_alt_msl;
      // ✓ FIX C4: At TERMINAL_HEIGHT_M (1m AGL), ALWAYS transition to TERMINAL.
      if (height_agl < TERMINAL_HEIGHT_M) {
        cmdL_target = 0.0f; cmdR_target = 0.0f;
        state = TERMINAL;
        break;
      }
      // ✓ v5.5 AUDIT FIX: Normalized triple-blend sink rate (same as GUIDED_DESCENT)
      // 50% baro + 25% EKF vD + 25% GNSS vD — consistent flare trigger across all states.
      {
        float ekf_vd = (ekf9.initialized && ekf9.healthy)
                       ? fmaxf((float)ekf9.x[EKF9_VD_IDX], 0.0f)
                       : (float)sink_rate_filtered;
        float gnss_vd = (gnss_fix_type >= 3 && fabs(gnss_vertical_velocity) > 0.01)
                        ? fmaxf((float)(-gnss_vertical_velocity), 0.0f)
                        : (float)sink_rate_filtered;
        float blended_sink = 0.5f * fmaxf((float)sink_rate_filtered, 0.0f) + 0.25f * ekf_vd + 0.25f * gnss_vd;
        float tti = (blended_sink > 0.5f) ? (float)(height_agl / blended_sink) : 99.0f;
        if (tti < FLARE_TRIGGER_TIME_S || height_agl < FLARE_HEIGHT_M) {
          flare_active = true;
          state = FLARE;
          break;
        }
      }
      // Guidance safety check — only affects steering, not altitude transitions
      if (!targetReceived || !guidanceSafe()) {
        resetGuidanceIntegrators();
        setServosNormalized(0, 0);
        break;
      }
      estimateWind();
      updateApproachPlan();
      predictLandingWithPhysics();
      // FAL guidance with inline fallback (avoids double wind estimation)
      if (!computeFALGuidance(height_agl)) {
        double desired_bearing = bearingRad(pred_lat, pred_lon, target_lat, target_lon);
        float altitude_gain_val = altitudeGain(height_agl,
            HEADING_GAIN_HIGH_ALT, HEADING_GAIN_LOW_ALT,
            HEADING_GAIN_HIGH_ALT_M, HEADING_GAIN_LOW_ALT_M);
        float turn_cmd = computeHeadingControl(desired_bearing, altitude_gain_val);
        // ✓ FIX: Damp lateral airspeed (slip), not ground velocity — same as predictive guidance fix
        if (ekf9.initialized) {
          double hdg = getHeadingRad();
          double air_n = ekf9.x[EKF9_VN_IDX] - ekf9.x[EKF9_WN_IDX];
          double air_e = ekf9.x[EKF9_VE_IDX] - ekf9.x[EKF9_WE_IDX];
          double vel_lat = -air_n * sin(hdg) + air_e * cos(hdg);
          turn_cmd -= (float)(LATERAL_DAMPING_GAIN * vel_lat);
        }
        turn_cmd = clampf(turn_cmd, -1.0f, 1.0f);
        turn_cmd = applyTurnRateFeedback(turn_cmd);
        float glide_cmd_val = applyGlideSlope(height_agl);
        cmdL_target = clampf(-turn_cmd + glide_cmd_val, 0.0f, 1.0f);
        cmdR_target = clampf( turn_cmd + glide_cmd_val, 0.0f, 1.0f);
      }
      { float wg_th = guidanceWindGain();
        if (wg_th < 1.0f) {
          float sym_th = 0.5f * (cmdL_target + cmdR_target);
          float diff_th = 0.5f * (cmdR_target - cmdL_target);
          diff_th *= wg_th;
          cmdL_target = clampf(sym_th - diff_th, 0.0f, 1.0f);
          cmdR_target = clampf(sym_th + diff_th, 0.0f, 1.0f);
        }
      }
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }

    // FLARE: adaptive brake step for dynamic lift transient. Bypass rate limiter.
    case FLARE: {
      flare_active = true;
      // ✓ FIXED: Adaptive flare brake based on ground-layer wind speed.
      // Uses lowest wind layer (0-5m AGL) instead of global EMA for accuracy.
      // High wind → full brake (maximize transient lift, wind decelerates vehicle).
      // Low wind → reduced brake (maintain some forward speed for smoother landing).
      double flare_wn, flare_we;
      getWindAtLayer(0, flare_wn, flare_we);
      float flare_wind = (float)sqrt(flare_wn * flare_wn + flare_we * flare_we);
      float flare_t = clampf((flare_wind - FLARE_WIND_LOW_MPS) /
                             (FLARE_WIND_HIGH_MPS - FLARE_WIND_LOW_MPS), 0.0f, 1.0f);
      float flare_brake = lerpf(FLARE_BRAKE_MIN, FLARE_BRAKE_MAX, flare_t);
      // FIX IMP-4: Disable crosswind correction when heading is unreliable.
      // Symmetric flare is safer than wrong-direction asymmetric flare.
      // Found by safety-auditor + heading-auditor.
      // ✓ v5.6: Raised confidence threshold from 0.2 to 0.4 — prevents wrong-direction
      // asymmetric flare when heading is unreliable (e.g. gyro drift, low speed).
      if (getHeadingConfidence() >= 0.4f) {
        float hdg = (float)getHeadingRad();
        float crosswind = (float)(-flare_wn * sin(hdg) + flare_we * cos(hdg));
        float diff = clampf(crosswind * FLARE_CROSSWIND_GAIN,
                           -FLARE_MAX_ASYMMETRY, FLARE_MAX_ASYMMETRY);
        // ✓ FIXED v5.0: Preserve full differential when near max brake.
        // Previously, flare_brake + diff would clip to 1.0, losing correction authority.
        float flare_L = flare_brake + diff;
        float flare_R = flare_brake - diff;
        if (flare_L > 1.0f) {
          flare_R -= (flare_L - 1.0f);  // transfer overshoot
          flare_L = 1.0f;
          flare_R = fmaxf(flare_R, 0.0f);
        }
        if (flare_R > 1.0f) {
          flare_L -= (flare_R - 1.0f);
          flare_R = 1.0f;
          flare_L = fmaxf(flare_L, 0.0f);
        }
        setServosNormalized(flare_L, flare_R);
      } else {
        // Degraded heading: symmetric flare (no crosswind correction)
        setServosNormalized(flare_brake, flare_brake);
      }
      double height_agl = fused_alt_msl - target_alt_msl;
      if (height_agl < TERMINAL_HEIGHT_M) {
        flare_active = false;
        state = TERMINAL;
      }
      break;
    }

    case TERMINAL: {
      flare_active = false;
      // ✓ FIXED v5.0: Maintain crosswind correction during terminal descent.
      // Previously commanded (0,0), allowing uncorrected crosswind drift
      // during the ~0.3s rate-limited brake release.
      double term_wn, term_we;
      getWindAtLayer(0, term_wn, term_we);
      float term_hdg = (float)getHeadingRad();
      float term_cw = (float)(-term_wn * sin(term_hdg) + term_we * cos(term_hdg));
      float term_diff = clampf(term_cw * FLARE_CROSSWIND_GAIN,
                              -FLARE_MAX_ASYMMETRY, FLARE_MAX_ASYMMETRY);
      // Differential only — no symmetric brake component
      setServosNormalized(fmaxf(term_diff, 0.0f), fmaxf(-term_diff, 0.0f));
      if (landingDetected()) {
        Serial.printf("FINAL ERROR: %.2f m\n",
                      distanceMeters(lat, lon, target_lat, target_lon));
        // Aggressive SD flush on landing — capture final data
        // ✓ v6.2: bracket with RTK service; landing is the last chance to
        // drain any corrections still in the queue before LANDED state.
        serviceRTKHotPath();
        if (logFile) logFile.flush();
        serviceRTKHotPath();
        state = LANDED;
      }
      break;
    }

    case LANDED:
      setServosNormalized(0, 0);
      break;

    default:
      setServosNormalized(0, 0);
      break;
  }

  // Status printing (1 Hz, offset from telemetry)
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 1000) {
    lastPrintMs = millis();
    const char* state_names[] = {
      "BOOT", "WAIT_DROP", "WAIT_STABLE", "GUIDED", "FINAL_APP",
      "TERMINAL_HOME", "FLARE", "TERMINAL", "LANDED"
    };
    const char* fix_names[] = {"No fix", "Dead reck", "2D", "3D", "3D+DGNSS", "Time-only"};
    const char* carr_names[] = {"None", "Float", "Fixed"};
    const char* fix_name = (gnss_fix_type <= 5) ? fix_names[gnss_fix_type] : "??";
    const char* carr_name = (gnss_carrier_solution <= 2) ? carr_names[gnss_carrier_solution] : "??";
    uint32_t rtk_age_s = last_rtcm_rx_ms ? (millis() - last_rtcm_rx_ms) / 1000 : 9999;
    Serial.printf("[%s] fix=%d(%s) RTK=%s sats=%d hacc=%lumm alt=%.0f sink=%.1f wind=%.1f pred=%.1f rtcm=%lu age=%lus sg=%.2f qlen=%u drop=%lu crcfail=%lu seqgaps=%lu gsping=%lu\n",
                  state_names[(int)state], gnss_fix_type, fix_name, carr_name, gnss_satellites,
                  gnss_h_accuracy_mm, fused_alt_msl - target_alt_msl,
                  sink_rate_filtered, wind_speed, predicted_error_m,
                  rtcm_packets_received, rtk_age_s, servo_gain_estimate,
                  (unsigned)rtk_q_len, (unsigned long)rtk_queue_drop_bytes,
                  (unsigned long)rtk_crc_fail_count, (unsigned long)rtk_seq_gaps,
                  (unsigned long)gs_ping_rx_count);
    if (target_unreachable) Serial.println("  *** TARGET UNREACHABLE — cross-wind mode ***");
  }

  kickWatchdog();       // state machine complete, before SD write + LoRa TX
  sendTelemetry1Hz();
  saveStateToSNVS();    // persist critical state for watchdog recovery

  // ✓ FIXED: 40ms loop period matches 25Hz GNSS rate (was 50ms for 20Hz)
  uint32_t loop_time = millis() - loop_start_ms;
  if (loop_time > 60) {
    Serial.printf("WARN: loop overrun %lu ms\n", loop_time);
  }
  if (loop_time < 40) delay(40 - loop_time);
}
