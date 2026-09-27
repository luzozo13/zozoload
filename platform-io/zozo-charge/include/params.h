#pragma once

#include <Arduino.h>  // For byte type definition

// Forward declarations
class MqttHandler;
class PZEM004Tv30; // PZEM type forward declaration

//-- Pin Assignments --//
// EV Charger Control Pins
#define CP_CTRL 18          // Control Pilot PWM output
#define FLT_CTRL 19         // Fault control output
#define REL_CTRL 21         // Main relay control
#define MT_CTRL_CLOSE 22    // Motor control - close
#define MT_CTRL_OPEN 23     // Motor control - open

// LED Pins  
#define L_R 25              // Red LED
#define L_G 26              // Green LED
#define L_B 27              // Blue LED

// Lock Control Pins
#define LOCK_ON 32          // Lock engage input
#define LOCK_OFF 33         // Lock disengage input

// Input Pins
#define PILOT_READ 34       // Control Pilot voltage reading (analog)
#define B_R 35              // Red button input
#define B_G 36              // Green button input  
#define B_B 39              // Blue button input

//-- PZEM Current Measurement --//
#define PZEM_RX_PIN 3      // PZEM RX pin (connect to PZEM TX)
#define PZEM_TX_PIN 1      // PZEM TX pin (connect to PZEM RX)
#define PZEM_ADDR 0x04      // PZEM device address
#define PZEM_UPDATE_INTERVAL 5000  // Update every 5 seconds
#define PZEM_TEST_MODE 0    // Set to 1 for simulated readings, 0 for real hardware

//-- PWM Configuration --//
#define CH_CP_CTRL 0        // PWM channel for Control Pilot
#define F_PWM 1000          // PWM frequency in Hz (1kHz)
#define PWM_RES 8           // PWM resolution in bits (8-bit: 0-255)

//-- EVSE State Definitions --//
#define STATE_INIT 0
#define STATE_A 1           // Vehicle not connected
#define STATE_B 2           // Vehicle connected, not ready
#define STATE_C 3           // Vehicle connected and ready/charging
#define STATE_SLEEPING 4    // CP set to +12V, waiting before opening relay (J1772 stop sequence)
#define STATE_FAULT -1      // Fault state

#define SLEEP_RELAY_OPEN_TIMEOUT_MS 3000  // Max ms to wait for EV to release before forcing relay open

//-- CP Fault Detection (state D/E: CP below TH_CD, e.g. CP shorted to ground) --//
// 0 = report only: a steady low CP is published as a "cp_fault_seen" event on
//     state/change, but the state machine ignores it (previous behaviour).
// 1 = enforce: a steady low CP goes to STATE_FAULT (relay open, FLT_CTRL high).
// Run with 0 first and check no event shows up during normal charging.
#define FAULT_DETECT_ENFORCE 0
#define FAULT_RETRY_MS       30000  // Min time held in STATE_FAULT before it may clear

//-- Debug Flags (bitmask — each flag gates its corresponding state topic) --//
#define DBG_DETAILS  0x01   // state/details  (EVSE state periodic)
#define DBG_CHANGE   0x02   // state/change   (EVSE state transitions)
#define DBG_PZEM     0x04   // state/pzem     (power meter telemetry)
#define DBG_COMM     0x08   // state/comm     (connection status)
#define DBG_TIME     0x10   // state/time     (NTP / uptime)

//-- NVS Persistence --//
// Bump NVS_SCHEMA_VERSION to force-reset all NVS settings on next upload
#define NVS_SCHEMA_VERSION  5
#define DBG_DEFAULT_FLAGS   0x1F  // All 5 flags on by default

//-- Solar Tracking Mode --//
// Closed-loop control: every SOLAR_LOOP_MS, compare the energy planner's setpoint
// (MQTT_EVSE_SETPOINT: solar production minus the other loads) vs EVSE
// consumption (PZEM). Surplus -> decrement PWM (more current, slow ramp-up);
// deficit -> increment PWM (less current, fast back-off). PWM clamped to
// [SOLAR_PWM_MIN, SOLAR_PWM_MAX]. Deadband prevents flapping near zero diff.
#define SOLAR_PWM_MIN     110   // Max charging current (lowest PWM duty)
#define SOLAR_PWM_MAX     221   // Min charging current = 8A (CP_AMP_8)
#define SOLAR_DEADBAND_W  500   // No change when |setpoint - evse_power| <= this
#define SOLAR_STEP_DOWN   5     // PWM decrement step (more current) on surplus
#define SOLAR_STEP_UP     10    // PWM increment step (less current) on deficit
#define SOLAR_LOOP_MS     10000 // Control loop period (ms)
// Setpoint freshness: the energy planner publishes at least every 30 s, and stops
// when the solar meter is silent. With no setpoint message for this long, the loop stops trusting
// the last value and falls back to minimum current (SOLAR_PWM_MAX) until data
// comes back, instead of charging from the grid on a stale surplus.
#define SOLAR_STALE_MS    180000
// Solar pause/resume (anti-flicker): when already at minimum current and still
// in deficit (or setpoint stale), pause charging instead of drawing from the
// grid. Minimum on/off times keep the contactor from cycling on unsteady sun.
#define SOLAR_PAUSE_AFTER_MS   600000   // Deficit at min current this long -> pause
#define SOLAR_MIN_ON_MS        900000   // Never pause sooner than this after charging (re)started
#define SOLAR_MIN_PAUSE_MS    1200000   // Once paused, stay paused at least this long
#define SOLAR_RESUME_W           2300   // Setpoint needed to resume (~8A x 230V + margin)
#define SOLAR_RESUME_AFTER_MS  600000   // Setpoint >= SOLAR_RESUME_W this long -> resume

//-- Unified config NVS defaults --//
#define NVS_DEFAULT_HOSTNAME      "zozo-charge"
#define NVS_DEFAULT_PZEM_ADDR     0x04
#define NVS_DEFAULT_PZEM_ACQ_RATE 5     // seconds
#define NVS_DEFAULT_PZEM_PUB_RATE 30    // seconds (state/pzem publish interval)

//-- Control Pilot Thresholds (ADC values) --//
// Based on J1772 specification voltage levels
#define TH_AB 3770          // Threshold between State A and B (~11V)
#define TH_BC 3135          // Threshold between State B and C (~9V)  
#define TH_CD 2400          // Threshold between State C and D (~6V)

//-- Control Pilot PWM Values --//
// J1772 compliant PWM duty cycles for current encoding
#define CP_AMP_8 221        // 8A charging current
#define CP_AMP_16 187       // 16A charging current
#define CP_AMP_BOOST 170    // Higher current (boost mode)
#define CP_12P 0            // +12V steady state (State A)
#define CP_12N 255          // -12V steady state (fault condition)

//-- Timing Constants --//
// OpenEVSE-style debouncing delays
#define DELAY_STATE_TRANSITION 250    // Standard state transition debounce (ms)
#define DELAY_STATE_TRANSITION_A 25   // Faster debounce for State A (ms)

// Publishing and communication intervals
#define UL_STATE_PUBLISH_INTERVAL 10000  // State publishing interval (ms)
#define MQTT_RECONNECT_INTERVAL   10000  // Min ms between MQTT reconnect attempts (non-blocking)
#define WIFI_CONNECT_TIMEOUT_MS   20000  // Max wait for WiFi in setup(), then continue offline
#define WDT_TIMEOUT_S             10     // Task watchdog: reboot (relay opens) if loop() hangs

// Legacy timing (for compatibility)
#define STATE_CHANGE_DEBOUNCE 10
#define I_DELAY_PILOT_LOOP 12

// Forward declare EVSEController (definition in include/EVSEController.h)
class EVSEController;

// Global EVSE Controller instance
extern EVSEController g_EvseController;
// Global PZEM instance (defined in main.cpp)
extern PZEM004Tv30 pzem;

//-- Function Prototypes --//
// Communication Functions
void reconnect();                     // MQTT reconnection handler