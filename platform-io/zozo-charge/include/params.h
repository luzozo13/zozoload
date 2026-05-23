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
#define PZEM_ADDR 0x01      // PZEM device address
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