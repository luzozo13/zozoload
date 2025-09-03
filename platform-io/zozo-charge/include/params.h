#pragma once

// Forward declaration
class MqttHandler;

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

//-- PWM Configuration --//
#define CH_CP_CTRL 0        // PWM channel for Control Pilot
#define F_PWM 1000          // PWM frequency in Hz (1kHz)
#define PWM_RES 8           // PWM resolution in bits (8-bit: 0-255)

//-- EVSE State Definitions --//
#define STATE_INIT 0
#define STATE_A 1           // Vehicle not connected
#define STATE_B 2           // Vehicle connected, not ready
#define STATE_C 3           // Vehicle connected and ready/charging
#define STATE_FAULT -1      // Fault state

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

//-- Global State Variables --//
// EVSE Controller Class (OpenEVSE pattern)
class EVSEController {
private:
  // MQTT Handler for communication
  MqttHandler* m_mqttHandler;         // Injected dependency

  // Control Pilot measurements
  int i_cpp_value;
  int i_cpp_max;
  int i_cpp_min;
  int i_charge_speed = CP_AMP_16;     // Default charge speed

  // EVSE State Machine Variables  
  int i_state_current = STATE_INIT;
  int i_state_previous = STATE_INIT;
  int i_state_meas = STATE_INIT;
  int i_debounce_cnt = 0;             // Legacy counter

  // Timer-based State Transition (OpenEVSE pattern)
  unsigned long ul_tmp_state_start = 0;
  int i_tmp_state = STATE_INIT;

  // Debug and Performance Monitoring
  unsigned long ul_loop_start_time = 0;
  unsigned long ul_loop_end_time = 0;
  unsigned long ul_readpilot_start_time = 0;
  unsigned long ul_readpilot_end_time = 0;
  unsigned long ul_loop_duration = 0;
  unsigned long ul_readpilot_duration = 0;
  unsigned long ul_last_state_publish = 0;

public:
  // Constructors
  EVSEController();                          // Default constructor (no MQTT)
  EVSEController(MqttHandler& mqttHandler);  // Constructor with MQTT dependency injection
  
  // Set MQTT handler after construction
  void setMqttHandler(MqttHandler& mqttHandler);

  // Getters for Control Pilot measurements
  int getCppValue() const { return i_cpp_value; }
  int getCppMax() const { return i_cpp_max; }
  int getCppMin() const { return i_cpp_min; }
  int getChargeSpeed() const { return i_charge_speed; }

  // Setters for Control Pilot measurements
  void setCppValue(int value) { i_cpp_value = value; }
  void setCppMax(int max_val) { i_cpp_max = max_val; }
  void setCppMin(int min_val) { i_cpp_min = min_val; }
  void setChargeSpeed(int speed) { i_charge_speed = speed; }

  // State getters
  int getCurrentState() const { return i_state_current; }
  int getPreviousState() const { return i_state_previous; }
  int getMeasuredState() const { return i_state_meas; }
  int getTmpState() const { return i_tmp_state; }
  unsigned long getTmpStateStart() const { return ul_tmp_state_start; }

  // State setters
  void setCurrentState(int state) { i_state_current = state; }
  void setPreviousState(int state) { i_state_previous = state; }
  void setMeasuredState(int state) { i_state_meas = state; }
  void setTmpState(int state) { i_tmp_state = state; }
  void setTmpStateStart(unsigned long time) { ul_tmp_state_start = time; }

  // Debug timing getters/setters
  unsigned long getLastStatePublish() const { return ul_last_state_publish; }
  void setLastStatePublish(unsigned long time) { ul_last_state_publish = time; }

  // Main Control Functions (OpenEVSE pattern)
  void update();                      // Main update loop
  int readPilot();                    // Read and process pilot signal
  
  // State Detection Functions
  bool isStateDiff();                 // Check if state has changed
  bool isFirstStateDiff();            // Check if this is first state change
  void startDiffTimer();              // Start state change timer
  bool isDiffSteady();                // Check if state change is stable

  // State Management Functions
  void setState();                    // Main state controller
  void setStateFault();               // Set fault state

  // Relay Control Functions (OpenEVSE pattern)
  void chargingOn();                  // Close relay - start charging
  void chargingOff();                 // Open relay - stop charging

  // Control Functions
  void updateChargeSpeed();           // Update charging current
  void publishState();                // Publish state via MQTT
};

// Global EVSE Controller instance
extern EVSEController g_EvseController;

//-- Function Prototypes --//
// Communication Functions
void mqttCallback(char* sz_topic, byte* b_payload, unsigned int ui_length);
void reconnect();                     // MQTT reconnection handler