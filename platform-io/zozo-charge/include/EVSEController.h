// EVSEController.h - Class declaration separated from params
#pragma once

#include "params.h" // pull in pin/constants

enum EvseMode {
  MODE_BOOST = 0,
  MODE_SOLAR = 1,
  MODE_CHEAP = 2
};

// Forward declarations
class MqttHandler;

// One-letter state code used in MQTT payloads (A, B, C, S=sleeping, F=fault, U=unknown)
inline char stateChar(int state) {
  switch (state) {
    case STATE_A:        return 'A';
    case STATE_B:        return 'B';
    case STATE_C:        return 'C';
    case STATE_SLEEPING: return 'S';
    case STATE_FAULT:    return 'F';
    default:             return 'U';
  }
}

class EVSEController {
private:
  // MQTT Handler for communication
  MqttHandler* m_mqttHandler;         // Injected dependency

  // Charging authorization
  bool b_charging_enabled = true;   // Default-on: charge immediately when car connects
  unsigned long ul_sleep_start_ms = 0;

  // CP fault detection (state D/E)
  unsigned long ul_fault_start_ms = 0;
  bool b_cp_low = false;              // CP max currently below TH_CD
  bool b_cp_low_reported = false;     // "cp_fault_seen" already published for this episode
  unsigned long ul_cp_low_start_ms = 0;

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

  // Current Measurement (PZEM)
  float f_current = 0.0;              // Measured current in Amps
  float f_voltage = 0.0;              // Measured voltage in Volts
  float f_power = 0.0;                // Measured power in Watts
  float f_energy = 0.0;               // Measured energy in kWh
  unsigned long ul_last_pzem_read = 0; // Last PZEM reading timestamp

  // Debug and Performance Monitoring
  unsigned long ul_loop_start_time = 0;
  unsigned long ul_loop_end_time = 0;
  unsigned long ul_readpilot_start_time = 0;
  unsigned long ul_readpilot_end_time = 0;
  unsigned long ul_loop_duration = 0;
  unsigned long ul_readpilot_duration = 0;
  unsigned long ul_last_state_publish = 0;

  EvseMode m_active_mode = MODE_SOLAR; // Default mode
  bool m_is_charge_cheap = false;

  // Solar tracking state
  bool b_solar_tracking = true;
  unsigned long ul_solar_last_step_ms = 0;
  // Power available for the car, from the energy planner (solar - other loads)
  unsigned long ul_setpoint_rx_ms = 0;
  bool b_setpoint_rx_seen = false;
  float f_setpoint_watts = 0.0f;

  // Solar pause/resume (anti-flicker)
  bool b_solar_paused = false;
  bool b_solar_soft_start = false;    // After a resume, restart at min current instead of 16A
  bool b_solar_was_charging = false;
  unsigned long ul_solar_charge_start_ms = 0;
  bool b_solar_deficit_timing = false;
  unsigned long ul_solar_deficit_start_ms = 0;
  unsigned long ul_solar_pause_start_ms = 0;
  bool b_solar_resume_timing = false;
  unsigned long ul_solar_resume_start_ms = 0;

  // Solar tracking control loop
  void applySolarTracking();
  void setSolarPaused(bool paused);
  void applyChargingGate(bool was_allowed);  // Act on a change of isChargeAllowed()
  void checkCpFault();                       // Report-only CP fault detection
  void publishSolarTelemetry(const char* action, float evse_power, float diff,
                             int pwm_current, int pwm_next);

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

  // Getters for Current measurements (PZEM)
  float getCurrent() const { return f_current; }
  float getVoltage() const { return f_voltage; }
  float getPower() const { return f_power; }
  float getEnergy() const { return f_energy; }

  // Setters for Control Pilot measurements
  void setCppValue(int value) { i_cpp_value = value; }
  void setCppMax(int max_val) { i_cpp_max = max_val; }
  void setCppMin(int min_val) { i_cpp_min = min_val; }
  void setChargeSpeed(int speed);

  // Charging authorization
  void setChargingEnabled(bool enabled);
  bool isChargingEnabled() const { return b_charging_enabled; }
  // Effective authorization: enabled by user/mode AND not paused by solar tracking
  bool isChargeAllowed() const { return b_charging_enabled && !b_solar_paused; }
  bool isSolarPaused() const { return b_solar_paused; }

  // Mode getters/setters
  void setChargeCheap(bool is_cheap) { m_is_charge_cheap = is_cheap; }
  bool isChargeCheap() const { return m_is_charge_cheap; }
  void setEvseMode(EvseMode mode);
  EvseMode getEvseMode() const { return m_active_mode; };

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
  // Hardware initialization moved from main.cpp
  void setupHardware();               // Initialize pins, PWM and safe states
  
  // Current Measurement Functions (PZEM)
  // PZEM sensor reading and publishing are handled at application level (main.cpp)
  
  // State Detection Functions
  bool isStateDiff();                 // Check if state has changed
  bool isFirstStateDiff();            // Check if this is first state change
  void startDiffTimer();              // Start state change timer
  bool isDiffSteady();                // Check if state change is stable

  // State Management Functions
  void setState();                    // Main state controller
  void setStateFault();               // Set fault state
  void enterSleep();                  // J1772 stop: CP to +12V then wait before opening relay

  // Relay Control Functions (OpenEVSE pattern)
  void chargingOn();                  // Close relay - start charging
  void chargingOff();                 // Open relay - stop charging

  // Control Functions
  void updateChargeSpeed();           // Update charging current
  void publishState();                // Publish state via MQTT
  void setMeasurements(float current, float voltage, float power, float energy);

  // Solar tracking control
  void setSolarTracking(bool enabled) { b_solar_tracking = enabled; }
  void setSetpointWatts(float watts);
  bool isSolarTracking() const { return b_solar_tracking; }
  float getSetpointWatts() const { return f_setpoint_watts; }
};
