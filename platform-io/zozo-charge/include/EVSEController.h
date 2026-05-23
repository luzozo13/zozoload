// EVSEController.h - Class declaration separated from params
#pragma once

#include "params.h" // pull in pin/constants

// Forward declarations
class MqttHandler;

class EVSEController {
private:
  // MQTT Handler for communication
  MqttHandler* m_mqttHandler;         // Injected dependency

  // Charging authorization
  bool b_charging_enabled = true;   // Default-on: charge immediately when car connects
  unsigned long ul_sleep_start_ms = 0;

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
  // Allow application to update measurement storage
  void setMeasurements(float current, float voltage, float power, float energy);
};
