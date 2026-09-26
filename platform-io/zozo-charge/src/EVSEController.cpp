//-- EVSE Controller Implementation --//
// ESP32 EVSE Controller - Class Implementation
// Following OpenEVSE architectural patterns

#include "EVSEController.h"
#include "MqttHandler.h"
#include "mqtt_config.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <PZEM004Tv30.h>

//-- EVSEController Class Implementation --//

EVSEController::EVSEController() : m_mqttHandler(nullptr) {
  // Default constructor - no MQTT handler
}

EVSEController::EVSEController(MqttHandler& mqttHandler) : m_mqttHandler(&mqttHandler) {
  // Constructor with dependency injection
}

void EVSEController::setMqttHandler(MqttHandler& mqttHandler) {
  m_mqttHandler = &mqttHandler;
}

//-- Pilot Signal Management --//

int EVSEController::readPilot() {
  i_cpp_max = 0;
  i_cpp_min = 4095;

  unsigned long ul_start_time = millis();
  while (millis() - ul_start_time < I_DELAY_PILOT_LOOP) { // Sample for I_DELAY_PILOT_LOOP ms
    i_cpp_value = analogRead(PILOT_READ);
    if (i_cpp_max < i_cpp_value) { i_cpp_max = i_cpp_value; }
    if (i_cpp_min > i_cpp_value) { i_cpp_min = i_cpp_value; }
  }

  if (i_cpp_max >= TH_AB) {
    i_state_meas = STATE_A;
  } else if (i_cpp_max >= TH_BC) {
    i_state_meas = STATE_B;
  } else if (i_cpp_max >= TH_CD) {
    i_state_meas = STATE_C;
  }
  return i_state_meas;
}

//-- Charge Speed Control --//

void EVSEController::setChargeSpeed(int speed) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  
  i_charge_speed = speed;
  
  // Apply immediately if currently charging (State C)
  if (i_state_current == STATE_C) {
    ledcWrite(CH_CP_CTRL, i_charge_speed);
  }
}

void EVSEController::setEvseMode(EvseMode mode) {
  m_active_mode = mode;

  switch (m_active_mode) {
    case MODE_BOOST:
      b_solar_tracking = false;
      setChargeSpeed(CP_AMP_16); // Max charge speed for boost mode
      setChargingEnabled(true);   
      break;

    case MODE_SOLAR:
      b_solar_tracking = true;
      setChargingEnabled(true);   
      break;

    case MODE_CHEAP:
      b_solar_tracking = false;
      setChargingEnabled(false);  
      break;
  }
}

void EVSEController::setSolarWatts(float watts) {
  f_solar_watts = watts;
  ul_solar_rx_ms = millis();
  b_solar_rx_seen = true;
}

void EVSEController::setChargingEnabled(bool enabled) {
  if (enabled == b_charging_enabled) return;
  b_charging_enabled = enabled;

  if (!enabled) {
    if (i_state_current == STATE_C) {
      enterSleep();
    } else if (i_state_current == STATE_B) {
      ledcWrite(CH_CP_CTRL, CP_12P);
    }
    return;
  }

  if (i_state_current == STATE_SLEEPING) {
    ledcWrite(CH_CP_CTRL, i_charge_speed);
    i_state_current = STATE_B;
  } else if (i_state_current == STATE_B) {
    ledcWrite(CH_CP_CTRL, i_charge_speed);
  } else if (i_state_current == STATE_C) {
    ledcWrite(CH_CP_CTRL, i_charge_speed);
    chargingOn();
  }
}

void EVSEController::enterSleep() {
  // J1772 stop sequence: signal "not ready" to EV before cutting power
  ledcWrite(CH_CP_CTRL, CP_12P);
  ul_sleep_start_ms = millis();
  i_state_current = STATE_SLEEPING;
}

void EVSEController::updateChargeSpeed() {
    // TODO: button-driven speed selection removed
    // Placeholder for future local input handling (e.g. rotary encoder, display)
}

//-- Charging Control --//

void EVSEController::chargingOn() {
  digitalWrite(REL_CTRL, HIGH);
}

void EVSEController::chargingOff() {
  digitalWrite(REL_CTRL, LOW);
}

//-- State Management --//

void EVSEController::setStateFault(){
  i_state_current = STATE_FAULT;
  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, HIGH);
}

void EVSEController::setState() {
  switch (i_state_meas) {
    case STATE_A:
      chargingOff();
      ledcWrite(CH_CP_CTRL, CP_12P);
      i_state_current = STATE_A;
      break;
    case STATE_B:
      chargingOff();
      ledcWrite(CH_CP_CTRL, b_charging_enabled ? i_charge_speed : CP_12P);
      i_state_current = STATE_B;
      break;
    case STATE_C:
      if (b_charging_enabled) {
        ledcWrite(CH_CP_CTRL, i_charge_speed);
        chargingOn();
      } else {
        ledcWrite(CH_CP_CTRL, CP_12P);
        chargingOff();
      }
      i_state_current = STATE_C;
      break;
    case STATE_FAULT:
    default:
      setStateFault();
      break;
  }
}

//-- State Transition Logic (OpenEVSE Pattern) --//

bool EVSEController::isStateDiff() {
  return i_state_meas != i_state_current;
}

bool EVSEController::isFirstStateDiff() {
  return i_state_meas != i_state_previous;
}

void EVSEController::startDiffTimer() {
  i_state_previous = i_state_meas;
  if (i_state_meas != i_tmp_state) {
    ul_tmp_state_start = millis();
    i_tmp_state = i_state_meas;
  }
}

bool EVSEController::isDiffSteady() {
  unsigned long ul_curms = millis();
  
  if (i_state_meas == i_tmp_state) {
    // State hasn't changed, check if enough time has passed
    unsigned long ul_delay_time = (i_state_meas == STATE_A) ? DELAY_STATE_TRANSITION_A : DELAY_STATE_TRANSITION;
    
    if ((ul_curms - ul_tmp_state_start) >= ul_delay_time) {
      return true; // State is stable
    }
  } else {
    // State changed again, reset timer
    ul_tmp_state_start = ul_curms;
    i_tmp_state = i_state_meas;
  }
  
  return false; // Not stable yet
}

//-- Main Update Method --//
void EVSEController::update() {
  if (i_state_current == STATE_SLEEPING) {
    readPilot();
    bool ev_released = (i_cpp_max >= TH_BC);
    bool timed_out = (millis() - ul_sleep_start_ms) >= SLEEP_RELAY_OPEN_TIMEOUT_MS;
    if (ev_released || timed_out) {
      chargingOff();
      i_state_current = STATE_B;
      if (m_mqttHandler) {
        m_mqttHandler->publishTransition('S', ev_released ? 'B' : 'T');
      }
    }
    return;
  }

  updateChargeSpeed();
  readPilot();

  if (isStateDiff()) {
    if (isFirstStateDiff()) {
      startDiffTimer();
    }
    else {
      if (isDiffSteady()) {
        // Store previous state as a character
        char c_prev_state = 'U';
        if (i_state_current == STATE_A) c_prev_state = 'A';
        else if (i_state_current == STATE_B) c_prev_state = 'B';
        else if (i_state_current == STATE_C) c_prev_state = 'C';
        else c_prev_state = 'U';

        setState();

        char c_new_state = 'U';
        if (i_state_current == STATE_A) c_new_state = 'A';
        else if (i_state_current == STATE_B) c_new_state = 'B';
        else if (i_state_current == STATE_C) c_new_state = 'C';
        else c_new_state = 'U';

        // Publish the transition (e.g. "B->C")
        if (m_mqttHandler) {
          m_mqttHandler->publishTransition(c_prev_state, c_new_state);
        }
      }
    }
  }
  
  if (millis() - ul_last_state_publish > UL_STATE_PUBLISH_INTERVAL) {
    publishState();
    // PZEM telemetry is read/published at application level (main.cpp)
    ul_last_state_publish = millis();
  }

  if (m_active_mode == MODE_SOLAR) {
    applySolarTracking();
  }
  else if (m_active_mode == MODE_CHEAP) {
    if(m_is_charge_cheap) {
      setChargeSpeed(CP_AMP_16); // Max charge speed for cheap mode
      setChargingEnabled(true);
    }
    else {
      setChargingEnabled(false);
    }
  }
}

//-- MQTT Publishing --//

void EVSEController::publishState() {
  if (!m_mqttHandler) return;  // No MQTT handler available
  
  // Convert state to character
  const char* sz_state_char = "Unknown";
  if (i_state_current == STATE_A)            sz_state_char = "A";
  else if (i_state_current == STATE_B)       sz_state_char = "B";
  else if (i_state_current == STATE_C)       sz_state_char = "C";
  else if (i_state_current == STATE_SLEEPING) sz_state_char = "S";

  // Use MqttHandler utility method
  m_mqttHandler->publishState(sz_state_char, i_cpp_max, i_cpp_min, b_charging_enabled);
}

//-- Solar Tracking Control Loop --//

void EVSEController::applySolarTracking() {
  if (!b_solar_tracking) return;

  int state = i_state_current;
  bool charging_effective = (state == STATE_C) && b_charging_enabled;

  // Do not adapt solar tracking setpoint unless charging is effectively active.
  // Before charging starts, keep a deterministic pre-charge setpoint at 16A.
  if (!charging_effective) {
    if (i_charge_speed != CP_AMP_16) {
      setChargeSpeed(CP_AMP_16);
      if (m_mqttHandler) m_mqttHandler->publishPwm();
    }
    return;
  }

  unsigned long now = millis();
  if (now - ul_solar_last_step_ms < SOLAR_LOOP_MS) return;
  ul_solar_last_step_ms = now;

  float evse_power = f_power;
  float diff = f_solar_watts - evse_power;
  int pwm_current = i_charge_speed;
  int pwm_next = pwm_current;
  const char* action = "hold";
  long solar_age_s = b_solar_rx_seen ? (long)((now - ul_solar_rx_ms) / 1000UL) : -1;

  if (!b_solar_rx_seen || now - ul_solar_rx_ms > SOLAR_STALE_MS) {
    pwm_next = SOLAR_PWM_MAX;
    action = "stale";
  } else if (diff > SOLAR_DEADBAND_W) {
    pwm_next -= SOLAR_STEP_DOWN;   // surplus: ramp up current (lower PWM)
    action = "down";
  } else if (diff < -SOLAR_DEADBAND_W) {
    pwm_next += SOLAR_STEP_UP;     // deficit: back off current (higher PWM)
    action = "up";
  }

  if (pwm_next < SOLAR_PWM_MIN) pwm_next = SOLAR_PWM_MIN;
  if (pwm_next > SOLAR_PWM_MAX) pwm_next = SOLAR_PWM_MAX;

  if (pwm_next != pwm_current) {
    setChargeSpeed(pwm_next);
    if (m_mqttHandler) m_mqttHandler->publishPwm();
  }

  // Publish debug telemetry every cycle
  if (m_mqttHandler && m_mqttHandler->isConnected()) {
    char ts[24];
    m_mqttHandler->getTimestamp(ts, sizeof(ts));
    char sz_msg[192];
    snprintf(sz_msg, sizeof(sz_msg),
      "{\"ts\":\"%s\",\"solar_w\":%.1f,\"solar_age_s\":%ld,\"evse_w\":%.1f,\"diff_w\":%.1f,\"pwm\":%d,\"pwm_next\":%d,\"action\":\"%s\"}",
      ts, f_solar_watts, solar_age_s, evse_power, diff, pwm_current, pwm_next, action);
    m_mqttHandler->publishSolarTracking(sz_msg);
  }
}

//-- Hardware setup (moved from main.cpp) --//
void EVSEController::setupHardware() {
  // Initialize pins
  pinMode(PILOT_READ, INPUT);
  pinMode(REL_CTRL, OUTPUT);
  pinMode(FLT_CTRL, OUTPUT);
  pinMode(B_R, INPUT_PULLUP);
  pinMode(B_G, INPUT_PULLUP);
  pinMode(B_B, INPUT_PULLUP);

  // Initialize PWM for Control Pilot
  ledcSetup(CH_CP_CTRL, F_PWM, PWM_RES);
  ledcAttachPin(CP_CTRL, CH_CP_CTRL);

  // Initialize in safe state
  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, LOW);
  ledcWrite(CH_CP_CTRL, CP_12P);
}

//-- Current Measurement (PZEM) --//





void EVSEController::setMeasurements(float current, float voltage, float power, float energy) {
  f_current = current;
  f_voltage = voltage;
  f_power = power;
  f_energy = energy;
}
