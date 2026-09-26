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
#if FAULT_DETECT_ENFORCE
  else {
    i_state_meas = STATE_FAULT;       // State D/E: CP shorted or pulled below ~6V
  }
#endif
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
  setSolarPaused(false);   // Any mode (re)selection cancels a solar pause

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
  bool was_allowed = isChargeAllowed();
  b_charging_enabled = enabled;
  applyChargingGate(was_allowed);
}

void EVSEController::setSolarPaused(bool paused) {
  if (paused == b_solar_paused) return;
  bool was_allowed = isChargeAllowed();
  b_solar_paused = paused;
  applyChargingGate(was_allowed);
}

void EVSEController::applyChargingGate(bool was_allowed) {
  bool allowed = isChargeAllowed();
  if (allowed == was_allowed) return;

  if (!allowed) {
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
  ledcWrite(CH_CP_CTRL, CP_12P);    // Steady +12V: no charge offer, CP stays readable
  ul_fault_start_ms = millis();
}

// Report-only detection (FAULT_DETECT_ENFORCE == 0): publish once per episode when
// the CP stays below TH_CD for the standard debounce time. No state change.
void EVSEController::checkCpFault() {
#if !FAULT_DETECT_ENFORCE
  if (i_cpp_max >= TH_CD) {
    b_cp_low = false;
    return;
  }
  unsigned long now = millis();
  if (!b_cp_low) {
    b_cp_low = true;
    b_cp_low_reported = false;
    ul_cp_low_start_ms = now;
  } else if (!b_cp_low_reported && now - ul_cp_low_start_ms >= DELAY_STATE_TRANSITION) {
    b_cp_low_reported = true;
    if (m_mqttHandler) m_mqttHandler->publishEvent("cp_fault_seen", i_cpp_max, i_cpp_min);
  }
#endif
}

void EVSEController::setState() {
  switch (i_state_meas) {
    case STATE_A:
      digitalWrite(FLT_CTRL, LOW);
      chargingOff();
      ledcWrite(CH_CP_CTRL, CP_12P);
      i_state_current = STATE_A;
      break;
    case STATE_B:
      digitalWrite(FLT_CTRL, LOW);
      chargingOff();
      ledcWrite(CH_CP_CTRL, isChargeAllowed() ? i_charge_speed : CP_12P);
      i_state_current = STATE_B;
      break;
    case STATE_C:
      digitalWrite(FLT_CTRL, LOW);
      if (isChargeAllowed()) {
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
  checkCpFault();

  // Stay in fault at least FAULT_RETRY_MS before a normal reading may clear it
  bool fault_hold = (i_state_current == STATE_FAULT) &&
                    (millis() - ul_fault_start_ms < FAULT_RETRY_MS);

  if (!fault_hold && isStateDiff()) {
    if (isFirstStateDiff()) {
      startDiffTimer();
    }
    else {
      if (isDiffSteady()) {
        char c_prev_state = stateChar(i_state_current);
        setState();
        char c_new_state = stateChar(i_state_current);

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
  
  char sz_state[2] = { stateChar(i_state_current), '\0' };
  m_mqttHandler->publishState(sz_state, i_cpp_max, i_cpp_min, b_charging_enabled);
}

//-- Solar Tracking Control Loop --//

void EVSEController::applySolarTracking() {
  if (!b_solar_tracking) {
    setSolarPaused(false);
    return;
  }

  int state = i_state_current;
  unsigned long now = millis();
  bool solar_fresh = b_solar_rx_seen && (now - ul_solar_rx_ms <= SOLAR_STALE_MS);

  // Car unplugged: a new session starts fresh (no pause, normal 16A pre-charge)
  if (state == STATE_A) {
    setSolarPaused(false);
    b_solar_soft_start = false;
  }

  // Paused: wait for the minimum pause time AND a steady surplus before resuming
  if (b_solar_paused) {
    b_solar_was_charging = false;
    if (now - ul_solar_last_step_ms < SOLAR_LOOP_MS) return;
    ul_solar_last_step_ms = now;

    if (solar_fresh && f_solar_watts >= SOLAR_RESUME_W) {
      if (!b_solar_resume_timing) {
        b_solar_resume_timing = true;
        ul_solar_resume_start_ms = now;
      }
    } else {
      b_solar_resume_timing = false;   // Any dip restarts the resume timer
    }

    bool min_pause_done = (now - ul_solar_pause_start_ms) >= SOLAR_MIN_PAUSE_MS;
    bool surplus_steady = b_solar_resume_timing &&
                          (now - ul_solar_resume_start_ms) >= SOLAR_RESUME_AFTER_MS;
    const char* action = "paused";
    if (min_pause_done && surplus_steady) {
      b_solar_soft_start = true;       // Restart at min current, then ramp up
      setChargeSpeed(SOLAR_PWM_MAX);
      setSolarPaused(false);
      if (m_mqttHandler) m_mqttHandler->publishPwm();
      action = "resume";
    }
    publishSolarTelemetry(action, 0.0f, f_solar_watts, i_charge_speed, i_charge_speed);
    return;
  }

  bool charging_effective = (state == STATE_C) && isChargeAllowed();

  // Do not adapt solar tracking setpoint unless charging is effectively active.
  // Before charging starts, keep a deterministic pre-charge setpoint: 16A, or
  // min current right after a solar resume.
  if (!charging_effective) {
    b_solar_was_charging = false;
    int pre_charge = b_solar_soft_start ? SOLAR_PWM_MAX : CP_AMP_16;
    if (i_charge_speed != pre_charge) {
      setChargeSpeed(pre_charge);
      if (m_mqttHandler) m_mqttHandler->publishPwm();
    }
    return;
  }

  if (!b_solar_was_charging) {
    // Charging just (re)started: arm the minimum on-time
    b_solar_was_charging = true;
    b_solar_soft_start = false;
    b_solar_deficit_timing = false;
    ul_solar_charge_start_ms = now;
  }

  if (now - ul_solar_last_step_ms < SOLAR_LOOP_MS) return;
  ul_solar_last_step_ms = now;

  float evse_power = f_power;
  float diff = f_solar_watts - evse_power;
  int pwm_current = i_charge_speed;
  int pwm_next = pwm_current;
  const char* action = "hold";

  if (!solar_fresh) {
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

  // Deficit at minimum current (or no solar data): time it, pause when it lasts
  bool deficit_at_min = (pwm_current >= SOLAR_PWM_MAX) && (!solar_fresh || diff < -SOLAR_DEADBAND_W);
  if (deficit_at_min) {
    if (!b_solar_deficit_timing) {
      b_solar_deficit_timing = true;
      ul_solar_deficit_start_ms = now;
    }
  } else {
    b_solar_deficit_timing = false;
  }

  if (b_solar_deficit_timing &&
      (now - ul_solar_deficit_start_ms) >= SOLAR_PAUSE_AFTER_MS &&
      (now - ul_solar_charge_start_ms) >= SOLAR_MIN_ON_MS) {
    b_solar_deficit_timing = false;
    b_solar_resume_timing = false;
    ul_solar_pause_start_ms = now;
    setSolarPaused(true);            // J1772 stop via enterSleep()
    if (m_mqttHandler) m_mqttHandler->publishPwm();
    action = "pause";
  }

  publishSolarTelemetry(action, evse_power, diff, pwm_current, pwm_next);
}

void EVSEController::publishSolarTelemetry(const char* action, float evse_power, float diff,
                                           int pwm_current, int pwm_next) {
  if (!m_mqttHandler || !m_mqttHandler->isConnected()) return;
  unsigned long now = millis();
  long solar_age_s = b_solar_rx_seen ? (long)((now - ul_solar_rx_ms) / 1000UL) : -1;
  long deficit_s = b_solar_deficit_timing ? (long)((now - ul_solar_deficit_start_ms) / 1000UL) : -1;
  long paused_s  = b_solar_paused ? (long)((now - ul_solar_pause_start_ms) / 1000UL) : -1;
  long resume_s  = (b_solar_paused && b_solar_resume_timing)
                   ? (long)((now - ul_solar_resume_start_ms) / 1000UL) : -1;
  char ts[24];
  m_mqttHandler->getTimestamp(ts, sizeof(ts));
  char sz_msg[300];
  snprintf(sz_msg, sizeof(sz_msg),
    "{\"ts\":\"%s\",\"solar_w\":%.1f,\"solar_age_s\":%ld,\"evse_w\":%.1f,\"diff_w\":%.1f,\"pwm\":%d,\"pwm_next\":%d,\"action\":\"%s\",\"deficit_s\":%ld,\"paused_s\":%ld,\"resume_s\":%ld}",
    ts, f_solar_watts, solar_age_s, evse_power, diff, pwm_current, pwm_next, action,
    deficit_s, paused_s, resume_s);
  m_mqttHandler->publishSolarTracking(sz_msg);
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
