// Pellet burner controller for Arduino Nano
// Features: state machine, safety checks, simple temperature control

#include <Arduino.h>

// --- Pin configuration ---
const uint8_t PIN_TEMP_SENSOR = A0;
const uint8_t PIN_START_BUTTON = 2;  // active LOW with pull-up
const uint8_t PIN_STOP_BUTTON = 3;   // active LOW with pull-up

const uint8_t PIN_FAN = 5;       // PWM
const uint8_t PIN_AUGER = 6;     // relay/MOSFET
const uint8_t PIN_IGNITER = 7;   // relay/MOSFET

// --- Temperature sensor (10k NTC) configuration ---
const float THERMISTOR_NOMINAL = 10000.0f; // resistance at 25C
const float TEMPERATURE_NOMINAL = 25.0f;   // in Celsius
const float B_COEFFICIENT = 3950.0f;       // beta value
const float SERIES_RESISTOR = 10000.0f;    // series resistor value

// --- Control targets ---
const float TARGET_TEMP_C = 70.0f;
const float MAX_TEMP_C = 95.0f;
const float MIN_FLAME_TEMP_C = 40.0f; // below this means flame-out

// --- Timings ---
const unsigned long IGNITION_TIME_MS = 300000; // 5 minutes
const unsigned long COOLDOWN_TIME_MS = 180000; // 3 minutes
const unsigned long AUGER_ON_MS = 800;
const unsigned long AUGER_OFF_MS = 3200;

// --- State machine ---
enum State {
  STATE_IDLE,
  STATE_IGNITION,
  STATE_RUN,
  STATE_COOLDOWN,
  STATE_FAULT
};

State state = STATE_IDLE;
unsigned long stateStartMs = 0;
unsigned long lastAugerToggleMs = 0;
bool augerOn = false;

// --- Helper functions ---
float readTemperatureC() {
  int adc = analogRead(PIN_TEMP_SENSOR);
  if (adc <= 0) {
    return -100.0f;
  }

  float resistance = SERIES_RESISTOR / ((1023.0f / adc) - 1.0f);
  float steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;
  return steinhart;
}

void setOutputs(bool fan, bool auger, bool igniter, uint8_t fanPwm) {
  analogWrite(PIN_FAN, fan ? fanPwm : 0);
  digitalWrite(PIN_AUGER, auger ? HIGH : LOW);
  digitalWrite(PIN_IGNITER, igniter ? HIGH : LOW);
}

bool buttonPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void enterState(State next) {
  state = next;
  stateStartMs = millis();
  lastAugerToggleMs = millis();
  augerOn = false;
}

void updateAugerCycle() {
  unsigned long now = millis();
  unsigned long interval = augerOn ? AUGER_ON_MS : AUGER_OFF_MS;
  if (now - lastAugerToggleMs >= interval) {
    augerOn = !augerOn;
    lastAugerToggleMs = now;
  }
}

void setup() {
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);

  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_AUGER, OUTPUT);
  pinMode(PIN_IGNITER, OUTPUT);

  setOutputs(false, false, false, 0);
}

void loop() {
  float tempC = readTemperatureC();
  bool startPressed = buttonPressed(PIN_START_BUTTON);
  bool stopPressed = buttonPressed(PIN_STOP_BUTTON);

  if (stopPressed && state != STATE_IDLE) {
    enterState(STATE_COOLDOWN);
  }

  if (tempC >= MAX_TEMP_C) {
    enterState(STATE_FAULT);
  }

  switch (state) {
    case STATE_IDLE:
      setOutputs(false, false, false, 0);
      if (startPressed) {
        enterState(STATE_IGNITION);
      }
      break;

    case STATE_IGNITION: {
      updateAugerCycle();
      setOutputs(true, augerOn, true, 200);

      if (tempC >= MIN_FLAME_TEMP_C) {
        enterState(STATE_RUN);
      }

      if (millis() - stateStartMs >= IGNITION_TIME_MS) {
        enterState(STATE_FAULT);
      }
      break;
    }

    case STATE_RUN: {
      updateAugerCycle();
      bool needHeat = tempC < TARGET_TEMP_C;
      uint8_t fanPwm = needHeat ? 220 : 160;

      setOutputs(true, needHeat ? augerOn : false, false, fanPwm);

      if (tempC < MIN_FLAME_TEMP_C) {
        enterState(STATE_FAULT);
      }
      break;
    }

    case STATE_COOLDOWN:
      setOutputs(true, false, false, 160);
      if (millis() - stateStartMs >= COOLDOWN_TIME_MS) {
        enterState(STATE_IDLE);
      }
      break;

    case STATE_FAULT:
      setOutputs(true, false, false, 255);
      if (startPressed) {
        enterState(STATE_COOLDOWN);
      }
      break;
  }

  delay(100);
}
