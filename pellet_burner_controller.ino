// Pellet burner controller for Arduino Nano
// Features: state machine, safety checks, simple temperature control

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>

// --- Pin configuration ---
const uint8_t PIN_TEMP_SENSOR = 12;  // DS18B20 data pin (supply/return sensors)
const uint8_t PIN_ENCODER_A = 2;     // active LOW with pull-up
const uint8_t PIN_ENCODER_B = 3;     // active LOW with pull-up
const uint8_t PIN_ENCODER_BTN = 4;   // active LOW with pull-up
const uint8_t PIN_ROOM_THERMOSTAT = A3; // active LOW with pull-up

const uint8_t PIN_FAN = 5;       // PWM
const uint8_t PIN_AUGER = 6;     // relay/MOSFET
const uint8_t PIN_IGNITER = 7;   // relay/MOSFET
const uint8_t PIN_GRATE_MOTOR = A2; // relay/MOSFET for moving grates
const uint8_t PIN_PUMP_RELAY = A1;  // relay for heating pump
const uint8_t PIN_PHOTO_SENSOR = A7; // analog photodetector

const uint8_t LCD_I2C_ADDRESS = 0x27;

const uint8_t PIN_TC_SCK = 13;
const uint8_t PIN_TC_CS = 10;
const uint8_t PIN_TC_SO = 11;

// --- Control targets ---
float targetTempC = 70.0f;
float maxTempC = 95.0f;
float minFlameTempC = 40.0f; // below this means flame-out (water)
float maxFlameTempC = 900.0f;
float minFlameThermoC = 120.0f;
float maxExhaustTempC = 250.0f;
float pumpOnTempC = 45.0f;

uint8_t fanMinPwm = 77;  // 30%
uint8_t fanMaxPwm = 204; // 80%
uint8_t ignitionFanPwm = 102; // 40%

const int PHOTO_THRESHOLD = 300;

// --- Timings ---
unsigned long startTimeMs = 15000;
unsigned long ignitionTimeMs = 120000; // 2 minutes
unsigned long stabilizationTimeMs = 30000;
unsigned long cooldownTimeMs = 180000; // 3 minutes
unsigned long augerOnMs = 1200;
unsigned long augerOffMs = 5000;
unsigned long ignitionAugerOnMs = 600;
unsigned long ignitionAugerOffMs = 8000;
unsigned long grateCycleMs = 600000; // 10 minutes
unsigned long grateOnMs = 5000;      // 5 seconds
unsigned long pelletMissingTimeoutMs = 20000;
unsigned long flameTimeoutMs = 60000;

float modulationBandC = 5.0f;

// --- State machine ---
enum State {
  STATE_OFF,
  STATE_START,
  STATE_IGNITION,
  STATE_STABILIZATION,
  STATE_WORK,
  STATE_MODULATION,
  STATE_CLEANING,
  STATE_STOP,
  STATE_ALARM
};

State state = STATE_OFF;
unsigned long stateStartMs = 0;
unsigned long lastAugerToggleMs = 0;
bool augerOn = false;
unsigned long lastGrateCycleMs = 0;
unsigned long grateStartMs = 0;
bool grateOn = false;
unsigned long lastPelletSeenMs = 0;
unsigned long lastFlameSeenMs = 0;
uint8_t currentFanPwm = 0;
bool currentAugerOn = false;
bool currentIgniterOn = false;
bool currentGrateOn = false;
bool currentPumpOn = false;
unsigned long lastEncoderPressMs = 0;
bool encoderPressConsumed = false;

OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature tempSensors(&oneWire);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 20, 4);
MAX6675 thermocouple(PIN_TC_SCK, PIN_TC_CS, PIN_TC_SO);

enum MenuMode {
  MENU_STATUS,
  MENU_EDIT_TARGET,
  MENU_EDIT_MAX,
  MENU_EDIT_MIN_FLAME,
  MENU_EDIT_IGNITION,
  MENU_EDIT_COOLDOWN,
  MENU_EDIT_AUGER_ON,
  MENU_EDIT_AUGER_OFF
};

MenuMode menuMode = MENU_STATUS;
unsigned long lastLcdUpdateMs = 0;
int8_t lastEncoderState = 0;

// --- Helper functions ---
float readTemperatureC() {
  tempSensors.requestTemperatures();
  float tempC = tempSensors.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return -100.0f;
  }
  return tempC;
}

float readReturnTemperatureC() {
  tempSensors.requestTemperatures();
  float tempC = tempSensors.getTempCByIndex(1);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return -100.0f;
  }
  return tempC;
}

float readExhaustTemperatureC() {
  return thermocouple.readCelsius();
}

void setOutputs(bool fan, bool auger, bool igniter, bool grateMotor, bool pump, uint8_t fanPwm) {
  currentFanPwm = fan ? fanPwm : 0;
  currentAugerOn = auger;
  currentIgniterOn = igniter;
  currentGrateOn = grateMotor;
  currentPumpOn = pump;
  analogWrite(PIN_FAN, fan ? fanPwm : 0);
  digitalWrite(PIN_AUGER, auger ? HIGH : LOW);
  digitalWrite(PIN_IGNITER, igniter ? HIGH : LOW);
  digitalWrite(PIN_GRATE_MOTOR, grateMotor ? HIGH : LOW);
  digitalWrite(PIN_PUMP_RELAY, pump ? HIGH : LOW);
}

bool buttonPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

int readEncoderDelta() {
  int a = digitalRead(PIN_ENCODER_A);
  int b = digitalRead(PIN_ENCODER_B);
  int8_t state = (a << 1) | b;
  int delta = 0;

  if (state != lastEncoderState) {
    if ((lastEncoderState == 0b00 && state == 0b01) ||
        (lastEncoderState == 0b01 && state == 0b11) ||
        (lastEncoderState == 0b11 && state == 0b10) ||
        (lastEncoderState == 0b10 && state == 0b00)) {
      delta = 1;
    } else if ((lastEncoderState == 0b00 && state == 0b10) ||
               (lastEncoderState == 0b10 && state == 0b11) ||
               (lastEncoderState == 0b11 && state == 0b01) ||
               (lastEncoderState == 0b01 && state == 0b00)) {
      delta = -1;
    }
    lastEncoderState = state;
  }

  return delta;
}

void enterState(State next) {
  state = next;
  stateStartMs = millis();
  lastAugerToggleMs = millis();
  augerOn = false;
}

void updateAugerCycle(unsigned long onMs, unsigned long offMs) {
  unsigned long now = millis();
  unsigned long interval = augerOn ? onMs : offMs;
  if (now - lastAugerToggleMs >= interval) {
    augerOn = !augerOn;
    lastAugerToggleMs = now;
  }
}

bool updateGrateCycle() {
  unsigned long now = millis();
  bool started = false;
  if (!grateOn && now - lastGrateCycleMs >= grateCycleMs) {
    grateOn = true;
    grateStartMs = now;
    started = true;
  }

  if (grateOn && now - grateStartMs >= grateOnMs) {
    grateOn = false;
    lastGrateCycleMs = now;
  }
  return started;
}

const char *stateLabel(State current) {
  switch (current) {
    case STATE_OFF:
      return "OFF";
    case STATE_START:
      return "START";
    case STATE_IGNITION:
      return "IGNITION";
    case STATE_STABILIZATION:
      return "STABLE";
    case STATE_WORK:
      return "WORK";
    case STATE_MODULATION:
      return "MOD";
    case STATE_CLEANING:
      return "CLEAN";
    case STATE_STOP:
      return "STOP";
    case STATE_ALARM:
      return "ALARM";
  }
  return "UNKNOWN";
}

void showStatusScreen(float supplyTempC, float returnTempC, float exhaustTempC) {
  lcd.setCursor(0, 0);
  lcd.print("MODE: ");
  lcd.print(stateLabel(state));
  lcd.print("            ");
  lcd.setCursor(0, 1);
  lcd.print("Flame: ");
  lcd.print(exhaustTempC, 0);
  lcd.print("C        ");
  lcd.setCursor(0, 2);
  lcd.print("Flow:");
  lcd.print(supplyTempC, 0);
  lcd.print(" Ret:");
  lcd.print(returnTempC, 0);
  lcd.print("  ");
  lcd.setCursor(0, 3);
  int fanPercent = (currentFanPwm * 100) / 255;
  lcd.print("Fan:");
  lcd.print(fanPercent);
  lcd.print("% Pel:");
  lcd.print(currentAugerOn ? "ON " : "OFF");
}

void showEditScreen(const char *title, float value, const char *suffix) {
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.print("            ");
  lcd.setCursor(0, 1);
  lcd.print(value, 1);
  lcd.print(suffix);
  lcd.print("             ");
  lcd.setCursor(0, 2);
  lcd.print("Encoder=Adj     ");
  lcd.setCursor(0, 3);
  lcd.print("Enc=Next Stop=Exit");
}

void showEditScreenMs(const char *title, unsigned long valueMs) {
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.print("            ");
  lcd.setCursor(0, 1);
  lcd.print(valueMs / 1000);
  lcd.print(" sec       ");
  lcd.setCursor(0, 2);
  lcd.print("Encoder=Adj     ");
  lcd.setCursor(0, 3);
  lcd.print("Enc=Next Stop=Exit");
}

void handleMenu(bool encoderPressed, int encoderDelta) {
  if (menuMode == MENU_STATUS) {
    if (encoderPressed) {
      menuMode = MENU_EDIT_TARGET;
    }
    return;
  }

  if (encoderPressed) {
    switch (menuMode) {
      case MENU_EDIT_TARGET:
        menuMode = MENU_EDIT_MAX;
        break;
      case MENU_EDIT_MAX:
        menuMode = MENU_EDIT_MIN_FLAME;
        break;
      case MENU_EDIT_MIN_FLAME:
        menuMode = MENU_EDIT_IGNITION;
        break;
      case MENU_EDIT_IGNITION:
        menuMode = MENU_EDIT_COOLDOWN;
        break;
      case MENU_EDIT_COOLDOWN:
        menuMode = MENU_EDIT_AUGER_ON;
        break;
      case MENU_EDIT_AUGER_ON:
        menuMode = MENU_EDIT_AUGER_OFF;
        break;
      case MENU_EDIT_AUGER_OFF:
        menuMode = MENU_STATUS;
        break;
      default:
        break;
    }
    return;
  }

  if (encoderDelta != 0) {
    switch (menuMode) {
      case MENU_EDIT_TARGET:
        targetTempC = max(0.0f, targetTempC + encoderDelta);
        break;
      case MENU_EDIT_MAX:
        maxTempC = max(0.0f, maxTempC + encoderDelta);
        break;
      case MENU_EDIT_MIN_FLAME:
        minFlameTempC = max(0.0f, minFlameTempC + encoderDelta);
        break;
      case MENU_EDIT_IGNITION:
        ignitionTimeMs = max(10000UL, ignitionTimeMs + (encoderDelta * 10000L));
        break;
      case MENU_EDIT_COOLDOWN:
        cooldownTimeMs = max(10000UL, cooldownTimeMs + (encoderDelta * 10000L));
        break;
      case MENU_EDIT_AUGER_ON:
        augerOnMs = max(100UL, augerOnMs + (encoderDelta * 100L));
        break;
      case MENU_EDIT_AUGER_OFF:
        augerOffMs = max(100UL, augerOffMs + (encoderDelta * 100L));
        break;
      default:
        break;
    }
  }

  if (millis() - lastLcdUpdateMs < 250) {
    return;
  }
  lastLcdUpdateMs = millis();

  switch (menuMode) {
    case MENU_EDIT_TARGET:
      showEditScreen("Target temp", targetTempC, "C");
      break;
    case MENU_EDIT_MAX:
      showEditScreen("Max temp", maxTempC, "C");
      break;
    case MENU_EDIT_MIN_FLAME:
      showEditScreen("Min flame", minFlameTempC, "C");
      break;
    case MENU_EDIT_IGNITION:
      showEditScreenMs("Ignition time", ignitionTimeMs);
      break;
    case MENU_EDIT_COOLDOWN:
      showEditScreenMs("Cooldown time", cooldownTimeMs);
      break;
    case MENU_EDIT_AUGER_ON:
      showEditScreenMs("Auger ON", augerOnMs);
      break;
    case MENU_EDIT_AUGER_OFF:
      showEditScreenMs("Auger OFF", augerOffMs);
      break;
    default:
      break;
  }
}

void setup() {
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);
  pinMode(PIN_ROOM_THERMOSTAT, INPUT_PULLUP);

  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_AUGER, OUTPUT);
  pinMode(PIN_IGNITER, OUTPUT);
  pinMode(PIN_GRATE_MOTOR, OUTPUT);
  pinMode(PIN_PUMP_RELAY, OUTPUT);

  setOutputs(false, false, false, false, false, 0);

  tempSensors.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Pellet Burner");

  lastEncoderState = (digitalRead(PIN_ENCODER_A) << 1) | digitalRead(PIN_ENCODER_B);
  lastPelletSeenMs = millis();
  lastFlameSeenMs = millis();
}

void loop() {
  float tempC = readTemperatureC();
  float returnTempC = readReturnTemperatureC();
  float exhaustTempC = readExhaustTemperatureC();
  bool encoderPressed = buttonPressed(PIN_ENCODER_BTN);
  int encoderDelta = readEncoderDelta();
  bool roomThermostatActive = buttonPressed(PIN_ROOM_THERMOSTAT);
  int photoValue = analogRead(PIN_PHOTO_SENSOR);
  bool pelletDetected = photoValue > PHOTO_THRESHOLD;
  bool pumpShouldRun = tempC >= pumpOnTempC;
  bool encoderPressedEvent = false;
  bool encoderLongPress = false;

  if (encoderPressed) {
    if (lastEncoderPressMs == 0) {
      lastEncoderPressMs = millis();
      encoderPressConsumed = false;
    } else if (!encoderPressConsumed && millis() - lastEncoderPressMs >= 1500) {
      encoderLongPress = true;
      encoderPressConsumed = true;
    }
  } else if (lastEncoderPressMs != 0) {
    if (!encoderPressConsumed) {
      encoderPressedEvent = true;
    }
    lastEncoderPressMs = 0;
  }

  if (menuMode == MENU_STATUS) {
    if (encoderPressedEvent) {
      menuMode = MENU_EDIT_TARGET;
      lcd.clear();
    }

    if (millis() - lastLcdUpdateMs >= 500) {
      lastLcdUpdateMs = millis();
      showStatusScreen(tempC, returnTempC, exhaustTempC);
    }
  } else {
    handleMenu(encoderPressedEvent, encoderDelta);
    if (encoderLongPress) {
      menuMode = MENU_STATUS;
    }
  }

  if (pelletDetected) {
    lastPelletSeenMs = millis();
  }
  if (exhaustTempC >= minFlameThermoC) {
    lastFlameSeenMs = millis();
  }

  if (encoderLongPress && state != STATE_OFF) {
    enterState(STATE_STOP);
  }

  if (tempC >= maxTempC || exhaustTempC >= maxFlameTempC || exhaustTempC >= maxExhaustTempC) {
    enterState(STATE_ALARM);
  }

  switch (state) {
    case STATE_OFF:
      setOutputs(false, false, false, false, pumpShouldRun, 0);
      if (encoderPressedEvent && roomThermostatActive && tempC < targetTempC) {
        enterState(STATE_START);
      }
      break;

    case STATE_START: {
      updateAugerCycle(400, 4000);
      bool grateStarted = updateGrateCycle();
      setOutputs(true, augerOn, false, grateOn, pumpShouldRun, fanMinPwm);
      if (grateStarted) {
        enterState(STATE_CLEANING);
        break;
      }
      if (millis() - stateStartMs >= startTimeMs) {
        enterState(STATE_IGNITION);
      }
      break;
    }

    case STATE_IGNITION: {
      updateAugerCycle(ignitionAugerOnMs, ignitionAugerOffMs);
      bool grateStarted = updateGrateCycle();
      setOutputs(true, augerOn, true, grateOn, pumpShouldRun, ignitionFanPwm);
      if (grateStarted) {
        enterState(STATE_CLEANING);
        break;
      }

      if (exhaustTempC >= minFlameThermoC || pelletDetected) {
        enterState(STATE_STABILIZATION);
      }

      if (millis() - stateStartMs >= ignitionTimeMs ||
          millis() - lastPelletSeenMs >= pelletMissingTimeoutMs ||
          millis() - lastFlameSeenMs >= flameTimeoutMs) {
        enterState(STATE_ALARM);
      }
      break;
    }

    case STATE_STABILIZATION: {
      updateAugerCycle(augerOnMs, augerOffMs);
      bool grateStarted = updateGrateCycle();
      setOutputs(true, augerOn, false, grateOn, pumpShouldRun, fanMinPwm);
      if (grateStarted) {
        enterState(STATE_CLEANING);
        break;
      }

      if (millis() - stateStartMs >= stabilizationTimeMs) {
        enterState(STATE_WORK);
      }

      if (millis() - lastFlameSeenMs >= flameTimeoutMs) {
        enterState(STATE_ALARM);
      }
      break;
    }

    case STATE_WORK: {
      updateAugerCycle(augerOnMs, augerOffMs);
      bool grateStarted = updateGrateCycle();
      uint8_t fanPwm = fanMaxPwm;
      setOutputs(true, augerOn, false, grateOn, pumpShouldRun, fanPwm);
      if (grateStarted) {
        enterState(STATE_CLEANING);
        break;
      }

      if (!roomThermostatActive) {
        enterState(STATE_STOP);
      } else if (tempC >= targetTempC - modulationBandC) {
        enterState(STATE_MODULATION);
      }

      if (millis() - lastFlameSeenMs >= flameTimeoutMs) {
        enterState(STATE_ALARM);
      }
      break;
    }

    case STATE_MODULATION: {
      updateAugerCycle(augerOnMs / 2, augerOffMs * 2);
      bool grateStarted = updateGrateCycle();
      setOutputs(true, augerOn, false, grateOn, pumpShouldRun, fanMinPwm);
      if (grateStarted) {
        enterState(STATE_CLEANING);
        break;
      }

      if (!roomThermostatActive) {
        enterState(STATE_STOP);
      } else if (tempC < targetTempC - modulationBandC) {
        enterState(STATE_WORK);
      }

      if (millis() - lastFlameSeenMs >= flameTimeoutMs) {
        enterState(STATE_ALARM);
      }
      break;
    }

    case STATE_CLEANING:
      updateGrateCycle();
      setOutputs(true, false, false, grateOn, pumpShouldRun, fanMinPwm);
      if (!grateOn) {
        enterState(roomThermostatActive ? STATE_WORK : STATE_STOP);
      }
      break;

    case STATE_STOP:
      updateGrateCycle();
      setOutputs(true, false, false, grateOn, pumpShouldRun, fanMinPwm);
      if (millis() - stateStartMs >= cooldownTimeMs) {
        enterState(STATE_OFF);
      }
      break;

    case STATE_ALARM:
      setOutputs(true, false, false, false, pumpShouldRun, 255);
      if (encoderPressedEvent) {
        enterState(STATE_STOP);
      }
      break;
  }

  delay(100);
}
