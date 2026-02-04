// Pellet burner controller for Arduino Nano
// Features: state machine, safety checks, simple temperature control

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>

// --- Pin configuration ---
const uint8_t PIN_TEMP_SENSOR = 12;  // DS18B20 data pin (supply/return sensors)
const uint8_t PIN_START_BUTTON = 2;  // active LOW with pull-up
const uint8_t PIN_STOP_BUTTON = 3;   // active LOW with pull-up
const uint8_t PIN_ENCODER_A = 8;     // active LOW with pull-up
const uint8_t PIN_ENCODER_B = 9;     // active LOW with pull-up
const uint8_t PIN_ENCODER_BTN = 4;   // active LOW with pull-up
const uint8_t PIN_ROOM_THERMOSTAT = A3; // active LOW with pull-up

const uint8_t PIN_FAN = 5;       // PWM
const uint8_t PIN_AUGER = 6;     // relay/MOSFET
const uint8_t PIN_IGNITER = 7;   // relay/MOSFET
const uint8_t PIN_GRATE_MOTOR = A2; // relay/MOSFET for moving grates
const uint8_t PIN_PHOTO_SENSOR = A7; // analog photodetector

const uint8_t LCD_I2C_ADDRESS = 0x27;

const uint8_t PIN_TC_SCK = 13;
const uint8_t PIN_TC_CS = 10;
const uint8_t PIN_TC_SO = 11;

// --- Control targets ---
float targetTempC = 70.0f;
float maxTempC = 95.0f;
float minFlameTempC = 40.0f; // below this means flame-out
float maxExhaustTempC = 250.0f;

const int PHOTO_THRESHOLD = 300;

// --- Timings ---
unsigned long ignitionTimeMs = 300000; // 5 minutes
unsigned long cooldownTimeMs = 180000; // 3 minutes
unsigned long augerOnMs = 800;
unsigned long augerOffMs = 3200;
unsigned long grateCycleMs = 600000; // 10 minutes
unsigned long grateOnMs = 5000;      // 5 seconds
unsigned long pelletMissingTimeoutMs = 20000;

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
unsigned long lastGrateCycleMs = 0;
unsigned long grateStartMs = 0;
bool grateOn = false;
unsigned long lastPelletSeenMs = 0;

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

void setOutputs(bool fan, bool auger, bool igniter, bool grateMotor, uint8_t fanPwm) {
  analogWrite(PIN_FAN, fan ? fanPwm : 0);
  digitalWrite(PIN_AUGER, auger ? HIGH : LOW);
  digitalWrite(PIN_IGNITER, igniter ? HIGH : LOW);
  digitalWrite(PIN_GRATE_MOTOR, grateMotor ? HIGH : LOW);
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

void updateAugerCycle() {
  unsigned long now = millis();
  unsigned long interval = augerOn ? augerOnMs : augerOffMs;
  if (now - lastAugerToggleMs >= interval) {
    augerOn = !augerOn;
    lastAugerToggleMs = now;
  }
}

void updateGrateCycle() {
  unsigned long now = millis();
  if (!grateOn && now - lastGrateCycleMs >= grateCycleMs) {
    grateOn = true;
    grateStartMs = now;
  }

  if (grateOn && now - grateStartMs >= grateOnMs) {
    grateOn = false;
    lastGrateCycleMs = now;
  }
}

const char *stateLabel(State current) {
  switch (current) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_IGNITION:
      return "IGNITION";
    case STATE_RUN:
      return "RUN";
    case STATE_COOLDOWN:
      return "COOLDOWN";
    case STATE_FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

void showStatusScreen(float supplyTempC, float returnTempC, float exhaustTempC) {
  lcd.setCursor(0, 0);
  lcd.print("State: ");
  lcd.print(stateLabel(state));
  lcd.print("        ");
  lcd.setCursor(0, 1);
  lcd.print("Sup:");
  lcd.print(supplyTempC, 1);
  lcd.print("C Ret:");
  lcd.print(returnTempC, 1);
  lcd.setCursor(0, 2);
  lcd.print("Exh:");
  lcd.print(exhaustTempC, 0);
  lcd.print("C Tgt:");
  lcd.print(targetTempC, 0);
  lcd.setCursor(0, 3);
  lcd.print("Start/Stop=Menu   ");
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

void handleMenu(bool startPressed, bool stopPressed, bool encoderPressed, int encoderDelta) {
  if (menuMode == MENU_STATUS) {
    if (startPressed) {
      menuMode = MENU_EDIT_TARGET;
    } else if (stopPressed) {
      menuMode = MENU_EDIT_TARGET;
    }
    return;
  }

  if (stopPressed) {
    menuMode = MENU_STATUS;
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
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);
  pinMode(PIN_ROOM_THERMOSTAT, INPUT_PULLUP);

  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_AUGER, OUTPUT);
  pinMode(PIN_IGNITER, OUTPUT);
  pinMode(PIN_GRATE_MOTOR, OUTPUT);

  setOutputs(false, false, false, false, 0);

  tempSensors.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Pellet Burner");

  lastEncoderState = (digitalRead(PIN_ENCODER_A) << 1) | digitalRead(PIN_ENCODER_B);
  lastPelletSeenMs = millis();
}

void loop() {
  float tempC = readTemperatureC();
  float returnTempC = readReturnTemperatureC();
  float exhaustTempC = readExhaustTemperatureC();
  bool startPressed = buttonPressed(PIN_START_BUTTON);
  bool stopPressed = buttonPressed(PIN_STOP_BUTTON);
  bool encoderPressed = buttonPressed(PIN_ENCODER_BTN);
  int encoderDelta = readEncoderDelta();
  bool roomThermostatActive = buttonPressed(PIN_ROOM_THERMOSTAT);
  int photoValue = analogRead(PIN_PHOTO_SENSOR);
  bool pelletDetected = photoValue > PHOTO_THRESHOLD;

  if (menuMode == MENU_STATUS) {
    if (startPressed || stopPressed) {
      menuMode = MENU_EDIT_TARGET;
      lcd.clear();
    }

    if (millis() - lastLcdUpdateMs >= 500) {
      lastLcdUpdateMs = millis();
      showStatusScreen(tempC, returnTempC, exhaustTempC);
    }
  } else {
    handleMenu(startPressed, stopPressed, encoderPressed, encoderDelta);
  }

  if (pelletDetected) {
    lastPelletSeenMs = millis();
  }

  if (stopPressed && state != STATE_IDLE) {
    enterState(STATE_COOLDOWN);
  }

  if (tempC >= maxTempC || exhaustTempC >= maxExhaustTempC) {
    enterState(STATE_FAULT);
  }

  switch (state) {
    case STATE_IDLE:
      setOutputs(false, false, false, false, 0);
      if (startPressed && roomThermostatActive) {
        enterState(STATE_IGNITION);
      }
      break;

    case STATE_IGNITION: {
      updateAugerCycle();
      updateGrateCycle();
      setOutputs(true, augerOn, true, grateOn, 200);

      if (tempC >= minFlameTempC) {
        enterState(STATE_RUN);
      }

      if (millis() - stateStartMs >= ignitionTimeMs) {
        enterState(STATE_FAULT);
      }

      if (millis() - lastPelletSeenMs >= pelletMissingTimeoutMs) {
        enterState(STATE_FAULT);
      }
      break;
    }

    case STATE_RUN: {
      updateAugerCycle();
      updateGrateCycle();
      bool needHeat = tempC < targetTempC && roomThermostatActive;
      uint8_t fanPwm = needHeat ? 220 : 160;

      setOutputs(true, needHeat ? augerOn : false, false, grateOn, fanPwm);

      if (tempC < minFlameTempC) {
        enterState(STATE_FAULT);
      }

      if (millis() - lastPelletSeenMs >= pelletMissingTimeoutMs) {
        enterState(STATE_FAULT);
      }
      break;
    }

    case STATE_COOLDOWN:
      updateGrateCycle();
      setOutputs(true, false, false, grateOn, 160);
      if (millis() - stateStartMs >= cooldownTimeMs) {
        enterState(STATE_IDLE);
      }
      break;

    case STATE_FAULT:
      setOutputs(true, false, false, false, 255);
      if (startPressed) {
        enterState(STATE_COOLDOWN);
      }
      break;
  }

  delay(100);
}
