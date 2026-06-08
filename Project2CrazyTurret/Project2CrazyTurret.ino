#include <Servo.h>
#include "U8glib.h"

/*
  Projekt: Servo- och laserstyrning med joystickar, knappar och OLED

  Kort förklaring:
  Programmet styr fyra servon med två joystickar.
  Det finns två lägen:
  - Dual: varje joystick styr två servon var
  - Sync: två servon följer de andra, så rörelsen blir synkad

  Knappar:
  - Slow: gör rörelsen långsammare
  - Fast: gör rörelsen snabbare
  - Mode kort tryck: byter mellan Dual och Sync
  - Mode långt tryck: slår på/av båda lasrarna

  Jag har försökt göra koden tydlig genom att:
  - dela upp programmet i funktioner
  - använda konstanter istället för "magiska tal"
  - ha egen debounce för varje knapp
  - skydda servovinklarna så de aldrig går utanför 0-180 grader
  - skriva kommentarer som förklarar varför saker görs
*/

// OLED

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

// Servon

Servo servoX;
Servo servoY;
Servo servoZ;
Servo servoW;

// Pinnar

const byte PIN_SERVO_X = 9;
const byte PIN_SERVO_Y = 10;
const byte PIN_SERVO_Z = 11;
const byte PIN_SERVO_W = 12;

const byte PIN_LASER_1 = 4;
const byte PIN_LASER_2 = 8;

const byte JOY_X_LEFT  = A0;
const byte JOY_Y_LEFT  = A1;
const byte JOY_X_RIGHT = A2;
const byte JOY_Y_RIGHT = A3;

const byte BTN_SLOW = 6;
const byte BTN_FAST = 7;
const byte BTN_MODE = 5;

// Konstanter för styrning

const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_START_ANGLE = 90;

// Joystickens mittläge är inte exakt 512 i verkligheten.
// Därför används en dödzon så servon inte rör sig av sig själv.
const int JOYSTICK_CENTER_LOW  = 472;
const int JOYSTICK_CENTER_HIGH = 552;

// Hastigheten styrs med delay mellan varje steg.
// Lägre värde = snabbare rörelse.
const int SPEED_MIN_DELAY = 1;
const int SPEED_MAX_DELAY = 60;
const int SPEED_CHANGE = 5;

// Ett servosteg per loop gör rörelsen mjukare.
const int SERVO_STEP = 1;

const unsigned long DEBOUNCE_TIME = 70;
const unsigned long MODE_HOLD_TIME = 700;

// Datatyper

// Detta gör att varje servo kan behandlas på samma sätt.
// Det gör koden lättare att bygga ut om man skulle lägga till fler servon.
struct ServoAxis {
  Servo *servo;
  byte servoPin;
  byte joystickPin;
  int angle;
  const char *name;
};

// Varje knapp har sin egen debounce.
// Det är bättre än en gemensam debounce-timer, eftersom knapparna då inte stör varandra.
struct ButtonState {
  byte pin;
  byte stableState;
  byte lastReading;
  unsigned long lastChangeTime;
  bool pressedEvent;
  bool releasedEvent;
};

// Globala variabler

ServoAxis axisX = { &servoX, PIN_SERVO_X, JOY_X_LEFT,  SERVO_START_ANGLE, "X" };
ServoAxis axisY = { &servoY, PIN_SERVO_Y, JOY_Y_LEFT,  SERVO_START_ANGLE, "Y" };
ServoAxis axisZ = { &servoZ, PIN_SERVO_Z, JOY_X_RIGHT, SERVO_START_ANGLE, "Z" };
ServoAxis axisW = { &servoW, PIN_SERVO_W, JOY_Y_RIGHT, SERVO_START_ANGLE, "W" };

ButtonState buttonSlow = { BTN_SLOW, HIGH, HIGH, 0, false, false };
ButtonState buttonFast = { BTN_FAST, HIGH, HIGH, 0, false, false };
ButtonState buttonMode = { BTN_MODE, HIGH, HIGH, 0, false, false };

int speedDelayMs = 15;

bool syncMode = false;
bool laserOn = true;

unsigned long modePressStart = 0;
bool modeButtonIsDown = false;
bool longPressAlreadyUsed = false;

// Funktionsdeklarationer

void initializeSystem();
void initializeServos();
void initializeButtons();
void initializeLasers();

void updateButton(ButtonState &button);
void handleButtons();
void handleSpeedButtons();
void handleModeButton();

void readAndControlServos();
void controlDualMode();
void controlSyncMode();

void updateAxisFromJoystick(ServoAxis &axis);
int getJoystickDirection(int value);
int limitServoAngle(int angle);

void writeAllServos();
void setLaserState(bool state);

void updateOLED();
void printSystemStatus();
void printAxisStatus(const ServoAxis &axis);

// Setup

void setup() {
  Serial.begin(9600);
  initializeSystem();

  Serial.println("System startat");
  printSystemStatus();

  updateOLED();
}

// Loop

void loop() {
  handleButtons();
  readAndControlServos();
  writeAllServos();

  delay(speedDelayMs);
}

// Initiering

void initializeSystem() {
  initializeServos();
  initializeButtons();
  initializeLasers();
}

void initializeServos() {
  axisX.servo->attach(axisX.servoPin);
  axisY.servo->attach(axisY.servoPin);
  axisZ.servo->attach(axisZ.servoPin);
  axisW.servo->attach(axisW.servoPin);

  writeAllServos();
}

void initializeButtons() {
  pinMode(buttonSlow.pin, INPUT_PULLUP);
  pinMode(buttonFast.pin, INPUT_PULLUP);
  pinMode(buttonMode.pin, INPUT_PULLUP);
}

void initializeLasers() {
  pinMode(PIN_LASER_1, OUTPUT);
  pinMode(PIN_LASER_2, OUTPUT);

  setLaserState(true);
}

// Knapphantering

void handleButtons() {
  updateButton(buttonSlow);
  updateButton(buttonFast);
  updateButton(buttonMode);

  handleSpeedButtons();
  handleModeButton();
}

void updateButton(ButtonState &button) {
  button.pressedEvent = false;
  button.releasedEvent = false;

  unsigned long now = millis();
  byte reading = digitalRead(button.pin);

  // Om läsningen ändras startar vi om debounce-tiden.
  if (reading != button.lastReading) {
    button.lastChangeTime = now;
    button.lastReading = reading;
  }

  // När signalen varit stabil tillräckligt länge räknas det som ett riktigt knapptryck.
  if ((now - button.lastChangeTime) >= DEBOUNCE_TIME && reading != button.stableState) {
    byte oldState = button.stableState;
    button.stableState = reading;

    if (oldState == HIGH && button.stableState == LOW) {
      button.pressedEvent = true;
    }

    if (oldState == LOW && button.stableState == HIGH) {
      button.releasedEvent = true;
    }
  }
}

void handleSpeedButtons() {
  bool changed = false;

  if (buttonSlow.pressedEvent) {
    speedDelayMs += SPEED_CHANGE;
    speedDelayMs = constrain(speedDelayMs, SPEED_MIN_DELAY, SPEED_MAX_DELAY);
    changed = true;

    Serial.print("Hastighet ändrad. Delay: ");
    Serial.print(speedDelayMs);
    Serial.println(" ms");
  }

  if (buttonFast.pressedEvent) {
    speedDelayMs -= SPEED_CHANGE;
    speedDelayMs = constrain(speedDelayMs, SPEED_MIN_DELAY, SPEED_MAX_DELAY);
    changed = true;

    Serial.print("Hastighet ändrad. Delay: ");
    Serial.print(speedDelayMs);
    Serial.println(" ms");
  }

  if (changed) {
    updateOLED();
  }
}

void handleModeButton() {
  unsigned long now = millis();

  if (buttonMode.pressedEvent) {
    modePressStart = now;
    modeButtonIsDown = true;
    longPressAlreadyUsed = false;
  }

  // Långt tryck används till lasern.
  if (modeButtonIsDown && buttonMode.stableState == LOW && !longPressAlreadyUsed) {
    if (now - modePressStart >= MODE_HOLD_TIME) {
      setLaserState(!laserOn);
      longPressAlreadyUsed = true;

      Serial.print("Laser: ");
      Serial.println(laserOn ? "ON" : "OFF");

      updateOLED();
    }
  }

  // Kort tryck används till lägesbyte.
  // Om långt tryck redan användes ska programmet inte också byta mode.
  if (buttonMode.releasedEvent && modeButtonIsDown) {
    if (!longPressAlreadyUsed) {
      syncMode = !syncMode;

      Serial.print("Mode bytt till: ");
      Serial.println(syncMode ? "Sync" : "Dual");

      updateOLED();
    }

    modeButtonIsDown = false;
    longPressAlreadyUsed = false;
  }
}

// Joystick och servostyrning

void readAndControlServos() {
  if (syncMode) {
    controlSyncMode();
  } else {
    controlDualMode();
  }
}

void controlDualMode() {
  // Dual mode:
  // Vänster joystick styr X och Y.
  // Höger joystick styr Z och W.
  updateAxisFromJoystick(axisX);
  updateAxisFromJoystick(axisY);
  updateAxisFromJoystick(axisZ);
  updateAxisFromJoystick(axisW);
}

void controlSyncMode() {
  /*
    Sync mode:
    Här styrs X med höger joystick X-led och Y med vänster joystick Y-led.
    Sedan kopieras X till Z och Y till W.
    Det gör att två servon rör sig likadant.
  */

  int oldX = axisX.angle;
  int oldY = axisY.angle;

  axisX.joystickPin = JOY_X_RIGHT;
  axisY.joystickPin = JOY_Y_LEFT;

  updateAxisFromJoystick(axisX);
  updateAxisFromJoystick(axisY);

  if (axisX.angle != oldX) {
    axisZ.angle = axisX.angle;
  }

  if (axisY.angle != oldY) {
    axisW.angle = axisY.angle;
  }

  // Sätter tillbaka standardkopplingarna så Dual mode alltid fungerar normalt.
  axisX.joystickPin = JOY_X_LEFT;
  axisY.joystickPin = JOY_Y_LEFT;
}

void updateAxisFromJoystick(ServoAxis &axis) {
  int joystickValue = analogRead(axis.joystickPin);
  int direction = getJoystickDirection(joystickValue);

  axis.angle += direction * SERVO_STEP;
  axis.angle = limitServoAngle(axis.angle);
}

int getJoystickDirection(int value) {
  if (value > JOYSTICK_CENTER_HIGH) {
    return 1;
  }

  if (value < JOYSTICK_CENTER_LOW) {
    return -1;
  }

  return 0;
}

int limitServoAngle(int angle) {
  return constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

void writeAllServos() {
  axisX.servo->write(axisX.angle);
  axisY.servo->write(axisY.angle);
  axisZ.servo->write(axisZ.angle);
  axisW.servo->write(axisW.angle);
}

// Laser

void setLaserState(bool state) {
  laserOn = state;

  digitalWrite(PIN_LASER_1, laserOn ? HIGH : LOW);
  digitalWrite(PIN_LASER_2, laserOn ? HIGH : LOW);
}

// OLED

void updateOLED() {
  u8g.firstPage();

  do {
    u8g.setFont(u8g_font_9x15);

    u8g.setPrintPos(4, 14);
    u8g.print("Mode:");
    u8g.setPrintPos(65, 14);
    u8g.print(syncMode ? "Sync" : "Dual");

    u8g.setPrintPos(4, 28);
    u8g.print("Speed:");
    u8g.setPrintPos(65, 28);
    u8g.print(speedDelayMs);
    u8g.print("ms");

    u8g.setPrintPos(4, 42);
    u8g.print("Laser:");
    u8g.setPrintPos(65, 42);
    u8g.print(laserOn ? "ON" : "OFF");

    u8g.setPrintPos(4, 60);
    u8g.print("X:");
    u8g.print(axisX.angle);
    u8g.print(" Y:");
    u8g.print(axisY.angle);

  } while (u8g.nextPage());
}

// Serial monitor

void printSystemStatus() {
  Serial.print("Mode: ");
  Serial.println(syncMode ? "Sync" : "Dual");

  Serial.print("Delay: ");
  Serial.print(speedDelayMs);
  Serial.println(" ms");

  Serial.print("Laser: ");
  Serial.println(laserOn ? "ON" : "OFF");

  printAxisStatus(axisX);
  printAxisStatus(axisY);
  printAxisStatus(axisZ);
  printAxisStatus(axisW);
}

void printAxisStatus(const ServoAxis &axis) {
  Serial.print("Servo ");
  Serial.print(axis.name);
  Serial.print(" startvinkel: ");
  Serial.println(axis.angle);
}