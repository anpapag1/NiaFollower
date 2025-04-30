#include <EEPROM.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
#include <FastLED.h>

// Ensure Bluetooth is enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Bluetooth serial communication
BluetoothSerial SerialBT;

// EEPROM configuration
#define EEPROM_SIZE 512
#define SETTINGS_ADDRESS 0

// Structure to store PID settings
struct Settings {
  double Kp;
  double Ki;
  double Kd;
  int baseSpeed;
};

// LED strip configuration
#define LED_PIN     18
#define NUM_LEDS    10
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

// LED array
CRGB leds[NUM_LEDS];

// Button and motor pin definitions
#define LED_BUILTIN 2
#define STRT_BTN 23
#define LED_BTN 22
#define AIN1 16
#define AIN2 17
#define BIN1 2
#define BIN2 15
#define STBY 4

// Sensor configuration
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// PID variables
double Kp;
double Ki;
double Kd;

// Serial command buffer
String inputString = "";
bool stringComplete = false;

// Robot state variables
uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int baseSpeed = 50;
bool robotEnabled = false;
bool manualControl = false;
int direction = 0;
uint8_t ledState = 0;

void setup() {
  // Initialize pins
  pinMode(STRT_BTN, INPUT_PULLUP);
  pinMode(LED_BTN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Initialize Bluetooth and Serial communication
  SerialBT.begin("NiaFollower");
  Serial.begin(500000);

  // Initialize EEPROM
  if (!EEPROM.begin(EEPROM_SIZE)) {
    SerialBT.println("Failed to initialize EEPROM");
    return;
  }
  loadSettings(); // Load saved PID settings

  delay(500);

  // Initialize LED strip
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(50);

  // Show startup animation
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(500);

  // Configure sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 26, 25, 33, 32, 35, 34, 39, 36}, SensorCount);
  qtr.setEmitterPin(27);

  // Display help message
  SerialBT.println("PID Line Follower Control");
  processCommand("h");
}

void loop() {
  // Update LED strip based on state
  updateLEDs(ledState);

  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Read Bluetooth serial data
  while (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // Handle LED button press
  if (digitalRead(LED_BTN) != HIGH) {
    ledState = (ledState + 1) % 3;
    Serial.println(ledState);
    while (digitalRead(LED_BTN) != HIGH) {}
  }

  // Handle start button press
  if (digitalRead(STRT_BTN) != HIGH) {
    Serial.println("Start button pressed");
    robotEnabled = !robotEnabled;
    while (digitalRead(STRT_BTN) != HIGH) {}
  }

  // Control robot movement
  if (manualControl) {
    motor_drive(baseSpeed - direction, baseSpeed + direction);
  } else {
    position = readLine(position, 4090);
    error = SensorCount / 2 * 1000 - position;
    PID_Linefollow(error);
  }
}

void processCommand(String command) {
  // Process incoming commands
  if (command.length() < 1) return;

  char cmd = command.charAt(0);
  // Serial.println(command);
  float value = 0;

  if (command.length() > 1) {
    value = command.substring(1).toFloat();
  }

  switch (cmd) {
    case 'p': // Set proportional gain
      Kp = value;
      Serial.println("Kp set to " + String(Kp, 4));
      break;
    case 'i': // Set integral gain
      Ki = value;
      Serial.println("Ki set to " + String(Ki, 4));
      break;
    case 'd': // Set derivative gain
      Kd = value;
      Serial.println("Kd set to " + String(Kd, 4));
      break;
    case 's': // Set base speed
      baseSpeed = (int)value;
      Serial.println("Base speed set to " + String(baseSpeed));
      break;
    case 't': // Toggle robot state
      robotEnabled = bool(value);
      Serial.println("Robot " + String(robotEnabled ? "enabled" : "disabled"));
      break;
    case 'w': // Save settings to EEPROM
      saveSettings();
      break;
    case 'm': // Enable/disable manual control
      manualControl = bool(value);
      Serial.println("Manual control " + String(manualControl ? "enabled" : "disabled"));
      break;
    case 'r': // Set speed and direction
      baseSpeed = command.substring(1, 4).toInt();
      baseSpeed = map(baseSpeed, 0, 300, -200, 200);
      direction = command.substring(4, 7).toInt();
      direction = map(direction, 70, 330, -70, 70);
      direction = pow(direction / 70.0, 3) * 70; // Make direction exponential
      if (baseSpeed < -10) direction = -direction;
      Serial.print(baseSpeed);
      Serial.print("\t");
      Serial.println(direction);
      break;
    case 'l': // Change LED state
      ledState = (ledState + 1) % 3;
      Serial.println("LED state changed to " + String(ledState));
      break;
    case 'h': // Display current settings
      SerialBT.println(String(Kp, 4) + " " + String(Ki, 4) + " " + String(Kd, 4) + " " + String(baseSpeed));
      break;
  }
}

void saveSettings() {
  // Save PID settings to EEPROM
  Settings settings = {
    Kp,
    Ki,
    Kd,
    baseSpeed
  };
  EEPROM.put(SETTINGS_ADDRESS, settings);
  EEPROM.commit();
  SerialBT.println("Settings saved to EEPROM");
  Serial.println("Settings saved to EEPROM");
  Serial.println("Kp: " + String(Kp, 4) + ", Ki: " + String(Ki, 4) + ", Kd: " + String(Kd, 4) + ", Base Speed: " + String(baseSpeed));
}

void loadSettings() {
  // Load PID settings from EEPROM
  Settings settings;
  EEPROM.get(SETTINGS_ADDRESS, settings);
  Kp = settings.Kp;
  Ki = settings.Ki;
  Kd = settings.Kd;
  baseSpeed = settings.baseSpeed;
  SerialBT.println("Settings loaded from EEPROM");
  Serial.println("Settings loaded from EEPROM");
  Serial.println("Kp: " + String(Kp, 4) + ", Ki: " + String(Ki, 4) + ", Kd: " + String(Kd, 4) + ", Base Speed: " + String(baseSpeed));
}

void PID_Linefollow(int error) {
  // Calculate PID values
  P = error;
  I = I + error;
  D = error - previousError;

  float PIDvalue = Kp * P + Ki * I + Kd * D;
  previousError = error;

  rsp = constrain(baseSpeed + PIDvalue, -255, 255);
  lsp = constrain(baseSpeed - PIDvalue, -255, 255);

  motor_drive(rsp, lsp);
}

int readLine(int lastposition, int blackLineValue) {
  // Read sensor values and calculate position
  qtr.read(sensorValues);

  int position = 0;
  int count = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > blackLineValue) {
      count++;
      position += i * 1000;
    }
  }
  
  if (count == SensorCount) {
    robotEnabled = false;
    position = 0;
  } else if (count != 0) {
    position = position / count;
  } else {
    if (lastposition > SensorCount / 2) {
      position = SensorCount * 1000;
    } else {
      position = 0;
    }
  }
  // debug
  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   if (sensorValues[i] > blackLineValue) {
  //     Serial.print("▢");
  //   } else {
  //     Serial.print("_");
  //   }
  // }
  // Serial.print("\t");
  // Serial.print("position: ");
  Serial.println(position);

  return position;
}

void motor_drive(int right, int left) {
  // Control motor movement
  digitalWrite(STBY, robotEnabled ? HIGH : LOW);

  if (position < SensorCount-1 * 1000 || PIDvalue == 0) {
    if (right > 0) {
      analogWrite(AIN1, right);
      analogWrite(AIN2, 0);
    } else {
      analogWrite(AIN1, 0);
      analogWrite(AIN2, abs(right));
    }
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 30);
  }

  if (position > 0 || PIDvalue == 0) {
    if (left > 0) {
      analogWrite(BIN1, left);
      analogWrite(BIN2, 0);
    } else {
      analogWrite(BIN1, 0);
      analogWrite(BIN2, abs(left));
    }
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 30);
  }
}

void updateLEDs(uint8_t state) {
  // Update LED strip based on state
  if (state == 0) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else if (state == 1) {
    if (robotEnabled) {
      if (0 < position && position < SensorCount * 1000) {
        fill_solid(leds, NUM_LEDS, CRGB::Green);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
      }
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Orange);
    }
  } else if (state == 2) {
    static uint8_t hue = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(hue + (i * 255 / NUM_LEDS), 255, 255);
    }
    hue++;
  }
  FastLED.show();
}