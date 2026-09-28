// ============================================================================
// Serial-Controlled TEC Controller - Arduino Sketch
// Receives commands from Python: "SET PWM <0-255> DIR <HEAT|COOL>"
// Emits measurement data: "Temperature (C): 27.73, Time (s): 645.06, PWM: 120, Heat/Cool: 1"
// ============================================================================

const int TEMP_PIN = A0;             // Thermistor voltage divider input
const int PIN_9 = 9;                 // H-bridge control pin 1 (PWM capable)
const int PIN_10 = 10;               // H-bridge control pin 2 (PWM capable)

// --- Thermistor Constants ---
const float V_REF = 5.0;             // Reference voltage (V)
const float R_FIXED = 100000.0;      // Fixed resistor (100k)
const float R0 = 100000.0;           // Thermistor resistance at 25°C (100k)
const float T0_KELVIN = 298.15;      // Reference temperature in Kelvin
const float BETA = 3950.0;           // Beta coefficient
const int SAMPLE_COUNT = 500;        // ADC samples to average per reading (100-1000)
const int REPORT_INTERVAL = 1000;    // Serial report interval in milliseconds
const float SOFTWARE_TEMP_LIMIT_C = 60.0; // Software shutdown limit for TEC output. Hardware thermal switch at ~70 C remains the independent final protection.

// --- Control State Variables ---
int currentPwm = 0;                  // PWM starts at 0 for safe initialization
bool isHeating = true;               // State flag: true = HEAT, false = COOL
bool safetyShutdownActive = false;   // Software thermal cutoff flag
unsigned long lastReportTime = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_9, OUTPUT);
  pinMode(PIN_10, OUTPUT);

  // --- Safety Behavior ---
  // Ensure both H-bridge inputs are LOW on startup so the TEC remains unpowered
  digitalWrite(PIN_9, LOW);
  digitalWrite(PIN_10, LOW);
}

void loop() {
  // 1. Check for and parse incoming serial commands from Python
  parseSerialCommands();

  // 2. Read the averaged temperature every loop so the software safety check can react immediately.
  float tempC = readTemperature();

  // 3. If the software limit is exceeded, disable H-bridge PWM immediately and keep the thermal output off.
  if (tempC > SOFTWARE_TEMP_LIMIT_C) {
    safetyShutdownActive = true;
    currentPwm = 0;
    setHBridgeDisabled();
  } else {
    safetyShutdownActive = false;
    applyHBridgeOutput();
  }

  // 4. Continue reporting telemetry to the Python GUI even while shutdown is active.
  if (millis() - lastReportTime >= REPORT_INTERVAL) {
    lastReportTime = millis();
    float timeSec = millis() / 1000.0;
    int heatCoolFlag = isHeating ? 1 : 0;

    Serial.print("Temperature (C): ");
    Serial.print(tempC, 2);
    Serial.print(", Time (s): ");
    Serial.print(timeSec, 2);
    Serial.print(", PWM: ");
    Serial.print(currentPwm);
    Serial.print(", Heat/Cool: ");
    Serial.print(heatCoolFlag);

    if (safetyShutdownActive) {
      Serial.print(", Safety: SHUTDOWN ACTIVE (temp > ");
      Serial.print(SOFTWARE_TEMP_LIMIT_C, 1);
      Serial.print(" C, both H-bridge PWM outputs set to 0)");
    } else {
      Serial.print(", Safety: OK");
    }
    Serial.println();
  }
}

// ============================================================================
// parseSerialCommands()
// Reads incoming command packets in format: "SET PWM <0-255> DIR <HEAT|COOL>"
// ============================================================================
void parseSerialCommands() {
  while (Serial.available() > 0) {
    // Read the line sent by Python until a newline character is encountered
    String command = Serial.readStringUntil('\n');
    command.trim(); // Strip extraneous spaces or '\r' characters

    // Command Validation: Check for required command prefix
    if (command.startsWith("SET PWM ")) {
      int dirIndex = command.indexOf(" DIR ");
      if (dirIndex != -1) {
        // Extract substring for PWM value between "SET PWM " (index 8) and " DIR "
        String pwmStr = command.substring(8, dirIndex);
        
        // Extract substring for direction string following " DIR " (index + 5)
        String dirStr = command.substring(dirIndex + 5);

        // Convert extracted text to integer
        int rawPwm = pwmStr.toInt();

        // --- Safety Behavior ---
        // Clamp raw numeric input to guarantee it remains in the valid PWM range (0-255)
        currentPwm = constrain(rawPwm, 0, 255);

        // Update direction flag based on command text
        if (dirStr == "HEAT") {
          isHeating = true;
        } else if (dirStr == "COOL") {
          isHeating = false;
        }
      }
    }
  }
}

// ============================================================================
// applyHBridgeOutput()
// Applies active PWM to the correct pin based on physical Part 3 calibration
// ============================================================================
void applyHBridgeOutput() {
  if (isHeating) {
    // Observed Heating: Pin 9 receives PWM, Pin 10 held LOW
    // (If your Part 3 testing showed Pin 10 heated, swap PIN_9 and PIN_10 here)
    analogWrite(PIN_9, currentPwm);
    digitalWrite(PIN_10, LOW);
  } else {
    // Observed Cooling: Pin 9 held LOW, Pin 10 receives PWM
    digitalWrite(PIN_9, LOW);
    analogWrite(PIN_10, currentPwm);
  }
}

void setHBridgeDisabled() {
  analogWrite(PIN_9, 0);
  analogWrite(PIN_10, 0);
  digitalWrite(PIN_9, LOW);
  digitalWrite(PIN_10, LOW);
}

// ============================================================================
// readTemperature()
// Performs ADC sample averaging on A0 and calculates temperature via Beta model
// ============================================================================
float readTemperature() {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(TEMP_PIN);
  }
  float avgAdc = sum / (float)SAMPLE_COUNT;

  float voltage = (avgAdc / 1023.0) * V_REF;
  if (voltage >= V_REF || voltage <= 0) return 0.0; // Voltage error check

  float resistance = R_FIXED * voltage / (V_REF - voltage);
  float inverseT = (1.0 / T0_KELVIN) + (1.0 / BETA) * log(resistance / R0);
  return (1.0 / inverseT) - 273.15;
}