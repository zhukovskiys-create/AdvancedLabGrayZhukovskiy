// ============================================================================
// Serial-Controlled TEC and Rod Temperature Controller - Arduino Sketch
// Receives commands from Python: "SET PWM <0-255> DIR <HEAT|COOL>"
// Logs the TEC-base temperature and five rod temperatures as CSV.
// ============================================================================

const int NUM_SENSORS = 6;
// A0 measures the TEC base; A1-A5 measure rod thermistors 1-5.
const int TEMP_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
const int PIN_9 = 9;                 // H-bridge control pin 1 (PWM capable)
const int PIN_10 = 10;               // H-bridge control pin 2 (PWM capable)

// --- Thermistor Constants ---
const float V_REF = 5.0;             // Reference voltage (V)
const float R_FIXED = 100000.0;      // Fixed resistor (100k)
const float R0 = 100000.0;           // Thermistor resistance at 25°C (100k)
const float T0_KELVIN = 298.15;      // Reference temperature in Kelvin
const float BETA = 3950.0;           // Beta coefficient
const int SAMPLE_COUNT = 500;        // ADC samples averaged for each thermistor
const unsigned long REPORT_INTERVAL = 5000;  // Serial report interval in milliseconds
const float SOFTWARE_TEMP_LIMIT_C = 60.0; // TEC-base software shutdown limit

// --- Control State Variables ---
int currentPwm = 0;                  // PWM starts at 0 for safe initialization
bool isHeating = true;               // State flag: true = HEAT, false = COOL
bool safetyShutdownActive = false;   // Software thermal cutoff flag
unsigned long lastReportTime = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_9, OUTPUT);
  pinMode(PIN_10, OUTPUT);

  // Keep both H-bridge inputs LOW at startup so the TEC remains unpowered.
  digitalWrite(PIN_9, LOW);
  digitalWrite(PIN_10, LOW);

  Serial.println("DATA,time_s,tec_C,rod1_C,rod2_C,rod3_C,rod4_C,rod5_C,pwm,heat_cool,setpoint_C,safety");
}

void loop() {
  // Check for and parse incoming serial commands from Python.
  parseSerialCommands();

  // The TEC-base thermistor alone controls the software temperature limit.
  float tecSafetyTempC = readTemperature(TEMP_PINS[0]);
  if (tecSafetyTempC > SOFTWARE_TEMP_LIMIT_C) {
    safetyShutdownActive = true;
    currentPwm = 0;
    setHBridgeDisabled();
  } else {
    safetyShutdownActive = false;
    applyHBridgeOutput();
  }

  // Acquire and report all six thermistors at the existing report interval.
  if (millis() - lastReportTime >= REPORT_INTERVAL) {
    lastReportTime = millis();

    float temperaturesC[NUM_SENSORS];
    // Read each channel in turn and convert its averaged ADC value to Celsius.
    for (int i = 0; i < NUM_SENSORS; i++) {
      temperaturesC[i] = readTemperature(TEMP_PINS[i]);
    }

    // Capture elapsed time only after all six temperature measurements finish.
    unsigned long elapsedTimeMs = millis();
    float timeSec = elapsedTimeMs / 1000.0;
    int heatCoolFlag = isHeating ? 1 : 0;

    // Keep each acquisition as one DATA CSV row; the external Python controller
    // owns the setpoint, so report NA here rather than inventing a value.
    Serial.print("DATA,");
    Serial.print(timeSec, 2);
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(",");
      Serial.print(temperaturesC[i], 2);
    }
    Serial.print(",");
    Serial.print(currentPwm);
    Serial.print(",");
    Serial.print(heatCoolFlag);
    Serial.print(",NA,");
    Serial.println(safetyShutdownActive ? "SHUTDOWN" : "OK");
  }
}

// ============================================================================
// parseSerialCommands()
// Reads incoming command packets in format: "SET PWM <0-255> DIR <HEAT|COOL>"
// ============================================================================
void parseSerialCommands() {
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("SET PWM ")) {
      int dirIndex = command.indexOf(" DIR ");
      if (dirIndex != -1) {
        String pwmStr = command.substring(8, dirIndex);
        String dirStr = command.substring(dirIndex + 5);

        int rawPwm = pwmStr.toInt();

        // Clamp raw numeric input to the valid PWM range (0-255).
        currentPwm = constrain(rawPwm, 0, 255);

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
    // Observed Heating: Pin 9 receives PWM, Pin 10 held LOW.
    analogWrite(PIN_9, currentPwm);
    digitalWrite(PIN_10, LOW);
  } else {
    // Observed Cooling: Pin 9 held LOW, Pin 10 receives PWM.
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
// readTemperature(pin)
// Averages ADC samples on the given analog pin and calculates temperature
// via the Beta model. Returns 0.0 on a voltage error.
// ============================================================================
float readTemperature(int pin) {
  // A dummy read gives the ADC multiplexer time to settle after channel changes.
  analogRead(pin);

  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(pin);
  }
  float avgAdc = sum / (float)SAMPLE_COUNT;

  float voltage = (avgAdc / 1023.0) * V_REF;
  if (voltage >= V_REF || voltage <= 0) return 0.0; // Voltage error check

  float resistance = R_FIXED * voltage / (V_REF - voltage);
  float inverseT = (1.0 / T0_KELVIN) + (1.0 / BETA) * log(resistance / R0);
  return (1.0 / inverseT) - 273.15;
}
