// ============================================================================
// Serial-Controlled TEC and Rod Temperature Controller
//
// Upload a schedule atomically:
// SCHEDULE,<count>,<duration_ms>,<mean_C>,<amplitude_C>,<period_s>,...
// Arduino replies SCHEDULE_ACK,OK,<count> or SCHEDULE_ACK,ERROR,<reason>.
//
// The GUI remains connected while the board executes each step. Phase carries
// across period changes; a new schedule starts paused at phase zero.
//
// Host actuator command:
// ACTUATOR,<RUNNING|PAUSED>,<mode>,<feedforward_pwm>,<raw_pwm>,
//          <cooling_ratio>,<signed_pwm>,<diagnostic_ua>,<diagnostic_ub>,
//          <diagnostic_omega_rad_s>
//
// One DATA CSV record is emitted every REPORT_INTERVAL. Its header documents
// the field order: elapsed_ms, state, mode, run segment and setpoint-sinusoid
// parameters, setpoint, feedforward PWM, TEC and five rod temperatures, raw
// controller PWM, cooling ratio, applied signed PWM, and diagnostic ua/ub/omega.
// PWM is signed (positive=heat, negative=cool); only one bridge direction is
// energized at a time. Diagnostic PWM modes bypass cooling compensation.
// ============================================================================

const int NUM_SENSORS = 6;
const int TEMP_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
const int PIN_9 = 9;
const int PIN_10 = 10;

const float V_REF = 5.0;
const float R_FIXED = 100000.0;
const float R0 = 100000.0;
const float T0_KELVIN = 298.15;
const float BETA = 3950.0;
const int SAMPLE_COUNT = 500;
const unsigned long REPORT_INTERVAL = 5000;
const float SOFTWARE_TEMP_LIMIT_C = 60.0;
const uint8_t MAX_SCHEDULE_STEPS = 8;
const unsigned long MAX_STEP_DURATION_MS = 86400000UL;
const float MAX_STEP_PERIOD_S = 86400.0;
const float MAX_DIAGNOSTIC_OMEGA_RAD_S = 6.28318530718;
const float TWO_PI_F = 6.28318530718;

struct ScheduleStep {
  unsigned long durationMs;
  float meanC;
  float amplitudeC;
  float periodS;
};

int requestedSignedPwm = 0;
int appliedSignedPwm = 0;
String operatingState = "PAUSED";
String controlMode = "pi";
ScheduleStep scheduleSteps[MAX_SCHEDULE_STEPS];
uint8_t scheduleCount = 0;
int scheduleStepIndex = -1;
unsigned long scheduleStepElapsedMs = 0;
unsigned long lastScheduleUpdateMs = 0;
float sinusoidPhaseRadians = 0.0;
float setpointCenterC = 30.0;
float sinusoidAmplitudeC = 0.0;
float sinusoidPeriodS = 120.0;
float setpointC = 30.0;
float feedforwardPwm = 0.0;
float rawControllerPwm = 0.0;
float coolingCompensationRatio = 2.0;
float diagnosticUaPwm = 0.0;
float diagnosticUbPwm = 0.0;
float diagnosticOmegaRadPerS = 0.0;
unsigned long diagnosticStartMs = 0;
bool safetyShutdownActive = false;
bool scheduleComplete = false;
unsigned long lastReportTime = 0;

bool parseFiniteFloat(const String &text, float &value) {
  const char *start = text.c_str();
  char *end = NULL;
  double parsed = strtod(start, &end);
  if (end == start || *end != '\0' || !isfinite(parsed)) return false;
  value = (float)parsed;
  return isfinite(value);
}

bool parseUnsignedLong(const String &text, unsigned long &value) {
  if (text.length() == 0) return false;
  for (unsigned int i = 0; i < text.length(); i++) {
    if (text[i] < '0' || text[i] > '9') return false;
  }
  char *end = NULL;
  unsigned long parsed = strtoul(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') return false;
  value = parsed;
  return true;
}

bool validControlMode(const String &mode) {
  return mode == "open_loop" || mode == "feedforward"
      || mode == "p" || mode == "pi"
      || mode == "raw_sine" || mode == "raw_offset_sine";
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_9, OUTPUT);
  pinMode(PIN_10, OUTPUT);
  setHBridgeDisabled();
  lastScheduleUpdateMs = millis();

  Serial.println(
    "DATA,elapsed_ms,operating_state,control_mode,run_segment,"
    "segment_duration_ms,segment_mean_C,segment_amplitude_C,segment_period_s,"
    "setpoint_C,"
    "feedforward_pwm,tec_C,rod1_C,rod2_C,rod3_C,rod4_C,rod5_C,"
    "raw_controller_pwm,cooling_compensation_ratio,applied_signed_pwm,"
    "diagnostic_ua_pwm,diagnostic_ub_pwm,diagnostic_omega_rad_s"
  );
}

void loop() {
  parseSerialCommands();
  unsigned long nowMs = millis();
  advanceSchedule(nowMs);
  updateRawDiagnosticCommand(nowMs);

  float tecSafetyTempC = readTemperature(TEMP_PINS[0]);
  safetyShutdownActive = tecSafetyTempC > SOFTWARE_TEMP_LIMIT_C;
  if (safetyShutdownActive) {
    requestedSignedPwm = 0;
    setHBridgeDisabled();
  } else {
    applyHBridgeOutput();
  }

  if (nowMs - lastReportTime >= REPORT_INTERVAL) {
    lastReportTime = nowMs;
    float temperaturesC[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      temperaturesC[i] = readTemperature(TEMP_PINS[i]);
    }

    printTelemetry(temperaturesC, nowMs);
  }
}

void parseSerialCommands() {
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("SCHEDULE,")) {
      parseScheduleCommand(command);
    } else if (command.startsWith("ACTUATOR,")) {
      parseActuatorCommand(command);
    } else if (command.startsWith("SET PWM ")) {
      parseLegacyCommand(command);
    }
  }
}

void acknowledgeScheduleError(const char *reason) {
  Serial.print("SCHEDULE_ACK,ERROR,");
  Serial.println(reason);
}

void parseScheduleCommand(const String &command) {
  String fields[2 + 4 * MAX_SCHEDULE_STEPS];
  int fieldCount = 0;
  int fieldStart = 0;
  while (fieldStart <= command.length()
      && fieldCount < (int)(2 + 4 * MAX_SCHEDULE_STEPS)) {
    int comma = command.indexOf(',', fieldStart);
    if (comma < 0) {
      fields[fieldCount++] = command.substring(fieldStart);
      fieldStart = command.length() + 1;
    } else {
      fields[fieldCount++] = command.substring(fieldStart, comma);
      fieldStart = comma + 1;
    }
  }
  if (fieldStart <= command.length()) {
    acknowledgeScheduleError("TOO_MANY_FIELDS");
    return;
  }
  if (fieldCount < 2 || fields[0] != "SCHEDULE") {
    acknowledgeScheduleError("BAD_FORMAT");
    return;
  }

  unsigned long parsedCount = 0;
  if (!parseUnsignedLong(fields[1], parsedCount)
      || parsedCount < 1 || parsedCount > MAX_SCHEDULE_STEPS) {
    acknowledgeScheduleError("BAD_COUNT");
    return;
  }
  uint8_t candidateCount = (uint8_t)parsedCount;
  if (fieldCount != 2 + 4 * candidateCount) {
    acknowledgeScheduleError("BAD_FIELD_COUNT");
    return;
  }

  ScheduleStep candidate[MAX_SCHEDULE_STEPS];
  unsigned long totalDurationMs = 0;
  int field = 2;
  for (uint8_t i = 0; i < candidateCount; i++) {
    float meanC, amplitudeC, periodS;
    unsigned long durationMs;
    if (!parseUnsignedLong(fields[field++], durationMs)
        || !parseFiniteFloat(fields[field++], meanC)
        || !parseFiniteFloat(fields[field++], amplitudeC)
        || !parseFiniteFloat(fields[field++], periodS)) {
      acknowledgeScheduleError("NON_NUMERIC_VALUE");
      return;
    }
    if (durationMs < 100 || durationMs > MAX_STEP_DURATION_MS
        || amplitudeC < 0.0 || amplitudeC > 30.0
        || periodS < 0.1 || periodS > MAX_STEP_PERIOD_S
        || meanC - amplitudeC < -10.0 || meanC + amplitudeC > 60.0) {
      acknowledgeScheduleError("STEP_OUT_OF_RANGE");
      return;
    }
    if (totalDurationMs > 0xFFFFFFFFUL - durationMs) {
      acknowledgeScheduleError("DURATION_OVERFLOW");
      return;
    }
    totalDurationMs += durationMs;
    candidate[i].durationMs = durationMs;
    candidate[i].meanC = meanC;
    candidate[i].amplitudeC = amplitudeC;
    candidate[i].periodS = periodS;
  }

  for (uint8_t i = 0; i < candidateCount; i++) {
    scheduleSteps[i] = candidate[i];
  }
  scheduleCount = candidateCount;
  scheduleStepIndex = 0;
  scheduleStepElapsedMs = 0;
  sinusoidPhaseRadians = 0.0;
  lastScheduleUpdateMs = millis();
  scheduleComplete = false;
  operatingState = "PAUSED";
  requestedSignedPwm = 0;
  setpointC = scheduleSteps[0].meanC;
  setpointCenterC = scheduleSteps[0].meanC;
  sinusoidAmplitudeC = scheduleSteps[0].amplitudeC;
  sinusoidPeriodS = scheduleSteps[0].periodS;
  Serial.print("SCHEDULE_ACK,OK,");
  Serial.println(scheduleCount);
}

void parseActuatorCommand(const String &command) {
  String fields[10];
  int fieldStart = 0;
  for (int i = 0; i < 9; i++) {
    int comma = command.indexOf(',', fieldStart);
    if (comma < 0) return;
    fields[i] = command.substring(fieldStart, comma);
    fieldStart = comma + 1;
  }
  fields[9] = command.substring(fieldStart);
  if (fields[0] != "ACTUATOR"
      || (fields[1] != "RUNNING" && fields[1] != "PAUSED")
      || !validControlMode(fields[2])) {
    return;
  }

  float feedforward, raw, ratio, signedPwm, diagnosticUa, diagnosticUb;
  float diagnosticOmega;
  if (!parseFiniteFloat(fields[3], feedforward)
      || !parseFiniteFloat(fields[4], raw)
      || !parseFiniteFloat(fields[5], ratio)
      || !parseFiniteFloat(fields[6], signedPwm)
      || !parseFiniteFloat(fields[7], diagnosticUa)
      || !parseFiniteFloat(fields[8], diagnosticUb)
      || !parseFiniteFloat(fields[9], diagnosticOmega)
      || ratio <= 0.0 || signedPwm < -255.0 || signedPwm > 255.0) {
    return;
  }
  if (fields[2] == "raw_sine") {
    if (diagnosticUa != 0.0 || diagnosticUb <= 0.0
        || diagnosticUb > 255.0 || diagnosticOmega <= 0.0
        || diagnosticOmega > MAX_DIAGNOSTIC_OMEGA_RAD_S) {
      return;
    }
  } else if (fields[2] == "raw_offset_sine") {
    if (diagnosticUa <= diagnosticUb || diagnosticUb <= 0.0
        || diagnosticUa + diagnosticUb > 255.0 || diagnosticOmega <= 0.0
        || diagnosticOmega > MAX_DIAGNOSTIC_OMEGA_RAD_S) {
      return;
    }
  } else if (diagnosticUa != 0.0 || diagnosticUb != 0.0
      || diagnosticOmega != 0.0) {
    return;
  }
  bool diagnosticMode = fields[2] == "raw_sine"
      || fields[2] == "raw_offset_sine";
  if (fields[1] == "RUNNING"
      && (!diagnosticMode
          && (scheduleCount == 0 || scheduleComplete)
          || safetyShutdownActive)) {
    return;
  }

  bool restartingDiagnostic = fields[2] != controlMode
      || (fields[1] == "RUNNING" && operatingState != "RUNNING");
  operatingState = fields[1];
  controlMode = fields[2];
  feedforwardPwm = feedforward;
  rawControllerPwm = raw;
  coolingCompensationRatio = ratio;
  requestedSignedPwm = (int)round(signedPwm);
  diagnosticUaPwm = diagnosticUa;
  diagnosticUbPwm = diagnosticUb;
  diagnosticOmegaRadPerS = diagnosticOmega;
  if (restartingDiagnostic) diagnosticStartMs = millis();
}

void updateRawDiagnosticCommand(unsigned long nowMs) {
  if (controlMode != "raw_sine" && controlMode != "raw_offset_sine") return;
  if (operatingState != "RUNNING" || safetyShutdownActive) {
    rawControllerPwm = 0.0;
    requestedSignedPwm = 0;
    return;
  }

  float elapsedS = (unsigned long)(nowMs - diagnosticStartMs) / 1000.0;
  float phase = fmod(diagnosticOmegaRadPerS * elapsedS, TWO_PI_F);
  float offset = controlMode == "raw_offset_sine" ? diagnosticUaPwm : 0.0;
  rawControllerPwm = offset + diagnosticUbPwm * sin(phase);
  requestedSignedPwm = constrain((int)round(rawControllerPwm), -255, 255);
}

void parseLegacyCommand(const String &command) {
  if (scheduleCount > 0) return;
  int directionIndex = command.indexOf(" DIR ");
  if (directionIndex < 0) return;
  int pwm = constrain(command.substring(8, directionIndex).toInt(), 0, 255);
  String direction = command.substring(directionIndex + 5);
  if (direction == "HEAT") {
    requestedSignedPwm = pwm;
  } else if (direction == "COOL") {
    requestedSignedPwm = -pwm;
  } else {
    return;
  }
  operatingState = pwm == 0 ? "PAUSED" : "RUNNING";
}

void advanceSchedule(unsigned long nowMs) {
  unsigned long remainingMs = nowMs - lastScheduleUpdateMs;
  lastScheduleUpdateMs = nowMs;
  if (scheduleCount == 0 || scheduleComplete || operatingState != "RUNNING") {
    return;
  }

  while (remainingMs > 0 && !scheduleComplete) {
    unsigned long stepRemainingMs =
        scheduleSteps[scheduleStepIndex].durationMs - scheduleStepElapsedMs;
    unsigned long segmentMs = min(remainingMs, stepRemainingMs);
    sinusoidPhaseRadians += TWO_PI_F * ((float)segmentMs / 1000.0)
        / scheduleSteps[scheduleStepIndex].periodS;
    while (sinusoidPhaseRadians >= TWO_PI_F) {
      sinusoidPhaseRadians -= TWO_PI_F;
    }
    scheduleStepElapsedMs += segmentMs;
    remainingMs -= segmentMs;

    if (scheduleStepElapsedMs == scheduleSteps[scheduleStepIndex].durationMs) {
      scheduleStepIndex++;
      scheduleStepElapsedMs = 0;
      if (scheduleStepIndex >= scheduleCount) {
        scheduleComplete = true;
        operatingState = "COMPLETE";
        requestedSignedPwm = 0;
        setpointC = scheduleSteps[scheduleCount - 1].meanC;
        return;
      }
    }
  }

  ScheduleStep &step = scheduleSteps[scheduleStepIndex];
  setpointCenterC = step.meanC;
  sinusoidAmplitudeC = step.amplitudeC;
  sinusoidPeriodS = step.periodS;
  setpointC = step.meanC + step.amplitudeC * sin(sinusoidPhaseRadians);
}

void printTelemetry(const float temperaturesC[NUM_SENSORS], unsigned long nowMs) {
  Serial.print("DATA,");
  Serial.print(nowMs);
  Serial.print(",");
  if (safetyShutdownActive) {
    Serial.print("SHUTDOWN");
  } else if (scheduleComplete
      && controlMode != "raw_sine" && controlMode != "raw_offset_sine") {
    Serial.print("COMPLETE");
  } else {
    Serial.print(operatingState);
  }
  Serial.print(",");
  Serial.print(controlMode);
  Serial.print(",");
  Serial.print(scheduleCount == 0 ? -1
      : (scheduleComplete ? scheduleCount - 1 : scheduleStepIndex));
  Serial.print(",");
  if (scheduleCount > 0) {
    const ScheduleStep &step = scheduleSteps[scheduleComplete
        ? scheduleCount - 1 : scheduleStepIndex];
    Serial.print(step.durationMs);
    Serial.print(",");
    Serial.print(step.meanC, 4);
    Serial.print(",");
    Serial.print(step.amplitudeC, 4);
    Serial.print(",");
    Serial.print(step.periodS, 4);
  } else {
    Serial.print("0,0.0000,0.0000,0.0000");
  }
  Serial.print(",");
  Serial.print(setpointC, 4);
  Serial.print(",");
  Serial.print(feedforwardPwm, 4);
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(",");
    Serial.print(temperaturesC[i], 2);
  }
  Serial.print(",");
  Serial.print(rawControllerPwm, 4);
  Serial.print(",");
  Serial.print(coolingCompensationRatio, 4);
  Serial.print(",");
  Serial.print(appliedSignedPwm);
  Serial.print(",");
  Serial.print(diagnosticUaPwm, 4);
  Serial.print(",");
  Serial.print(diagnosticUbPwm, 4);
  Serial.print(",");
  Serial.println(diagnosticOmegaRadPerS, 6);
}

void applyHBridgeOutput() {
  appliedSignedPwm = 0;
  bool diagnosticMode = controlMode == "raw_sine"
      || controlMode == "raw_offset_sine";
  if (!safetyShutdownActive && (!scheduleComplete || diagnosticMode)
      && operatingState == "RUNNING") {
    appliedSignedPwm = constrain(requestedSignedPwm, -255, 255);
  }

  // Turn both outputs off before selecting a direction; never drive both.
  digitalWrite(PIN_9, LOW);
  digitalWrite(PIN_10, LOW);
  if (appliedSignedPwm > 0) {
    analogWrite(PIN_9, appliedSignedPwm);
  } else if (appliedSignedPwm < 0) {
    analogWrite(PIN_10, -appliedSignedPwm);
  }
}

void setHBridgeDisabled() {
  analogWrite(PIN_9, 0);
  analogWrite(PIN_10, 0);
  digitalWrite(PIN_9, LOW);
  digitalWrite(PIN_10, LOW);
  appliedSignedPwm = 0;
}

float readTemperature(int pin) {
  analogRead(pin);
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(pin);
  }
  float avgAdc = sum / (float)SAMPLE_COUNT;
  float voltage = (avgAdc / 1023.0) * V_REF;
  if (voltage >= V_REF || voltage <= 0.0) return 0.0;

  float resistance = R_FIXED * voltage / (V_REF - voltage);
  float inverseT = (1.0 / T0_KELVIN)
      + (1.0 / BETA) * log(resistance / R0);
  return (1.0 / inverseT) - 273.15;
}
