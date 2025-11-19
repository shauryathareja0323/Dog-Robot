#include <math.h>
#include <ODriveArduino.h>

// ----------------- Teensy + ODrive UART Setup -----------------
HardwareSerial& odrive_serial = Serial2;  // Use Serial2 for ODrive
ODriveArduino odrive(odrive_serial);

// ----------------- Leg Geometry -----------------
const float L1 = 20.0;  // Hip to knee length (cm)
const float L2 = 20.0;  // Knee to foot length (cm)
const float ENC_STEP_DEG = 36.0; // 1 encoder step = 36°
const float GEAR_RATIO = 10.0;   // Gear ratio
const float HIP_NEUTRAL = -90.0; // Neutral angle

// Joint limits
const float HIP_MIN = -60.0;
const float HIP_MAX = 60.0;
const float KNEE_MIN = -91.0;
const float KNEE_MAX = 91.0;

// ----------------- Motion Parameters -----------------
float hip_target_turns = 0.0;
float knee_target_turns = 0.0;

float hip_curr_turns = 0.0;
float knee_curr_turns = 0.0;

float max_speed = 0.002;   // max turns per update (safety)
unsigned long lastUpdate = 0;
const int updateInterval = 10; // 100 Hz update

// Oscillation state
static float y = -29.0;       // starting Y position
static bool goingDown = true; // direction flag

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  odrive_serial.begin(115200);

  Serial.println("Teensy IK + ODrive (Smooth Linear Increment)");

  // Put ODrive in CLOSED_LOOP_CONTROL
  odrive_serial.println("w axis0.requested_state 8");
  odrive_serial.println("w axis1.requested_state 8");
  delay(20);
}

// ----------------- Main Loop -----------------
void loop() {
  unsigned long now = millis();

  // 1. Smooth linear oscillation of Y
  if (goingDown) {
    y -= 0.1;  // small step for smooth motion
    if (y <= -39.0) goingDown = false;
  } else {
    y += 0.1;
    if (y >= -29.0) goingDown = true;
  }

  float x = 0.0;  // fixed X

  // 2. Calculate IK for (x, y)
  float hip_angle_rel, knee_angle_rel;
  if (inverseKinematics(x, y, hip_angle_rel, knee_angle_rel)) {
    hip_target_turns = (hip_angle_rel / ENC_STEP_DEG) / GEAR_RATIO;
    knee_target_turns = (knee_angle_rel / ENC_STEP_DEG) / GEAR_RATIO;
  }

  // 3. Update motors at fixed rate
  if (now - lastUpdate >= updateInterval) {
    lastUpdate = now;

    // Smooth speed-limited motion
    hip_curr_turns = moveTowards(hip_curr_turns, hip_target_turns, max_speed);
    knee_curr_turns = moveTowards(knee_curr_turns, -knee_target_turns, max_speed);

    odrive.SetPosition(0, hip_curr_turns * 10.0);  // scaling for ODrive
    odrive.SetPosition(1, knee_curr_turns * 10.0);
    delay(5);

    // Debug output
    Serial.print("y=");
    Serial.print(y, 3);
    Serial.print("  hip=");
    Serial.print(hip_curr_turns*10, 4);
    Serial.print("  knee=");
    Serial.println(knee_curr_turns*10, 4);
  }
}

// ----------------- Smooth Move Function -----------------
float moveTowards(float current, float target, float maxStep) {
  if (fabs(target - current) <= maxStep) return target;
  return (target > current) ? current + maxStep : current - maxStep;
}

// ----------------- IK Function -----------------
bool inverseKinematics(float x, float y, float &hip_angle_rel, float &knee_angle_rel) {
  float d = sqrt(x * x + y * y);
  if (d > (L1 + L2) || d < fabs(L1 - L2)) return false;

  float cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2);
  cos_knee = constrain(cos_knee, -1.0, 1.0);
  float knee_angle_rad = PI - acos(cos_knee);
  knee_angle_rel = knee_angle_rad * 180.0 / PI;

  float cos_hip_offset = (d * d + L1 * L1 - L2 * L2) / (2 * L1 * d);
  cos_hip_offset = constrain(cos_hip_offset, -1.0, 1.0);
  float hip_offset = acos(cos_hip_offset);
  float base_angle = atan2(y, x);
  float hip_angle_abs = base_angle - hip_offset;
  float hip_angle_deg = hip_angle_abs * 180.0 / PI;

  hip_angle_rel = hip_angle_deg - HIP_NEUTRAL;

  if (hip_angle_rel < HIP_MIN || hip_angle_rel > HIP_MAX) return false;
  if (knee_angle_rel < KNEE_MIN || knee_angle_rel > KNEE_MAX) return false;

  return true;
}
