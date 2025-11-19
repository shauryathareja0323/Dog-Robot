#include <SoftwareSerial.h>
#include <math.h>

// ----------------- Serial & ODrive Setup -----------------
SoftwareSerial odriveSerial(10, 11); // RX, TX for ODrive UART

// ----------------- Leg Geometry -----------------
const float L1 = 20.0;  // Hip to knee length (cm)
const float L2 = 20.0;  // Knee to foot length (cm)
const float ENC_STEP_DEG = 36.0; // 1 encoder step = 36°
const float GEAR_RATIO = 10.0;   // Gearbox ratio
const float HIP_NEUTRAL = -90.0; // Reference angle

// Joint limits
const float HIP_MIN = -60.0;
const float HIP_MAX = 60.0;
const float KNEE_MIN = -91.0;
const float KNEE_MAX = 91.0;

// ----------------- Control Variables -----------------
float hip_target_turns = 0.0;
float knee_target_turns = 0.0;

unsigned long lastUpdate = 0;
const int updateInterval = 20; // 50 Hz update

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  odriveSerial.begin(115200);

  Serial.println("Dog Leg IK + ODrive Control Ready");
  Serial.println("Enter coordinates as: x,y (cm)");

  // Set ODrive to closed-loop control mode
  sendODriveCommand("w axis0.requested_state 8");
  delay(50);
  sendODriveCommand("w axis1.requested_state 8");
  delay(50);
}

// ----------------- Main Loop -----------------
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("Invalid input! Use: x,y");
      return;
    }

    float x = input.substring(0, commaIndex).toFloat();
    float y = input.substring(commaIndex + 1).toFloat();

    float hip_angle_rel, knee_angle_rel;
    if (inverseKinematics(x, y, hip_angle_rel, knee_angle_rel)) {
      // Convert to encoder steps
      float hip_steps = hip_angle_rel / ENC_STEP_DEG;
      float knee_steps = knee_angle_rel / ENC_STEP_DEG;

      // Convert steps → turns (divide by gear ratio)
      hip_target_turns = hip_steps / GEAR_RATIO;
      knee_target_turns = knee_steps / GEAR_RATIO;

      Serial.print("IK OK -> Hip: ");
      Serial.print(hip_angle_rel);
      Serial.print("° (");
      Serial.print(hip_target_turns, 4);
      Serial.print(" turns), Knee: ");
      Serial.print(knee_angle_rel);
      Serial.print("° (");
      Serial.print(knee_target_turns, 4);
      Serial.println(" turns)");
    } else {
      Serial.println("No IK solution (out of range).");
    }
  }

  // Send commands to ODrive at 50 Hz
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();
    sendInputPos(0, hip_target_turns);
    sendInputPos(1, -knee_target_turns);
  }
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

// ----------------- ODrive Command Functions -----------------
void sendInputPos(int axis, float pos) {
  String cmd = "w axis" + String(axis) + ".controller.input_pos " + String(pos*10, 4);
  sendODriveCommand(cmd);
}

void sendODriveCommand(String cmd) {
  odriveSerial.println(cmd);
}
