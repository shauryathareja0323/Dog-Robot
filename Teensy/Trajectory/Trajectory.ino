#include <math.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing helper
template<class T> inline Print& operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
template<> inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive Serial
HardwareSerial& odrive_serial = Serial2;
ODriveArduino odrive(odrive_serial);

// Link lengths
const float L1 = 20.0;  // Hip to knee length (cm)
const float L2 = 20.0;  // Knee to foot length (cm)

// Encoder conversion constants
const float ENC_STEP_DEG = 36.0; // 1 encoder step = 36°
const float HIP_NEUTRAL = -90.0; // Neutral angle

// Joint limits
const float HIP_MIN = -61.0;
const float HIP_MAX = 61.0;
const float KNEE_MIN = -91.0;
const float KNEE_MAX = 135.0;

// Motion params
float hip_target_turns = 0.0;
float knee_target_turns = 0.0;

unsigned long lastUpdate = 0;

// ---- Trajectory Parameters ----
const float X_FIXED = 0.0;     // Fixed X
const float Y_CENTER = -30.0;  // Center of oscillation
const float Y_AMPLITUDE = 10.0; // Oscillation amplitude
const float FREQUENCY = 0.5;   // Hz (cycles per second)
const int UPDATE_MS = 5;       // Update every 5 ms (~200 Hz)

void setup() {
  odrive_serial.begin(115200);
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ODrive Smooth Trajectory Example");

  // Set ODrive parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
  }

  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  if (!odrive.run_state(0, requested_state, false)) return;
  delay(50);
  if (!odrive.run_state(1, requested_state, false)) return;

  // Set PID gains
  odrive_serial << "w axis0.controller.config.pos_gain " << 60.0 << '\n';
  odrive_serial << "w axis1.controller.config.pos_gain " << 60.0 << '\n';
  odrive_serial << "w axis0.controller.config.vel_gain " << 1.0 << '\n';
  odrive_serial << "w axis1.controller.config.vel_gain " << 1.0 << '\n';
  odrive_serial << "w axis0.controller.config.vel_integrator_gain " << 2.0 << '\n';
  odrive_serial << "w axis1.controller.config.vel_integrator_gain " << 2.0 << '\n';
}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_MS) {
    lastUpdate = now;

    // ---- Smooth Trajectory for Y ----
    float t = now / 1000.0;  // time in seconds
    float y = Y_CENTER + Y_AMPLITUDE * sin(2 * PI * FREQUENCY * t);

    // ---- Inverse Kinematics ----
    float hip_angle_rel, knee_angle_rel;
    if (inverseKinematics(X_FIXED, y, hip_angle_rel, knee_angle_rel)) {
      hip_target_turns = (hip_angle_rel / ENC_STEP_DEG);
      knee_target_turns = (knee_angle_rel / ENC_STEP_DEG);

      // Send to ODrive
      odrive.SetPosition(0, hip_target_turns);
      odrive.SetPosition(1, -knee_target_turns);
    }

    // Debug info
    Serial.print("t=");
    Serial.print(t, 2);
    Serial.print("  y=");
    Serial.print(y, 3);
    Serial.print("  hip=");
    Serial.print(hip_target_turns, 4);
    Serial.print("  knee=");
    Serial.println(knee_target_turns, 4);
  }
}

// ---- Inverse Kinematics ----
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
