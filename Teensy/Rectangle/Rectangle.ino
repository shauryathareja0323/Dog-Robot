// includes
#include <math.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Teensy 3 and 4 (all versions) - Serial2
// pin 9: RX - connect to ODrive TX
// pin 10: TX - connect to ODrive RX
HardwareSerial& odrive_serial = Serial2;
// ODrive object
ODriveArduino odrive(odrive_serial);

const float L1 = 20.0;  // Hip to knee length (cm)
const float L2 = 20.0;  // Knee to foot length (cm)
const float ENC_STEP_DEG = 36.0; // 1 encoder step = 36°
const float HIP_NEUTRAL = -90.0; // Neutral angle

// Joint limits
const float HIP_MIN = -61.0;
const float HIP_MAX = 61.0;
const float KNEE_MIN = -91.0;
const float KNEE_MAX = 135.0;

// ----------------- Motion Parameters -----------------
float hip_target_turns = 0.0f;
float knee_target_turns = 0.0f;

// Rectangle coordinates (X, Y)
const int NUM_POINTS = 4;
int dist_y = 0;            // default rectangle width
int last_dist_y = dist_y;  // previous value tracker

int currentPoint = 0;
unsigned long lastMove = 0;
const unsigned long moveDelay = 200; // ms per corner

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ODriveArduino Rectangle Path");
  Serial.println("Enter rectangle width (dist_y in cm):");

  // Basic ODrive config
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(0, requested_state, false)) return;
  delay(50);
  if(!odrive.run_state(1, requested_state, false)) return;
  delay(50);

  float posGainKnee = 20.0;
  float posGainHips = 60.0;  
  float velGain = 0.1;      
  float integrator = 0.2;  

  odrive_serial << "w axis" << 0 << ".controller.config.pos_gain " << posGainHips << '\n';
  odrive_serial << "w axis" << 1 << ".controller.config.pos_gain " << posGainKnee << '\n';

  odrive_serial << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
  odrive_serial << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

  odrive_serial << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  odrive_serial << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
}

void loop() {
  // ----------------- Handle User Input -----------------
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // read line until Enter
    input.trim(); // remove spaces and \r

    if (input.length() > 0) {
      int new_val = input.toInt();
      if (new_val >= 0 && new_val != dist_y) {
        last_dist_y = dist_y;
        dist_y = new_val;
        Serial.print("Updated dist_y = ");
        Serial.println(dist_y);
      }
    }
  }

  // ----------------- Motion Loop -----------------
  unsigned long now = millis();

  float rectX[NUM_POINTS] = { 0.0, (float)dist_y, (float)dist_y, 0.0 };   // cm
  float rectY[NUM_POINTS] = { -39.0, -39.0, -29.0, -29 };

  if (now - lastMove > moveDelay) {
    // get next rectangle corner
    float x = rectX[currentPoint];
    float y = rectY[currentPoint];

    // calculate IK
    float hip_angle_rel, knee_angle_rel;
    if (inverseKinematics(x, y, hip_angle_rel, knee_angle_rel)) {
      hip_target_turns = (hip_angle_rel / ENC_STEP_DEG);
      knee_target_turns = (knee_angle_rel / ENC_STEP_DEG);

      // send to ODrive
      odrive.SetPosition(0, hip_target_turns);
      odrive.SetPosition(1, -knee_target_turns);

      // debug print
      // Serial.print("Point ");
      // Serial.print(currentPoint);
      // Serial.print(" -> X=");
      // Serial.print(x);
      // Serial.print(" Y=");
      // Serial.print(y);
      // Serial.print(" | hip=");
      // Serial.print(hip_target_turns, 3);
      // Serial.print(" turns, knee=");
      // Serial.println(knee_target_turns, 3);
    } else {
      Serial.println("IK failed for this point!");
    }

    // go to next corner
    currentPoint = (currentPoint + 1) % NUM_POINTS;
    lastMove = now;
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
