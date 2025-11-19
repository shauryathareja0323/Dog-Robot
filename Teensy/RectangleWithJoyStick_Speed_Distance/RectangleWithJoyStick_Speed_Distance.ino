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
HardwareSerial& odrive_serial = Serial2;
ODriveArduino odrive(odrive_serial);

// Leg geometry
const float L1 = 20.0;  
const float L2 = 20.0;  
const float ENC_STEP_DEG = 36.0; 
const float HIP_NEUTRAL = -90.0; 

// Joint limits
const float HIP_MIN = -61.0;
const float HIP_MAX = 61.0;
const float KNEE_MIN = -91.0;
const float KNEE_MAX = 135.0;

// ----------------- Motion Parameters -----------------
float hip_target_turns = 0.0f;
float knee_target_turns = 0.0f;

const int NUM_POINTS = 4;
int dist_y = 0;            
int currentPoint = 0;
unsigned long lastMove = 0;
unsigned long moveDelay = 500; 

// ----------------- Joystick Pins -----------------
const int joyX = 2;  // Horizontal PWM input
const int joyY = 3;  // Vertical PWM input

void setup() {
  odrive_serial.begin(115200);
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ODrive + Joystick Rectangle Path");

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

  odrive_serial << "w axis0.controller.config.pos_gain " << posGainHips << '\n';
  odrive_serial << "w axis1.controller.config.pos_gain " << posGainKnee << '\n';
  odrive_serial << "w axis0.controller.config.vel_gain " << velGain << '\n';
  odrive_serial << "w axis1.controller.config.vel_gain " << velGain << '\n';
  odrive_serial << "w axis0.controller.config.vel_integrator_gain " << integrator << '\n';
  odrive_serial << "w axis1.controller.config.vel_integrator_gain " << integrator << '\n';

  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);
}

void loop() {
  // ----------------- Read Joysticks -----------------
  unsigned long xJoy = pulseIn(joyX, HIGH, 25000UL);
  unsigned long yJoy = pulseIn(joyY, HIGH, 25000UL);

  // Serial.print("X: ");
  // Serial.print(xJoy);
  // Serial.print("   Y:");
  // Serial.println(yJoy);

  // ----------------- Deadzone Logic (X only) -----------------
  if (xJoy <= 1000UL) {
    // X below threshold → stop movement
    return;
  }

  // ----------------- Map Joystick Values -----------------
  // X controls moveDelay (990 → 2000 → 500 → 200 ms)
  xJoy = constrain(xJoy, 990UL, 2000UL);
  moveDelay = map(xJoy, 990, 2000, 500, 200);

  // Y controls step size (1500 → 2000 → 0 → 8 cm)
  if (yJoy < 1500UL) {
    dist_y = 0;
  } else {
    yJoy = constrain(yJoy, 1500UL, 2000UL);
    dist_y = map(yJoy, 1500, 2000, 1, 8);
  }

  // ----------------- Motion Loop -----------------
  unsigned long now = millis();

  float rectX[NUM_POINTS] = { 0.0, (float)dist_y, (float)dist_y, 0.0 };   // cm
  float rectY[NUM_POINTS] = { -39.0, -39.0, -29.0, -29.0 };

  if (now - lastMove > moveDelay) {
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

      Serial.print("Point ");
      Serial.print(currentPoint);
      Serial.print(" | XJoy=");
      Serial.print(xJoy);
      Serial.print(" YJoy=");
      Serial.print(yJoy);
      Serial.print(" | Delay=");
      Serial.print(moveDelay);
      Serial.print(" ms, dist_y=");
      Serial.println(dist_y);
    } else {
      Serial.println("IK failed for this point!");
    }

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
