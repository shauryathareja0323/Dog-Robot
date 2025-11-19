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
ODriveArduino odrive1(Serial1);
ODriveArduino odrive2(Serial2);
ODriveArduino odrive3(Serial3);
ODriveArduino odrive4(Serial4);
ODriveArduino odrive5(Serial5);
ODriveArduino odrive6(Serial6);

// const float L1 = 20.0;
// const float L2 = 20.0;
// const float ENC_STEP_DEG = 36.0;
// const float HIP_NEUTRAL = -90.0;

// // Joint limits
// const float HIP_MIN = -61.0;
// const float HIP_MAX = 61.0;
// const float KNEE_MIN = -91.0;
// const float KNEE_MAX = 135.0;

// // ----------------- Motion Parameters -----------------
// float hip_target_turns = 0.0f;
// float knee_target_turns = 0.0f;

// // Rectangle coordinates (X, Y)
// const int NUM_POINTS = 4;
// int dist_y = -8;
// int last_dist_y = dist_y;

// int currentPoint = 0;
// unsigned long lastMove = 0;
// const unsigned long moveDelay = 200; // ms per corner

void setup() {
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ODriveArduino Rectangle Path");
  Serial.println("Enter in format: odrive_number,axis_number,position");
  Serial.println("Example: 2,1,0.45");

  for (int axis = 0; axis < 2; ++axis) {
    //Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
    Serial1 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  for (int axis = 0; axis < 2; ++axis) {
    //Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
    Serial2 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  for (int axis = 0; axis < 2; ++axis) {
    //Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
    Serial3 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  for (int axis = 0; axis < 2; ++axis) {
    //Serial4 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
    Serial4 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  for (int axis = 0; axis < 2; ++axis) {
    //Serial5 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
    Serial5 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  for (int axis = 0; axis < 2; ++axis) {
    //Serial6 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
    Serial6 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  // Basic ODrive config setup (unchanged)
  // ... your configuration code remains same ...
  
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive1.run_state(0, requested_state, false)) return;
  if(!odrive1.run_state(1, requested_state, false)) return;
  if(!odrive2.run_state(0, requested_state, false)) return;
  if(!odrive2.run_state(1, requested_state, false)) return;
  if(!odrive3.run_state(0, requested_state, false)) return;
  if(!odrive3.run_state(1, requested_state, false)) return;
  if(!odrive4.run_state(0, requested_state, false)) return;
  if(!odrive4.run_state(1, requested_state, false)) return;
  if(!odrive5.run_state(0, requested_state, false)) return;
  if(!odrive5.run_state(1, requested_state, false)) return;
  if(!odrive6.run_state(0, requested_state, false)) return;
  if(!odrive6.run_state(1, requested_state, false)) return;

  float posGainKnee = 40.0;
  float posGainHips = 60.0;  
  float velGain = 0.1;      
  float integrator = 0.2;  

  Serial1 << "w axis" << 0 << ".controller.config.pos_gain " << posGainHips << '\n';
  Serial1 << "w axis" << 1 << ".controller.config.pos_gain " << posGainHips << '\n';
  delay(50);

  Serial2 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
  Serial2 << "w axis" << 1 << ".controller.config.pos_gain " << posGainKnee << '\n';
  delay(50);

  Serial3 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
  Serial3 << "w axis" << 1 << ".controller.config.pos_gain " << posGainKnee << '\n';
  delay(50);

  Serial4 << "w axis" << 0 << ".controller.config.pos_gain " << posGainHips << '\n';
  Serial4 << "w axis" << 1 << ".controller.config.pos_gain " << posGainHips << '\n';
  delay(50);

  Serial5 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
  Serial5 << "w axis" << 1 << ".controller.config.pos_gain " << posGainKnee << '\n';
  delay(50);

  Serial6 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
  Serial6 << "w axis" << 1 << ".controller.config.pos_gain " << posGainKnee << '\n';
  delay(50);

  Serial1 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
  Serial1 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';
  delay(50);

  Serial2 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
  Serial2 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';
  delay(50);

  Serial3 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
  Serial3 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';
  delay(50);

  Serial4 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
  Serial4 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';
  delay(50);

  Serial5 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
  Serial5 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';
  delay(50);

  Serial6 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
  Serial6 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';
  delay(50);

  Serial1 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  Serial1 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  delay(50);

  Serial2 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  Serial2 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  delay(50);

  Serial3 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  Serial3 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  delay(50);

  Serial4 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  Serial4 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  delay(50);

  Serial5 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  Serial5 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  delay(50);

  Serial6 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  Serial6 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';
  delay(50);

  // odrive1.SetPosition(0, -0.07);
  // odrive1.SetPosition(1, 0.40);
  // delay(50);    
  // odrive4.SetPosition(0, -0.28);
  // odrive4.SetPosition(1, -0.10);
  // delay(50); // For stand
  // odrive1.SetPosition(0, 0.00);
  // odrive1.SetPosition(1, 0.35);
  // delay(50);    
  // odrive4.SetPosition(0, -0.20);
  // odrive4.SetPosition(1, -0.05);
  // delay(5000); 
  // odrive2.SetPosition(0, -2.6);
  // odrive2.SetPosition(1, 0.3);
  // delay(50);
  // odrive3.SetPosition(0, 2.0);
  // odrive3.SetPosition(1, 0.5);
  // delay(50);
  // odrive5.SetPosition(0, 1.90);
  // odrive5.SetPosition(1, 0.50);
  // delay(50);
  // odrive6.SetPosition(0, -2.50);
  // odrive6.SetPosition(1, -0.75);
  // delay(5000);
}

void loop() {
  //----------------- NEW INPUT HANDLER -----------------
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int comma1 = input.indexOf(',');
      int comma2 = input.indexOf(',', comma1 + 1);

      if (comma1 != -1 && comma2 != -1) {
        int odriveNum = input.substring(0, comma1).toInt();
        int axisNum = input.substring(comma1 + 1, comma2).toInt();
        float pos = input.substring(comma2 + 1).toFloat();

        Serial.print("Moving ODrive ");
        Serial.print(odriveNum);
        Serial.print(" Axis ");
        Serial.print(axisNum);
        Serial.print(" to position ");
        Serial.println(pos);

        switch (odriveNum) {
          case 1: odrive1.SetPosition(axisNum, pos); break;
          case 2: odrive2.SetPosition(axisNum, pos); break;
          case 3: odrive3.SetPosition(axisNum, pos); break;
          case 4: odrive4.SetPosition(axisNum, pos); break;
          case 5: odrive5.SetPosition(axisNum, pos); break;
          case 6: odrive6.SetPosition(axisNum, pos); break;
          default:
            Serial.println("Invalid ODrive number (use 1–6).");
            break;
        }
      } else {
        Serial.println("Invalid format. Use: odrive,axis,position (e.g., 3,1,0.75)");
      }
    }
  }

  // You can still keep your motion loop or disable it
  // to test manual commands.

  // odrive1.SetPosition(0, -0.07);
  // odrive1.SetPosition(1, +0.42);
  // odrive4.SetPosition(0, -0.25);
  // odrive4.SetPosition(1, -0.12);
  // odrive2.SetPosition(0, -0.40);
  // odrive2.SetPosition(1, +0.39);
  // odrive3.SetPosition(0, -0.07);
  // odrive3.SetPosition(1, -0.08);
  // odrive5.SetPosition(0, +0.32);
  // odrive5.SetPosition(1, +0.30);
  // odrive6.SetPosition(0, -0.21);
  // odrive6.SetPosition(1, +0.18);
}

// ----------------- IK Function (unchanged) -----------------
// bool inverseKinematics(float x, float y, float &hip_angle_rel, float &knee_angle_rel) {
//   float d = sqrt(x * x + y * y);
//   if (d > (L1 + L2) || d < fabs(L1 - L2)) return false;

//   float cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2);
//   cos_knee = constrain(cos_knee, -1.0, 1.0);
//   float knee_angle_rad = PI - acos(cos_knee);
//   knee_angle_rel = knee_angle_rad * 180.0 / PI;

//   float cos_hip_offset = (d * d + L1 * L1 - L2 * L2) / (2 * L1 * d);
//   cos_hip_offset = constrain(cos_hip_offset, -1.0, 1.0);
//   float hip_offset = acos(cos_hip_offset);
//   float base_angle = atan2(y, x);
//   float hip_angle_abs = base_angle - hip_offset;
//   float hip_angle_deg = hip_angle_abs * 180.0 / PI;

//   hip_angle_rel = hip_angle_deg - HIP_NEUTRAL;

//   if (hip_angle_rel < HIP_MIN || hip_angle_rel > HIP_MAX) return false;
//   if (knee_angle_rel < KNEE_MIN || knee_angle_rel > KNEE_MAX) return false;

//   return true;
// }
