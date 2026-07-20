#include <Servo.h>
#include <math.h>

// Digital pin configuration for 6 servos
const int BASE_PIN = 9;
const int SHOULDER_PIN = 10;
const int ELBOW_PIN = 11;
const int PITCH_PIN = 6;
const int ROLL_PIN = 5;
const int GRIP_PIN = 3;

// Analog pin assignments for the three potentiometers
const int POT_X_PIN = A0;
const int POT_Y_PIN = A1;
const int POT_Z_PIN = A2;

Servo base_s, sh_s, el_s, pitch_s, roll_s, grip_s;

// Link lengths (measured in cm)
const float L1 = 10.0; 
const float L2 = 10.0; 

// Track current operational vector coordinates in 3D space
float cur_x = 0.0;
float cur_y = 10.0;
float cur_z = 5.0; 

// Standby home parameters for structural recovery and startup safety
const float home_x = 0.0;
const float home_y = 10.0;
const float home_z = 5.0; 

// Core geometric tracking engine with automatic vector clamping
void set_arm_position(float x, float y, float z) {
  float r = sqrt(x*x + y*y);
  float d = sqrt(r*r + z*z);
  float max_reach = L1 + L2 - 0.5; // Set safe maximum reach to 19.5cm
  
  // Vector Clamping: Dynamically scale target vector if dial input exceeds physical linkage envelope
  if (d > max_reach) {
    float scale = max_reach / d;
    x *= scale;
    y *= scale;
    z *= scale;
    
    // Recalculate radial and spatial distances post-clamping
    r = sqrt(x*x + y*y);
    d = max_reach;
  }
  
  float t1 = atan2(y, x) * 180.0 / M_PI;
  
  // Law of Cosines calculation for internal link relationships
  float c_alpha = (L1*L1 + d*d - L2*L2) / (2.0 * L1 * d);
  c_alpha = constrain(c_alpha, -1.0, 1.0); // Guard against floating-point precision overflow
  float alpha = acos(c_alpha);
  
  float c_beta = (L1*L1 + L2*L2 - d*d) / (2.0 * L1 * L2);
  c_beta = constrain(c_beta, -1.0, 1.0);
  float beta = acos(c_beta);
  
  float pitch_offset = atan2(z, r);
  
  float t2 = (pitch_offset + alpha) * 180.0 / M_PI;
  float t3 = beta * 180.0 / M_PI;
  
  // Match computed geometric configurations to direct physical hardware orientation limits
  int s1 = constrain(int(t1 + 90), 0, 180); 
  int s2 = constrain(int(t2), 0, 180);
  int s3 = constrain(int(180 - t3), 0, 180); // Inverted due to mechanical bracket assembly orientation
  
  base_s.write(s1);
  sh_s.write(s2);
  el_s.write(s3);
  
  // Lock auxiliary wrist positions level and gripper ready during manual drive
  pitch_s.write(90);
  roll_s.write(90);
  grip_s.write(45);
}

// Linear interpolation trajectory logic used strictly during setup stabilization
void sweep_to(float tx, float ty, float tz, int subdivisions) {
  for (int step = 1; step <= subdivisions; step++) {
    float percent = (float)step / (float)subdivisions;
    float step_x = cur_x + percent * (tx - cur_x);
    float step_y = cur_y + percent * (ty - cur_y);
    float step_z = cur_z + percent * (tz - cur_z);
    
    set_arm_position(step_x, step_y, step_z);
    delay(35); 
  }
  cur_x = tx;
  cur_y = ty;
  cur_z = tz;
}

// Homing utility sequence
void go_home() {
  Serial.println("Executing initialization sweep to home posture...");
  sweep_to(home_x, home_y, home_z, 45); 
  Serial.println("Arm secured at primary standby coordinates.");
}

void setup() {
  Serial.begin(9600);
  Serial.println("System boot initiated. Powering up analog mapping infrastructure...");

  // Staggering pin attachments with 120ms breaks to spread out current spike draw
  base_s.attach(BASE_PIN);     delay(120);
  sh_s.attach(SHOULDER_PIN);   delay(120);
  el_s.attach(ELBOW_PIN);      delay(120);
  pitch_s.attach(PITCH_PIN);   delay(120);
  roll_s.attach(ROLL_PIN);     delay(120);
  grip_s.attach(GRIP_PIN);     delay(120);
  
  // Transition safely to base tracking coordinates before handing over manual control
  go_home();
  
  Serial.println("System operational. Potentiometer target mapping active.");
  delay(1000);
}

void loop() {
  // Read multi-axis potentiometer voltages (returns values between 0 and 1023)
  int raw_x = analogRead(POT_X_PIN);
  int raw_y = analogRead(POT_Y_PIN);
  int raw_z = analogRead(POT_Z_PIN);
  
  // Calibrated workspace mapping bounds
  float target_x = 4.0 + ((raw_x / 1023.0) * 10.0);   // X: 4.0cm to 14.0cm
  float target_y = -8.0 + ((raw_y / 1023.0) * 16.0);  // Y: -8.0cm to 8.0cm
  float target_z = 2.0 + ((raw_z / 1023.0) * 10.0);   // Z: 2.0cm to 12.0cm
  
  // Process target through the kinematic solver
  set_arm_position(target_x, target_y, target_z);
  
  cur_x = target_x;
  cur_y = target_y;
  cur_z = target_z;
  
  // Diagnostic print loop operating on a 300ms window
  static unsigned long last_print = 0;
  if (millis() - last_print > 300) {
    Serial.print("Target XYZ -> X: "); Serial.print(cur_x);
    Serial.print(" | Y: "); Serial.print(cur_y);
    Serial.print(" | Z: "); Serial.println(cur_z);
    last_print = millis();
  }
  
  delay(40); // Baseline control refresh interval
}
