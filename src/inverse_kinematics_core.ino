#include <Servo.h>
#include <math.h>

// Grouping pins for easier assembly wiring layout
const int BASE_PIN = 9;
const int SHOULDER_PIN = 10;
const int ELBOW_PIN = 11;
const int PITCH_PIN = 6;
const int ROLL_PIN = 5;
const int GRIP_PIN = 3;

Servo base_s, sh_s, el_s, pitch_s, roll_s, grip_s;

// Constants using estimated lengths (updating with calipers on August 24)
const float L1 = 10.0; 
const float L2 = 10.0; 

// Initial standby coordinate
float cur_x = 0.0;
float cur_y = 10.0;
float cur_z = 5.0; 

// Autonomous movement target checklist
float path[][3] = {
  {12.0,  0.0,  8.0},  
  { 8.0,  8.0,  5.0},  
  {10.0, -5.0,  7.0},  
  { 0.0, 10.0,  5.0}   
};

const int path_len = sizeof(path) / sizeof(path[0]);
int target_idx = 0;

// Core geometric engine
void set_arm_position(float x, float y, float z) {
  float r = sqrt(x*x + y*y);
  float d = sqrt(r*r + z*z);
  
  // Boundary guardrail: don't tear the hardware apart if point is out of range
  if (d > (L1 + L2)) {
    Serial.println("Warning: Target out of physical envelope bounds");
    return; 
  }
  
  float t1 = atan2(y, x) * 180.0 / M_PI;
  
  // Law of Cosines to solve the interior shoulder/elbow triangle
  float c_alpha = (L1*L1 + d*d - L2*L2) / (2.0 * L1 * d);
  float alpha = acos(c_alpha);
  
  float c_beta = (L1*L1 + L2*L2 - d*d) / (2.0 * L1 * L2);
  float beta = acos(c_beta);
  
  float pitch_offset = atan2(z, r);
  
  float t2 = (pitch_offset + alpha) * 180.0 / M_PI;
  float t3 = beta * 180.0 / M_PI;
  
  // Mapping geometric math angles directly to physical 0-180 degree micro-servos
  int s1 = constrain(int(t1 + 90), 0, 180); // Center standard base rotation
  int s2 = constrain(int(t2), 0, 180);
  int s3 = constrain(int(180 - t3), 0, 180); // Inverted due to mechanical bracket orientation
  
  base_s.write(s1);
  sh_s.write(s2);
  el_s.write(s3);
  
  // Keeping wrist level and gripper ready during transit
  pitch_s.write(90);
  roll_s.write(90);
  grip_s.write(45);
}

// Linear interpolation (Lerp) to break up jerky movements
void sweep_to(float tx, float ty, float tz, int subdivisions) {
  for (int step = 1; step <= subdivisions; step++) {
    float percent = (float)step / (float)subdivisions;
    
    // Lerp math structure: original + delta * percent
    float step_x = cur_x + percent * (tx - cur_x);
    float step_y = cur_y + percent * (ty - cur_y);
    float step_z = cur_z + percent * (tz - cur_z);
    
    set_arm_position(step_x, step_y, step_z);
    delay(35); // Gives analog servos time to physically travel to target increment
  }
  
  // Set position tracking state values
  cur_x = tx;
  cur_y = ty;
  cur_z = tz;
}

void setup() {
  base_s.attach(BASE_PIN);
  sh_s.attach(SHOULDER_PIN);
  el_s.attach(ELBOW_PIN);
  pitch_s.attach(PITCH_PIN);
  roll_s.attach(ROLL_PIN);
  grip_s.attach(GRIP_PIN);
  
  Serial.begin(9600);
  Serial.println("Ready. Interpolation controller sweep active.");
}

void loop() {
  float tx = path[target_idx][0];
  float ty = path[target_idx][1];
  float tz = path[target_idx][2];
  
  sweep_to(tx, ty, tz, 40); // 40 sub-steps over roughly 1.4 seconds of transit
  
  target_idx = (target_idx + 1) % path_len;
  delay(2500); 
}
