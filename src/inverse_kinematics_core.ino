#include <Servo.h>
#include <math.h>

const int BASE_PIN = 9;
const int SHOULDER_PIN = 10;
const int ELBOW_PIN = 11;
const int PITCH_PIN = 6;
const int ROLL_PIN = 5;
const int GRIP_PIN = 3;

Servo base_s, sh_s, el_s, pitch_s, roll_s, grip_s;

const float L1 = 10.0; 
const float L2 = 10.0; 

// Initial tracking variables matching the hardware standby posture
float cur_x = 0.0;
float cur_y = 10.0;
float cur_z = 5.0; 

// Permanent safe coordinates for system startup and return paths
const float home_x = 0.0;
const float home_y = 10.0;
const float home_z = 5.0; 

float path[][3] = {
  {12.0,  0.0,  8.0},  
  { 8.0,  8.0,  5.0},  
  {10.0, -5.0,  7.0}   
};

const int path_len = sizeof(path) / sizeof(path[0]);
int target_idx = 0;

void set_arm_position(float x, float y, float z) {
  float r = sqrt(x*x + y*y);
  float d = sqrt(r*r + z*z);
  
  if (d > (L1 + L2)) {
    Serial.println("Warning: Target out of physical bounds!");
    return; 
  }
  
  float t1 = atan2(y, x) * 180.0 / M_PI;
  float c_alpha = (L1*L1 + d*d - L2*L2) / (2.0 * L1 * d);
  float alpha = acos(c_alpha);
  float c_beta = (L1*L1 + L2*L2 - d*d) / (2.0 * L1 * L2);
  float beta = acos(c_beta);
  float pitch_offset = atan2(z, r);
  
  float t2 = (pitch_offset + alpha) * 180.0 / M_PI;
  float t3 = beta * 180.0 / M_PI;
  
  int s1 = constrain(int(t1 + 90), 0, 180); 
  int s2 = constrain(int(t2), 0, 180);
  int s3 = constrain(int(180 - t3), 0, 180); 
  
  base_s.write(s1);
  sh_s.write(s2);
  el_s.write(s3);
  
  pitch_s.write(90);
  roll_s.write(90);
  grip_s.write(45);
}

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

// Moves the chassis smoothly from its current coordinates to the home position
void go_home() {
  Serial.println("Executing homing routine...");
  sweep_to(home_x, home_y, home_z, 45); 
  Serial.println("Arm locked at home posture.");
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing hardware power rails...");

  // Delaying pin attachments to spread out electric current draw spikes
  base_s.attach(BASE_PIN);     delay(120);
  sh_s.attach(SHOULDER_PIN);   delay(120);
  el_s.attach(ELBOW_PIN);      delay(120);
  pitch_s.attach(PITCH_PIN);   delay(120);
  roll_s.attach(ROLL_PIN);     delay(120);
  grip_s.attach(GRIP_PIN);     delay(120);
  
  // Anchor arm to starting frame parameters
  go_home();
  
  Serial.println("Boot clear. Commencing main path script in 2 seconds.");
  delay(2000); 
}

void loop() {
  float tx = path[target_idx][0];
  float ty = path[target_idx][1];
  float tz = path[target_idx][2];
  
  sweep_to(tx, ty, tz, 40); 
  delay(2000); 
  
  target_idx++;
  
  // Home the robot at the end of a process cycle before starting over
  if (target_idx >= path_len) {
    Serial.println("Path complete. Resetting sequence via home node.");
    go_home();
    delay(3500); 
    target_idx = 0;
  }
}
