#include <Servo.h>
#include <math.h> // Includes trigonometric math functions like acos, atan2, sqrt

// Assign servos to names
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

// --- STEP 1: DEFINE ROBOT LINKS (PLACEHOLDERS) ---
const float L1 = 10.0; // Upper arm length in cm (Update this on Aug 24)
const float L2 = 10.0; // Forearm length in cm  (Update this on Aug 24)

void setup() {
  baseServo.attach(9);
  shoulderServo.attach(10);
  elbowServo.attach(11);
  
  Serial.begin(9600); // Open serial monitor to watch calculations
  
  // --- STEP 4: TEST RUN ---
  // Let's command the arm to reach forward to coordinate X=12, Y=0, Z=8
  move_to_coordinate(12.0, 0.0, 8.0);
}

void loop() {
  // Keeping this clean while testing coordinates
}

// --- STEP 2 & 3: THE INVERSE KINEMATICS ENGINE ---
void move_to_coordinate(float x, float y, float z) {
  
  // 1. Calculate Base Rotation (Theta 1)
  float theta1_rad = atan2(y, x);
  float theta1_deg = theta1_rad * 180.0 / M_PI; // Convert to degrees
  
  // 2. Calculate projected horizontal radius (r) on the ground
  float r = sqrt(x*x + y*y);
  
  // 3. Calculate distance from shoulder to target point (D)
  float D = sqrt(r*r + z*z);
  
  // Safety Boundary Check: Is the point too far out of reach?
  if (D > (L1 + L2)) {
    Serial.println("ERROR: Target out of reach!");
    return;
  }
  
  // 4. Law of Cosines to solve for interior triangle angles
  float cos_alpha = (L1*L1 + D*D - L2*L2) / (2.0 * L1 * D);
  float alpha_rad = acos(cos_alpha);
  
  float cos_beta = (L1*L1 + L2*L2 - D*D) / (2.0 * L1 * L2);
  float beta_rad = acos(cos_beta);
  
  // Angle of elevation from ground to target coordinate
  float ground_angle_rad = atan2(z, r);
  
  // 5. Compute absolute joint angles
  float theta2_deg = (ground_angle_rad + alpha_rad) * 180.0 / M_PI; // Shoulder joint
  float theta3_deg = (beta_rad) * 180.0 / M_PI;                     // Elbow joint
  
  // Map standard geometry calculations to safe physical servo positions (0-180)
  int servo1 = constrain(int(theta1_deg + 90), 0, 180);
  int servo2 = constrain(int(theta2_deg), 0, 180);
  int servo3 = constrain(int(180 - theta3_deg), 0, 180); // Inverted to handle joint hinge direction
  
  // --- STEP 5: DRIVE THE SIMULATION ---
  baseServo.write(servo1);
  shoulderServo.write(servo2);
  elbowServo.write(servo3);
  
  // Print values to the Serial Monitor for tracking
  Serial.print("Base: "); Serial.print(servo1);
  Serial.print(" | Shoulder: "); Serial.print(servo2);
  Serial.print(" | Elbow: "); Serial.println(servo3);
}
