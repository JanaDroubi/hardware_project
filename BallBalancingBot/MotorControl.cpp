#include "MotorControl.h"

// --- Stepper motor objects ---
AccelStepper motorA(AccelStepper::DRIVER, 3, 2);  // STEP, DIR
AccelStepper motorC(AccelStepper::DRIVER, 5, 4);
AccelStepper motorB(AccelStepper::DRIVER, 7, 6);
MultiStepper motors;

// --- Global variables ---
long int pos[3] = {0,0,0};
double speed[3] = {0,0,0};
double speed_prev[3] = {0,0,0};

// --- Initialize motors ---
void motor_init() {
  motors.addStepper(motorA);
  motors.addStepper(motorB);
  motors.addStepper(motorC);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  delay(2000);
  digitalWrite(ENA, LOW);
}

// --- Convert angle to steps ---
long int angle_to_steps(double angle) {
  return round((3200.0 / 360.0) * angle);
}

// --- Convert steps to angle ---
double steps_to_angle(int steps) {
  return (360.0 / 3200.0) * steps;
}

// --- Home motors ---
void home_motors() {
  motorA.setMaxSpeed(1000);
  motorB.setMaxSpeed(1000);
  motorC.setMaxSpeed(1000);

  pos[0] = angle_to_steps(ANGLE_TO_ORIGIN_A);
  pos[1] = angle_to_steps(ANGLE_TO_ORIGIN_B);
  pos[2] = angle_to_steps(ANGLE_TO_ORIGIN_C);

  motors.moveTo(pos);
  motors.runSpeedToPosition();

  motorA.setCurrentPosition(0);
  motorB.setCurrentPosition(0);
  motorC.setCurrentPosition(0);
}

// --- Move to zero ---
void go_home() {
  for (int i = 0; i < 3; i++) pos[i] = 0;
  motors.moveTo(pos);
  motors.runSpeedToPosition();
}

// --- Move to angle ---
void move_to_angle(double theta_deg, double phi_deg, double h, double speed[3]) {
  CalculatedAngles result = get_angles(theta_deg, phi_deg, h);
  pos[0] = angle_to_steps(result.thetaA);
  pos[1] = angle_to_steps(result.thetaB);
  pos[2] = angle_to_steps(result.thetaC);

  motorA.setMaxSpeed(speed[0]); motorA.setAcceleration(speed[0]*50);
  motorB.setMaxSpeed(speed[1]); motorB.setAcceleration(speed[1]*50);
  motorC.setMaxSpeed(speed[2]); motorC.setAcceleration(speed[2]*50);

  motors.moveTo(pos);
  for (int i = 0; i < 10; i++) {
    if (motors.run()) break;
  }
}

// --- Speed controller ---
void speed_controller(double speed[3]) {
  static double current_pos[3];
  for (int i = 0; i < 3; i++) {
    speed_prev[i] = speed[i];
    if(i==0) current_pos[i] = motorA.currentPosition();
    if(i==1) current_pos[i] = motorB.currentPosition();
    if(i==2) current_pos[i] = motorC.currentPosition();
    speed[i] = abs(current_pos[i] - pos[i]) * ks;
    speed[i] = constrain(speed[i], speed_prev[i]-300, speed_prev[i]+300);
    speed[i] = constrain(speed[i], 0, 1300);
  }
}

// --- Test motor speed ---
void test_motor_speed() {
  motorA.setMaxSpeed(4000);
  motorA.setAcceleration(10000);
  motorA.moveTo(500);
  unsigned long start = millis();
  while(motorA.distanceToGo()!=0) motorA.run();
  unsigned long duration = millis() - start;
  Serial.println("Time to move 500 steps: " + String(duration) + "ms");
  Serial.println("Actual speed: " + String(500.0 / (duration / 1000.0)) + " steps/sec");
}
