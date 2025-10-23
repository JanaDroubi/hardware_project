#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "InverseKinematics.h"
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Screen.h"

// --- Pins ---
#define ENA 0  // Enable pin (LOW = enabled)

// --- Stepper motor objects ---
extern AccelStepper motorA;
extern AccelStepper motorB;
extern AccelStepper motorC;
extern MultiStepper motors;

// --- Motor parameters ---
#define ANGLE_TO_ORIGIN_A 71
#define ANGLE_TO_ORIGIN_B 66.75
#define ANGLE_TO_ORIGIN_C 61.95
#define ks 100.0

extern long int pos[3];      // Step positions
extern double speed[3];
extern double speed_prev[3];

// --- Function prototypes ---
void motor_init();
long int angle_to_steps(double angle);
double steps_to_angle(int steps);
void home_motors();
void go_home();
void move_to_angle(double theta_deg, double phi_deg, double h, double speed[3]);
void speed_controller(double speed[3]);
void test_motor_speed();

#endif
