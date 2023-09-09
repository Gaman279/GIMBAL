#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>

using namespace webots;

// Constants for PID control
const double KP = 0.5;  // Proportional gain
const double KI = 0.1;  // Integral gain
const double KD = 0.2;  // Derivative gain

// Variables for PID control
double last_error[2] = {0, 0};   // Previous error values for derivative term
double integral[2] = {0, 0};     // Accumulated error for integral term

// Target angles for stabilization (in radians)
double target_angles[2] = {0, 0};  // Desired gimbal angles (roll, pitch)

// Function to calculate control signal
double* calculate_control_signal(double current_angles[], double delta_time) {
  // Calculate error for each axis
  double errors[2] = {target_angles[0] - current_angles[0], target_angles[1] - current_angles[1]};

  // Calculate proportional term
  double proportional[2] = {KP * errors[0], KP * errors[1]};

  // Calculate integral term
  integral[0] += errors[0] * delta_time;
  integral[1] += errors[1] * delta_time;
  integral[0] *= KI;
  integral[1] *= KI;

  // Calculate derivative term
  double derivative[2] = {(errors[0] - last_error[0]) / delta_time, (errors[1] - last_error[1]) / delta_time};
  derivative[0] *= KD;
  derivative[1] *= KD;

  // Calculate control signal
  static double control_signal[2];
  control_signal[0] = proportional[0] + integral[0] + derivative[0];
  control_signal[1] = proportional[1] + integral[1] + derivative[1];

  // Update previous error values
  last_error[0] = errors[0];
  last_error[1] = errors[1];

  return control_signal;
}

int main(int argc, char **argv) {
  // Create a Webots robot instance
  Robot *robot = new Robot();

  // Get devices and enable sensors
  PositionSensor *gimbal_roll_sensor = robot->getPositionSensor("roll");
  PositionSensor *gimbal_pitch_sensor = robot->getPositionSensor("pitch");
  gimbal_roll_sensor->enable(16);  // Enable position sensor with 16ms refresh rate
  gimbal_pitch_sensor->enable(16); // Enable position sensor with 16ms refresh rate

  Motor *gimbal_roll_motor = robot->getMotor("motor3");
  Motor *gimbal_pitch_motor = robot->getMotor("motor4");
  gimbal_roll_motor->setPosition(INFINITY);  // Enable position control
  gimbal_pitch_motor->setPosition(INFINITY); // Enable position control

  // Enable keyboard for teleoperation
  Keyboard *keyboard = robot->getKeyboard();
  keyboard->enable(16);  // Enable keyboard input with 16ms refresh rate

  // Time step for simulation
  int time_step = (int)robot->getBasicTimeStep();

  // Main control loop
  while (robot->step(time_step) != -1) {
    // Get current gimbal angles from sensors (in radians)
    double current_angles[2] = {gimbal_roll_sensor->getValue(), gimbal_pitch_sensor->getValue()};

    // Calculate control signal
    double *control_signal = calculate_control_signal(current_angles, time_step / 1000.0);  // Convert time_step to seconds

    // Apply control signal to adjust gimbal angles
    gimbal_roll_motor->setVelocity(control_signal[0]);
    gimbal_pitch_motor->setVelocity(control_signal[1]);

    // Teleoperate gimbal with keyboard inputs
    int key = keyboard->getKey();
    if (key == Keyboard::UP) {  // Pitch up
      target_angles[1] += 0.1;
    } else if (key == Keyboard::DOWN) {  // Pitch down
      target_angles[1] -= 0.1;
    } else if (key == Keyboard::LEFT) {  // Roll left
      target_angles[0] -= 0.1;
    } else if (key == Keyboard::RIGHT) {  // Roll right
      target_angles[0] += 0.1;
    }
  }

  delete robot;
  return 0;
}
