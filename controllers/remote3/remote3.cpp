#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 64
#define POSITION_THRESHOLD 0.1 // Adjust the threshold as needed

using namespace webots;

int main(int argc, char **argv) {
  
  Robot *robot = new Robot();
  
  Keyboard kb;
  
  Motor *wheels[4];
  
  PositionSensor *posSensors[4];
  
  Camera *cm;
  cm = robot->getCamera("CAM");
  cm->enable(TIME_STEP);
  
  char wheels_names[4][8] = {"motor1", "motor2", "motor3", "motor4"};
  
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
    posSensors[i] = wheels[i]->getPositionSensor();
    posSensors[i]->enable(TIME_STEP);
  }
  
  kb.enable(TIME_STEP);
  double pitchSpeed = 0.0;
  double rollSpeed = 0.0;
  double droneSpeed = 0.0;
  double motor3Angle = 0.0;
  double motor4Angle = 0.0;
  
  while (robot->step(TIME_STEP) != -1) {
    int key = kb.getKey();
    
    switch (key) {
      case 'W': 
        pitchSpeed = 10.0;
        break;
      case 'S':
        pitchSpeed = -10.0;
        break;
      case 'A':
        rollSpeed = -10.0;
        break;
      case 'D':
        rollSpeed = 10.0;
        break;
      case 'M':
        droneSpeed = 10.0;
        break;
      case 'N':
        droneSpeed = -10.0;
        break;
      default:
        rollSpeed = 0.0;
        pitchSpeed = 0.0;
        droneSpeed = 0.0;
        break;
    }
    
    wheels[2]->setVelocity(rollSpeed);  // Stabilize motor3
    wheels[3]->setVelocity(pitchSpeed);
    wheels[1]->setVelocity(droneSpeed);
    
    // Increase torque and force for motor2
    wheels[1]->setTorque(10.0);  // Adjust the torque value as needed
    wheels[1]->setForce(10.0);   // Adjust the force value as needed
    
    // Update motor angles for motor3 and motor4
    motor3Angle += wheels[2]->getVelocity() * TIME_STEP / 1000.0;
    motor4Angle += wheels[3]->getVelocity() * TIME_STEP / 1000.0;
    
    // Apply motor angle correction to stabilize the solid attached to motor3 and motor4
    // Assuming you have some solid attached to motor3 and motor4, update the angles as needed.
    // For example, if you want to keep them at 0 degrees:
    double desiredAngle = 0.0;
    wheels[2]->setPosition(desiredAngle - motor3Angle);
    wheels[3]->setPosition(desiredAngle - motor4Angle);
  }
  
  delete robot;
  return 0;  // EXIT_SUCCESS
}

