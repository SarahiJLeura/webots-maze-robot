#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6
#define NUM_MOTORS 4
#define NUM_SENSORS 2

using namespace webots;

// For simplicity, variables will be global

Motor* motors[NUM_MOTORS]; // 4 motors: one for each wheel
DistanceSensor *ds[NUM_SENSORS]; // 2 front-facing distance sensors
Camera *camera;

void initializeMotors(Robot *robot);
void initializeSensors(Robot *robot);
void initializeCamera(Robot *robot);
char detectArrow();

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  
  initializeSensors(robot);
  initializeMotors(robot);
  initializeCamera(robot);
  
  int retrocedeCounter = 0; // Counter to reverse the robot before turning
  int avoidObstacleCounter = 0; // Counter to rotate the robot
  char arrow = 'N';
  
  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;
 
    if (avoidObstacleCounter > 0) {
      if (retrocedeCounter>0){
        retrocedeCounter--;
        
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
      }else{
        avoidObstacleCounter--;
        
        if (arrow == 'L') {
            leftSpeed = 2.0;
            rightSpeed = -1.0;
        } else if (arrow == 'R') {
            leftSpeed = -1.0;
            rightSpeed = 2.0;
        }else if (arrow == 'S'){
            leftSpeed = 0;
            rightSpeed = 0;
        }
      }
    } 
    else { // read sensors
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (ds[i]->getValue() < 1000.0){
          avoidObstacleCounter = 41;
          retrocedeCounter=15;
          arrow = detectArrow();
        }
      }
      
    }

    motors[0]->setVelocity(leftSpeed);
    motors[1]->setVelocity(leftSpeed);
    motors[2]->setVelocity(rightSpeed);
    motors[3]->setVelocity(rightSpeed);
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}

// Initialize the motors
void initializeMotors(Robot *robot){
  char motor_names[4][8] = {"motor_1", "motor_2", "motor_3", "motor_4"};
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i] = robot->getMotor(motor_names[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }
}

// Initialize the distance sensors
void initializeSensors(Robot *robot){
  char dsNames[NUM_SENSORS][10] = {"ds_right", "ds_left"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
}

// Initialize the camera
void initializeCamera(Robot *robot){
  camera = robot->getCamera("CAM");
  camera->enable(TIME_STEP);
}

// Check the arrow color: blue = left, green = right, red = final goal
char detectArrow() {
    const unsigned char *image = camera->getImage();
    int width = camera->getWidth();
    int height = camera->getHeight();

    int blueCount = 0, greenCount = 0, redCount = 0;

    // Analyze the image for arrow colors
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int red = camera->imageGetRed(image, width, x, y);
            int green = camera->imageGetGreen(image, width, x, y);
            int blue = camera->imageGetBlue(image, width, x, y);

            if (blue > red && blue > green) blueCount++;
            if (green > blue && green > red) greenCount++;
            if (red > blue && red > green) redCount++;
        }
    }

    if (blueCount > greenCount && blueCount > redCount) return 'L'; // Left arrow
    if (greenCount > blueCount && greenCount > redCount) return 'R'; // Right arrow
    if (redCount > blueCount && redCount > greenCount) return 'S'; // Stop (goal)

    return 'N'; // No arrow detected
}