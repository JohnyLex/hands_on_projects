/* meArm library York Hack Space May 2014
 * A simple control library for Phenoptix' meArm
 * Usage:
 *   meArm arm;
 *   arm.begin(1, 10, 9, 6);
 *   arm.openGripper();
 *   arm.gotoPoint(-80, 100, 140);
 *   arm.closeGripper();
 *   arm.gotoPoint(70, 200, 10);
 *   arm.openGripper();
 */
#include <Arduino.h>
#include "ik.h"
#include "meArm.h"
#include <Servo.h>

bool setup_servo (ServoInfo& svo, const int n_min, const int n_max,
                  const float a_min, const float a_max)
{
    float n_range = n_max - n_min;
    float a_range = a_max - a_min;

    // Must have a non-zero angle range
    if(a_range == 0) return false;

    // Calculate gain and zero
    svo.gain = n_range / a_range;
    svo.zero = n_min - svo.gain * a_min;

    // Set limits
    svo.n_min = n_min;
    svo.n_max = n_max;

    return true;
}

int angle2pwm (const ServoInfo& svo, const float angle)
{
    float pwm = 0.5f + svo.zero + svo.gain * angle;
    if (pwm > svo.n_max) { pwm = svo.n_max; Serial.print("x");}
    if (pwm < svo.n_min) { pwm = svo.n_min; Serial.print("n");}
    return int(pwm);
}

//Full constructor with calibration data
meArm::meArm(int sweepMinBase, int sweepMaxBase, float angleMinBase, float angleMaxBase,
          int sweepMinShoulder, int sweepMaxShoulder, float angleMinShoulder, float angleMaxShoulder,
          int sweepMinElbow, int sweepMaxElbow, float angleMinElbow, float angleMaxElbow,
          int sweepMinGripper, int sweepMaxGripper, float angleMinGripper, float angleMaxGripper) {
  //calroutine();
  setup_servo(_svoBase, sweepMinBase, sweepMaxBase, angleMinBase, angleMaxBase);
  setup_servo(_svoShoulder, sweepMinShoulder, sweepMaxShoulder, angleMinShoulder, angleMaxShoulder);
  setup_servo(_svoElbow, sweepMinElbow, sweepMaxElbow, angleMinElbow, angleMaxElbow);
  setup_servo(_svoGripper, sweepMinGripper, sweepMaxGripper, angleMinGripper, angleMaxGripper);
}

void meArm::begin(int pinBase, int pinShoulder, int pinElbow, int pinGripper) {
  _pinBase = pinBase;
  _pinShoulder = pinShoulder;
  _pinElbow = pinElbow;
  _pinGripper = pinGripper;
  _base.attach(_pinBase);
  _shoulder.attach(_pinShoulder);
  _elbow.attach(_pinElbow);
  _gripper.attach(_pinGripper);

  goDirectlyTo(HOME_X, HOME_Y, HOME_Z);
  closeGripper();
}

//Set servos to reach a certain point directly without caring how we get there 
void meArm::goDirectlyTo(float x, float y, float z) {
  float radBase,radShoulder,radElbow;
  if (solve(x, y, z, radBase, radShoulder, radElbow)) {
    int pwm;
    Serial.print("base "); 
    pwm = angle2pwm(_svoBase,radBase); _base.write(pwm);
    Serial.print(pwm); Serial.print(" "); 
    Serial.print(DEGREES(radBase));
    
    Serial.print(" shoulder "); 
    pwm = angle2pwm(_svoShoulder,radShoulder); _shoulder.write(pwm);
    Serial.print(pwm); Serial.print(" "); 
    Serial.print(DEGREES(radShoulder));

    Serial.print(" elbow "); 
    pwm = angle2pwm(_svoElbow,radElbow); _elbow.write(pwm);
    Serial.print(pwm); Serial.print(" "); 
    Serial.print(DEGREES(radElbow));

    Serial.print(" x "); Serial.print(x);
    Serial.print(" y "); Serial.print(y);
    Serial.print(" z "); Serial.print(z);
    _x = x; _y = y; _z = z;  // Maybe should FK to figure out actual after PWM limits
    Serial.println("");
  } else {
    Serial.println("Position is not reachable.");     
  }
}

//Travel smoothly from current point to another point
void meArm::gotoPoint(float x, float y, float z) {
  //Starting points - current pos
  float x0 = _x; 
  float y0 = _y; 
  float z0 = _z;
  float dist = sqrt((x0-x)*(x0-x)+(y0-y)*(y0-y)+(z0-z)*(z0-z));
  int step = 5;
  for (int i = 0; i<dist; i+= step) {
    goDirectlyTo(x0 + (x-x0)*i/dist, y0 + (y-y0) * i/dist, z0 + (z-z0) * i/dist);
    delay(30);
  }
  goDirectlyTo(x, y, z);
  delay(50);
}

//Check to see if possible
bool meArm::isReachable(float x, float y, float z) {
  float radBase,radShoulder,radElbow;
  return (solve(x, y, z, radBase, radShoulder, radElbow));
}

//Grab something
void meArm::openGripper() {
  _gripper.write(CLAW_OPEN_PWM);
  delay(50);
}

//Let go of something
void meArm::closeGripper() {
  _gripper.write(CLAW_CLOSED_PWM);
  delay(50);
}

//Current x, y and z
float meArm::getX() {
  return _x;
}
float meArm::getY() {
  return _y;
}
float meArm::getZ() {
  return _z;
}


