#include <Servo.h> 
#include "configuration.h"
#include "meArm.h"
#include "ros.h"
#include <geometry_msgs/Pose.h>

meArm arm(
  BASE_MIN_PWM,     BASE_MAX_PWM,     BASE_MIN_ANGLE_RAD,     BASE_MAX_ANGLE_RAD,
  SHOULDER_MIN_PWM, SHOULDER_MAX_PWM, SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD,
  ELBOW_MIN_PWM,    ELBOW_MAX_PWM,    ELBOW_MIN_ANGLE_RAD,    ELBOW_MAX_ANGLE_RAD,
  CLAW_MIN_PWM,     CLAW_MAX_PWM,     CLAW_MIN_ANGLE_RAD,     CLAW_MAX_ANGLE_RAD);

ros::NodeHandle nh;

void callback(const geometry_msgs::Pose& pos){
  //arm.gotoPoint(pos.position.x, pos.position.y, pos.position.z);
  arm.goDirectlyTo(pos.position.x, pos.position.y, pos.position.z);
}

ros::Subscriber<geometry_msgs::Pose> sub("position", &callback );

void setup() 
{ 
  arm.begin(BASE_PIN, SHOULDER_PIN, ELBOW_PIN, CLAW_PIN);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() 
{ 
  nh.spinOnce();
  delay(1);
} 

