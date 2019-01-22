#include <Servo.h> 
#include "configuration.h"
#include "meArm.h"
#include "ros.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

meArm arm(
  BASE_MIN_PWM,     BASE_MAX_PWM,     BASE_MIN_ANGLE_RAD,     BASE_MAX_ANGLE_RAD,
  SHOULDER_MIN_PWM, SHOULDER_MAX_PWM, SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD,
  ELBOW_MIN_PWM,    ELBOW_MAX_PWM,    ELBOW_MIN_ANGLE_RAD,    ELBOW_MAX_ANGLE_RAD,
  CLAW_MIN_PWM,     CLAW_MAX_PWM,     CLAW_MIN_ANGLE_RAD,     CLAW_MAX_ANGLE_RAD);


ros::NodeHandle nh;

std_msgs::Bool bool_msg;
ros::Publisher done_tapping("done_tapping", &bool_msg);

void callback(const geometry_msgs::Pose& pos_diff){
  if (pos_diff.position.z == 0.0) {
    arm.gotoPoint(arm.getX()+pos_diff.position.x, arm.getY()+pos_diff.position.y, arm.getZ());
    bool_msg.data = false;
    delay(200);
    done_tapping.publish(&bool_msg);
  } else {
    arm.goDirectlyTo(arm.getX(), arm.getY()+pos_diff.position.y, arm.getZ()+pos_diff.position.z);
    delay(300);
    arm.goDirectlyTo(arm.getX(), arm.getY()-pos_diff.position.y, arm.getZ()-pos_diff.position.z);
    bool_msg.data = true;
    done_tapping.publish(&bool_msg);
  }
}

ros::Subscriber<geometry_msgs::Pose> sub("position_diff", &callback );

void setup() 
{ 
  arm.begin(BASE_PIN, SHOULDER_PIN, ELBOW_PIN, CLAW_PIN);
  arm.closeGripper();
  arm.gotoPoint(HOME_X, HOME_Y, HOME_Z);
  
  nh.initNode();
  nh.advertise(done_tapping);
  nh.subscribe(sub);
}

void loop() 
{ 
  nh.spinOnce();
  delay(1);
} 


