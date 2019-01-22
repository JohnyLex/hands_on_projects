
#pragma once 

#define DEGREES(rads)  (((rads)*180.0)/(PI))
#define RADIANS(degs)  (((degs)*(PI))/(180.0))

#define SHOULDER_MAX_PWM         150
#define SHOULDER_MIN_PWM         83
#define SHOULDER_MAX_ANGLE_RAD   RADIANS(15)
#define SHOULDER_MIN_ANGLE_RAD   RADIANS(90)
#define SHOULDER_HOME_PWM        90 // in PWM counts
#define SHOULDER_PIN             6

#define ELBOW_MAX_PWM         155
#define ELBOW_MIN_PWM         55
#define ELBOW_MAX_ANGLE_RAD   RADIANS(32)
#define ELBOW_MIN_ANGLE_RAD   RADIANS(-67)
#define ELBOW_HOME_PWM        90     // in PWM counts
#define ELBOW_PIN             9

#define CLAW_MAX_PWM          87
#define CLAW_MIN_PWM          45
#define CLAW_CLOSED_PWM       85
#define CLAW_OPEN_PWM         65
#define CLAW_HOME_PWM         75  // in PWM counts
#define CLAW_PIN              10
#define CLAW_MAX_ANGLE_RAD   RADIANS(0)
#define CLAW_MIN_ANGLE_RAD   RADIANS(90)

#define BASE_MAX_PWM          160
#define BASE_MIN_PWM          0
#define BASE_MAX_ANGLE_RAD    RADIANS(-90)
#define BASE_MIN_ANGLE_RAD    RADIANS(70)
#define BASE_HOME_PWM         70 // in PWM counts or as an angle (degrees), they are the same
#define BASE_PIN              5

#define HOME_X  -7
#define HOME_Y  145
#define HOME_Z  5        // empirically figured these out

static const float L1=80;  // Shoulder to elbow length (mm)
static const float L2=80;  // Elbow to wrist length (mm)
static const float L3=68;  // Length from wrist to hand PLUS base centre to shoulder (mm)


