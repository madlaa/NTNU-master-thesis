#include <iostream>
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <fstream>

#include "../ur_modern_driver-master/include/ur_driver.h"
#include "../kinematics/ur_kin.h"
#include "../vision/vision.h"
#include "../force/force.h"



// DEFINE THE APPROX LENGTH OF THE SPANNERS HERE (LENGTH = DIST FROM MASS CENTER POINT TO CENTER OF ROTATION)
#define wrench10 0.0750

// OFFSET FROM END-EFFECTOR TO GRIPPER POINT
#define GRIPPEROFFSET_X 0.13000//0.12000
#define GRIPPEROFFSET_Y 0.00350
#define GRIPPEROFFSET_Z 0.035//0.01310

// DISTANCE FROM CIRCLE TO VALVE
#define CIRCLE_OFFSET 0.055

// MOUNTING ANGLE OFFSET BETWEEN F/T SENSOR AND GRIPPER
#define TCP_OFFSET 9

// ASSUMING THAT A 10 MM VALVE IS DETECTED
#define DESIRED_TOOL 1

// BOARD ORIENTATION IN WORLD FRAME
#define GAMMA -1.570796326794897
#define BETA 6.283185307179586
#define ALPHA -1.570796326794897


void getq(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double q[6]);
void RobotWait(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double pose_target[6]);
void GripperControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int gripPos);
void GripperSpeedControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int gripPos);
void upBack(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int up);
void moveValve(UrDriver *ur5, std::condition_variable *rt_msg_cond_, std::vector<cv::Vec3f> *circles, cv::Point2f *hexPoint, double BoltWorld[3], int *successDetect);
void moveSimpleJoint(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a);
void moveSimpleJointDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double qGoal[6], double v, double a);
void moveSimpleCart(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a);
void moveSimpleCartDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], double v, double a);
void moveTaskSpace(double q[6], double X_CHANGE, double Y_CHANGE, double Z_CHANGE, double T06_NEW[16]);
void rgbControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int status);