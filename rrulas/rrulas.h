#include <iostream>
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <fstream>

#include "../ur_modern_driver-master/include/ur_driver.h"
#include "../kinematics/ur_kin.h"
#include "../force/force.h"

// MOUNTING ANGLE OFFSET BETWEEN F/T SENSOR AND PROTOTYPE 1.1 (PHYSICAL INTERFACE BETWEEN END-EFFECTOR AND PATIENT) 
#define TCP_OFFSET 0


void getq(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double q[6]);
void RobotWait(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double pose_target[6]);
void moveSimpleJoint(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a);
void moveSimpleJointDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double qGoal[6], double v, double a);
void moveSimpleCart(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a);
void moveSimpleCartDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], double v, double a);
void moveTaskSpace(double q[6], double X_CHANGE, double Y_CHANGE, double Z_CHANGE, double T06_NEW[16]);
void gluInvertMatrix(const double m[16], double invOut[16]);
void rgbControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int status);
