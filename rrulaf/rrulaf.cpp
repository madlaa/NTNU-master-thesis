/*Written by Mads Johan Laastad as part of a master thesis in Cybernetics and Robotics at the Norwegian University of Science and Technology (NTNU) during the spring of 2017. The author can not guarrantee for the safety of anyone that desides to use this code in their own projects. This module is named rrulaf after the title of the master thesis it is part of, rrulaf is an acronyme for "Robotic Rehabilitation of Upper-Limb After Stroke". Feel free to contact the autor at laastad.m@gmail.com if you have any questions.
# NETWORK
#
# The IP address can be found in the PolyScope interface (tablet) of the robot.
# SETUP Robot -> Setup NETWORK (requires password: "ngr12") -> IP address
UR5_IP = "10.42.0.63"
UR5_HOSTNAME = 'ur-2012208984' #requires dns.
*/
#include "rrulaf.h"

void getq(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double q[6])
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_->wait(locker);
	}
	std::vector<double> q_vector = ur5->rt_interface_->robot_state_->getQActual();
	std::copy(q_vector.begin(), q_vector.end(), q);
	ur5->rt_interface_->robot_state_->setDataPublished();
}

void RobotWait(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double pose_target[6])
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	double startTime = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	double goal_tol = 0.0075;  
	while(1)
	{
	gettimeofday(&tp, NULL);
	double timeStamp = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	double elapsTime = (timeStamp-startTime)/1000;
	  
		std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
		rt_msg_cond_->wait(locker);
	}
		ur5->rt_interface_->robot_state_->setDataPublished();
		std::vector<double> q_actual = ur5->rt_interface_->robot_state_->getQActual();
	if(((fabs(q_actual[0] - pose_target[0])) < goal_tol) &&
	   ((fabs(q_actual[1] - pose_target[1])) < goal_tol) &&
	   ((fabs(q_actual[2] - pose_target[2])) < goal_tol) &&
	   ((fabs(q_actual[3] - pose_target[3])) < goal_tol) &&
	   ((fabs(q_actual[4] - pose_target[4])) < goal_tol) &&
	   ((fabs(q_actual[5] - pose_target[5])) < goal_tol)) break;
	   
	if(elapsTime > 15)
	{
	  std::cout << "RobotWait is stuck - breaking out!" << std::endl;
	  break;
	}
	}
	usleep(1000000);
}
/*

void upBack(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int up, double up_value, double back_value)
{
	if(up == 1)
	{
	// UP
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
		rt_msg_cond_->wait(locker);
	}
		ur5->rt_interface_->robot_state_->setDataPublished();
	double qU[6];
	getq(ur5, rt_msg_cond_, qU);
	double T06U[16];
	ur_kinematics::forward(qU, T06U);
	T06U[11] = T06U[11]+up_value;
	double qGoalU[6];
	getnearest(T06U, qU, qGoalU);
	char TargetU[200];
	sprintf(TargetU, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoalU[0], qGoalU[1], qGoalU[2], qGoalU[3], qGoalU[4], qGoalU[5], 0.05, 0.05);
	ur5->rt_interface_->addCommandToQueue(TargetU);  
	RobotWait(ur5, rt_msg_cond_, qGoalU);
	}
	
	// BACK
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_->wait(locker);
	}
	ur5->rt_interface_->robot_state_->setDataPublished();
	double qB[6];
	getq(ur5, rt_msg_cond_, qB);
	double T06B[16];
	ur_kinematics::forward(qB, T06B);
	T06B[3] = T06B[3]-back_value;
	double qGoalB[6];
	getnearest(T06B, qB, qGoalB);
	char TargetB[200];
	sprintf(TargetB, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
			qGoalB[0], qGoalB[1], qGoalB[2], qGoalB[3], qGoalB[4], qGoalB[5], 0.05, 0.05);
	ur5->rt_interface_->addCommandToQueue(TargetB);
	RobotWait(ur5, rt_msg_cond_, qGoalB);
}*/
/*
void moveValve(UrDriver *ur5, std::condition_variable *rt_msg_cond_, std::vector<cv::Vec3f> *circles, cv::Point2f *hexPoint, double BoltWorld[3], int *successDetect)
{
	int tempRadius = 0;
	int indexMAX;
	for(unsigned int j = 0; j < circles->size(); j++)
	{
	if((*circles)[j][2] > tempRadius)
	{
		tempRadius = (*circles)[j][2];
		cv::Point2f BoltCenter(cvRound((*circles)[j][0]), cvRound((*circles)[j][1]));
		*successDetect = 1;
		indexMAX = j;
	}
	}
	if(*successDetect == 1)
	{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
		rt_msg_cond_->wait(locker);
	}
		ur5->rt_interface_->robot_state_->setDataPublished();
	
	double q[6];
	getq(ur5, rt_msg_cond_, q);
	double ZcBolt;
	calcDistance(tempRadius*2, 0.08, &ZcBolt);
	std::cout << "Distance to valve from camera: " << ZcBolt << std::endl;
	cv::Point2f BoltCenter(hexPoint->x, hexPoint->y);
	int uBolt = BoltCenter.x;
	int vBolt = BoltCenter.y;
	double T06[16];
	ur_kinematics::forward(q, T06);
	image2base(T06, ZcBolt, uBolt, vBolt, BoltWorld);
	std::cout << "Valve position in robot base frame: " << BoltWorld[0] << " " << BoltWorld[1] << " " << BoltWorld[2] << std::endl;
	double rot[9];	
	getRotation(T06, rot);
	double TGoal[16];
	getTransformation(rot, T06[3], BoltWorld[1]+0.01, BoltWorld[2]+0.02, TGoal);
	double qBoltGoal[6];
	getnearest(TGoal, q, qBoltGoal);
	//qBoltGoal[5] = qBoltGoal[5]-TCP_OFFSET*M_PI/180;
	char TargetPoseBolt[200];
	sprintf(TargetPoseBolt, "movej([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qBoltGoal[0], qBoltGoal[1], qBoltGoal[2], qBoltGoal[3], qBoltGoal[4], qBoltGoal[5], 1.0, 1.0);
	ur5->rt_interface_->addCommandToQueue(TargetPoseBolt);  
	RobotWait(ur5, rt_msg_cond_, qBoltGoal);
	
	}
	else
	{
	std::cout << "Valve is not detected!" << std::endl;
	}
}
*/  
void moveSimpleJoint(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_->wait(locker);
	}
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	double q[6];  
	getq(ur5, rt_msg_cond_, q);
	double qGoal[6];
	getnearest(T06, q, qGoal);
	qGoal[5] = qGoal[5]-((angle_offset+TCP_OFFSET)*M_PI/180);
	char TargetString[200];
	sprintf(TargetString, "movej([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString);  
	RobotWait(ur5, rt_msg_cond_, qGoal);
}

void moveSimpleJointDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double qGoal[6], double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_->wait(locker);
	}
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	char TargetString[200];
	sprintf(TargetString, "movej([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString);  
	RobotWait(ur5, rt_msg_cond_, qGoal);
}

void moveSimpleCart(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_->wait(locker);
	}
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	double q[6];
	getq(ur5, rt_msg_cond_, q);
	double qGoal[6];
	getnearest(T06, q, qGoal);
	qGoal[5] = qGoal[5]-((angle_offset+TCP_OFFSET)*M_PI/180);
	char TargetString[200];
	sprintf(TargetString, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString); 
	RobotWait(ur5, rt_msg_cond_, qGoal);
}

void moveSimpleCartDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_->wait(locker);
	}
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	double q[6];
	getq(ur5, rt_msg_cond_, q);
	double qGoal[6];
	getnearest(T06, q, qGoal);	
	char TargetString[200];
	sprintf(TargetString, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString); 
	RobotWait(ur5, rt_msg_cond_, qGoal);
}
/*
void gluInvertMatrix(const double m[16], double invOut[16]) //Inverts a 4x4 matrix
{
    double inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        std::cout << "Matrix inversion failed - determinant is zero!" << std::endl;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

}
*/
/*
void moveTaskSpace(double q[6], double X_CHANGE, double Y_CHANGE, double Z_CHANGE, double T06_NEW[16])
{
	double T06[16];
	ur_kinematics::forward(q, T06);
	double INV_T06[16];
	gluInvertMatrix(T06, INV_T06);
	INV_T06[3]  = INV_T06[3]  + X_CHANGE;
	INV_T06[7]  = INV_T06[7]  + Y_CHANGE;
	INV_T06[11] = INV_T06[11] + Z_CHANGE;
	gluInvertMatrix(INV_T06, T06_NEW);
}
*/
void rgbControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int status)
{
	char TargetString[200];
	
	if(status==1)
	sprintf(TargetString, "set_tool_voltage(12)\n");
	
	if(status==0)
	sprintf(TargetString, "set_tool_voltage(0)\n");
	
	ur5->rt_interface_->addCommandToQueue(TargetString);
	usleep(1000000);
}
  

  
  
  
  
int main()
{
  
	// ROBOT CONNECTION
	std::condition_variable rt_msg_cond_;
	std::condition_variable msg_cond_;
	UrDriver ur5(rt_msg_cond_, msg_cond_,"10.42.0.63",5000);
	ur5.start();
	std::cout << "Connecting to robot ..." << std::endl;
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	std::cout << "Waiting for data ..." << std::endl;
	while (!ur5.rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_.wait(locker);
	}	
	std::cout << "Data received!" << std::endl;
	
	
	
	// Approximate starting position
	double qStart[6] = {0.1585, -2.5167, -1.3030, 0.6209, 1.3051, 0}; //-1.04719755
	
	// Reading and print current position
	/*
	double q[6];
	getq(&ur5, &rt_msg_cond_, q);	
	char TargetString[200];
	sprintf(TargetString, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n", q[0], q[1], q[2], q[3], q[4], q[5], 0.2, 0.2);
	std::cout << "Current pose is: " << TargetString << std::endl;
	*/
	
	// MOVE TO STARING POINT
	std::cout << "Moving to staring location. " << std::endl;
	moveSimpleJointDirect(&ur5, &rt_msg_cond_, qStart, 1, 1);
	//moveSimpleJoint(&ur5, &rt_msg_cond_, qStart, 0, 0.2, 0.2);

	// FORCE CONTROL
	int force_mode = 1; // TriggerAssisted, Challange based, etc.
	std::cout << "Initializing force control. \n" << std::endl;
	pthread_t forceID; //this is done inside force.cpp
	//startFT(&forceID);
	//std::cout << "Please enter the desired mode. \n Enter 1 for Compliance mode: " << std::endl;
	//std::cin >> force_mode; 
	simpleForceControl(&ur5, &rt_msg_cond_, 200, force_mode, 3);//forceControl(&ur5, &rt_msg_cond_, 5, 2, 5.0);
	//simpleForceControl(&ur5, &rt_msg_cond_, 20, 2, 3);
	usleep(10000);
	
	stopFT(&forceID);
	std::cout << "Shutting down force control. \n";
	ur5.halt();
	std::cout << "Disconnected!\n";
	return 0;
}
