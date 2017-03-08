#include "mbzirc.h"

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

void GripperControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int gripPos)
{
    char closeGrip[200];
    sprintf(closeGrip, "modbus_set_output_register(\"Position_Req\", %.3d, False)\n", gripPos);
    ur5->rt_interface_->addCommandToQueue(closeGrip);
}

void GripperSpeedControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int gripPos)
{
    char closeGrip[200];
    sprintf(closeGrip, "modbus_set_output_register(\"Speed_Force_Req\", %.3d, False)\n", gripPos);
    ur5->rt_interface_->addCommandToQueue(closeGrip);
}

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
}

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
    
    
    /*
    GripperSpeedControl(&ur5, &rt_msg_cond_, 40000);
    usleep(50000);
    GripperControl(&ur5, &rt_msg_cond_, 50);
    usleep(50000);
    */
    
    
    // APPROX. START LOCATIONS
    //double qBolt[6] = {2.06957, -2.18337, 1.97259, 0.21851, 0.498663, 2.98847};
    //double qTool[6] = {1.33906, -0.967229, 0.397516, 0.565406, -0.235619, 2.98923};
    double qStart[6] = {0.44, -0.475, -0.375, 1.325, 0.777, 0525}; //Approximate starting position
    
    
    // START LOOKING FOR VALVE
    moveSimpleJointDirect(&ur5, &rt_msg_cond_, qStart, 0.5, 0.5);
    /*
    
  
    
    std::vector<cv::Vec3f> circles;
    cv::Point2f hexPoint;
    int successDetect = 0;
    double BoltWorld[3];    
    for(int i = 0; i < 3; i++)
    {
	BoltDetection(&circles, &hexPoint);	
	moveValve(&ur5, &rt_msg_cond_, &circles, &hexPoint, BoltWorld, &successDetect);
    }
    
    
    
    if(successDetect == 1)
    {	
	
        // ALIGN VALVE
        double rot[9];
	trans3D(GAMMA, BETA, ALPHA, rot); 
	
	double T06_VALVE_GOAL_ALIGN[16];
	getTransformation(rot, BoltWorld[0]-0.2, BoltWorld[1]+GRIPPEROFFSET_Y, BoltWorld[2]-GRIPPEROFFSET_Z, T06_VALVE_GOAL_ALIGN);
	moveSimpleJoint(&ur5, &rt_msg_cond_, T06_VALVE_GOAL_ALIGN, 0, 0.2, 0.2);
	
	
	
	// APPROACH VALVE
	double T06_VALVE_GOAL[16];
	getTransformation(rot, BoltWorld[0]-GRIPPEROFFSET_X-CIRCLE_OFFSET, BoltWorld[1]+GRIPPEROFFSET_Y, BoltWorld[2]-GRIPPEROFFSET_Z, T06_VALVE_GOAL);
	moveSimpleCart(&ur5, &rt_msg_cond_, T06_VALVE_GOAL, 0, 0.05, 0.05);
	
	
	upBack(&ur5, &rt_msg_cond_, 0, 0, 0.075);
	
	

        
	// START LOOKING FOR TOOL	
	moveSimpleJointDirect(&ur5, &rt_msg_cond_, qTool, 0.5, 0.5);
	
	
	
	rgbControl(&ur5, &rt_msg_cond_, 1);
	
	
	
	
	
	
	cv::Point2d TOOL_IMAGE_CENTER_1;
	int TOOL_CENTER_DIST_1;
	ToolDetection(DESIRED_TOOL, &TOOL_IMAGE_CENTER_1, &TOOL_CENTER_DIST_1);
	double ZcTool_1;
	calcDistance(TOOL_CENTER_DIST_1, 0.05, &ZcTool_1);
	int uTool_1 = TOOL_IMAGE_CENTER_1.x;
	int vTool_1 = TOOL_IMAGE_CENTER_1.y;
	if((uTool_1 == 0 && vTool_1 == 0) || isinf(ZcTool_1) == 1)
	{
	    std::cout << "Improper picture ... Exiting program!" << std::endl;
	}
	else
	{
	    double T06_Tool_1[16];
	    ur_kinematics::forward(qTool, T06_Tool_1);    
	    double ToolWorld_1[3];
	    image2base(T06_Tool_1, ZcTool_1, uTool_1, vTool_1, ToolWorld_1);
	    std::cout << "Tool position in robot base frame: " << ToolWorld_1[0] << " " << ToolWorld_1[1] << " " << ToolWorld_1[2] << std::endl;
	    std::cout << "Distance to tool from camera: " << ZcTool_1 << std::endl;
	    
	    // FIRST TOOL APPROACH
	    double TGoalTool_1[16];              
	    getTransformation(rot, ToolWorld_1[0]-0.25, ToolWorld_1[1]-0.02, ToolWorld_1[2]+0.025, TGoalTool_1);
	    moveSimpleJoint(&ur5, &rt_msg_cond_, TGoalTool_1, 0, 0.5, 0.5);
	    GripperControl(&ur5, &rt_msg_cond_, 0);
	    usleep(2000000);
	    
	    // SECOND TOOL DETECTION
	    cv::Point2d TOOL_IMAGE_CENTER_2;
	    int TOOL_CENTER_DIST_2;
	    ToolDetection(DESIRED_TOOL, &TOOL_IMAGE_CENTER_2, &TOOL_CENTER_DIST_2);
	    int uTool_2 = TOOL_IMAGE_CENTER_2.x;
	    int vTool_2 = TOOL_IMAGE_CENTER_2.y;
	    double q[6];
	    getq(&ur5, &rt_msg_cond_, q);
	    double T06_Tool_2[16];  
	    ur_kinematics::forward(q, T06_Tool_2);
	    double ZcTool_2;
	    calcDistance(TOOL_CENTER_DIST_2, 0.05, &ZcTool_2);
	    double ToolWorld_2[3];
	    image2base(T06_Tool_2, ZcTool_2, uTool_2, vTool_2, ToolWorld_2);
	    std::cout << "Tool position in robot base frame: " << ToolWorld_2[0] << " " << ToolWorld_2[1] << " " << ToolWorld_2[2] << std::endl;
	    std::cout << "Distance to tool from camera: " << ZcTool_2 << std::endl;
	    
	    
	    
	    rgbControl(&ur5, &rt_msg_cond_, 0);
	    
	    
	    // APPROACH TOOL
	    GripperControl(&ur5, &rt_msg_cond_, 150);
	    usleep(2000000);
	    double TGoalTool_2[16];              
	    getTransformation(rot, ToolWorld_2[0]-GRIPPEROFFSET_X, ToolWorld_2[1]+GRIPPEROFFSET_Y, ToolWorld_2[2]-GRIPPEROFFSET_Z, TGoalTool_2);
	    double wayPoint[16];
	    memcpy(&wayPoint, &TGoalTool_2, sizeof(TGoalTool_2));
	    wayPoint[3] = wayPoint[3]-0.025;
	    moveSimpleJoint(&ur5, &rt_msg_cond_, wayPoint, 0, 0.5, 0.5);
	    double rot75[9];
	    double vertical_angle = 7.5*M_PI/180;
	    trans3D(GAMMA+vertical_angle, BETA, ALPHA, rot75); 
	    double T06_TOOL_75[16];
	    getTransformation(rot75, TGoalTool_2[3]+0.01, TGoalTool_2[7], TGoalTool_2[11]-0.03, T06_TOOL_75);
	    moveSimpleCart(&ur5, &rt_msg_cond_, T06_TOOL_75, 0, 0.05, 0.05);
	    
	    
	
	   
	    
	    // PICK UP TOOL
	    GripperControl(&ur5, &rt_msg_cond_, 255);
	    usleep(2000000);
	    
	   
	    
	   
	    
	    upBack(&ur5, &rt_msg_cond_, 1, 0.002, 0.075);
	    
	    
	    
	    
	    
	    // BACK TO VALVE
	    double wrench_length = wrench10-GRIPPEROFFSET_Z;
	    double nut_radius = 0.010/2;
	   
	    double T06_VALVE_GOAL_BACK[16];
	    memcpy(&T06_VALVE_GOAL_BACK, &T06_VALVE_GOAL, sizeof(T06_VALVE_GOAL));
	    T06_VALVE_GOAL_BACK[3]  = T06_VALVE_GOAL[3]; 
	    T06_VALVE_GOAL_BACK[7]  = T06_VALVE_GOAL[7]+sin(15*M_PI/180)*nut_radius+sin(15*M_PI/180)*wrench_length+0.005;
	    T06_VALVE_GOAL_BACK[11] = T06_VALVE_GOAL[11]+cos(15*M_PI/180)*nut_radius+cos(15*M_PI/180)*wrench_length+0.03;
	    moveSimpleJoint(&ur5, &rt_msg_cond_, T06_VALVE_GOAL_BACK, 15, 0.5, 0.5);
	    
	    
	 
	    
	    
	    
	    // FORCE CONTROL
	    pthread_t forceID;
	    startFT(&forceID);
	    
	    forceControl(&ur5, &rt_msg_cond_, 5, 2, 5.0);
	    forceControl(&ur5, &rt_msg_cond_, 20, 1, 5.0);
	    
	    stopFT(&forceID);
	    
	    
	    
	    
	    

	    // PLACE TOOL
	    double q_BACK[6];
	    getq(&ur5, &rt_msg_cond_, q_BACK);
	    double T[16];
	    moveTaskSpace(q_BACK, 0, 0.05, 0, T);
	    moveSimpleCartDirect(&ur5, &rt_msg_cond_, T, 0.05, 0.05);		
	    upBack(&ur5, &rt_msg_cond_, 0, 0, 0.075);
	    wayPoint[3] = wayPoint[3]-0.020;
	    moveSimpleJoint(&ur5, &rt_msg_cond_, wayPoint, 0, 0.5, 0.5);
	    moveSimpleCart(&ur5, &rt_msg_cond_, T06_TOOL_75, 0, 0.05, 0.05);
	    GripperControl(&ur5, &rt_msg_cond_, 150);
	    usleep(2000000);
	    wayPoint[3] = wayPoint[3]-0.060;
	    moveSimpleJoint(&ur5, &rt_msg_cond_, wayPoint, 0, 0.2, 0.2);
	    
        }*/
    }
    ur5.halt();
    std::cout << "Disconnected!\n";
    return 0;  
}
