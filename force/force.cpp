#include "force.h"

#include "ujac_c.c"
#include "ufwdkin_c.c"

int initFT = 1;
int forceSleepTime = 10000; //usleep(10000) ~ 90 Hz --> run a bit faster than the fastest human reaction time -- currently involuntary muscle contractions at 24ms ~ 41,6 Hz
//Fastest possible hardware update rate for the UR5 is set at 8ms ~ 125 Hz equivalent to usleep(8000)
double rawFTdata[6];
double biasTF[3]; // Tool frame (or TCP frame)


void *getFTData(void *arg)
{
	int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
	struct sockaddr_in addr;		/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36];			/* The raw response data received from the Net F/T. */
	int i;					/* Generic loop/array index. */
	int err;				/* Error status of operations. */
	//char const *AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	/* The names of the force and torque axes. */
	


	/* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1)
	{
		exit(1);
	}

	*(uint16*)&request[0] = htons(0x1234); 	/* standard header. */
	*(uint16*)&request[2] = htons(COMMAND); 	/* per table 9.1 in Net F/T user manual. */
	*(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

	/* Sending the request. */
	he = gethostbyname("10.42.0.8");


	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);

	err = connect(socketHandle, (struct sockaddr *)&addr, sizeof(addr));
	if (err == -1)
	{
		exit(2);
	}
	send(socketHandle, request, 8, 0);

	
	int scale_factor = 1000000; //Scales down to Newton [N] and Newtonmeter [Nm]
	while(initFT)
	{
		/* Receiving the response. */
		recv( socketHandle, response, 36, 0 );
		resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
		resp.ft_sequence = ntohl(*(uint32*)&response[4]);
		resp.status = ntohl(*(uint32*)&response[8]);
		for( i = 0; i < 6; i++ )
		{
			resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
		}


		double Fx = (double)resp.FTData[0]/scale_factor;
		double Fy = (double)resp.FTData[1]/scale_factor;
		double Fz = (double)resp.FTData[2]/scale_factor;
		double Tx = (double)resp.FTData[3]/scale_factor;
		double Ty = (double)resp.FTData[4]/scale_factor;
		double Tz = (double)resp.FTData[5]/scale_factor;
	
		rawFTdata[0] = Fx; rawFTdata[1] = Fy; rawFTdata[2] = Fz; rawFTdata[3] = Tx; rawFTdata[4] = Ty; rawFTdata[5] = Tz;
		usleep(3800); // 4000 microseconds ~ 250 Hz is current FT broadcast frequency
		//usleep830 ~ 1200 Hz --> run a bit faster than FT broadcast frequency -- currently 1000 Hz
	}
}

void startFT(pthread_t *forceID)
{
	//pthread_t forceID;
	if(pthread_create(forceID, NULL, &getFTData, NULL))
	{
		std::cout << "Error: force/torque thread not created!" << std::endl;
		//return -1;
	}
	std::cout << "Broadcasting force/torque data!" << std::endl;
	usleep(200000);
}

void stopFT(pthread_t *forceID)
{
	initFT = 0;
	pthread_join(*forceID, NULL);
	std::cout << "Force thread joined - stopping force/torque data acquisition!" << std::endl;
	usleep(2000000);
}

void rot_z(double angle, double rot[9])
{
    rot[0] = cos(angle);  rot[1] = -sin(angle); rot[2] = 0;
    rot[3] = sin(angle);  rot[4] = cos(angle);  rot[5] = 0;
    rot[6] = 0;           rot[7] = 0;           rot[8] = 1;
}

void forceTransformation(double ft_in[3], double ft_out[3])
{
	double theta = 2.61799388;//0.523598; approx. 30 degrees offset due to FT sensor mounting points
	double rot[9];
	rot_z(theta, rot);
	
	double ft_wrist[4];
	ft_wrist[0] = rot[0]*ft_in[0] + rot[1]*ft_in[1] + rot[2]*ft_in[2]; 
	ft_wrist[1] = rot[3]*ft_in[0] + rot[4]*ft_in[1] + rot[5]*ft_in[2];
	ft_wrist[2] = rot[6]*ft_in[0] + rot[7]*ft_in[1] + rot[8]*ft_in[2];
	ft_wrist[3] = 1;

	ft_out[0] = ft_wrist[0];
	ft_out[1] = ft_wrist[1];
	ft_out[2] = ft_wrist[2];
}

void rotate(gsl_vector *res,gsl_matrix *R, gsl_vector *inp,gsl_vector *t1,gsl_vector *t2)
{
	t1->data=inp->data;
	t2->data=res->data;
	gsl_blas_dgemv(CblasNoTrans ,1.0,R, t1,0.0,t2); 
	t1->data=&inp->data[3];
	t2->data=&res->data[3];
	gsl_blas_dgemv(CblasNoTrans ,1.0,R, t1,0.0,t2); 
}  



void gravityCompensation(std::vector<double> q, double biasWF[3])
{	
	gsl_matrix *R = gsl_matrix_calloc(3,3);
	gsl_matrix *invR = gsl_matrix_calloc(3,3);
	
	int signum;
	double apar[6] = {0,-0.42500,-0.39225,0,0,0};
	double dpar[6] = {0.089159,0,0,0.10915,0.09465,0.0823};
	double gravityCompFTdata[3] = {0, 0, 0};
	gsl_permutation *p = gsl_permutation_alloc(3);

	tfrotype tfkin;
	R->data=tfkin.R;
	ufwdkin(&tfkin,q.data(),apar,dpar);

	gsl_linalg_LU_decomp(R, p, &signum);
	gsl_linalg_LU_invert (R, p, invR);
	
	biasTF[0] = biasTF[1] = biasTF[2] = 0;
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3 ;j++)
		{
			gravityCompFTdata[i] += gsl_matrix_get(invR, i, j)*biasWF[j];
		}
	}
	forceTransformation(gravityCompFTdata, biasTF);
}

void solveInverseJacobian(std::vector<double> q, double vw[6], double qd[6])
{
	gsl_vector *x = gsl_vector_alloc(6);
	gsl_vector *vw_ = gsl_vector_alloc(6);
	gsl_vector *t1 = gsl_vector_alloc(3);
	gsl_vector *t2 = gsl_vector_alloc(3);
	gsl_vector *vw_w = gsl_vector_alloc(6);
	gsl_matrix *R = gsl_matrix_alloc(3,3);
	gsl_matrix *A = gsl_matrix_alloc(6,6);
	gsl_permutation *p = gsl_permutation_alloc(6);
	
	int signum;
	double apar[6] = {0,-0.42500,-0.39225,0,0,0};
	double dpar[6] = {0.089159,0,0,0.10915,0.09465,0.0823};
	double fqd;
	double sat = 0;
	
	gsl_vector_set(vw_,0,vw[0]);
	gsl_vector_set(vw_,1,vw[1]);
	gsl_vector_set(vw_,2,vw[2]);
	gsl_vector_set(vw_,3,vw[3]);
	gsl_vector_set(vw_,4,vw[4]);
	gsl_vector_set(vw_,5,vw[5]);
	
	
	tfrotype tfkin;
	R->data=tfkin.R;
	ufwdkin(&tfkin,q.data(),apar,dpar);
	rotate(vw_w,R,vw_,t1,t2);
	
	
	ujac(A->data,q.data(),apar,dpar);
	
	gsl_linalg_LU_decomp(A,p,&signum);

	gsl_linalg_LU_solve(A,p,vw_w,x);
	for (int k=0;k<6;k++)
	{
		qd[k] = gsl_vector_get(x,k); 
		fqd = fabs(qd[k]);
		if (qd[k] > 1 || qd[k] < -1)
		{
			sat = fqd > sat ? fqd : sat;   // ?: is a ternary operator
		}
	}
	if(sat > 0)
	{
		for (int i = 0; i < 6; i++)
		{
			qd[i] = qd[i]/sat*1;
		}
	}
}

//called by adjustForce(sq[5], q[5], current_TCP_Frame_adjusted);
void adjustForce(double sq6, double cq6, double outF[3]) //Adjusting for weight of equptment? Initializing circular motion
{
	double magX = -11.4048;
	double magY = -11.4560;

	outF[0] = 5+magX*sin(sq6-cq6+0.4);
	outF[1] = 1+magY*sin(sq6/4+cq6+1)+magY;
}

void forceControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int run_time, int force_mode, double f_ref, double t_ref, double buoyancy, double disturbance_scale)
{
	pthread_t forceID;
	startFT(&forceID);	
	
	
	std::cout << "Force control initiated - starting data logging ..." << std::endl;
	std::ofstream forcelog;
	forcelog.open("../data/logs/forcelog", std::ofstream::out);
	//Control system for translation forces
	double integrator_Fx = 0, integrator_Fy = 0, integrator_Fz = 0;
	double derivator_Fx = 0, derivator_Fy = 0, derivator_Fz = 0;
	double Kp = 0, Ki = 0, Kd = 0;
	
	double error_Fx = 0, error_Fy = 0, error_Fz = 0;
	double prior_error_Fx = 0, prior_error_Fy = 0, prior_error_Fz = 0;
	//double disturbance_Fx = 0, disturbance_Fy = 0, disturbance_Fz = 0;
	
	double u_Fx = 0, u_Fy = 0, u_Fz = 0;
	
	//Control system for rotational forces
	//ToDo: Add threshold force referance. Remove F/T Bias at the F/T Net Box interface.
	double integrator_Tx = 0, integrator_Ty = 0, integrator_Tz = 0;
	double derivator_Tx = 0, derivator_Ty = 0, derivator_Tz = 0;
	double Kp_T = 0, Ki_T = 0, Kd_T = 0;
	
	double error_Tx = 0, error_Ty = 0, error_Tz = 0;
	double prior_error_Tx = 0, prior_error_Ty = 0, prior_error_Tz = 0;
	//double disturbance_Tx = 0, disturbance_Ty = 0, disturbance_Tz = 0;
	
	double u_Tx = 0, u_Ty = 0, u_Tz = 0;
	
	
	//=======================================================================
	double speed[6] = {0,0,0,0,0,0};
	
	
	double ref_TCP_Frame[3];
	double current_TCP_Frame[3];
	
	double vw[6];
 
	double biasWF[3] = {rawFTdata[0], rawFTdata[1], rawFTdata[2]}; //World frame (or base frame)
	
	double biasFT[3] = {-rawFTdata[0], -rawFTdata[1], -rawFTdata[2]};
	double biasTorque[3] = {rawFTdata[3], rawFTdata[4], rawFTdata[5]};
	
	
	double ref_FT_Frame[3] = {rawFTdata[0]-biasFT[0], rawFTdata[1]-biasFT[1], rawFTdata[2]-biasFT[2]};
	
	
	double Fangle = -1.209406804939513; //-1.57079633;
	double f_refx = f_ref*cos(Fangle);
	double f_refy = f_ref*sin(Fangle);
	
	
	
	
	forceTransformation(ref_FT_Frame, ref_TCP_Frame);
	ref_TCP_Frame[0] = ref_TCP_Frame[0]+f_refx; //f_refx can be used for setting a bouyancy?
	ref_TCP_Frame[1] = ref_TCP_Frame[1]+f_refy;
	

	
	double startTime = ur5->rt_interface_->robot_state_->getTime();
	std::vector<double> sq = ur5->rt_interface_->robot_state_->getQActual(); //sq = Start Q
	
	int i = 0; 
	int iter = run_time/0.008;
	double angular_speed = -2.25*M_PI/run_time;
	
	
	if(fabs(angular_speed) > 0.5) 
	{
		//ur5->setSpeed(0,0,0,0,0,0,1);
		std::cout << "============================= STOPPING! ============================" << std::endl;
		std::cout << "Run-time too low -- setting angular speed to 1 rad/s" << std::endl;
	}
	
	//Doublecheck PID controller tuning
	Kp = 0.001;//0.005; //Original: -0.005 // Tuned: -0.008
	Ki = 0;//0.0002; //Original: -0.000025 //Tuned: -0.00085 or -0.0003
	Kd = 0.0001;//0.0002;//0.000045; //Original: -0.000025 //Tuned: -0.00045
	
	Kp_T = 0.3; //Original: -0.005;
	Ki_T = 0.003; //Original: -0.000025;
	Kd_T = 0.000045;
	
	double disturbances[6] = {0,0,0,0,0,0}; //Fx, Fy, Fz, Tx, Ty, Tz
	srand(time(NULL));
	double randomDuration = rand() % 375 + 125;
	double randomDisturbances[6] = {0,0,0,0,0,0}; 
	//double prior_randomDisturbances[6] = {0,0,0,0,0,0};
	
	double testTime = 500; //4 seconds test
	double test_force = 5; //5 Newton step responce
	
	std::cout << "======================== FORCE CONTROL ACTIVE ========================" << std::endl;
	
	while(i<iter)
	{
	
		std::mutex msg_lock;
		std::unique_lock<std::mutex> locker(msg_lock);
		while (!ur5->rt_interface_->robot_state_->getDataPublished())
		{
			rt_msg_cond_->wait(locker);
		}
		
		//double Forces[3] = {rawFTdata[0]-biasTF[2], rawFTdata[1]-biasTF[1], rawFTdata[2]-biasTF[0]};
		
		
		//double Torques[3] = {rawFTdata[3]-biasTorque[3], rawFTdata[4]-biasTorque[4], rawFTdata[5]-biasTorque[5]};
		double Torques[3] = {rawFTdata[3], rawFTdata[4], rawFTdata[5]};
		double Forces[3] = {rawFTdata[0]+biasFT[0], rawFTdata[1]+biasFT[1], rawFTdata[2]+biasFT[2]};
		
		double timeStamp = ur5->rt_interface_->robot_state_->getTime();
		double elapsTime = timeStamp-startTime;
		
			  
		std::vector<double> q = ur5->rt_interface_->robot_state_->getQActual();
		
		
		double current_FT_Frame[3] = {rawFTdata[0]-biasFT[0], rawFTdata[1]-biasFT[1], rawFTdata[2]-biasFT[2]};
		forceTransformation(current_FT_Frame, current_TCP_Frame);
		
		gravityCompensation(q, biasWF); 
		
		double current_TCP_Frame_adjusted[3];
		adjustForce(sq[5], q[5], current_TCP_Frame_adjusted); // Used for compansating force indused by the equiptment? Remove?
		double new_TCP_Frame[3];
		for(int j=0; j<3; j++)
		{
			new_TCP_Frame[j] = current_TCP_Frame[j]-current_TCP_Frame_adjusted[j];
		}
		
		
		/*
		error_Fx = ref_TCP_Frame[0]-new_TCP_Frame[0];
		error_Fy = ref_TCP_Frame[1]-new_TCP_Frame[1];
   		error_Fz = ref_TCP_Frame[2]-new_TCP_Frame[2];
   		*/
   		
   		//Only update error if one of the forces exeedes a given threshhold (badly implemented low-pass filer)
   		//Vibrations in the manipulator transends to the end effector and gets interpeted as new inputs without this lowpass filter
   		/*
   		if(fabs(Forces[0]) < (2+f_ref) && fabs(Forces[1]) < (2+f_ref) && fabs(Forces[2]) < (2+f_ref))
   		{
			error_Fx = error_Fx/1.2; //Using [variable/1.2] to ensure a "soft" stopping behaviour
			error_Fy = error_Fy/1.2;
			error_Fz = error_Fz/1.2;
			
			integrator_Fx = integrator_Fx/1.2;
			integrator_Fy = integrator_Fy/1.2;
			integrator_Fz = integrator_Fz/1.2;
			
			
		}
		else
		{
			//ToDo: Give the different modes access to the error variables. Generates the desired buoyancy without the uncontrolled desent. 
			error_Fx = Forces[0];// + disturbances[0];
			error_Fy = Forces[1];// + buoyancy;// + disturbances[1];
			error_Fz = Forces[2];// + disturbances[2];
		}
		*/
		error_Fz = test_force + Forces[2];
		/*
		if (testTime < 100 && testTime > 50)
		{
			error_Fy = test_force;
		}
		else
		{
			error_Fy = 0;
		}*/
		if (testTime < 0)
		{
			ur5->setSpeed(0,0,0,0,0,0,1);
			break;
		}
		testTime = testTime -1;
		
		
		if(fabs(Torques[0]) < (1+t_ref) && fabs(Torques[1]) < (1+t_ref) && fabs(Torques[2]) < (0.4+t_ref))
		{
			error_Tx = error_Tx/1.2;
			error_Ty = error_Ty/1.2;
	   		error_Tz = error_Tz/1.2;
	   		
	   		integrator_Tx = integrator_Tx/1.2;
			integrator_Ty = integrator_Ty/1.2;
			integrator_Tz = integrator_Tz/1.2;
		}
		else
		{
			error_Tx = Torques[0];
			error_Ty = Torques[1];
	   		error_Tz = Torques[2];
		}

		
		if(fabs(error_Fx) > 50 || fabs(error_Fy) > 50 || fabs(error_Fz) > 50)
		{
			ur5->setSpeed(0,0,0,0,0,0,1);
			std::cout << "============================= STOPPING! ============================" << std::endl;
			std::cout << "Force levels too large - stopping force control!" << std::endl;
			break;
		}
		
		if(fabs(error_Tx) > 25 || fabs(error_Ty) > 25 || fabs(error_Tz) > 25)
		{
			ur5->setSpeed(0,0,0,0,0,0,1);
			std::cout << "============================= STOPPING! ============================" << std::endl;
			std::cout << "Torque levels too large - stopping force control!" << std::endl;
			break;
		}
		
		//GENERATING DISTURANCES
		if (force_mode == 3 && randomDuration < 1) 
		{
			/*
			for (int i = 0; i<6; i++)
			{
				prior_disturbances[i] = disturbances[i];
			}
			*/
			//Random disturbance force between -10 and 9 Newton 
			// --> scaled down by (1/150)*disturbance_scale before implemented in controller
			randomDisturbances[0] = (rand() % 30)-15;
			randomDisturbances[1] = (rand() % 30)-15;
			randomDisturbances[2] = (rand() % 30)-15;
			
			randomDisturbances[3] = (rand() % 20)-10;
			randomDisturbances[4] = (rand() % 20)-10;
			randomDisturbances[5] = (rand() % 20)-10;
			
			randomDuration = rand() % 500 + 250; //Random duration between 2-4 seconds
		}
		
		
		//===============Controller=====================
		//Translational forces - 3DOF
		integrator_Fx = integrator_Fx + error_Fx;
		integrator_Fy = integrator_Fy + error_Fy;
		integrator_Fz = integrator_Fz + error_Fz;
		
		derivator_Fx = error_Fx - prior_error_Fx;
		derivator_Fy = error_Fy - prior_error_Fy;
		derivator_Fz = error_Fz - prior_error_Fz;
		//ToDo: Implement disturbance as a BIAS. The current solution does not give the desired outcome
		u_Fx = Kp*error_Fx + Ki*integrator_Fx + Kd*derivator_Fx;// + disturbances[0];
		u_Fy = Kp*error_Fy + Ki*integrator_Fy + Kd*derivator_Fy;// + disturbances[1];
		u_Fz = Kp*error_Fz + Ki*integrator_Fz + Kd*derivator_Fz;// + disturbances[2];
		
		//Rotational torques
		integrator_Tx = integrator_Tx + error_Tx;
		integrator_Ty = integrator_Ty + error_Ty;
		integrator_Tz = integrator_Tz + error_Tz;
		
		derivator_Tx = error_Tx - prior_error_Tx;
		derivator_Ty = error_Ty - prior_error_Ty;
		derivator_Tz = error_Tz - prior_error_Tz;
		
		u_Tx = Kp_T*error_Tx + Ki_T*integrator_Tx + Kd_T*derivator_Tx + disturbances[3];
		u_Ty = Kp_T*error_Ty + Ki_T*integrator_Ty + Kd_T*derivator_Ty + disturbances[4];
		u_Tz = Kp_T*error_Tz + Ki_T*integrator_Tz + Kd_T*derivator_Tz + disturbances[5];
		
		//Set different modes to exert a different rehabilitation strategy
		if(force_mode == 1) // Compliance mode
		{
			vw[0] = u_Fx;
			vw[1] = u_Fy; 
			vw[2] = u_Fz; 
			vw[3] = 0;//u_Tx;
			vw[4] = 0;//u_Ty;
			vw[5] = 0;//u_Tz;
		}
		if(force_mode == 2) //Buoyancy mode
		{
			vw[0] = u_Fx;
			vw[1] = u_Fy;// + buoyancy; 
			vw[2] = u_Fz; 
			vw[3] = 0;//u_Tx;
			vw[4] = 0;//u_Ty;
			vw[5] = 0;//u_Tz;
			
			//biasFT[1] = -buoyancy; // This generates a constant force on the Y-axis.
		}
		if(force_mode == 3) //Random mode
		{
			vw[0] = u_Fx;
			vw[1] = u_Fy; 
			vw[2] = u_Fz; 
			vw[3] = 0;//u_Tx;
			vw[4] = 0;//u_Ty;
			vw[5] = 0;//u_Tz;
			
			/*
			for (int i = 0; i<6; i++)
			{
				if (disturbances[i] > (randomDisturbances[i]/150)*disturbance_scale && fabs(disturbances[i]) > 0.00001)
				{
					disturbances[i] = disturbances[i]/1.2; //Gentle step-down
					//prior_disturbances[i] = disturbances[i];
				}
				else if (disturbances[i] < (randomDisturbances[i]/150)*disturbance_scale && fabs(disturbances[i]) > 0.00001)
				{
					disturbances[i] = disturbances[i]*1.2; //Gentle step-up
				}
				else
				{
					disturbances[i] = (randomDisturbances[i]/150)*disturbance_scale;
				}
				/*
				else if (randomDisturdances[i] != prior_randomDisturbances[i] && disturbances[i] < ((randomDisturdances[i])/150)*disturbance_scale)
				{
					disturbances[i] = disturbance[i]*1.2; //Gentle step-up
				}
				else
				{
					disturbances[i] = (randomDisturbances[i]/150)*disturbance_scale;
				}
			}
			*/
			for (int j = 0; j<6; j++)
			{
				disturbances[j] = ((randomDisturbances[j]/150)*disturbance_scale);///randomDuration;
			}
			
			randomDuration = randomDuration-1;
		}
		if(force_mode == 4) //2-plane mode
		{
			vw[0] = u_Fx;
			vw[1] = 0;
			vw[2] = u_Fz;
			vw[3] = 0;
			vw[4] = 0;
			vw[5] = 0;//angular_speed;
		}
		
		//FUTURE WORK
		//Suggested modes: Syrup mode (less agressive controller parameters), 2-plane mode, 1-plane mode, functional reach
	
		solveInverseJacobian(q, vw, speed);
		
		
		ur5->rt_interface_->robot_state_->setDataPublished();
		ur5->setSpeed(speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], 20);
		
		
		prior_error_Fx = error_Fx; 
		prior_error_Fy = error_Fy;
		prior_error_Fz = error_Fz;
		
		prior_error_Tx = error_Tx; 
		prior_error_Ty = error_Ty;
		prior_error_Tz = error_Tz;
		
		
		forcelog << elapsTime << " " << speed[0] << " " << speed[1] << " " << speed[2] << " " << speed[3] << " " << speed[4] << " " << speed[5] << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << " " << rawFTdata[0] << " " << rawFTdata[1] << " " << rawFTdata[2] << " " << rawFTdata[3] << " " << rawFTdata[4] << " " << rawFTdata[5] << " " << Forces[0] << " " << Forces[1] << " " << Forces[2] << " " << Torques[0] << " " << Torques[1] << " " << Torques[2] << " " << error_Fx << " " << error_Fy << " " << error_Fz << " " << error_Tx << " " << error_Ty << " " << error_Tz << " " << u_Fx << " " << u_Fy << " " << u_Fz << " " << u_Tx << " " << u_Ty << " " << u_Tz << " " << biasFT[0] << " " << biasFT[1] << " " << biasFT[2] << " " << biasTF[0] << " " << biasTF[1] << " " << biasTF[2] << " "  << "\n";
		
		
		i = i+1;
		usleep(forceSleepTime);
		
		
	}	
	
	//stopFT(&forceID);
	
	forcelog.close();
}

