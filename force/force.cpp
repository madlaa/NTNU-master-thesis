#include "force.h"

#include "ujac_c.c"
#include "ufwdkin_c.c"

int initFT = 1;
double rawFTdata[6];

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
	//he = gethostbyname(argv[1]);
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
		//std::cout << "Current F/T readings on X axis are: \n" << rawFTdata[0] << std::endl;
		usleep(830);//usleep830 ~ 1200 Hz --> run a bit faster than FT broadcast frequency -- currently 1000 Hz
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
	usleep(500000);
}

void stopFT(pthread_t *forceID)
{
	initFT = 0;
	pthread_join(*forceID, NULL);
	std::cout << "Force thread joined - stopping force/torque data acquisition!" << std::endl;
	usleep(2000000);
}

void forceTransformation(double ft_in[3], double ft_out[3])
{
	ft_in = ft_out;
	/*
	double theta = 2.2328;
	double rot[9];
	rot_z(theta, rot); //Replace?

	double ft_wrist[4];
	ft_wrist[0] = rot[0]*ft_in[0] + rot[1]*ft_in[1] + rot[2]*ft_in[2]; 
	ft_wrist[1] = rot[3]*ft_in[0] + rot[4]*ft_in[1] + rot[5]*ft_in[2];
	ft_wrist[2] = rot[6]*ft_in[0] + rot[7]*ft_in[1] + rot[8]*ft_in[2];
	ft_wrist[3] = 1;

	ft_out[0] = ft_wrist[0];
	ft_out[1] = ft_wrist[1];
	ft_out[2] = ft_wrist[2];
	*/
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


void adjustForce(double sq6, double cq6, double outF[3])
{
	double magX = -11.4048;
	double magY = -11.4560;

	outF[0] = 5+magX*sin(sq6-cq6+0.4);
	outF[1] = 1+magY*sin(sq6/4+cq6+1)+magY;
}

void simpleForceControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int run_time, int rotation_on, double f_ref)
{




	pthread_t forceID;
	startFT(&forceID); //Results in two threads reading F/T data
  
	std::cout << "Force control initiated - starting compliance mode ...\n";
	std::ofstream forcelog;
	forcelog.open("../data/logs/forcelog", std::ofstream::out);
	//Control system for translation forces
	double integrator_Fx = 0;
	double integrator_Fy = 0;
	double integrator_Fz = 0;
  
	double Kp = -0.005; //Doublecheck PI controller tuning
	double Ki = -0.000025;
	
	double error_Fx = 0;
	double error_Fy = 0;
	double error_Fz = 0;
	
	double u_Fx = 0;
	double u_Fy = 0;
	double u_Fz = 0;
	
	//Control system for rotational forces
	double integrator_Tx = 0;
	double integrator_Ty = 0;
	double integrator_Tz = 0;
  
	double Kp_T = -0.005; //Doublecheck PI controller tuning
	double Ki_T = -0.000025;
	
	double error_Tx = 0;
	double error_Ty = 0;
	double error_Tz = 0;
	
	double u_Tx = 0;
	double u_Ty = 0;
	double u_Tz = 0;
	
	double speed[6] = {0,0,0,0,0,0};
	
	
	double ref_TCP_Frame[3];
	double current_TCP_Frame[3];
	
	double vw[6];
 
	
 
	double biasFT[3] = {rawFTdata[0], rawFTdata[1], rawFTdata[2]};
	double biasTorque[3] = {rawFTdata[3], rawFTdata[4], rawFTdata[5]};
	
	
	double ref_FT_Frame[3] = {rawFTdata[0]-biasFT[0], rawFTdata[1]-biasFT[1], rawFTdata[2]-biasFT[2]};
	
	
	double Fangle = -1.209406804939513;
	double f_refx = f_ref*cos(Fangle);
	double f_refy = f_ref*sin(Fangle);
	
	
	
	
	forceTransformation(ref_FT_Frame, ref_TCP_Frame);
	ref_TCP_Frame[0] = ref_TCP_Frame[0]+f_refx;
	ref_TCP_Frame[1] = ref_TCP_Frame[1]+f_refy;
	

	
	double startTime = ur5->rt_interface_->robot_state_->getTime();
	std::vector<double> sq = ur5->rt_interface_->robot_state_->getQActual();
	
	int i = 0; 
	int iter = run_time/0.008;
	double angular_speed = -2.25*M_PI/run_time;
	
	if(fabs(angular_speed) > 0.5) 
	{
		angular_speed = -0.35;
		std::cout << "Run-time too low -- setting angular speed to -0.35 rad/s" << std::endl;
	}
	
	while(i<iter)
	{
	
		std::mutex msg_lock;
		std::unique_lock<std::mutex> locker(msg_lock);
		while (!ur5->rt_interface_->robot_state_->getDataPublished())
		{
			rt_msg_cond_->wait(locker);
		}



		double Torques[3] = {rawFTdata[3]-biasTorque[3], rawFTdata[4]-biasTorque[4], rawFTdata[5]-biasTorque[5]};
		
		double timeStamp = ur5->rt_interface_->robot_state_->getTime();
		double elapsTime = timeStamp-startTime;
		
			  
		std::vector<double> q = ur5->rt_interface_->robot_state_->getQActual();
		
		
		double current_FT_Frame[3] = {rawFTdata[0]-biasFT[0], rawFTdata[1]-biasFT[1], rawFTdata[2]-biasFT[2]};
		forceTransformation(current_FT_Frame, current_TCP_Frame);
		
		
		
		double current_TCP_Frame_adjusted[3];
		adjustForce(sq[5], q[5], current_TCP_Frame_adjusted);
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
   		error_Fx = rawFTdata[0]-biasFT[0];//rawFTdata[0];
		error_Fy = rawFTdata[1]-biasFT[1];//rawFTdata[1];
   		error_Fz = rawFTdata[2]-biasFT[2];//rawFTdata[2];
		
		error_Tx = rawFTdata[3]-biasTorque[3];//Torques[0];
		error_Ty = rawFTdata[4]-biasTorque[4];//Torques[1];
   		error_Tz = rawFTdata[5]-biasTorque[5];//Torques[2];
		
		
		if(fabs(error_Fx) > 30 || fabs(error_Fy) > 30 || fabs(error_Fz) > 30)
		{
			ur5->setSpeed(0,0,0,0,0,0,1);
			std::cout << "Force levels too large - stopping control!" << std::endl;
			break;
		}
		
		
		
		//Controller
		integrator_Fx = integrator_Fx + error_Fx;
		integrator_Fy = integrator_Fy + error_Fy;
		integrator_Fz = integrator_Fz + error_Fz;
		
		integrator_Tx = integrator_Tx + error_Tx;
		integrator_Ty = integrator_Ty + error_Ty;
		integrator_Tz = integrator_Tz + error_Tz;
		
		
		u_Fx = Kp*error_Fx + Ki*integrator_Fx;
		u_Fy = Kp*error_Fy + Ki*integrator_Fy;
		u_Fz = Kp*error_Fz + Ki*integrator_Fz;
		
		u_Tx = Kp_T*error_Tx + Ki_T*integrator_Tx;
		u_Ty = Kp_T*error_Ty + Ki_T*integrator_Ty;
		u_Tz = Kp_T*error_Tz + Ki_T*integrator_Tz;
		
		if(rotation_on == 1)
		{
			vw[0] = u_Fx;
			vw[1] = u_Fy, 
			vw[2] = -u_Fz; 
			vw[3] = 0;//u_Tx;
			vw[4] = 0;//u_Ty;
			vw[5] = 0;//u_Tz;
		}
		if(rotation_on == 2)
		{
			vw[0] = u_Fx;
			vw[1] = u_Fy; 
			vw[2] = 0; 
			vw[3] = 0;
			vw[4] = 0;
			vw[5] = 0;
		}
		if(rotation_on == 3)
		{
			vw[0] = 0;
			vw[1] = 0; 
			vw[2] = 0; 
			vw[3] = 0;
			vw[4] = 0;
			vw[5] = 0;
		}
		if(rotation_on == 4)
		{
			vw[0] = 0;
			vw[1] = 0; 
			vw[2] = 0; 
			vw[3] = 0;
			vw[4] = 0;
			vw[5] = angular_speed;
		}
	
	
		solveInverseJacobian(q, vw, speed);
	
	
		//std::cout << "Does this compute? ...\n";
		ur5->rt_interface_->robot_state_->setDataPublished();
		ur5->setSpeed(speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], 1);
	
		//std::cout << elapsTime << "\t" << i << std::endl;
		//std::cout << speed[0] << " " << speed[1] << " " << speed[2] << " " << speed[3] << " " << speed[4] << " " << speed[5] << "\n";
	
	
		forcelog << elapsTime << " " << speed[0] << " " << speed[1] << " " << speed[2] << " " << speed[3] << " " << speed[4] << " " << speed[5] << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << " " << u_Fx << " " << u_Fy << " " << u_Fz << " " << current_TCP_Frame[0] << " " << current_TCP_Frame[1] << " " << current_TCP_Frame[2] << " " << ref_TCP_Frame[0] << " " << ref_TCP_Frame[1] << " " << ref_TCP_Frame[2] << " " << current_TCP_Frame_adjusted[0] << " " << current_TCP_Frame_adjusted[1] << " " << new_TCP_Frame[0] << " " << new_TCP_Frame[1] << " " << Torques[0] << " " << Torques[1] << " " << Torques[2] << "\n";
		
		
		i=i+1;
	}	
	//stopFT(&forceID);
	forcelog.close();
}

