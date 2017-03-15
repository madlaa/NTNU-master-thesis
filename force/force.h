#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <stdint.h>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>

#include "../ur_modern_driver-master/include/ur_driver.h"
#include "../kinematics/ur_kin.h"

#define PORT 49152 
#define COMMAND 2 
#define NUM_SAMPLES 0

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

typedef struct response_struct 
{
  uint32 rdt_sequence;
  uint32 ft_sequence;
  uint32 status;
  int32 FTData[6];
} RESPONSE;

typedef struct
{
  double R[9];
  double O[3];
} tfrotype;

void *getFTData(void *arg);
void startFT(pthread_t *forceID);
void stopFT(pthread_t *forceID);
void forceControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int run_time, int force_mode, double user_parameters[6], double f_ref, double t_ref);
void forceTransformation(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double ft_in[3], double ft_out[3]);
void adjustForce(double sq6, double cq6, double outF[3]);
void rotate(gsl_vector *res,gsl_matrix *R, gsl_vector *inp,gsl_vector *t1,gsl_vector *t2);
void solveInverseJacobian(std::vector<double> q, double vw[6], double qd[6]);


