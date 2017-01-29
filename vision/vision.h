#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <sys/time.h>
#include <chrono>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// IMAGE DIMENSIONS
#define WIDTH 640
#define HEIGHT 480


// CAMERA INTRINSICS
#define FOCAL_X 674.6964
#define FOCAL_Y 673.0493
#define P_X 312.0442
#define P_Y 248.0589


void rot_x(double angle, double rot[9]);
void rot_y(double angle, double rot[9]);
void rot_z(double angle, double rot[9]);
void matmul4(double T[16], double v_in[4], double v_out[4]);
void threshCB(int pos, void* param);
void BoltDetection(std::vector<cv::Vec3f> *circles, cv::Point2f *hexPoint);
void ToolDetection(int sTool, cv::Point2d *exportCenter, int *exportDistance);
void trans3D(double gamma, double beta, double alpha, double resMat[9]);
void getTransformation(double rotGoal[9], double x, double y, double z, double T[16]);
void getRotation(double T[16], double rot[9]);
void calcDistance(int v, double Y_C, double *Z);
void gluInvertMatrix(const double m[16], double invOut[16]);
void image2base(double T[16], double Z_c, int u, int v, double baseXYZ[3]);