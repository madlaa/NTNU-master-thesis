#include "vision.h"

void threshCB(int pos, void* param){}

class exportTool
{
    public:
        double Area;
        cv::Point2d Center;
};

bool areaSort(exportTool i, exportTool j)
{
    return (i.Area>j.Area); 
}

void ToolDetection(int sTool, cv::Point2d *exportCenter, int *exportDistance)
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    double startTime = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  
  
    cv::VideoCapture cap;
    cap.open(CV_CAP_DC1394); 
    
    
    cap.set(CV_CAP_PROP_GAMMA,0);
    cap.set(CV_CAP_PROP_GAIN,30);
    cap.set(CV_CAP_PROP_FPS,60);
    cap.set(CV_CAP_PROP_EXPOSURE,1000);
    
  
    cv::namedWindow("Gray",CV_WINDOW_AUTOSIZE); 
    cv::namedWindow("Mask",CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Objects",CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Grayscale Histogram",CV_WINDOW_AUTOSIZE );
      
    cv::moveWindow("Gray",0,0);
    cv::moveWindow("Mask",0,700);
    cv::moveWindow("Objects",1400,0);
    cv::moveWindow("Grayscale Histogram",2200,200);
    
    double fontScale = 1;
    int thickness = 2;
    int threshVal = 40;
    
 
    cv::createTrackbar("Threshold","Gray",&threshVal,255,threshCB,NULL);
    
    
    int histSize = 256;
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;
    cv::Mat b_hist, g_hist, r_hist;
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    
    cv::Mat b_histE, g_histE, r_histE;
    
    
    
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;
    

    cv::Mat frame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat colorFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat grey(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat tresholdedFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat blurredFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat edgeFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat normFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat dilateFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat erodeFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    
    
    
    int erosion_size = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		      cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
                      cv::Point(erosion_size, erosion_size));
    
    
    
    double xc_ref = 128;
    double Kp = 1;
    double Ki = 0.05;
    double integrator = 0;
    double xc;
    double error;
    double exposure;
    
    
    while(1)
    {
	gettimeofday(&tp, NULL);
	double timeStamp = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	double elapsTime = (timeStamp-startTime)/1000;
      
        cap >> frame;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::cvtColor(frame,colorFrame,CV_BayerBG2RGB,0);
	cv::cvtColor(colorFrame,grey,CV_RGB2GRAY);	
	cv::normalize(grey, normFrame, 0, 255, cv::NORM_MINMAX);

	
	
	cv::calcHist(&normFrame, 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
        cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));
	cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
        for(int i = 1; i < histSize; i++)
	{
            cv::line(histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1))),
                     cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
                     cv::Scalar( 255, 0, 0), 2, 8, 0);
	}
        
        
        
        
	double num = 0;
	double den = 0;
	for(int i = 1; i < histSize; i++)
	{
	  num += i*b_hist.at<float>(i);
	  den += b_hist.at<float>(i);
	}
	xc = num/den;
	
	
	
	error = xc_ref-xc;
	integrator = integrator + error;
	exposure = Kp*error + Ki*integrator;
	cap.set(CV_CAP_PROP_EXPOSURE,exposure);
        
        
        

        cv::GaussianBlur(normFrame, blurredFrame, cv::Size(3,3), 0, 0);
        
	
	
	cv::Sobel(blurredFrame, grad_x, ddepth, 1, 0, 3, scale, delta);
	cv::convertScaleAbs(grad_x, abs_grad_x);
	cv::Sobel(blurredFrame, grad_y, ddepth, 0, 1, 3, scale, delta);
	cv::convertScaleAbs(grad_y, abs_grad_y);
	cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edgeFrame);
        
  
  
	
	
	cv::calcHist(&edgeFrame, 1, 0, cv::Mat(), b_histE, 1, &histSize, &histRange, uniform, accumulate);
        cv::Mat histImageE(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));
	cv::normalize(b_histE, b_histE, 0, histImageE.rows, cv::NORM_MINMAX, -1, cv::Mat());
	
	
	
	
	

	cv::threshold(edgeFrame,tresholdedFrame,threshVal,255,cv::THRESH_BINARY);
	
	

	cv::dilate(tresholdedFrame, dilateFrame, element);
	cv::erode(dilateFrame, erodeFrame, element);
	
	
	
	cv::Mat contourOutput = erodeFrame.clone();
	cv::findContours(contourOutput,contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	std::vector<cv::Moments> mu(contours.size());
	for(unsigned int i = 0; i < contours.size(); i++ )
	{
	    mu[i] = moments(contours[i],false);
	}
	std::vector<cv::Point2d> mc( contours.size());
	for(unsigned int i = 0; i < contours.size(); i++ )
	{
	    mc[i] = cv::Point2d(mu[i].m10/mu[i].m00,mu[i].m01/mu[i].m00);
	}
	std::vector<cv::Rect> boundRect( contours.size());
	std::vector<std::vector<cv::Point> > contours_poly( contours.size());
	std::vector<exportTool> exportData(contours.size());
	int ExportCount = 0;
	cv::Mat drawing = cv::Mat::zeros(colorFrame.size(),CV_8UC3);
	cv::Point object_center_position;
	int temp = 0;
	for(unsigned int i = 0; i< contours.size(); i++ )
	{
		cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 5, true );
		boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
		double ratio = (boundRect[i].br().x-boundRect[i].tl().x)/(boundRect[i].br().y-boundRect[i].tl().y);
		if(ratio > 3 && ratio < 7 && cv::contourArea(contours_poly[i]) != 0)
		{
		    if(cv::contourArea(contours_poly[i]) > 3000 && cv::contourArea(contours_poly[i]) < 40000)
		    {
	                cv::Scalar color = cv::Scalar( 255,0,0 );
			cv::drawContours( drawing, contours, i, color, -1, 8, hierarchy, 2, cv::Point() );
			color = cv::Scalar( 0,0,255 );
			cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
			double rectArea;
			rectArea = (boundRect[i].br().x-boundRect[i].tl().x) * (boundRect[i].br().y-boundRect[i].tl().y);
			//std::cout << rectArea << std::endl;
			cv::circle(drawing, mc[i], 5, color, -1, 8, 0);
			object_center_position = mc[i];
			std::string text = "Spanner";
			cv::Point textOrg(object_center_position);
			cv::putText(drawing, text, textOrg, 0, fontScale,
			cv::Scalar::all(255), thickness, 8);
			exportData[ExportCount].Area = rectArea;//cv::contourArea(contours_poly[i]);
			exportData[ExportCount].Center = object_center_position;
			ExportCount = ExportCount+1;
		    }
		}	
	}
	
	
	
	if(ExportCount > 1)
	{
	  temp = exportData[0].Center.y-exportData[1].Center.y;
	}
	
	
	
	cv::imshow("Gray", grey);
	cv::imshow("Mask", tresholdedFrame);
	cv::imshow("Objects", drawing);
	cv::imshow("Grayscale Histogram", histImage);

	
	

	if(cv::waitKey(1) >= 27 || elapsTime > 6)
	//if(cv::waitKey(1) >= 27)
	{
	    std::ofstream histolog;
	    histolog.open("../data/logs/tool_hist", std::ofstream::out);
	    for(int i = 1; i < histSize; i++)
	    {
	      histolog << i << " " << b_hist.at<float>(i) << " " << b_histE.at<float>(i) << "\n";
	    }
	    histolog.close();
	    
	    cv::imwrite("../data/images/tools_RAW.jpg", frame);
	    cv::imwrite("../data/images/tools_COLOR.jpg", colorFrame);
	    cv::imwrite("../data/images/tools_GRAY.jpg", grey);
	    cv::imwrite("../data/images/tools_THRESH.jpg", tresholdedFrame);
	    cv::imwrite("../data/images/tools_OBJECTS.jpg", drawing);
	    cv::imwrite("../data/images/tools_HISTO.jpg", histImage);
	    cv::imwrite("../data/images/tools_EDGES.jpg", edgeFrame);
	    cv::imwrite("../data/images/tools_NORM.jpg", normFrame);
	    cv::imwrite("../data/images/tools_BLUR.jpg", blurredFrame);
	    cv::imwrite("../data/images/tools_DILATE.jpg", dilateFrame);
	    cv::imwrite("../data/images/tools_ERODE.jpg", erodeFrame);
	    
	    
	    
	    cv::destroyWindow("Gray");
	    cv::destroyWindow("Mask");
	    cv::destroyWindow("Objects");
	    cv::destroyWindow("Grayscale Histogram");

	    
	    
	    for(int i = 0; i < 20; i++)
	    {
	        cv::waitKey(1);
	    }
	    
	    
	    if(ExportCount > 0)
	    {
	      sort(exportData.begin(), exportData.begin()+ExportCount+1, areaSort);
	    }
	    
	    for(int i = 0; i < ExportCount; i++)
	    {
	        std::cout << "Area: " << exportData[i].Area << "\t" << "Center: " << exportData[i].Center << std::endl;
	    }

	    
		      
	    
		      
	    switch(sTool)
	    {
		// TOOL SIZE 10 MM.
		case 1:
		{
		    *exportDistance = temp;
		    *exportCenter = exportData[ExportCount-1].Center;
		    
		    std::cout << "... exporting: " << *exportCenter << std::endl;
		    std::cout << "... exporting: " << *exportDistance << std::endl;
		    
		}break; 
		
		
		// TODO
		case 2:
		    *exportCenter = exportData[4].Center;
		    *exportDistance = temp;
		    std::cout << "... exporting: " << *exportCenter << std::endl;
		    std::cout << "... exporting: " << *exportDistance << std::endl;
		    break;
		case 3:
		    *exportCenter = exportData[3].Center;
		    *exportDistance = temp;
		    std::cout << "... exporting: " << *exportCenter << std::endl;
		    std::cout << "... exporting: " << *exportDistance << std::endl;
		    break;
		case 4:
		    *exportCenter = exportData[2].Center;
		    *exportDistance = temp;
		    std::cout << "... exporting: " << *exportCenter << std::endl;
		    std::cout << "... exporting: " << *exportDistance << std::endl;
		    break;
		case 5:
		    *exportCenter = exportData[1].Center;
		    *exportDistance = temp;
		    std::cout << "... exporting: " << *exportCenter << std::endl;
		    std::cout << "... exporting: " << *exportDistance << std::endl;
		    break;
		case 6:
		    *exportCenter = exportData[0].Center;
		    *exportDistance = temp;
		    std::cout << "... exporting: " << *exportCenter << std::endl;
		    std::cout << "... exporting: " << *exportDistance << std::endl;
		    break;
	    }	   
	    break;
	}
    }
}

void BoltDetection(std::vector<cv::Vec3f> *circles, cv::Point2f *hexPoint)
{
  
    struct timeval tp;
    gettimeofday(&tp, NULL);
    double startTime = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  
  
    cv::VideoCapture cap;
    cap.open(CV_CAP_DC1394); 
    
    
    cap.set(CV_CAP_PROP_GAMMA,0);
    cap.set(CV_CAP_PROP_GAIN,30);
    cap.set(CV_CAP_PROP_FPS,60);
    cap.set(CV_CAP_PROP_EXPOSURE,1000);
  
  
    cv::namedWindow("Color",CV_WINDOW_AUTOSIZE); 
    cv::namedWindow("Mask",CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Objects",CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Grayscale Histogram",CV_WINDOW_AUTOSIZE);
    
    
    cv::moveWindow("Color",0,0);
    cv::moveWindow("Mask",0,700);
    cv::moveWindow("Objects",700,700);
    cv::moveWindow("Grayscale Histogram",700,0);
    
    
    int threshVal  = 20;
    cv::createTrackbar("Threshold","Mask",&threshVal,255,threshCB,NULL);
   
    
    double fontScale = 1;
    int thickness = 2;
    
    
    int histSize = 256;
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;
    cv::Mat b_hist, g_hist, r_hist;
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    
    cv::Mat frame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat colorFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat grey(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat tresholdedFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat normFrame(WIDTH,HEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat houghFrame(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
    
    
    double xc_ref = 128;
    double Kp = 2;
    double Ki = 0.5;
    double integrator = 0;
    double xc;
    double error;
    double exposure;
    
    
    
    while(1)
    {
      	gettimeofday(&tp, NULL);
	double timeStamp = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	double elapsTime = (timeStamp-startTime)/1000;
	
	
	cap >> frame;
	cv::cvtColor(frame,colorFrame,CV_BayerBG2RGB,0);
	cv::cvtColor(colorFrame,grey,CV_RGB2GRAY );
	cv::normalize(grey, normFrame, 0, 255, cv::NORM_MINMAX);
	
	
	cv::calcHist(&grey, 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
        cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
        for(int i = 1; i < histSize; i++)
	{
            cv::line(histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1))),
                     cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
                     cv::Scalar( 255, 0, 0), 2, 8, 0);
	}
	
	
	
	double num = 0;
	double den = 0;
	for(int i = 1; i < histSize; i++)
	{
	  num += i*b_hist.at<float>(i);
	  den += b_hist.at<float>(i);
	}
	xc = num/den;
	

	error = xc_ref-xc;
	integrator = integrator + error;
	exposure = Kp*error + Ki*integrator;
	cap.set(CV_CAP_PROP_EXPOSURE,exposure);
	
	
	
	
	cv::Mat houghFrame = colorFrame.clone();
	cv::HoughCircles(normFrame, *circles, CV_HOUGH_GRADIENT,1, normFrame.rows/4, 75, 40, 75, 150);
	for(size_t i = 0; i < circles->size(); i++)
	{
	    cv::Point2f center(cvRound((*circles)[i][0]), cvRound((*circles)[i][1]));
	    int radius = (*circles)[i][2];
	    circle(houghFrame, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
	    //circle(houghFrame, center, radius/10, cv::Scalar(0,255,0), -1, 8, 0);
	    std::string text = "Valve";
	    cv::putText(houghFrame, text, center, 0, fontScale, cv::Scalar::all(255), thickness, 8);   
        }
	
	
	

	
        cv::threshold(normFrame,tresholdedFrame,threshVal,255,cv::THRESH_BINARY_INV);
        std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
        cv::Mat contourOutput = tresholdedFrame.clone();
	cv::findContours(contourOutput,contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        std::vector<cv::Moments> mu(contours.size());
	for(unsigned int i = 0; i < contours.size(); i++)
	{
	    mu[i] = moments(contours[i],false);
	}
	std::vector<cv::Point2f> mc( contours.size());
	for(unsigned int i = 0; i < contours.size(); i++)
	{
	    mc[i] = cv::Point2f(mu[i].m10/mu[i].m00,mu[i].m01/mu[i].m00);
	}
	cv::Mat drawing = cv::Mat::zeros(colorFrame.size(),CV_8UC3);
	std::vector<cv::Rect> boundRect( contours.size());
	std::vector<std::vector<cv::Point> > contours_poly( contours.size());
	for(unsigned int i = 0; i< contours.size(); i++ )
	{
	  cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
	  if(cv::contourArea(contours_poly[i]) > 1000 && cv::contourArea(contours_poly[i]) < 10000)
	  {
	      boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));  
	      cv::drawContours(drawing, contours, i, cv::Scalar(255,0,0), -1, 8, hierarchy, 2, cv::Point());
	      cv::rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0,0,255), 2, 8, 0);
	      cv::circle(drawing, mc[i], 5, cv::Scalar(0,0,255), -1, 8, 0);
	      *hexPoint = mc[i];
	  } 
	}
	
	
	//std::cout << xc << "\t" << exposure << "\t" << *hexPoint << std::endl;
	
	
	cv::imshow("Color",houghFrame);
	cv::imshow("Mask",tresholdedFrame);
	cv::imshow("Objects",drawing);
	cv::imshow("Grayscale Histogram", histImage);
	
	
	//if(cv::waitKey(1) >= 27)
	if(cv::waitKey(1) >= 27 || elapsTime > 3)
	{ 
	    std::ofstream histolog;
	    histolog.open("../data/logs/valve_hist", std::ofstream::out);
	    for(int i = 1; i < histSize; i++)
	    {
	      histolog << i << " " << b_hist.at<float>(i) << "\n";
	    }
	    histolog.close();
	  
	  
	    cv::imwrite("../data/images/valve_RAW.jpg", frame);
	    cv::imwrite("../data/images/valve_COLOR.jpg", colorFrame);
	    cv::imwrite("../data/images/valve_GREY.jpg", grey);
	    cv::imwrite("../data/images/valve_MASK.jpg", tresholdedFrame);
	    cv::imwrite("../data/images/valve_OBJECT.jpg", drawing);
	    cv::imwrite("../data/images/valve_HIST.jpg", histImage);
	    cv::imwrite("../data/images/valve_NORM.jpg", normFrame);
	    cv::imwrite("../data/images/valve_HOUGH.jpg", houghFrame);
	    
	    cv::destroyWindow("Color");
	    cv::destroyWindow("Mask");
	    cv::destroyWindow("Objects");
	    cv::destroyWindow("Grayscale Histogram");
	
	    std::cout << xc << "\t" << exposure << "\t" << *hexPoint << std::endl;
	    

	    for(int i = 0; i < 16; i++)
	    {
	        cv::waitKey(1);
	    }
	    break;
	}
    }
}

void matmul4(double T[16], double v_in[4], double v_out[4])
{
    v_out[0] = T[0]*v_in[0]+T[1]*v_in[1]+T[2]*v_in[2]+T[3]*v_in[3];
    v_out[1] = T[4]*v_in[0]+T[5]*v_in[1]+T[6]*v_in[2]+T[7]*v_in[3];
    v_out[2] = T[8]*v_in[0]+T[9]*v_in[1]+T[10]*v_in[2]+T[11]*v_in[3];
    v_out[3] = T[12]*v_in[0]+T[13]*v_in[1]+T[14]*v_in[2]+T[15]*v_in[3];
}

void image2base(double T[16], double Z_c, int u, int v, double baseXYZ[3])
{
    double X_c = ((u-P_X)/FOCAL_X)*Z_c;
    double Y_c = ((v-P_Y)/FOCAL_Y)*Z_c;

    double camFrame[4];
    camFrame[0] = X_c;
    camFrame[1] = Y_c;
    camFrame[2] = Z_c;
    camFrame[3] = 1;
    
    double cam2Wrist[16];
    cam2Wrist[0]  = -0.0394; cam2Wrist[1]  = -0.9994; cam2Wrist[2]  = 0;  cam2Wrist[3]  = 0; 
    cam2Wrist[4]  = 0.9994;  cam2Wrist[5]  = -0.0349; cam2Wrist[6]  = 0;  cam2Wrist[7]  = 0.0340; 
    cam2Wrist[8]  = 0;       cam2Wrist[9]  = 0;       cam2Wrist[10] = 1;  cam2Wrist[11] = 0.007;
    cam2Wrist[12] = 0;       cam2Wrist[13] = 0;       cam2Wrist[14] = 0;  cam2Wrist[15] = 1;

    double wrist[4];
    matmul4(cam2Wrist,camFrame,wrist);
     
    double base[4];
    matmul4(T,wrist,base);

    baseXYZ[0] = base[0];
    baseXYZ[1] = base[1];
    baseXYZ[2] = base[2];  
}

void rot_x(double angle, double rot[9])
{
    rot[0] = 1; rot[1] = 0;          rot[2] = 0;
    rot[3] = 0; rot[4] = cos(angle); rot[5] = -sin(angle);
    rot[6] = 0; rot[7] = sin(angle); rot[8] = cos(angle);
}


void rot_y(double angle, double rot[9])
{
    rot[0] = cos(angle);  rot[1] = 0; rot[2] = sin(angle);
    rot[3] = 0;           rot[4] = 1; rot[5] = 0;
    rot[6] = -sin(angle); rot[7] = 0; rot[8] = cos(angle);
}

void rot_z(double angle, double rot[9])
{
    rot[0] = cos(angle);  rot[1] = -sin(angle); rot[2] = 0;
    rot[3] = sin(angle);  rot[4] = cos(angle);  rot[5] = 0;
    rot[6] = 0;           rot[7] = 0;           rot[8] = 1;
}

void trans3D(double gamma, double beta, double alpha, double resMat[9])
{
    resMat[0] = cos(alpha)*cos(beta);
    resMat[1] = cos(alpha)*sin(beta)*sin(gamma)-cos(gamma)*sin(alpha);
    resMat[2] = sin(alpha)*sin(gamma)+cos(alpha)*cos(gamma)*sin(beta);
  
    resMat[3] = cos(beta)*sin(alpha);
    resMat[4] = cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma);
    resMat[5] = cos(gamma)*sin(alpha)*sin(beta)-cos(alpha)*sin(gamma);
  
    resMat[6] = -sin(beta);
    resMat[7] = cos(beta)*sin(gamma);
    resMat[8] = cos(beta)*cos(gamma);
}

void getTransformation(double rotGoal[9], double x, double y, double z, double T[16])
{
    T[0] = rotGoal[0];   T[1] = rotGoal[1];  T[2] = rotGoal[2];   T[3] = x;
    T[4] = rotGoal[3];   T[5] = rotGoal[4];  T[6] = rotGoal[5];   T[7] = y;
    T[8] = rotGoal[6];   T[9] = rotGoal[7];  T[10] = rotGoal[8];  T[11] = z;
    T[12] = 0;           T[13] = 0;          T[14] = 0;           T[15] = 1;
}

void getRotation(double T[16], double rot[9])
{
    rot[0] = T[0]; rot[1] = T[1]; rot[2] = T[2];
    rot[3] = T[4]; rot[4] = T[5]; rot[5] = T[6];
    rot[6] = T[8]; rot[7] = T[9]; rot[8] = T[10];
}

void calcDistance(int v, double Y_C, double *Z)
{
    *Z = (FOCAL_Y*Y_C)/v;
}

void gluInvertMatrix(const double m[16], double invOut[16])
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