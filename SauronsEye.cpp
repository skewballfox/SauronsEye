/*
 * SauronsEye.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: daedalus
 */
#include <iostream> //for standard I/O
#include <chrono>
#include <ctime>



#include <opencv2/core/core.hpp> //Basic OpenCV structures (cv::Mat, Scalar)
#include "opencv2/videoio.hpp"
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/imgproc.hpp> //to use the putText function
#include <opencv2/imgcodecs.hpp> //to use imWrite

using namespace std;
using namespace cv;

string NowToString();
void display_information(Mat &frame, string room_status);
Mat set_delta(Mat &average, Mat gray_frame);
Mat set_contours(Mat delta_frame);


//for debugging
Scalar get_average(Mat array);

int main(void)
{
	string room_status="Unoccupied";
	Mat average;

	VideoCapture feed(0);  //open the default camera



	if (!feed.isOpened()) //check if we suceeded
	{
		cout << "we did it";
		return -1;
	}


	while(1)
	{
		Mat frame, gray_frame, delta_frame, contour_frame; //Capture frame-by-frame
		feed >> frame;
		//cout << "frame: \n" << frame<< endl;
		cvtColor(frame,gray_frame, cv::COLOR_BGR2GRAY);
		//cout << "frame: \n" << frame<< endl;
		//cout<< "gray_frame: \n" <<gray_frame << endl;
		GaussianBlur(gray_frame,gray_frame,Size(21,21),0);
		//cout<< gray_frame << endl;

		delta_frame=set_delta(average,gray_frame);

		contour_frame=set_contours(delta_frame);


		//if the frame is empty break immediately
		if (frame.empty())
			break;
		display_information(frame, room_status);
		imshow("Frame", delta_frame);




		//press ESC on keyboard to exit
		char c=(char)waitKey(25);
		if(c==27)
			break;
		if (c==67)
			imwrite("/home/daedalus/test.bmp",frame);
	}

	//when everything is done, release the video capture object
	feed.release();

	//closes all the frames
	destroyAllWindows();
	return 0;
	}

void display_information(Mat &frame, string room_status)
{

	   const string timestamp=NowToString();
	   const string occupation="Room Status: "+room_status;

	   cv::putText(frame,timestamp,Point(0,20), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
	   cv::putText(frame,occupation,Point(0,470), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
}
string NowToString()
{
	time_t result = std::time(nullptr);
    return asctime(std::localtime(&result));
}

Mat set_delta(Mat &average, Mat gray_frame)
{
	Mat delta_frame, scaled_average;
	//delta_frame is the storage for the difference between the accumulated average
	//and the current gray frame. scaled average is for storing the output of convertScaleAbs()
	//otherwise the depth of global average is 0 and causes an error on the second loop
	if (average.empty()==1)
	{
		gray_frame.convertTo(average, CV_32FC(gray_frame.channels()));
	}

	accumulateWeighted(gray_frame, average, .5);
	convertScaleAbs(average, scaled_average);
	absdiff(gray_frame,scaled_average,delta_frame);

	return delta_frame;
}

Mat set_contours(Mat delta_frame)
{
	Mat thresh, cnts;
	double delta_thresh=5;
	double max_value=255;
	cout << "pass 1\n";
	threshold(delta_frame,thresh, delta_thresh,max_value,THRESH_BINARY);
	cout << "average of thresh is: "<< get_average(thresh);
	dilate(thresh, thresh,Mat(),Point(-1,-1),2);
	cout << "pass 3\n";
	findContours(thresh,(Mat(),cnts,Mat()),RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point(0,0));
	cout << "pass 4\n";
    return cnts;

}
Scalar get_average(Mat array)
{
	vector<Mat> channels;
	split(array, channels);
	Scalar m = mean(channels[0]);
	return m;

}











