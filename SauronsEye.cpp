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
vector<vector<Point> > set_contours(Mat delta_frame);
void check_room_status(string &room_status, Mat &frame,vector<vector<Point> > contours);



//for debugging
Scalar get_average(Mat array);

int main(void)
{

	Mat average;

	VideoCapture feed(0);  //open the default camera



	if (!feed.isOpened()) //check if we suceeded
	{

		return -1;
	}


	while(1)
	{
		Mat frame, gray_frame, delta_frame; //Capture frame-by-frame

		vector<vector<Point> > contours;
		string room_status="Unoccupied";
		feed >> frame;

		cvtColor(frame,gray_frame, cv::COLOR_BGR2GRAY);

		GaussianBlur(gray_frame,gray_frame,Size(21,21),0);


		delta_frame=set_delta(average,gray_frame);




		contours=set_contours(delta_frame);
	    check_room_status(room_status,frame,contours);


		//if the frame is empty break immediately
		if (frame.empty())
			break;
		display_information(frame, room_status);
		imshow("Frame", frame);




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

vector<vector<Point> > set_contours(Mat delta_frame)
{
	Mat thresh;
	vector<vector<Point> > contours;
	double delta_thresh=5;
	double max_value=255;

	threshold(delta_frame,thresh, delta_thresh,max_value,THRESH_BINARY);

	dilate(thresh, thresh,noArray(),Point(-1,-1),2);

	findContours(thresh,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

    return contours;

}
void check_room_status(string &room_status,Mat &frame, vector<vector <Point> > contours)
{
    for(int i =0;i<contours.size(); i++)
    {
        vector<Point> cnts=contours[i];
    	if (contourArea(cnts)<5000)
        	continue;
        Rect rect=boundingRect(cnts);
        Point pt1, pt2;
        pt1.x = rect.x;
        pt1.y = rect.y;
        pt2.x = rect.x + rect.width;
        pt2.y = rect.y + rect.height;
        rectangle(frame, pt1, pt2, (0,255,0), 2);
        room_status="Occupied";
    }
}
Scalar get_average(Mat array)
{
	vector<Mat> channels;
	split(array, channels);
	Scalar m = mean(channels[0]);
	return m;

}











