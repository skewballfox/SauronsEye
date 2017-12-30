/*
 * SauronsEye.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: daedalus
 */
#include <iostream> //for standard I/O
#include <chrono>
#include <ctime>
#include <algorithm>//for that amazing one liner in upload to remove whitespace
#include <experimental/filesystem>
#include <stdlib.h>

#include <curl/curl.h>

#include <opencv2/core/core.hpp> //Basic OpenCV structures (cv::Mat, Scalar)
#include "opencv2/videoio.hpp"
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/imgproc.hpp> //to use the putText function
#include <opencv2/imgcodecs.hpp> //to use imWrite

namespace fs=std::experimental::filesystem;
using namespace std;
using namespace cv;


const chrono::duration<double> min_upload_seconds=chrono::duration<double>(3.0);
const int min_motion_frames=8;



int motion_counter=0;
auto last_uploaded = chrono::system_clock::now();




//methods for motion detection
Mat set_delta(Mat &average, Mat gray_frame);
vector<vector<Point> > set_contours(Mat delta_frame);
void check_room_status(string &room_status, Mat &frame,vector<vector<Point> > contours);

//for displaying the necessary information on the image
void display_information(Mat &frame, string room_status);
//for formating time in order to display it
string NowToString();

//methods for cloud integration
void check_motion_counter(string room_status, auto timestamp, Mat frame);
void upload(Mat frame);
void send_to_drive(string file_name);
void send_to_dropbox(string file_name);


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
        auto timestamp=chrono::system_clock::now();
		vector<vector<Point> > contours;
		string room_status="Unoccupied";
		feed >> frame;

		cvtColor(frame,gray_frame, cv::COLOR_BGR2GRAY);

		GaussianBlur(gray_frame,gray_frame,Size(21,21),0);


		delta_frame=set_delta(average,gray_frame);




		contours=set_contours(delta_frame);
	    check_room_status(room_status,frame,contours);

	    //start the steps for determining whether to upload to cloud
	    check_motion_counter(room_status, timestamp, frame);


		//if the frame is empty break immediately
		if (frame.empty())
			break;
		display_information(frame, room_status);
		imshow("Frame", frame);




		//press ESC on keyboard to exit
		//press C to save
		char c=(char)waitKey(25);
		if(c==27)
			break;
		if (c==67)
			imwrite("/home/daedalus/test.jpeg",frame);
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

void check_motion_counter(string room_status, auto timestamp, Mat frame )
{
	if (room_status==("Occupied"))
	{
		chrono::duration<double> elapsed_time=timestamp-last_uploaded;
		if (elapsed_time>=min_upload_seconds)
		{
			motion_counter++;
			if (motion_counter>=min_motion_frames)
			{
				upload(frame);
				last_uploaded=timestamp;
				motion_counter=0;
			}
		}
	}
	else
	{
		motion_counter=0;
	}
}

void upload(Mat frame)
{

    string file_name=fs::current_path()+"/"+NowToString()+".jpg";
    file_name.erase (std::remove (file_name.begin(), file_name.end(), ' '), file_name.end());
    file_name.erase (std::remove (file_name.begin(), file_name.end(), '\n'), file_name.end());
    cout << "upload file: "<<file_name<<endl;
    imwrite(file_name,frame);
    send_to_drive(file_name);

}

void send_to_drive(string file_name)
{
	//cout<<"work in progress";
    string cmd="python "+fs::current_path()+"/"+"drive.py "+file_name;
    cout<<cmd<<endl;
	system((cmd).c_str());
}

void send_to_dropbox(string file_name)
{
	cout<<"work in progress"<<endl;

}

Scalar get_average(Mat array)
{
	vector<Mat> channels;
	split(array, channels);
	Scalar m = mean(channels[0]);
	return m;

}











