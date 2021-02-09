#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <iostream>
#include "microRobot.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include <opencv2/core/core.hpp>
#include "microrobot.h"
#include <opencv2/flann/flann.hpp>
#include <algorithm>
#include "FlyCapture2.h"
#include "SerialClass.h"
#include "controller.h"
#include <fstream>
#include <chrono> 
using namespace std::chrono;
using std::ofstream;

using namespace cv;
using namespace std;
using namespace FlyCapture2;

# define pi_def           3.14159265358979323846  /* pi */

class frameProcessor {
private:
	Rect2d crop_rect;
	const String window_capture_name = "Video Capture";
	const String window_detection_name = "Object Detection";

	FlyCapture2::Error error;
	FlyCapture2::Image rawImage;
	FlyCapture2::Image rgbImage;

	Camera camera;
	CameraInfo camInfo;
	clock_t start, end;
	double elapsed;

	int sum_goals = 0;
	int sum_waypoints = 0;

	//For recording images while actuation
	int count_act = 0;
	int count_act_max;
public:
	std::chrono::time_point<std::chrono::system_clock> current_time;
	std::chrono::time_point<std::chrono::system_clock> start_time;

	bool writeFrame = false;
	bool getInput = false;
	bool RecordedImages = false; //If older images are used
	bool CameraImages = false; //For input from cameras
	bool ManualControl = false; //Manual Control using w-a-s-d
	bool ArduinoControl = false; //To switch on arduino control
	bool AutoControl = false;
	bool FeedbackControl = false;
	bool WriteFail = true;
	bool PresetGoals = false;
	bool cropFrame = false;
	bool FeedbackControl_Orientation = false;
	bool RecordSelectImage = false;
	cv::Point pt; //Mouse callback
	vector<vector<cv::Point2f>> pt_goals;

	const int max_value_H = 360 / 2;
	const int max_value = 255;
	int low_H = 0, low_S = 0, low_V = 0;
	int high_H = max_value_H, high_S = max_value, high_V = max_value;

	Mat src, dst;
	int morph_elem = 0;
	int morph_size = 0;
	int morph_operator = 0;
	int const max_operator = 4;
	int const max_elem = 2;
	int const max_kernel_size = 21;
	int erosion_elem = 0;
	int erosion_size = 0;
	int dilation_elem = 0;
	int dilation_size = 0;

	int num_robots = 2;
	int frame_begin;
	int frame_end;
	int frame_num;
	String frame_base_add;
	int cur_frame_num;
	Mat cur_frame;
	int write_image = 0;
	int write_image_num;

	//int phase_image;
	//int dir_image; //0 for x, 1 for y

	cv::Point2f mid_platform_x;
	cv::Point2f mid_platform_y;

	vector<Point2f> goal_p_0;
	vector<Point2f> goal_p_1;
	vector<Point2f> goal_p_2;
	vector<Point2f> goal_p_3;
	Point2f temp_p;

	char image_name[130];
	//namedWindow(window_capture_name);
	// Output display window name
	std::string windowNameOutput;

	Mat frame, frame_HSV, frame_threshold, frame_temp;
	Scalar color = Scalar(100, 255, 255); // B G R values
	Scalar color_red = Scalar(0, 0, 255);
	Scalar color_blue = Scalar(255, 0, 0);

	const char *arduino_add;
	int quad_select;

	//For feedback control
	int num_goals;

	//For writing images
	String img_file;

	//Writing to file
	ofstream outdata; // outdata is like cin

	float err_thres = 4;
	microRobot robots[10];
	Rect2f temp_frame;
	controller control;
	bool arduino_set_controller = false;

	void reset() {
		bool writeFrame = false;
		bool getInput = false;
		bool RecordedImages = false; //If older images are used
		bool CameraImages = false; //For input from cameras
		bool ManualControl = false; //Manual Control using w-a-s-d
		bool ArduinoControl = false; //To switch on arduino control
		bool AutoControl = false;
		bool FeedbackControl = false;
		bool WriteFail = true;
		bool PresetGoals = false;
		bool cropFrame = false;
		bool FeedbackControl_Orientation = false;
		bool RecordSelectImage = false;
		bool arduino_set_controller = false;
	}
	//p_goals is used when feedback control with zero goal points are initiated
	void p_goals() {
		//Zone 0 - U
		temp_p.x = 522;
		temp_p.y = 101;
		goal_p_0.push_back(temp_p);
		temp_p.y = 259;
		goal_p_0.push_back(temp_p);
		temp_p.x = 655;
		goal_p_0.push_back(temp_p);
		temp_p.y = 101;
		goal_p_0.push_back(temp_p);
		goal_p_0.push_back(temp_p);

		//Zone 1 - P
		temp_p.x = 145;
		temp_p.y = 259;
		goal_p_1.push_back(temp_p);
		temp_p.y = 101;
		goal_p_1.push_back(temp_p);
		temp_p.x = 278;
		goal_p_1.push_back(temp_p);
		temp_p.y = 187;
		goal_p_1.push_back(temp_p);
		temp_p.x = 145;
		goal_p_1.push_back(temp_p);

		//Zone 2 - W
		temp_p.x = 145;
		temp_p.y = 463;
		goal_p_2.push_back(temp_p);
		temp_p.y = 621;
		goal_p_2.push_back(temp_p);
		temp_p.x = 213;
		temp_p.y = 560;
		goal_p_2.push_back(temp_p);
		temp_p.x = 278;
		temp_p.y = 621;
		goal_p_2.push_back(temp_p);
		temp_p.y = 463;
		goal_p_2.push_back(temp_p);

		//Zone 2 - L
		temp_p.x = 522;
		temp_p.y = 463;
		goal_p_3.push_back(temp_p);
		temp_p.y = 621;
		goal_p_3.push_back(temp_p);
		temp_p.x = 655;
		goal_p_3.push_back(temp_p);
		goal_p_3.push_back(temp_p);
		goal_p_3.push_back(temp_p);

		num_goals = 5;
	}
	void crop_frame(int x_pos, int y_pos, int height_frame, int width_frame) {
		crop_rect.x = x_pos;
		crop_rect.y = y_pos;
		crop_rect.width = width_frame;
		crop_rect.height = height_frame;
		cropFrame = true;
	}

	void set_numRobots(int d) {
		num_robots = d;
	}

	void write_frame(int y) {
		int d1, d2, d3, d4;
		String image_name;
		//int x = cur_frame_num;
		d1 = y / 1000;
		d2 = (y % 1000) / 100;
		d3 = (y % 100) / 10;
		d4 = y % 10;
		image_name = img_file + to_string(d1) + to_string(d2) + to_string(d3) + to_string(d4) + ".bmp";
		cv::imwrite(image_name, frame);
	}

	void Morphology_Operations(int, void*)
	{
		int operation = morph_operator + 2;
		Mat element = getStructuringElement(morph_elem, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
		cv::morphologyEx(src, dst, operation, element);
		//imshow(window_capture_name, dst);
	}

	void setHSV_color(char color_choice) {
		morph_elem = 0;
		morph_size = 2;
		morph_operator = 0;

		switch (color_choice) {
		case 'r': {
			//Red
			/*
			low_H = 0;
			high_H = 85;
			low_S = 0;
			high_S = 142;
			low_V = 0;
			high_V = 255;
			*/
			/* //PUWL Experiment
			low_H = 0;
			high_H = 116;
			low_S = 126;
			high_S = 255;
			low_V = 133;
			high_V = 255;
			*/
			/*
			//Zoom Test
			low_H = 122;
			high_H = 180;
			low_S = 116;
			high_S = 184;
			low_V = 0;
			high_V = 255;
			*/
			//Zoom Test 2
			low_H = 0;
			high_H = 180;
			low_S = 118;
			high_S = 255;
			low_V = 183;
			high_V = 190;
			break;
		}
		case 'b': {

			//Blue Color
			/*
			low_H = 101;
			high_H = 142;
			low_S = 0;
			high_S = 255;
			low_V = 0;
			high_V = 255;
			*/
			/* //PUWL Experiement
			low_H = 108;
			high_H = 161;
			low_S = 52;
			high_S = 178;
			low_V = 59;
			high_V = 142;
			*/
			/*
			//Zoom Experiment
			low_H = 97;
			high_H = 180;
			low_S = 176;
			high_S = 255;
			low_V = 0;
			high_V = 255;
			*/
			//Zoom Experiment 2
			low_H = 37;
			high_H = 180;
			low_S = 123;
			high_S = 214;
			low_V = 114;
			high_V = 180;
			break;
		}
		default: {
			low_H = 0;
			high_H = 180;
			low_S = 0;
			high_S = 255;
			low_V = 0;
			high_V = 210;
			break;
		}
		}
	}

	void detect_color(char color_select, int num_robots, Mat frame, vector<Point2f> &mc_temp) {
		//Use contours to detect multiple blobs
		switch (color_select) {
		case 'r': {
			setHSV_color('r');
			break;
		}
		case 'b': {
			setHSV_color('b');
			break; }
		default: {
			cout << "Error in selecting color" << endl;
			break;
		}
		}
		Mat frame_HSV, frame_threshold;
		Mat canny_output;
		vector<vector<Point> > contours; //Contours
		vector<Vec4i> hierarchy; //For contours
		Scalar color = Scalar(100, 255, 255); // B G R values

		cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
		inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
		src = frame_threshold;
		Morphology_Operations(0, 0);
		Canny(dst, canny_output, 50, 150, 3);
		// find contours
		//findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<vector<Point> >hull(contours.size());

		vector<Moments> mu(contours.size()); //Moments
											 //vector<Moments> mu_temp(contours.size()); //Moments
											 // get the moments
		for (int i = 0; i < contours.size(); i++)
		{
			convexHull(Mat(contours[i]), hull[i], false);
			//mu_temp[i] = moments(contours[i], false);
			mu[i] = moments(contours[i], false);

		}

		// get the centroid of figures.
		vector<Point2f> mc(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}
		mc_temp = mc;
	}

	void detect_color_crop(char color_select, int num_robots, Mat frame, vector<Point2f> &mc_temp, vector<Rect2f> pos_crop) {
		//Use robot position estimates to crop camera frame to multiple smaller frames that find moments of threshold image
		switch (color_select) {
		case 'r': {
			setHSV_color('r');
			break;
		}
		case 'b': {
			setHSV_color('b');
			break; }
		default: {
			cout << "Error in selecting color" << endl;
			break;
		}
		}


		Mat frame_HSV, frame_threshold, thr, gray, frame_crop;
		Mat canny_output;
		vector<vector<Point> > contours; //Contours
		vector<Vec4i> hierarchy; //For contours
		Scalar color = Scalar(100, 255, 255); // B G R values

		for (int i = 0; i < num_robots; i++) {

			frame_crop = frame(pos_crop[i]);
			cvtColor(frame_crop, frame_HSV, COLOR_BGR2HSV);
			inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
			src = frame_threshold;
			Morphology_Operations(0, 0);
			// find moments of the image
			//cvtColor(src, gray, COLOR_BGR2GRAY);
			//threshold(gray, thr, 100, 255, THRESH_BINARY);
			Moments m = moments(src, true);
			Point2f p(m.m10 / m.m00, m.m01 / m.m00);
			mc_temp[i].x = p.x + pos_crop[i].x;
			mc_temp[i].y = p.y + pos_crop[i].y;

		}
	}

	int find_robot_ind(vector<Point2f> mc_red, int num_robots, int robot_ind, Point2f prev_pos) {
		double min_value = 100;
		double comp_value;
		int min_ind = -1;

		if (mc_red.size() > num_robots) {
			cout << "More points detected";
			waitKey(0);
		}

		if (mc_red.size() < num_robots) {
			cout << "Less points detected";
			waitKey(0);
		}

		for (int y = 0; y < num_robots; y++) {
			comp_value = norm(mc_red[y] - prev_pos);

			if (comp_value < min_value) {
				min_value = comp_value;
				min_ind = y;
			}
		}
		return min_ind;
	}

	void displayOutput(std::string wn) {

		windowNameOutput = wn;
		//cv::namedWindow(windowNameOutput, WINDOW_AUTOSIZE);
		cv::namedWindow(windowNameOutput, WINDOW_NORMAL);

	}

	//Set method of 
	void setFrameProcess(String img_filename, int first_frame, int last_frame) {
		frame_begin = first_frame;
		frame_end = last_frame;
		frame_base_add = img_filename;
		RecordedImages = true;
		CameraImages = false;
		writeFrame = false;
	}

	bool setFrameProcess(String image_file_loc) {
		img_file = image_file_loc;
		writeFrame = true;
		// Connect the camera
		error = camera.Connect(0);
		if (error != PGRERROR_OK)
		{
			std::cout << "Failed to connect to camera" << std::endl;
			return false;
		}

		// Get the camera info and print it out
		error = camera.GetCameraInfo(&camInfo);
		if (error != PGRERROR_OK)
		{
			std::cout << "Failed to get camera info from camera" << std::endl;
			return false;
		}
		std::cout << camInfo.vendorName << " "
			<< camInfo.modelName << " "
			<< camInfo.serialNumber << std::endl;

		error = camera.StartCapture();
		if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
		{
			std::cout << "Bandwidth exceeded" << std::endl;
			return false;
		}
		else if (error != PGRERROR_OK)
		{
			std::cout << "Failed to start image capture" << std::endl;
			return false;
		}
		frame_begin = 0;
		frame_end = 10000;
		RecordedImages = false;
		CameraImages = true;
		return true;
	}

	void readFrame(int y) {
		int d1, d2, d3, d4;

		//int x = cur_frame_num;
		d1 = y / 1000;
		d2 = (y % 1000) / 100;
		d3 = (y % 100) / 10;
		d4 = y % 10;
		String image_name;
		image_name = frame_base_add + to_string(d1) + to_string(d2) + to_string(d3) + to_string(d4) + ".bmp";

		frame_temp = imread(image_name);
		
		cur_frame = frame_temp(crop_rect);
	}

	static void CallBackFunc(int event, int x, int y, int flags, void* userdata)
	{
		if (event == EVENT_LBUTTONDOWN)
		{
			cv::Point* ptPtr = (cv::Point*)userdata;
			ptPtr->x = x;
			ptPtr->y = y;
		}

	}

	void setInput(bool condition) {
		getInput = condition;
	}
	/*
	void set_arduino(char *ard_add, bool manual_control) {
		if (manual_control == true) {
			FeedbackControl = false;
			ArduinoControl = true;
			ManualControl = true;
			arduino_add = ard_add;
		}
		else {
			cout << "Wrong arduino settings";
		}
	}
	*/

	void set_arduino(char *ard_add, bool manual_control, int no_goals) {
		if (manual_control == false) {
			FeedbackControl = true;
			num_goals = no_goals;
			ManualControl = manual_control;
			ArduinoControl = true;
			arduino_add = ard_add;
		}
		else {
			FeedbackControl = false;
			ArduinoControl = true;
			ManualControl = true;
			arduino_add = ard_add;
			quad_select = no_goals; //select quadrant is no_goals
		}
	}

	void set_arduino() {
		ArduinoControl = false;
		cout << "No Arduino Connected" << endl;
	}

	/*
	void set_goals(bool manual, int no_goals) {
		if(manual==false){
			FeedbackControl = true;
			num_goals = no_goals;
			ManualControl = manual;
		}

	}
	void set_goals(bool manual) {
		FeedbackControl = false;
		ManualControl = true;

	}
	*/
	void get_point(Point2f &pt_temp) {
		char key_select = 'b';
		while (key_select != 'a') {
			//cout << key_select << endl;
			key_select = waitKey(0);
		}
		pt_temp = static_cast<cv::Point2f>(pt); //Converts int to float for computation
	}


	void run_setup(float zoom_multiplier, int num_robots_new, int num_goals_new, int frame_num) {
		num_robots = num_robots_new;
		num_goals = num_goals_new;
		cv::setMouseCallback(windowNameOutput, CallBackFunc, &pt);
		//cv::Point pt(-1, -1);
		cv::Point2f pt_float(-1, -1);
		char input_key = '0';
		//This will run only the first time

		if (frame_num == 0) {
			//cout << frame_num;
			error = camera.RetrieveBuffer(&rawImage);
			if (error != PGRERROR_OK)
			{
				std::cout << "capture error" << std::endl;
			}
			rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);
			// convert to OpenCV Mat
			unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
			//cv::Mat cur_frame = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
			cur_frame = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
			//frame_temp = cur_frame;
			current_time = std::chrono::system_clock::now();

			temp_frame.width = 50 * zoom_multiplier;
			temp_frame.height = 50 * zoom_multiplier;
			if (cropFrame == true) {
				frame = cur_frame(crop_rect);
			}
			else {
				frame = cur_frame;
			}

			imshow(windowNameOutput, frame);

			set_file(); //Set up data file for output
			if (ArduinoControl == true) {
				control.set_arduino(arduino_add, zoom_multiplier);
			}
			else {
				control.set_arduino();
			}
			//Midpoints of the workspace
			cout << "Click midpoint of workspace (Motion X) and Press 'a'" << endl;
			get_point(pt_float);
			mid_platform_x = pt_float;
			control.mid_point_x = mid_platform_x;

			cout << "Click midpoint of workspace (Motion Y) and Press 'a'" << endl;
			get_point(pt_float);
			mid_platform_y = pt_float;
			control.mid_point_y = mid_platform_y;
			arduino_set_controller = false;
		}
		
		for (int k = 0; k < num_robots; k++) {
			robots[k].clear_goal();
			robots[k].set_midpoint(mid_platform_x.x, mid_platform_x.y, mid_platform_y.x, mid_platform_y.y);
			//Get positions of robots

			cout << "Click position of Robot " << k + 1 << "and Press 'a'" << endl;
			if (ManualControl == true) {
				robots[k].set_id(k, true);
			}
			else
			{
				robots[k].set_id(k, false);
			}
			get_point(pt_float);
			robots[k].initialize_pos(pt_float);
			pt.x = -1;
			pt.y = -1;
			cout << robots[k].pos << endl;
		}
		int ind = 0;
		if (num_goals == 0) {
			PresetGoals = true;
			p_goals();
		}
		if (FeedbackControl == true) {
			for (int k = 0; k < num_robots; k++) {
				//circle(frame, robots[k].pos_b, 4, color_blue, -1, 8, 0);
				//circle(frame, robots[k].pos, 4, color, -1, 8, 0);

				if (PresetGoals == false) {
					pt_float = { (-1, -1) };
					cout << "Click goal locations of Robot " << k + 1 << endl;
					robots[k].num_goals = num_goals;
					for (int j = 0; j < num_goals; j++) {
						get_point(pt_float);
						robots[k].set_goal(pt_float);
						pt.x = -1;
						pt.y = -1;
						cout << "Goal points for Robot " << k + 1 << ":" << robots[k].goals << endl;
					}
				}
				else {

					for (int j = 0; j < num_goals; j++) {
						switch (robots[k].id) {
						case 0: {
							robots[k].set_goal(goal_p_0[j]);
							break;
						}
						case 1: {
							robots[k].set_goal(goal_p_1[j]);
							break;
						}
						case 2: {
							robots[k].set_goal(goal_p_2[j]);
							break;
						}
						case 3: {
							robots[k].set_goal(goal_p_3[j]);
							break;
						}
						}

					}
				}
			}
		}
	}

	int run_continous(int pause_time, int act_time, float zoom_multiplier, int frame_begin)
	{
		count_act_max = pause_time / act_time;
		char input_key = '0';
		int last_frame = frame_begin + 1;
		sum_goals = 0;
		sum_waypoints = 0;
		start_time = std::chrono::system_clock::now();

		for (int x = frame_begin; x <= frame_end; x++) {

			cur_frame_num = x;
			if (RecordedImages == true) {
				readFrame(x);

			}
			else if (CameraImages == true) { //Camera Capture option
				error = camera.RetrieveBuffer(&rawImage);
				if (error != PGRERROR_OK)
				{
					std::cout << "capture error" << std::endl;
					continue;
				}

				rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

				// convert to OpenCV Mat
				unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
				//cv::Mat cur_frame = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
				cur_frame = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
				//frame_temp = cur_frame;
				current_time = std::chrono::system_clock::now();

			}


			if (cropFrame == true) {
				frame = cur_frame(crop_rect);
			}
			else {
				frame = cur_frame;
			}
			for (int q = 0; q < num_robots; q++) {
				//circle(frame, robots[q].goals[robots[q].goal_ind], 4, color_red, -1, 8, 0);
			}
			if (ManualControl == true) {
				imshow(windowNameOutput, frame);
			}

			//imshow(windowNameOutput, cur_frame);
			if (ManualControl == false) {
				vector<Point2f> mc_red(num_robots);
				vector<Point2f> mc_blue(num_robots);

				vector<Rect2f> robot_crop_frames;

				for (int s = 0; s < num_robots; s++) {
					//temp_frame.width = 50 * zoom_multiplier;
					//temp_frame.height = 50 * zoom_multiplier;
					//temp_frame.x = robots[s].pos.x;
					//temp_frame.y = robots[s].pos.y;
					for (int s = 0; s < num_robots; s++) {
						temp_frame.x = robots[s].pos.x - temp_frame.width / 2;;
						temp_frame.y = robots[s].pos.y - temp_frame.height / 2;

						if (robots[s].pos.x - temp_frame.width / 2 <= 0)
							temp_frame.x = 0;
						if (1279 - robots[s].pos.x - temp_frame.width / 2 <= 0)
							temp_frame.x = 1279 - temp_frame.width;

						if (robots[s].pos.y - temp_frame.height / 2 <= 0)
							temp_frame.y = 0;
						if (1023 - robots[s].pos.y - temp_frame.height / 2 <= 0)
							temp_frame.y = 1023 - temp_frame.height;

						robot_crop_frames.push_back(temp_frame);
					}
				}
				//This will get mc_red and mc_blue positions for all robots using contours
				//detect_color('r', num_robots, frame, mc_red);
				//detect_color('b', num_robots, frame, mc_blue);


				detect_color_crop('r', num_robots, frame, mc_red, robot_crop_frames);
				detect_color_crop('b', num_robots, frame, mc_blue, robot_crop_frames);

				for (int q = 0; q < num_robots; q++) {
					//This is for contours
					//robots[q].set_red(mc_red[find_robot_ind(mc_red, num_robots, q, robots[q].pos)]);
					//robots[q].set_blue(mc_blue[find_robot_ind(mc_blue, num_robots, q, robots[q].pos)]);

					robots[q].set_red(mc_red[q]);
					robots[q].set_blue(mc_blue[q]);

					if (x == frame_begin) {
						robots[q].set_vector();
						robots[q].update_pos(err_thres*zoom_multiplier);
					}
					else {
						robots[q].update_pos(err_thres*zoom_multiplier);
					}
					//circle(frame, robots[q].pos_r, 4, color_red, -1, 8, 0);
					//circle(frame, robots[q].pos_b, 4, color_blue, -1, 8, 0);
					//circle(frame, robots[q].pos, 4, color, -1, 8, 0);
					//robots[q].print_data();
					if (x != frame_begin) {
						imshow(windowNameOutput, frame);
					}
				}
			}

			if (ArduinoControl == true) {
				if (ManualControl == true) {
					if (input_key != '0') {
						if (RecordSelectImage == true) { //This is for recording images in phase sequence
							write_image_num = control.manual_control_phase(input_key, quad_select);
							if (write_image_num % 100 != 99) {
								write_image = 1;
							}
						}
						else {
							control.manual_control(input_key, quad_select);
						}
					}
					if (input_key == 'q') {
						return last_frame;
					}
					input_key = '0';
				}
				else {
					if (input_key == 'q') {
						cout << last_frame;
						return last_frame;
					}


					if (count_act >= count_act_max) {
						if (FeedbackControl == true) {
							//Need to use error of robot to update goal index
							vector<Point2f> err_rec;
							vector<int> zones_rec;
							vector<int> zones_rec_x;
							vector<int> zones_rec_y;
							vector<int> zones_rec_base_y;

							for (int n = 0; n < num_robots; n++) {
								err_rec.push_back(robots[n].error_pos);
								zones_rec.push_back(robots[n].zone);
								zones_rec_base_y.push_back(robots[n].zone_y_base);
								zones_rec_x.push_back(robots[n].zone_x);
								zones_rec_y.push_back(robots[n].zone_y);
							}

							//control.auto_control(err_rec, zones_rec, num_robots);
							control.auto_control_cross(err_rec, zones_rec_x, zones_rec_y, zones_rec, zones_rec_base_y, num_robots);

							for (int v = 0; v < num_robots; v++) {
								sum_goals += robots[v].end_goal;
								sum_waypoints += robots[v].mid_goal; //mid_goal is a integer that keeps track of current waypoint
							}
							if (sum_waypoints == num_robots) { //To make sure all robots reach their waypoints before moving to next
								for (int v = 0; v < num_robots; v++) {
									robots[v].reached_waypoint();
								}
							}
							if (sum_goals >= num_robots) {
								cout << last_frame;
								return last_frame;
							}
							sum_goals = 0;
							sum_waypoints = 0;

						}
						count_act = 0;
					}

				}
			}

			//imshow(windowNameOutput, frame);
			for (int u = 0; u < num_robots; u++) {
				write_file(robots[u], u, x);
			}

			if (writeFrame == true) {

				if (cropFrame == true) {
					frame = cur_frame(crop_rect);
				}
				else {
					frame = cur_frame;
				}
				if (RecordSelectImage == false) {
					write_frame(x);
				}
				else
				{
					//if (write_image == 1) {
					write_frame(write_image_num);
					write_image = 0;
					//}
				}
			}
			//cout << "Frame No: " << x << endl;

			//input_key = waitKey(pause_time);


			input_key = waitKey(act_time);
			count_act++;

			last_frame = x + 1;
			//waitKey(0);
		}
		cout << last_frame;
		return last_frame;
	}

	/*
	int run(int pause_time, float zoom_multiplier)
	{

		//microRobot robots[10];
		cv::setMouseCallback(windowNameOutput, CallBackFunc, &pt);

		Rect2f temp_frame;
		temp_frame.width = 50 * zoom_multiplier;
		temp_frame.height = 50 * zoom_multiplier;


		controller control;

		set_file(); //Set up data file for output

		if (ArduinoControl == true) {
			control.set_arduino(arduino_add, zoom_multiplier);
		}
		else {
			control.set_arduino();
		}


		//cv::Point pt(-1, -1);
		cv::Point2f pt_float(-1, -1);

		char input_key = '0';

		for (int x = frame_begin; x <= frame_end; x++) {

			cur_frame_num = x;
			if (RecordedImages == true) {
				readFrame(x);

			}
			else if (CameraImages == true) { //Camera Capture option
				error = camera.RetrieveBuffer(&rawImage);
				if (error != PGRERROR_OK)
				{
					std::cout << "capture error" << std::endl;
					continue;
				}

				rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

				// convert to OpenCV Mat
				unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
				//cv::Mat cur_frame = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
				cur_frame = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
				//frame_temp = cur_frame;
			}

			if (x == frame_begin) {
				if (cropFrame == true) {
					frame = cur_frame(crop_rect);
				}
				else {
					frame = cur_frame;
				}
				imshow(windowNameOutput, frame);
				cout << "Click midpoint of workspace (Motion X) and Press 'a'" << endl;
				get_point(pt_float);
				mid_platform_x = pt_float;
				control.mid_point_x = mid_platform_x;

				cout << "Click midpoint of workspace (Motion Y) and Press 'a'" << endl;
				get_point(pt_float);
				mid_platform_y = pt_float;
				control.mid_point_y = mid_platform_y;
				//Get initial positions of robots
				for (int k = 0; k < num_robots; k++) {

					robots[k].set_midpoint(mid_platform_x.x, mid_platform_x.y, mid_platform_y.x, mid_platform_y.y);

					cout << "Click position of Robot " << k + 1 << "and Press 'a'" << endl;
					if (ManualControl == true) {
						robots[k].set_id(k, true);
					}
					else
					{
						robots[k].set_id(k, false);
					}
					get_point(pt_float);
					robots[k].initialize_pos(pt_float);
					pt.x = -1;
					pt.y = -1;
					cout << robots[k].pos << endl;
				}
				int ind = 0;
				if (num_goals == 0) {
					PresetGoals = true;
					p_goals();
				}
				if (FeedbackControl == true) {
					for (int k = 0; k < num_robots; k++) {
						//circle(frame, robots[k].pos_b, 4, color_blue, -1, 8, 0);
						//circle(frame, robots[k].pos, 4, color, -1, 8, 0);

						if (PresetGoals == false) {
							pt_float = { (-1, -1) };
							cout << "Click goal locations of Robot " << k + 1 << endl;
							robots[k].num_goals = num_goals;
							for (int j = 0; j < num_goals; j++) {
								get_point(pt_float);
								robots[k].set_goal(pt_float);
								pt.x = -1;
								pt.y = -1;
								cout << "Goal points for Robot " << k + 1 << ":" << robots[k].goals << endl;
							}
						}
						else {

							for (int j = 0; j < num_goals; j++) {
								switch (robots[k].id) {
								case 0: {
									robots[k].set_goal(goal_p_0[j]);
									break;
								}
								case 1: {
									robots[k].set_goal(goal_p_1[j]);
									break;
								}
								case 2: {
									robots[k].set_goal(goal_p_2[j]);
									break;
								}
								case 3: {
									robots[k].set_goal(goal_p_3[j]);
									break;
								}
								}

							}
						}
					}
				}
			}
			else {
				if (cropFrame == true) {
					frame = cur_frame(crop_rect);
				}
				else {
					frame = cur_frame;
				}
				for (int q = 0; q < num_robots; q++) {
					//circle(frame, robots[q].goals[robots[q].goal_ind], 4, color_red, -1, 8, 0);
				}
				if (ManualControl == true) {
					imshow(windowNameOutput, frame);
				}
			}

			//imshow(windowNameOutput, cur_frame);
			if (ManualControl == false) {
				vector<Point2f> mc_red(num_robots);
				vector<Point2f> mc_blue(num_robots);

				vector<Rect2f> robot_crop_frames;

				for (int s = 0; s < num_robots; s++) {
					//temp_frame.width = 50 * zoom_multiplier;
					//temp_frame.height = 50 * zoom_multiplier;
					//temp_frame.x = robots[s].pos.x;
					//temp_frame.y = robots[s].pos.y;
					for (int s = 0; s < num_robots; s++) {
						temp_frame.x = robots[s].pos.x - temp_frame.width / 2;;
						temp_frame.y = robots[s].pos.y - temp_frame.height / 2;

						if (robots[s].pos.x - temp_frame.width / 2 <= 0)
							temp_frame.x = 0;
						if (1279 - robots[s].pos.x - temp_frame.width / 2 <= 0)
							temp_frame.x = 1279 - temp_frame.width;

						if (robots[s].pos.y - temp_frame.height / 2 <= 0)
							temp_frame.y = 0;
						if (1023 - robots[s].pos.y - temp_frame.height / 2 <= 0)
							temp_frame.y = 1023 - temp_frame.height;

						robot_crop_frames.push_back(temp_frame);
					}
				}
				//This will get mc_red and mc_blue positions for all robots using contours
				//detect_color('r', num_robots, frame, mc_red);
				//detect_color('b', num_robots, frame, mc_blue);


				detect_color_crop('r', num_robots, frame, mc_red, robot_crop_frames);
				detect_color_crop('b', num_robots, frame, mc_blue, robot_crop_frames);

				for (int q = 0; q < num_robots; q++) {
					//This is for contours
					//robots[q].set_red(mc_red[find_robot_ind(mc_red, num_robots, q, robots[q].pos)]);
					//robots[q].set_blue(mc_blue[find_robot_ind(mc_blue, num_robots, q, robots[q].pos)]);

					robots[q].set_red(mc_red[q]);
					robots[q].set_blue(mc_blue[q]);

					if (x == frame_begin) {
						robots[q].set_vector();
						robots[q].update_pos(err_thres*zoom_multiplier);
					}
					else {
						robots[q].update_pos(err_thres*zoom_multiplier);
					}
					//circle(frame, robots[q].pos_r, 4, color_red, -1, 8, 0);
					//circle(frame, robots[q].pos_b, 4, color_blue, -1, 8, 0);
					//circle(frame, robots[q].pos, 4, color, -1, 8, 0);
					//robots[q].print_data();
					if (x != frame_begin) {
						imshow(windowNameOutput, frame);
					}
				}
			}

			if (ArduinoControl == true) {
				if (ManualControl == true) {
					if (input_key != '0') {
						if (RecordSelectImage == true) {
							write_image_num = control.manual_control_phase(input_key, quad_select);
							if (write_image_num % 100 != 99) {
								write_image = 1;
							}
						}
						else {
							control.manual_control(input_key, quad_select);
						}
					}
					if (input_key == 'q') {
						return 0;
					}
					input_key = '0';
				}
				else {
					if (input_key == 'q') {
						return 0;
					}
					if (FeedbackControl == true) {
						//Need to use error of robot to update goal index
						vector<Point2f> err_rec;
						vector<int> zones_rec;
						vector<int> zones_rec_x;
						vector<int> zones_rec_y;
						vector<int> zones_rec_base_y;
						if (FeedbackControl_Orientation == false) {
							for (int n = 0; n < num_robots; n++) {
								err_rec.push_back(robots[n].error_pos);
								zones_rec.push_back(robots[n].zone);
								//zones_rec_x.push_back(robots[n].zone_x);
								//zones_rec_y.push_back(robots[n].zone_y);
							}

							control.auto_control(err_rec, zones_rec, num_robots);
							//control.auto_control_cross(err_rec, zones_rec_x, zones_rec_y , num_robots);

							for (int v = 0; v < num_robots; v++) {
								sum_goals += robots[v].end_goal;
								sum_waypoints += robots[v].mid_goal; //mid_goal is a integer that keeps track of current waypoint
							}
							if (sum_waypoints == num_robots) { //To make sure all robots reach their waypoints before moving to next
								for (int v = 0; v < num_robots; v++) {
									robots[v].reached_waypoint();
								}
							}
							if (sum_goals >= num_robots) {
								return 0;
							}
							sum_goals = 0;
							sum_waypoints = 0;
						}
						else {
							for (int n = 0; n < num_robots; n++) {
								err_rec.push_back(robots[n].error_pos);
								zones_rec.push_back(robots[n].zone);
								zones_rec_base_y.push_back(robots[n].zone_y_base);
								zones_rec_x.push_back(robots[n].zone_x);
								zones_rec_y.push_back(robots[n].zone_y);
							}

							//control.auto_control(err_rec, zones_rec, num_robots);
							control.auto_control_cross(err_rec, zones_rec_x, zones_rec_y, zones_rec, zones_rec_base_y, num_robots);

							for (int v = 0; v < num_robots; v++) {
								sum_goals += robots[v].end_goal;
								sum_waypoints += robots[v].mid_goal; //mid_goal is a integer that keeps track of current waypoint
							}
							if (sum_waypoints == num_robots) { //To make sure all robots reach their waypoints before moving to next
								for (int v = 0; v < num_robots; v++) {
									robots[v].reached_waypoint();
								}
							}
							if (sum_goals >= num_robots) {
								return 0;
							}
							sum_goals = 0;
							sum_waypoints = 0;
						}
					}
				}
			}
			//imshow(windowNameOutput, frame);
			for (int u = 0; u < num_robots; u++) {
				write_file(robots[u], u, x);
			}

			if (writeFrame == true) {

				if (cropFrame == true) {
					frame = cur_frame(crop_rect);
				}
				else {
					frame = cur_frame;
				}
				if (RecordSelectImage == false) {
					write_frame(x);
				}
				else
				{
					if (write_image == 1) {
						write_frame(write_image_num);
						write_image = 0;
					}
				}
			}
			//cout << "Frame No: " << x << endl;

			input_key = waitKey(pause_time);

			//waitKey(0);
		}
		return 0;
	}
	*/
	void set_file() {

		outdata.open("results.dat"); // opens the file
		if (!outdata) { // file couldn't be opened
			cerr << "Error: file could not be opened" << endl;
			WriteFail = true;
		}
		else
		{
			WriteFail = false;
			for (int u = 0; u < num_robots; u++) {
				outdata << "TimeStamp\t" << "Fig_No\t" << "robotID\t" << "robotPosX\t" << "robotPosY\t" << "robotPos_RX\t" << "robotPos_RY\t" << "robotPos_BX\t" << "robotPos_BY\t" << "robotErrorX\t" << "robotErrorY\t" << "Zone\t" << "GoalIndex\t" << "PhaseX\t" << "PhaseY\t";
			}

			outdata << endl;
		}
	}
	void write_file(microRobot r, int robot_ind, int fig_num) {
		std::chrono::duration<double, std::milli> current_time_d = current_time-start_time;
		if (robot_ind == 0) {
			outdata << current_time_d.count() << "\t" << fig_num << "\t";
		}
		if (WriteFail == false) {
			outdata << r.id << "\t" << r.pos.x << "\t" << r.pos.y << "\t" << r.pos_r.x << "\t" << r.pos_r.y << "\t" << r.pos_b.x << "\t" << r.pos_b.y << "\t" << r.error_pos.x << "\t" << r.error_pos.y << "\t" << r.zone << "\t" << r.goal_ind << "\t" << control.phase_x_int[r.zone] << "\t" << control.phase_y_int[r.zone] << "\t";
			if (robot_ind == num_robots - 1) {
				outdata << endl;
			}
		}
	}
};

