#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;

class params_experiment {
public:
	cv::String window_name;
	cv::String file_path_base;
	int num_robots;
	int start_frame;
	int end_frame;
	cv::Rect2i crop_frame_val;
	String arduino_add;
	int pause_time;

	params_experiment() {
		window_name = "Video Output";
		num_robots = 2;
		file_path_base = "C:/Users/john1360/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/object_tracking_example/Samples/frame_";
		start_frame = 275;
		end_frame = 300;
		crop_frame_val.x = 315;
		crop_frame_val.y = 300;
		crop_frame_val.width = 300;
		crop_frame_val.height = 300;
		arduino_add = "\\\\.\\COM8";
		pause_time = 100;
	}

	params_experiment(String filename) {
		// Object to read from file 
		ifstream file_obj;

		// Opening file in input mode 
		file_obj.open(filename, ios::in);

		// Reading from file into object "obj" 
		file_obj.read((char*)this, sizeof(this));

	}

	void display_params() {
		cout << "Window Name: " << this->window_name;
		cout << "Number of Robots: " << this->num_robots << endl;
		cout << "File Path: " << this->file_path_base << endl;
		cout << "Start #: " << this->start_frame << endl;
		cout << "End #: " << this->end_frame << endl;
		cout << "Crop Values: " << this->crop_frame_val << endl;
		cout << "Arduino Number: " << this->arduino_add << endl;
		cout << "Pause Time: " << this->pause_time << endl;
	}

	void write_params(String filename_write) {

	}
	/*
	~params_experiment() {
		ofstream file_obj;
		file_obj.open("Settings2.txt", ios::app);
		file_obj.write((char*)this, sizeof(this));
	}
	*/
};