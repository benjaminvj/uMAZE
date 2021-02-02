#include "frameProcessor.h"
#include "params_experiment.h"
#include <iostream>
#include <fstream>
#include <chrono> 
using namespace std;
using namespace std::chrono;

int main()
{
	int first_time = 0;


	int select_mode = 1;
	char feedback_key = '0';
	frameProcessor processor;
	//Set up output window
	processor.displayOutput("Video Capture");

	int pause_time = 100; //pause between frames, set lower for higher speeds of motion
	int act_time = 50;
	int number_of_robots = 1;
	int number_of_goals = 4; //Set 0 goals to run PUWL
	float zoom_multiplier = 4;
	int quad_select = 1; //selected quadrant for control	

	int frame_num = 0;
	int choice_mode = 0;
	
	while (select_mode != 7) {
		processor.reset();

		//Reset options for manual and auto
		cout << "0 for 'auto'" << endl;
		cout << "1 for 'manual'" << endl;
		cout << "2 for changing pause time" << endl;
		cout << "3 for 'running PUWL motion'" << endl;
		cout << "4 for 'crossing zones'" << endl;
		cout << "5 for 'manual recording of zone'" << endl;
		//cout << "6 for continuous auto with robot definitions" << endl;
		cout << "7 for quit" << endl;
		cout << "Specify Mode:";
		cin >> select_mode;
		//Specify for modes:
		auto start = high_resolution_clock::now();


		switch (select_mode) {
		case 0: { //auto mode
			
			//Set process as frames from the computer
			processor.setFrameProcess("D:/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/LIVE_RESULTS_CONT/frame_"); //For pointgrey flycap
			//If image needs to be cropped, specify here
			//processor.crop_frame(217, 116, 768, 747);
			//Set up arduino for auto Control
			processor.set_arduino("\\\\.\\COM8", false, number_of_goals);

			cout << "Enter number of robots to control: ";
			cin >> number_of_robots;
			cout << "Enter number of goals for each robot: ";
			cin >> number_of_goals;

			//Set number of robots
			processor.set_numRobots(number_of_robots);

			processor.run_setup(zoom_multiplier, number_of_robots, number_of_goals, frame_num);
			frame_num = processor.run_continous(pause_time, act_time, zoom_multiplier, frame_num);
			cout << "frame_num: " << frame_num << endl;
			break; }
		case 1: { //manual mode
			cout << "Enter quadrant to control (0-3):";
			cin >> quad_select;
			//processor.setFrameProcess(fig_path, 275, 350);
			processor.setFrameProcess("D:/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/LIVE_RESULTS_CONT/frame_"); //For pointgrey flycap
			processor.set_arduino("\\\\.\\COM8", true, quad_select);
			//processor.run(pause_time, zoom_multiplier);
			processor.set_numRobots(number_of_robots);
			processor.run_setup(zoom_multiplier, number_of_robots, number_of_goals, frame_num);
			frame_num = processor.run_continous(pause_time, act_time, zoom_multiplier, frame_num);
			break; }
		case 2: { 
			//Set number of robots
			
			//Set process as frames from the computer
			processor.setFrameProcess("D:/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/LIVE_RESULTS_CONT/frame_"); //For pointgrey flycap
			//If image needs to be cropped, specify here
			//processor.crop_frame(217, 116, 768, 747);
			//Set up arduino for auto Control
			processor.set_arduino("\\\\.\\COM8", false, number_of_goals);
			processor.set_numRobots(number_of_robots);

			cout << "Enter number of robots to control: ";
			cin >> number_of_robots;
			cout << "Enter number of goals for each robot: ";
			cin >> number_of_goals;
			cout << "Enter pause time";
			cin >> pause_time;
			processor.run_setup(zoom_multiplier, number_of_robots, number_of_goals, frame_num);
			frame_num = processor.run_continous(pause_time, act_time, zoom_multiplier, frame_num);
			cout << "frame_num: " << frame_num << endl; 
			break;
			//cout << "Option Disabled" << endl;
			/*
			//manual mode with existing images
			//	const String window_capture_name = "Video Capture";
			const String fig_path = "D:/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/object_tracking_example/Samples/frame_";
			frameProcessor processor;
			//Set number of robots
			processor.set_numRobots(number_of_robots);
			//Set up output window
			processor.displayOutput("Video Capture");
			//Set process as frames from the computer
			processor.setFrameProcess(fig_path, 275, 350);
			//If image needs to be cropped, specify here
			//processor.crop_frame(217, 116, 768, 747);
			//Set up arduino for manual control or comment it out for no arduino - and last number is quadrant to control
			processor.set_arduino("\\\\.\\COM8", true, 1);
			//processor.run(pause_time, zoom_multiplier);
			processor.run_setup(zoom_multiplier, number_of_robots, number_of_goals, frame_num);
			frame_num = processor.run_continous(pause_time, zoom_multiplier, frame_num);
			break;
			*/}
		case 3: { //PUWL Option
			//Set number of robots
			processor.set_numRobots(number_of_robots);
			//Set process as frames from the computer
			processor.setFrameProcess("D:/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/LIVE_RESULTS/frame_"); //For pointgrey flycap
			//If image needs to be cropped, specify here
			//processor.crop_frame(217, 116, 768, 747);
			//Set up arduino for auto Control
			processor.set_arduino("\\\\.\\COM8", false, 0);
			//processor.run(pause_time, zoom_multiplier);
			processor.run_setup(zoom_multiplier, number_of_robots, number_of_goals, frame_num);
			frame_num = processor.run_continous(pause_time, act_time, zoom_multiplier, frame_num);
			break; }
		case 4: { //auto_with Orientation 
			//Set number of robots
			processor.set_numRobots(number_of_robots);
			//Set process as frames from the computer
			processor.setFrameProcess("D:/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/LIVE_RESULTS/frame_"); //For pointgrey flycap
			//If image needs to be cropped, specify here
			//processor.crop_frame(217, 116, 768, 747);
			//Set up arduino for auto Control
			processor.set_arduino("\\\\.\\COM8", false, number_of_goals);
			processor.FeedbackControl_Orientation = true;
			//processor.run(pause_time, zoom_multiplier);
			processor.run_setup(zoom_multiplier, number_of_robots, number_of_goals, frame_num);
			frame_num = processor.run_continous(pause_time, act_time, zoom_multiplier, frame_num);

			break; }
		case 5: {//Manual Mode with Select image
			cout << "Enter quadrant to control (0-3):";
			cin >> quad_select;
			//Set number of robots
			processor.set_numRobots(number_of_robots);
			//processor.setFrameProcess(fig_path, 275, 350);
			processor.setFrameProcess("D:/OneDrive - purdue.edu/Research/Serpentine Coil/Benji/Visual/repos/LIVE_RESULTS/frame_"); //For pointgrey flycap
			processor.RecordSelectImage = true;
			//If image needs to be cropped, specify here
			//processor.crop_frame(217, 116, 768, 747);
			//Set up arduino for manual control or comment it out for no arduino - and last number is quadrant to control
			processor.set_arduino("\\\\.\\COM8", true, quad_select);
			//processor.run(pause_time, zoom_multiplier);
			processor.run_setup(zoom_multiplier, number_of_robots, number_of_goals, frame_num);
			frame_num = processor.run_continous(pause_time, act_time, zoom_multiplier, frame_num);

			break; }
		case 7: {
			return 0;
		}
		default: break;
		}
		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop - start);
		cout << "Time taken by function: " << duration.count() << " microseconds" << endl;
		cout << "Process has ended." << endl;
		//reset processor;
		
		//feedback_key = waitKey(0);
	}

	//cout << "Process has ended! Click any button to close";
	//waitKey(0);
	//return 0;
}

