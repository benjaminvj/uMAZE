#include "SerialClass.h"
#include <iostream>
#include <string>
# define pi_def           3.14159265358979323846  /* pi */

class controller {



public:
	//Arduino Variables
	Serial* port;
	char data[4] = "";
	char command[2] = "";
	int datalength = 4;  //length of the data,
	int readResult = 0;
	char key_char[13] = "1,2000,3,4,5";
	//const char digits_def[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };


	cv::Point2f mid_point_x;
	cv::Point2f mid_point_y;

	int quadrants[4] = { 1,2,3,4 };
	int move_x[4] = { 0,0,0,0 };
	int move_y[4] = { 0,0,0,0 };
	int res[4][4] = { {1,1,-1,-1},{ 1,1,-1,-1 },{ 1,1,-1,-1 },{ 1,1,-1,-1 } }; //resolution of dir movement for each (dir,zone)
	int On_time = 20;
	int select_quad = 3;
	int send_arduino = 0;
	char send_to_arduino[19];
	char send_ind[1];
	char currents_all[16];

	const int phase_max = 16;
	double phaseStep = 2 * pi_def / phase_max;

	double phase_x[4] = { 0,0,0,0 };//Phase of each quad
	double phase_y[4] = { 0,0,0,0 };
	int phase_x_int[4] = { 0,0,0,0 };
	int phase_y_int[4] = { 0,0,0,0 };
	int free_x[4] = { 0,0,0,0 };
	int free_y[4] = { 0,0,0,0 };

	int coil_index[4][4] = { {15,13,3,1},{11,9,2,0},{10,8,6,4},{14,12,7,5} };
	int coils_x[4][2] = { {15,13},{11,9},{10,8},{14,12} };
	int coils_y[4][2] = { {3,1},{2,0},{6,4},{7,5} };

	//int zoom_multiplier = 1;
	float err_thres = 4;

	void set_thres(int zoom_multiplier) {
		err_thres = err_thres * zoom_multiplier;
	}

	void set_currents() {
		for (int quad = 0; quad < 4; quad++) {
			switch (phase_x_int[quad]) {
			case 0: {
				currents_all[coils_x[quad][0]] = '0';
				currents_all[coils_x[quad][1]] = '0';
				break;
			}
			case 1: {
				currents_all[coils_x[quad][0]] = '1';
				currents_all[coils_x[quad][1]] = '1';
				break;
			}
			case 2: {
				currents_all[coils_x[quad][0]] = '2';
				currents_all[coils_x[quad][1]] = '2';
				break;
			}
			case 3: {
				currents_all[coils_x[quad][0]] = '3';
				currents_all[coils_x[quad][1]] = '3';
				break;
			}
			case 4: {
				currents_all[coils_x[quad][0]] = '4';
				currents_all[coils_x[quad][1]] = '4';
				break;
			}
			case 5: {
				currents_all[coils_x[quad][0]] = '5';
				currents_all[coils_x[quad][1]] = '5';
				break;
			}
			case 6: {
				currents_all[coils_x[quad][0]] = '6';
				currents_all[coils_x[quad][1]] = '6';
				break;
			}
			case 7: {
				currents_all[coils_x[quad][0]] = '7';
				currents_all[coils_x[quad][1]] = '7';
				break;
			}
			case 8: {
				currents_all[coils_x[quad][0]] = '8';
				currents_all[coils_x[quad][1]] = '8';
				break;
			}
			case 9: {
				currents_all[coils_x[quad][0]] = '9';
				currents_all[coils_x[quad][1]] = '9';
				break;
			}
			case 10: {
				currents_all[coils_x[quad][0]] = 'A';
				currents_all[coils_x[quad][1]] = 'A';
				break;
			}
			case 11: {
				currents_all[coils_x[quad][0]] = 'B';
				currents_all[coils_x[quad][1]] = 'B';
				break;
			}
			case 12: {
				currents_all[coils_x[quad][0]] = 'C';
				currents_all[coils_x[quad][1]] = 'C';
				break;
			}
			case 13: {
				currents_all[coils_x[quad][0]] = 'D';
				currents_all[coils_x[quad][1]] = 'D';
				break;
			}
			case 14: {
				currents_all[coils_x[quad][0]] = 'E';
				currents_all[coils_x[quad][1]] = 'E';
				break;
			}
			case 15: {
				currents_all[coils_x[quad][0]] = 'F';
				currents_all[coils_x[quad][1]] = 'F';
				break;
			}
			}
			switch (phase_y_int[quad]) {
			case 0: {
				currents_all[coils_y[quad][0]] = '0';
				currents_all[coils_y[quad][1]] = '0';
				break;
			}
			case 1: {
				currents_all[coils_y[quad][0]] = '1';
				currents_all[coils_y[quad][1]] = '1';
				break;
			}
			case 2: {
				currents_all[coils_y[quad][0]] = '2';
				currents_all[coils_y[quad][1]] = '2';
				break;
			}
			case 3: {
				currents_all[coils_y[quad][0]] = '3';
				currents_all[coils_y[quad][1]] = '3';
				break;
			}
			case 4: {
				currents_all[coils_y[quad][0]] = '4';
				currents_all[coils_y[quad][1]] = '4';
				break;
			}
			case 5: {
				currents_all[coils_y[quad][0]] = '5';
				currents_all[coils_y[quad][1]] = '5';
				break;
			}
			case 6: {
				currents_all[coils_y[quad][0]] = '6';
				currents_all[coils_y[quad][1]] = '6';
				break;
			}
			case 7: {
				currents_all[coils_y[quad][0]] = '7';
				currents_all[coils_y[quad][1]] = '7';
				break;
			}
			case 8: {
				currents_all[coils_y[quad][0]] = '8';
				currents_all[coils_y[quad][1]] = '8';
				break;
			}
			case 9: {
				currents_all[coils_y[quad][0]] = '9';
				currents_all[coils_y[quad][1]] = '9';
				break;
			}
			case 10: {
				currents_all[coils_y[quad][0]] = 'A';
				currents_all[coils_y[quad][1]] = 'A';
				break;
			}
			case 11: {
				currents_all[coils_y[quad][0]] = 'B';
				currents_all[coils_y[quad][1]] = 'B';
				break;
			}
			case 12: {
				currents_all[coils_y[quad][0]] = 'C';
				currents_all[coils_y[quad][1]] = 'C';
				break;
			}
			case 13: {
				currents_all[coils_y[quad][0]] = 'D';
				currents_all[coils_y[quad][1]] = 'D';
				break;
			}
			case 14: {
				currents_all[coils_y[quad][0]] = 'E';
				currents_all[coils_y[quad][1]] = 'E';
				break;
			}
			case 15: {
				currents_all[coils_y[quad][0]] = 'F';
				currents_all[coils_y[quad][1]] = 'F';
				break;
			}
			}
			//current override
			switch (free_y[quad]) {
			case 1: {
				currents_all[coils_y[quad][0]] = '0' + phase_max/4;// '4';
				currents_all[coils_y[quad][1]] = '0';
				break; }
			}
			switch (free_x[quad]) {
			case 1: {
				currents_all[coils_x[quad][0]] = '0' + phase_max / 4;// '4';
				currents_all[coils_x[quad][1]] = '0';
				break; }
			}

		}

	}

	void set_arduino(const char *arduino_addr, float zoom_num) {
		port = new Serial(arduino_addr);
		if (port->IsConnected()) std::cout << "Connected!" << std::endl;
		cout << "Wait for 2 seconds" << endl;
		//ArduinoControl = true;
		//ManualControl = true;
		set_thres(zoom_num);
		cout << "Error Threshold set to " << err_thres << " with zoom multiplier " << zoom_num << endl;
		Sleep(1000);
		set_currents();
		Sleep(1000);
		sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
		Sleep(20);
		cout << send_to_arduino << endl;
	}

	void set_arduino() {
		cout << "Arduino Not Connected" << endl;
	}

	void manual_control(char key, int select_quad_ip) {
		select_quad = select_quad_ip;
		reset_free();
		switch (key) {
		case 'w': {
			move_y[select_quad] = 1 * res[select_quad][2];
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			move_y[select_quad] = 0;
			phase_y[select_quad] = phase_y[select_quad] + phaseStep * res[select_quad][2];
			if (phase_y[select_quad] >= (2 * pi_def) || phase_y[select_quad] <= -(2 * pi_def))
			{
				phase_y[select_quad] = 0;
			}
			phase_y_int[select_quad] = phase_y_int[select_quad] + 1 * res[select_quad][2];
			if (phase_y_int[select_quad] >= phase_max)
			{
				phase_y_int[select_quad] = phase_max - phase_y_int[select_quad];
			}
			if (phase_y_int[select_quad] <= -1)
			{
				phase_y_int[select_quad] = phase_max + phase_y_int[select_quad];
			}
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "Y Phase : " << phase_y_int[select_quad];
			break; }
		case 's': {move_y[select_quad] = -1 * res[select_quad][3];
			phase_y[select_quad] = phase_y[select_quad] - phaseStep * res[select_quad][3];
			if (phase_y[select_quad] >= (2 * pi_def) || phase_y[select_quad] <= -(2 * pi_def))
			{
				phase_y[select_quad] = 0;
			}
			phase_y_int[select_quad] = phase_y_int[select_quad] - 1 * res[select_quad][3];
			if (phase_y_int[select_quad] >= phase_max)
			{
				phase_y_int[select_quad] = phase_max - phase_y_int[select_quad];
			}
			if (phase_y_int[select_quad] <= -1)
			{
				phase_y_int[select_quad] = phase_max + phase_y_int[select_quad];
			}
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "Y Phase : " << phase_y_int[select_quad];
			//cout << phase_y[select_quad] << endl;
			move_y[select_quad] = 0;
			break; }
		case 'd': {move_x[select_quad] = 1 * res[select_quad][0];
			phase_x[select_quad] = phase_x[select_quad] + phaseStep * res[select_quad][0];
			if (phase_x[select_quad] >= (2 * pi_def) || phase_x[select_quad] <= -(2 * pi_def)) {
				phase_x[select_quad] = 0;
			}
			phase_x_int[select_quad] = phase_x_int[select_quad] + 1 * res[select_quad][0];
			if (phase_x_int[select_quad] >= phase_max)
			{
				phase_x_int[select_quad] = phase_max - phase_x_int[select_quad];
			}
			if (phase_x_int[select_quad] <= -1)
			{
				phase_x_int[select_quad] = phase_max + phase_x_int[select_quad];
			}
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "X Phase : " << phase_x_int[select_quad];
			//cout << phase_x[select_quad] << endl;
			move_x[select_quad] = 0;
			break; }
		case 'a': {move_x[select_quad] = -1 * res[select_quad][1];
			phase_x[select_quad] = phase_x[select_quad] - phaseStep * res[select_quad][1];
			if (phase_x[select_quad] >= (2 * pi_def) || phase_x[select_quad] <= -(2 * pi_def))
			{
				phase_x[select_quad] = 0;
			}
			phase_x_int[select_quad] = phase_x_int[select_quad] - 1 * res[select_quad][1];
			if (phase_x_int[select_quad] >= phase_max)
			{
				phase_x_int[select_quad] = phase_max - phase_x_int[select_quad];
			}
			if (phase_x_int[select_quad] <= -1)
			{
				phase_x_int[select_quad] = phase_max + phase_x_int[select_quad];
			}
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "X Phase : " << phase_x_int[select_quad];
			move_x[select_quad] = 0;
			//cout << phase_x[select_quad] << endl;
			break; }
		case '0': {cout << "Key setup ok" << endl;
			break; }
				  //default: {cout << "No key Input" << endl;
				  //	break; };	
		};

		if (send_arduino == 1) {
			int msglen = strlen(send_to_arduino);
			if (port->WriteData(send_to_arduino, msglen));   //write to arduino	
													  //Sleep(30);
			//printf("\n(writing success)\n");
			//std::cout << key_char << std::endl;
			Sleep(On_time);
			send_arduino = 0;

		}

	}

	void arduino_char_gen() {
		//Need zone number, moving dir.
		sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
		Sleep(20);
		send_arduino = 1;
		//cout << send_to_arduino << endl;
		phase_x[select_quad] = phase_x[select_quad] - phaseStep;
		if (phase_x[select_quad] >= (2 * pi_def) || phase_x[select_quad] <= -(2 * pi_def))
		{
			phase_x[select_quad] = 0;
		}
		move_x[select_quad] = 0;
		//cout << phase_x[select_quad] << endl;
	}


	void auto_control(vector<cv::Point2f> errors, vector<int> zones, int num_robots) {

		for (int k = 0; k < num_robots; k++) {
			//cout << zones[k] << " zone for " << k + 1 << "robot" << endl;

			if (abs(errors[k].x) > abs(errors[k].y)) {
				if ((errors[k].x > err_thres) && (errors[k].x > 0)) {
					move_x[zones[k]] = -1 * res[zones[k]][0];
					//mov_x[zones[k]] = move_x[zones[k]] + 1;
					phase_x[zones[k]] = phase_x[zones[k]] + phaseStep;
					phase_x_int[zones[k]] = phase_x_int[zones[k]] - 1 * res[zones[k]][0];
					send_arduino = 1;
				}
				else {
					if ((errors[k].x < -err_thres) && (errors[k].x < 0)) {
						move_x[zones[k]] = 1 * res[zones[k]][1];
						//mov_x[zones[k]] = move_x[zones[k]] + 1;
						phase_x[zones[k]] = phase_x[zones[k]] - phaseStep;
						phase_x_int[zones[k]] = phase_x_int[zones[k]] + 1 * res[zones[k]][1];
						send_arduino = 1;
					}
				}
			}
			else {
				if ((errors[k].y < -err_thres) && (errors[k].y < 0)) {
					move_y[zones[k]] = -1 * res[zones[k]][2];
					//mov_y[zones[k]] = move_y[zones[k]] + 1;
					phase_y[zones[k]] = phase_y[zones[k]] + phaseStep;
					phase_y_int[zones[k]] = phase_y_int[zones[k]] - 1 * res[zones[k]][2];
					send_arduino = 1;
				}
				else {
					if ((errors[k].y > err_thres) && (errors[k].y > 0)) {
						move_y[zones[k]] = 1 * res[zones[k]][3];
						//mov_y[zones[k]] = move_y[zones[k]] + 1;
						phase_y[zones[k]] = phase_y[zones[k]] - phaseStep;
						phase_y_int[zones[k]] = phase_y_int[zones[k]] + 1 * res[zones[k]][3];
						send_arduino = 1;
					}
				}
			}
			if (phase_x_int[zones[k]] >= phase_max)
			{
				phase_x_int[zones[k]] = phase_max - phase_x_int[zones[k]];
			}
			if (phase_x_int[zones[k]] <= -1)
			{
				phase_x_int[zones[k]] = phase_max + phase_x_int[zones[k]];
			}
			if (phase_y_int[zones[k]] >= phase_max)
			{
				phase_y_int[zones[k]] = phase_max - phase_y_int[zones[k]];
			}
			if (phase_y_int[zones[k]] <= -1)
			{
				phase_y_int[zones[k]] = phase_max + phase_y_int[zones[k]];
			}
			if (phase_x[zones[k]] >= (2 * pi_def) || phase_x[zones[k]] <= -(2 * pi_def))
			{
				phase_x[zones[k]] = 0;
			}
		}

		set_currents();
		sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
		Sleep(20);
		cout << send_to_arduino << endl;

		if (send_arduino == 1) {

			//int msglen = strlen(send_to_arduino);
			//if (port->WriteData(send_to_arduino, msglen));   //write to arduino
													  //Sleep(30);
			int msglen = strlen(send_to_arduino);
			if (port->WriteData(send_to_arduino, msglen));   //write to arduino

			//printf("\n(writing success)\n");
			//std::cout << key_char << std::endl;
			Sleep(On_time);
			send_arduino = 0;
			for (int c = 0; c < 4; c++) {
				move_x[c] = 0;
				move_y[c] = 0;
			}

		}

	}

	void adjust_phase(int quad, int dir) {
		//dir 0 1 2 3 (+x, -x, +y, -y)
		switch (dir) {
		case 0: {	move_x[quad] = -1 * res[quad][0];
			phase_x[quad] = phase_x[quad] + phaseStep;
			phase_x_int[quad] = phase_x_int[quad] - 1 * res[quad][0];
			break;
		}
		case 1: {move_x[quad] = 1 * res[quad][1];
			phase_x[quad] = phase_x[quad] - phaseStep;
			phase_x_int[quad] = phase_x_int[quad] + 1 * res[quad][1];
			send_arduino = 1;
			break;
		}
		case 2: {	move_y[quad] = -1 * res[quad][2];
			phase_y[quad] = phase_y[quad] + phaseStep;
			phase_y_int[quad] = phase_y_int[quad] - 1 * res[quad][2];
			send_arduino = 1;
			break;
		}
		case 3: {move_y[quad] = 1 * res[quad][3];
			phase_y[quad] = phase_y[quad] - phaseStep;
			phase_y_int[quad] = phase_y_int[quad] + 1 * res[quad][3];
			send_arduino = 1;
			break;
		}
		}
		//cout << "Quad: " << quad << " phase_y_int: " << phase_y_int[quad] << "Dir:" << dir << endl;
		//cout << "Quad: " << quad << " phase_x_int: " << phase_x_int[quad] << "Dir:" << dir << endl;
	}

	void check_phase() {
		for (int p = 0; p < 4; p++) {
			if (phase_x_int[p] >= phase_max)
			{
				phase_x_int[p] = phase_max - phase_x_int[p];
			}
			if (phase_x_int[p] <= -1)
			{
				phase_x_int[p] = phase_max + phase_x_int[p];
			}
			if (phase_y_int[p] >= phase_max)
			{
				phase_y_int[p] = phase_max - phase_y_int[p];
			}
			if (phase_y_int[p] <= -1)
			{
				phase_y_int[p] = phase_max + phase_y_int[p];
			}
			if (phase_x[p] >= (2 * pi_def) || phase_x[p] <= -(2 * pi_def))
			{
				phase_x[p] = 0;
			}
			if (phase_y[p] >= (2 * pi_def) || phase_y[p] <= -(2 * pi_def))
			{
				phase_y[p] = 0;
			}
		}
	}
	void reset_free() {
		for (int k = 0; k < 4; k++) {
			free_x[k] = 0;
			free_y[k] = 0;
		}
	}

	void auto_control_cross(vector<cv::Point2f> errors, vector<int> zones_x, vector<int> zones_y, vector<int> zones_x_base, vector<int> zones_y_base, int num_robots) {
		//cout << "Zone Y: " << zones_y[0] << endl;
		reset_free();

		for (int k = 0; k < num_robots; k++) {
			//cout << zones[k] << " zone for " << k + 1 << "robot" << endl;
				//Motion in X
			//cout << "Zone X:" << zones_x[k] << endl;
			//cout << "Zone Y: " << zones_y[k] << endl;


			if (zones_y[k] == 7) {
				if (zones_x[k] == 0 || zones_x[k] == 1 || zones_x[k] == 5) {
					free_y[0] = 1;
					free_y[1] = 1;
				}
				if (zones_x[k] == 2 || zones_x[k] == 3 || zones_x[k] == 6) {
					free_y[2] = 1;
					free_y[3] = 1;
				}
			}

			if ((zones_x[k] != 7) && (zones_x[k] != 5) && (zones_x[k] != 6)) {
				if ((errors[k].x > err_thres) && (errors[k].x > 0)) {
					move_x[zones_x[k]] = -1 * res[zones_x[k]][0];
					//mov_x[zones[k]] = move_x[zones[k]] + 1;
					phase_x[zones_x[k]] = phase_x[zones_x[k]] + phaseStep;
					phase_x_int[zones_x[k]] = phase_x_int[zones_x[k]] - 1 * res[zones_x[k]][0];
					send_arduino = 1;
				}
				else {
					if ((errors[k].x < -err_thres) && (errors[k].x < 0)) {
						move_x[zones_x[k]] = 1 * res[zones_x[k]][1];
						//mov_x[zones[k]] = move_x[zones[k]] + 1;
						phase_x[zones_x[k]] = phase_x[zones_x[k]] - phaseStep;
						phase_x_int[zones_x[k]] = phase_x_int[zones_x[k]] + 1 * res[zones_x[k]][1];
						send_arduino = 1;
					}
				}
			}
			else {
				if (zones_x[k] == 5) {
					//cout << "Zone 5 x" << endl;
					//free_y[0] = 1;
					//free_y[1] = 1;
					if (zones_x_base[k] == 0) {
						if (phase_x_int[0] != phase_x_int[1])
							phase_x_int[1] = phase_x_int[0];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[0] != phase_y_int[1])
							phase_y_int[1] = phase_y_int[0];
					}
					else {
						if (phase_x_int[0] != phase_x_int[1])
							phase_x_int[0] = phase_x_int[1];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[0] != phase_y_int[1])
							phase_y_int[0] = phase_y_int[1];
					}
					if ((errors[k].x > err_thres) && (errors[k].x > 0)) {
						adjust_phase(0, 0);
						adjust_phase(1, 0);
						send_arduino = 1;
					}
					else if ((errors[k].x < -err_thres) && (errors[k].x < 0)) {
						adjust_phase(0, 1);
						adjust_phase(1, 1);
						send_arduino = 1;
					}
				}
				else if (zones_x[k] == 6) {
					//free_y[2] = 1;
					//free_y[3] = 1;
					if (zones_x_base[k] == 2) {
						if (phase_x_int[2] != phase_x_int[3])
							phase_x_int[3] = phase_x_int[2];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[2] != phase_y_int[3])
							phase_y_int[3] = phase_y_int[2];
					}
					else {
						if (phase_x_int[2] != phase_x_int[3])
							phase_x_int[2] = phase_x_int[3];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[2] != phase_y_int[3])
							phase_y_int[2] = phase_y_int[3];
					}
					if ((errors[k].x > err_thres) && (errors[k].x > 0)) {
						adjust_phase(2, 0);
						adjust_phase(3, 0);
						send_arduino = 1;
					}
					else if ((errors[k].x < -err_thres) && (errors[k].x < 0)) {
						adjust_phase(2, 1);
						adjust_phase(3, 1);
						send_arduino = 1;
					}
				}
			}
			/*
			if (zones_y[k] == 7) {
				if (zones_x[k] == 0 || zones_x[k] == 1) {
					free_y[0] = 1;
					free_y[1] = 1;
				}
				if (zones_x[k] == 2 || zones_x[k] == 3) {
					free_y[2] = 1;
					free_y[3] = 1;
				}
			}*/

			if (zones_x[k] == 7) {
				if (zones_y[k] == 1 || zones_y[k] == 2 || zones_y[k] == 5) {
					free_x[1] = 1;
					free_x[2] = 1;
				}
				if (zones_y[k] == 0 || zones_y[k] == 3 || zones_y[k] == 6) {
					free_x[0] = 1;
					free_x[3] = 1;
				}
			}

			//Motion in Y
			if (zones_y[k] < 5) {
				if ((errors[k].y < -err_thres) && (errors[k].y < 0)) {
					move_y[zones_y[k]] = -1 * res[zones_y[k]][2];
					//mov_y[zones[k]] = move_y[zones[k]] + 1;
					phase_y[zones_y[k]] = phase_y[zones_y[k]] + phaseStep;
					phase_y_int[zones_y[k]] = phase_y_int[zones_y[k]] - 1 * res[zones_y[k]][2];
					send_arduino = 1;
				}
				else {
					if ((errors[k].y > err_thres) && (errors[k].y > 0)) {
						move_y[zones_y[k]] = 1 * res[zones_y[k]][3];
						//mov_y[zones[k]] = move_y[zones[k]] + 1;
						phase_y[zones_y[k]] = phase_y[zones_y[k]] - phaseStep;
						phase_y_int[zones_y[k]] = phase_y_int[zones_y[k]] + 1 * res[zones_y[k]][3];
						send_arduino = 1;
					}
				}
				if (zones_x[k] == 7) {
					if (zones_y[k] == 0 || zones_y[k] == 1) {
						free_x[1] = 1;
						free_x[2] = 1;
					}
				}
			}
			else {
				if (zones_y[k] == 5) {
					//free_x[1] = 1;
					//free_x[2] = 1;
					if (zones_y_base[k] == 1) {
						if (phase_y_int[1] != phase_y_int[2])
							phase_y_int[2] = phase_y_int[1];
						if (phase_x_int[1] != phase_x_int[2])
							phase_x_int[2] = phase_x_int[1];
					}
					else {
						if (phase_y_int[1] != phase_y_int[2])
							phase_y_int[1] = phase_y_int[2];
						if (phase_x_int[1] != phase_x_int[2])
							phase_x_int[1] = phase_x_int[2];
					}
					if ((errors[k].y < -err_thres) && (errors[k].y < 0)) {
						adjust_phase(1, 2);
						adjust_phase(2, 2);
						send_arduino = 1;
					}
					else {
						if ((errors[k].y > err_thres) && (errors[k].y > 0)) {
							adjust_phase(1, 3);
							adjust_phase(2, 3);
							send_arduino = 1;
						}
					}
				}
				else if (zones_y[k] == 6) {
					free_x[0] = 1;
					free_x[3] = 1;
					//cout << "Zone 6: " <<zones_y[k] << "Zone y base: " << zones_y_base[k] << endl;
					if (zones_y_base[k] == 0) {
						if (phase_y_int[0] != phase_y_int[3])
							phase_y_int[3] = phase_y_int[0];
						if (phase_x_int[0] != phase_x_int[3])
							phase_x_int[3] = phase_x_int[0];
						//phase_x_int[0] = 0;
						//free_x[zones_x_base[k]] = 1;
					}
					else {
						if (phase_y_int[0] != phase_y_int[3])
							phase_y_int[0] = phase_y_int[3];
						if (phase_x_int[0] != phase_x_int[3])
							phase_x_int[0] = phase_x_int[3];
						//phase_x_int[3] = 0;
						//free_x[zones_x_base[k]] = 1;
					}
					if ((errors[k].y < -err_thres) && (errors[k].y < 0)) {
						adjust_phase(0, 2);
						adjust_phase(3, 2);
						send_arduino = 1;
					}
					else {
						if ((errors[k].y > err_thres) && (errors[k].y > 0)) {
							adjust_phase(0, 3);
							adjust_phase(3, 3);
							send_arduino = 1;
						}
					}
				}

			}
		}
		check_phase();
		set_currents();
		sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
		Sleep(20);
		cout << send_to_arduino << endl;

		if (send_arduino == 1) {

			//int msglen = strlen(send_to_arduino);
			//if (port->WriteData(send_to_arduino, msglen));   //write to arduino
													  //Sleep(30);
			int msglen = strlen(send_to_arduino);
			if (port->WriteData(send_to_arduino, msglen));   //write to arduino

			//printf("\n(writing success)\n");
			//std::cout << key_char << std::endl;
			Sleep(On_time);
			send_arduino = 0;
			for (int c = 0; c < 4; c++) {
				move_x[c] = 0;
				move_y[c] = 0;
			}

		}

	}

	int manual_control_phase(char key, int select_quad_ip) {
		int write_image_num = 9;
		int image_phase	 = 99;
		int image_xy_num;
		select_quad = select_quad_ip;
		switch (key) {
		case 'w': {
			move_y[select_quad] = 1 * res[select_quad][2];
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			move_y[select_quad] = 0;
			phase_y[select_quad] = phase_y[select_quad] + phaseStep * res[select_quad][2];
			if (phase_y[select_quad] >= (2 * pi_def) || phase_y[select_quad] <= -(2 * pi_def))
			{
				phase_y[select_quad] = 0;
			}
			phase_y_int[select_quad] = phase_y_int[select_quad] + 1 * res[select_quad][2];
			if (phase_y_int[select_quad] >= phase_max)
			{
				phase_y_int[select_quad] = phase_max - phase_y_int[select_quad];
			}
			if (phase_y_int[select_quad] <= -1)
			{
				phase_y_int[select_quad] = phase_max + phase_y_int[select_quad];
			}
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "Y Phase : " << phase_y_int[select_quad];
			image_phase = phase_y_int[select_quad];
			image_xy_num = 1;
			break; }
		case 's': {move_y[select_quad] = -1 * res[select_quad][3];
			phase_y[select_quad] = phase_y[select_quad] - phaseStep * res[select_quad][3];
			if (phase_y[select_quad] >= (2 * pi_def) || phase_y[select_quad] <= -(2 * pi_def))
			{
				phase_y[select_quad] = 0;
			}
			phase_y_int[select_quad] = phase_y_int[select_quad] - 1 * res[select_quad][3];
			if (phase_y_int[select_quad] >= phase_max)
			{
				phase_y_int[select_quad] = phase_max - phase_y_int[select_quad];
			}
			if (phase_y_int[select_quad] <= -1)
			{
				phase_y_int[select_quad] = phase_max + phase_y_int[select_quad];
			}
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "Y Phase : " << phase_y_int[select_quad];
			//cout << phase_y[select_quad] << endl;
			move_y[select_quad] = 0;
			image_phase = phase_y_int[select_quad];
			image_xy_num = 1;
			break; }
		case 'd': {move_x[select_quad] = 1 * res[select_quad][0];
			phase_x[select_quad] = phase_x[select_quad] + phaseStep * res[select_quad][0];
			if (phase_x[select_quad] >= (2 * pi_def) || phase_x[select_quad] <= -(2 * pi_def)) {
				phase_x[select_quad] = 0;
			}
			phase_x_int[select_quad] = phase_x_int[select_quad] + 1 * res[select_quad][0];
			if (phase_x_int[select_quad] >= phase_max)
			{
				phase_x_int[select_quad] = phase_max - phase_x_int[select_quad];
			}
			if (phase_x_int[select_quad] <= -1)
			{
				phase_x_int[select_quad] = phase_max + phase_x_int[select_quad];
			}
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "X Phase : " << phase_x_int[select_quad];
			//cout << phase_x[select_quad] << endl;
			move_x[select_quad] = 0;
			image_phase = phase_x_int[select_quad];
			image_xy_num = 0;
			break; }
		case 'a': {move_x[select_quad] = -1 * res[select_quad][1];
			phase_x[select_quad] = phase_x[select_quad] - phaseStep * res[select_quad][1];
			if (phase_x[select_quad] >= (2 * pi_def) || phase_x[select_quad] <= -(2 * pi_def))
			{
				phase_x[select_quad] = 0;
			}
			phase_x_int[select_quad] = phase_x_int[select_quad] - 1 * res[select_quad][1];
			if (phase_x_int[select_quad] >= phase_max)
			{
				phase_x_int[select_quad] = phase_max - phase_x_int[select_quad];
			}
			if (phase_x_int[select_quad] <= -1)
			{
				phase_x_int[select_quad] = phase_max + phase_x_int[select_quad];
			}
			//sprintf(send_to_arduino, "<%d%d%d%d%d%d%d%d%d%d%d%d%d>", 4, 0, move_x[0] + 1, move_y[0] + 1, 1, move_x[1] + 1, move_y[1] + 1, 2, move_x[2] + 1, move_y[2] + 1, 3, move_x[3] + 1, move_y[3] + 1);
			set_currents();
			sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
			//Sleep(20);
			send_arduino = 1;
			cout << send_to_arduino << endl;
			cout << "X Phase : " << phase_x_int[select_quad];
			move_x[select_quad] = 0;
			image_phase = phase_x_int[select_quad];
			image_xy_num = 0;
			//cout << phase_x[select_quad] << endl;
			break; }
		default: {cout << "Key setup ok" << endl;
			image_xy_num = 9;
			break;
		}
		};
		write_image_num = select_quad_ip * 1000 + image_xy_num * 100 + image_phase;

		if (send_arduino == 1) {
			int msglen = strlen(send_to_arduino);
			if (port->WriteData(send_to_arduino, msglen));   //write to arduino	
													  //Sleep(30);
			//printf("\n(writing success)\n");
			//std::cout << key_char << std::endl;
			Sleep(On_time);
			send_arduino = 0;

		}
		return write_image_num;
	}

	void auto_control_phase(vector<cv::Point2f> errors, vector<int> zones_x, vector<int> zones_y, vector<int> zones_x_base, vector<int> zones_y_base, int num_robots) {
		//cout << "Zone Y: " << zones_y[0] << endl;
		reset_free();

		for (int k = 0; k < num_robots; k++) {

			//cout << "Zone X:" << zones_x[k] << endl;
			//cout << "Zone Y: " << zones_y[k] << endl;

			if (zones_y[k] == 7) {
				if (zones_x[k] == 0 || zones_x[k] == 1 || zones_x[k] == 5) {
					free_y[0] = 1;
					free_y[1] = 1;
				}
				if (zones_x[k] == 2 || zones_x[k] == 3 || zones_x[k] == 6) {
					free_y[2] = 1;
					free_y[3] = 1;
				}
			}

			if ((zones_x[k] != 7) && (zones_x[k] != 5) && (zones_x[k] != 6)) {
				if ((errors[k].x > err_thres) && (errors[k].x > 0)) {
					move_x[zones_x[k]] = -1 * res[zones_x[k]][0];
					//mov_x[zones[k]] = move_x[zones[k]] + 1;
					phase_x[zones_x[k]] = phase_x[zones_x[k]] + phaseStep;
					phase_x_int[zones_x[k]] = phase_x_int[zones_x[k]] - 1 * res[zones_x[k]][0];
					send_arduino = 1;
				}
				else {
					if ((errors[k].x < -err_thres) && (errors[k].x < 0)) {
						move_x[zones_x[k]] = 1 * res[zones_x[k]][1];
						//mov_x[zones[k]] = move_x[zones[k]] + 1;
						phase_x[zones_x[k]] = phase_x[zones_x[k]] - phaseStep;
						phase_x_int[zones_x[k]] = phase_x_int[zones_x[k]] + 1 * res[zones_x[k]][1];
						send_arduino = 1;
					}
				}
			}
			else {
				if (zones_x[k] == 5) {
					//cout << "Zone 5 x" << endl;
					//free_y[0] = 1;
					//free_y[1] = 1;
					if (zones_x_base[k] == 0) {
						if (phase_x_int[0] != phase_x_int[1])
							phase_x_int[1] = phase_x_int[0];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[0] != phase_y_int[1])
							phase_y_int[1] = phase_y_int[0];
					}
					else {
						if (phase_x_int[0] != phase_x_int[1])
							phase_x_int[0] = phase_x_int[1];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[0] != phase_y_int[1])
							phase_y_int[0] = phase_y_int[1];
					}
					if ((errors[k].x > err_thres) && (errors[k].x > 0)) {
						adjust_phase(0, 0);
						adjust_phase(1, 0);
						send_arduino = 1;
					}
					else if ((errors[k].x < -err_thres) && (errors[k].x < 0)) {
						adjust_phase(0, 1);
						adjust_phase(1, 1);
						send_arduino = 1;
					}
				}
				else if (zones_x[k] == 6) {
					//free_y[2] = 1;
					//free_y[3] = 1;
					if (zones_x_base[k] == 2) {
						if (phase_x_int[2] != phase_x_int[3])
							phase_x_int[3] = phase_x_int[2];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[2] != phase_y_int[3])
							phase_y_int[3] = phase_y_int[2];
					}
					else {
						if (phase_x_int[2] != phase_x_int[3])
							phase_x_int[2] = phase_x_int[3];
						//free_y[zones_y_base[k]] = 1;
						if (phase_y_int[2] != phase_y_int[3])
							phase_y_int[2] = phase_y_int[3];
					}
					if ((errors[k].x > err_thres) && (errors[k].x > 0)) {
						adjust_phase(2, 0);
						adjust_phase(3, 0);
						send_arduino = 1;
					}
					else if ((errors[k].x < -err_thres) && (errors[k].x < 0)) {
						adjust_phase(2, 1);
						adjust_phase(3, 1);
						send_arduino = 1;
					}
				}
			}
			/*
			if (zones_y[k] == 7) {
				if (zones_x[k] == 0 || zones_x[k] == 1) {
					free_y[0] = 1;
					free_y[1] = 1;
				}
				if (zones_x[k] == 2 || zones_x[k] == 3) {
					free_y[2] = 1;
					free_y[3] = 1;
				}
			}*/

			if (zones_x[k] == 7) {
				if (zones_y[k] == 1 || zones_y[k] == 2 || zones_y[k] == 5) {
					free_x[1] = 1;
					free_x[2] = 1;
				}
				if (zones_y[k] == 0 || zones_y[k] == 3 || zones_y[k] == 6) {
					free_x[0] = 1;
					free_x[3] = 1;
				}
			}

			//Motion in Y
			if (zones_y[k] < 5) {
				if ((errors[k].y < -err_thres) && (errors[k].y < 0)) {
					move_y[zones_y[k]] = -1 * res[zones_y[k]][2];
					//mov_y[zones[k]] = move_y[zones[k]] + 1;
					phase_y[zones_y[k]] = phase_y[zones_y[k]] + phaseStep;
					phase_y_int[zones_y[k]] = phase_y_int[zones_y[k]] - 1 * res[zones_y[k]][2];
					send_arduino = 1;
				}
				else {
					if ((errors[k].y > err_thres) && (errors[k].y > 0)) {
						move_y[zones_y[k]] = 1 * res[zones_y[k]][3];
						//mov_y[zones[k]] = move_y[zones[k]] + 1;
						phase_y[zones_y[k]] = phase_y[zones_y[k]] - phaseStep;
						phase_y_int[zones_y[k]] = phase_y_int[zones_y[k]] + 1 * res[zones_y[k]][3];
						send_arduino = 1;
					}
				}
				if (zones_x[k] == 7) {
					if (zones_y[k] == 0 || zones_y[k] == 1) {
						free_x[1] = 1;
						free_x[2] = 1;
					}
				}
			}
			else {
				if (zones_y[k] == 5) {
					//free_x[1] = 1;
					//free_x[2] = 1;
					if (zones_y_base[k] == 1) {
						if (phase_y_int[1] != phase_y_int[2])
							phase_y_int[2] = phase_y_int[1];
						if (phase_x_int[1] != phase_x_int[2])
							phase_x_int[2] = phase_x_int[1];
					}
					else {
						if (phase_y_int[1] != phase_y_int[2])
							phase_y_int[1] = phase_y_int[2];
						if (phase_x_int[1] != phase_x_int[2])
							phase_x_int[1] = phase_x_int[2];
					}
					if ((errors[k].y < -err_thres) && (errors[k].y < 0)) {
						adjust_phase(1, 2);
						adjust_phase(2, 2);
						send_arduino = 1;
					}
					else {
						if ((errors[k].y > err_thres) && (errors[k].y > 0)) {
							adjust_phase(1, 3);
							adjust_phase(2, 3);
							send_arduino = 1;
						}
					}
				}
				else if (zones_y[k] == 6) {
					free_x[0] = 1;
					free_x[3] = 1;
					//cout << "Zone 6: " <<zones_y[k] << "Zone y base: " << zones_y_base[k] << endl;
					if (zones_y_base[k] == 0) {
						if (phase_y_int[0] != phase_y_int[3])
							phase_y_int[3] = phase_y_int[0];
						if (phase_x_int[0] != phase_x_int[3])
							phase_x_int[3] = phase_x_int[0];
						//phase_x_int[0] = 0;
						//free_x[zones_x_base[k]] = 1;
					}
					else {
						if (phase_y_int[0] != phase_y_int[3])
							phase_y_int[0] = phase_y_int[3];
						if (phase_x_int[0] != phase_x_int[3])
							phase_x_int[0] = phase_x_int[3];
						//phase_x_int[3] = 0;
						//free_x[zones_x_base[k]] = 1;
					}
					if ((errors[k].y < -err_thres) && (errors[k].y < 0)) {
						adjust_phase(0, 2);
						adjust_phase(3, 2);
						send_arduino = 1;
					}
					else {
						if ((errors[k].y > err_thres) && (errors[k].y > 0)) {
							adjust_phase(0, 3);
							adjust_phase(3, 3);
							send_arduino = 1;
						}
					}
				}

			}
		}
		check_phase();
		set_currents();
		//sprintf(send_to_arduino, "<%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c>", currents_all[0], currents_all[1], currents_all[2], currents_all[3], currents_all[4], currents_all[5], currents_all[6], currents_all[7], currents_all[8], currents_all[9], currents_all[10], currents_all[11], currents_all[12], currents_all[13], currents_all[14], currents_all[15]);
		//Sleep(20);
		cout << send_to_arduino << endl;

		if (send_arduino == 1) {

			//int msglen = strlen(send_to_arduino);
			//if (port->WriteData(send_to_arduino, msglen));   //write to arduino
													  //Sleep(30);
			int msglen = strlen(send_to_arduino);
			if (port->WriteData(send_to_arduino, msglen));   //write to arduino

			//printf("\n(writing success)\n");
			//std::cout << key_char << std::endl;
			Sleep(On_time);
			send_arduino = 0;
			for (int c = 0; c < 4; c++) {
				move_x[c] = 0;
				move_y[c] = 0;
			}

		}

	}

};
