#pragma once
#include <opencv2/core/core.hpp>
#include <stdio.h>
using namespace std;
class microRobot {
private:

public:
	cv::Point2f mid_point_x;
	cv::Point2f mid_point_y;
	int id;
	cv::Point2f pos_r; //Position of red blob
	cv::Point2f pos_b; //Position of blue blob
	cv::Point2f pos; //Position of 250um magnet
	cv::Point2f p; //p is vector between red-blue blob
	cv::Point2f q; //q is vector between pos-blue blob
	cv::Point2f r; //r is vector between pos-red blob
	cv::Point2f pr;
	cv::Point2f pb;

	float q_norm;
	float r_norm;

	double phi; //Constant angle between p and q
	double theta; //Angle between blue and pos
	double eta; //Angle between red and pos
	double offset_angle; //For future to offset the angle between clicked point on robot as reference angle

	int end_goal = 0;
	int mid_goal = 0;
	bool all_waypoints = false;

	cv::Point2f const_q;

	vector<cv::Point2f> goals;
	vector<double> goals_angles; //orientation direction towards goal
	vector<int> goals_phase_x;
	vector<int> goals_phase_y;

	int num_goals = 0;
	int goal_ind = 0;
	cv::Point2f error_pos;
	double error_angle;
	int zone;
	int zone_x;
	int zone_y;
	int zone_y_base;
	float gap_x;
	float gap_y;

	int x_id;
	int y_id;
	float phase_gap;
	float phase_begin_x;
	float phase_begin_y;

	bool ManualControl;

	void set_midpoint(float midx_x, float midx_y, float midy_x, float midy_y) {
		mid_point_x.x = midx_x;
		mid_point_x.y = midx_y;
		mid_point_y.x = midy_x;
		mid_point_y.y = midy_y;
		gap_x = mid_point_x.x - mid_point_y.x;
		gap_y = mid_point_y.y - mid_point_x.y;
		phase_gap = (gap_x/4+gap_y/2)/2;
		phase_begin_x = std::fmod(mid_point_x.x, phase_gap);
		phase_begin_y = std::fmod(mid_point_x.y, phase_gap);

	}

	void reached_waypoint() {
		all_waypoints = true;
	}

	void set_id(int i, bool manual) { //Set ID number of robot
		id = i;
		if (manual == true) {
			ManualControl = true;
		}
		else
		{
			ManualControl = false;
		}
		end_goal = 0;
		mid_goal = 0;
		goal_ind = 0;
		all_waypoints = false;
	}

	void initialize_pos(cv::Point2f point_pos) { //Initial pos based on user-clicks
		pos = point_pos;
		update_zone();
	}

	void set_red(cv::Point2f point_red) { //Recognized red blobs from image
		pos_r = point_red;
	}

	void set_blue(cv::Point2f point_blue) { //Recognized blue blobs from image
		pos_b = point_blue;
	}

	void set_vector() { //Set up constants Q and phi for calculation of pos
		p = pos_r - pos_b;
		q = pos - pos_b;
		r = pos - pos_r;

		//p.y *= -1;
		//q.y *= -1;
		//p.y *= -1;
		theta = atan2(-p.y, p.x);
		phi = atan2(-q.y, q.x) - theta;
		eta = atan2(-r.y, r.x) - theta;
		//phi = acos(p.dot(q) / (norm(p)*norm(q)));
		q_norm = static_cast<float>(norm(q)); //phi
		r_norm = static_cast<float>(norm(r)); //eta
	}

	void update_zone() {
		int factor_gap = 1.5;
		//cout << "Midpoint : " << mid_point << endl;
		if (pos.x > mid_point_x.x) {
			if (pos.y > mid_point_x.y) {
				zone = 3;
			}
			else {
				zone = 0;
			}
		}
		else {
			if (pos.y > mid_point_x.y) {
				zone = 2;
			}
			else {
				zone = 1;
			}
		}

		if (pos.x > mid_point_y.x) {
			if (pos.y > mid_point_y.y) {
				zone_y_base = 3;
			}
			else {
				zone_y_base = 0;
			}
		}
		else {
			if (pos.y > mid_point_y.y) {
				zone_y_base = 2;
			}
			else {
				zone_y_base = 1;
			}
		}


		//Zones_x for crossing zones
		if (pos.x > mid_point_x.x + gap_x * factor_gap) {
			if (pos.y > mid_point_x.y + gap_y * factor_gap) {
				zone_x = 3;
			}
			else {
				if (pos.y < mid_point_x.y - gap_y * factor_gap) {
					zone_x = 0;
				}
				else {
					zone_x = 7;
				}
			}
		}
		else {
			if (pos.x < mid_point_x.x - gap_x * factor_gap) {
				if (pos.y > mid_point_x.y + gap_y * factor_gap) {
					zone_x = 2;
				}
				else {
					if (pos.y < mid_point_x.y - gap_y * factor_gap) {
						zone_x = 1;
					}
					else {
						zone_x = 7;
					}
				}
			}
			else {
				if (pos.y < mid_point_x.y - gap_y * factor_gap) {
					zone_x = 5;
				}
				else {
					if (pos.y > mid_point_x.y + gap_y * factor_gap) {
						zone_x = 6;
					}
					else {
						zone_x = 7;
					}
				}
			}
		}

		//Zones_y for crossing zones
		//Zones_x for crossing zones
		if (pos.x > mid_point_y.x + gap_y * factor_gap) {
			if (pos.y > mid_point_y.y + gap_x * factor_gap) {
				zone_y = 3;
			}
			else {
				if (pos.y < mid_point_y.y - gap_x * factor_gap) {
					zone_y = 0;
				}
				else {
					zone_y = 6;
				}
			}
		}
		else {
			if (pos.x < mid_point_y.x - gap_y * factor_gap) {
				if (pos.y > mid_point_y.y + gap_x * factor_gap) {
					zone_y = 2;
				}
				else {
					if (pos.y < mid_point_y.y - gap_x * factor_gap) {
						zone_y = 1;
					}
					else {
						zone_y = 5;
					}
				}
			}
			else {
				zone_y = 7;
			}
		}

		//Find x_id and y_id for each zone
		x_id = floor((pos.x - phase_begin_x + 1.5*phase_gap) / phase_gap);
		y_id = floor((pos.y - phase_begin_y + 1.5*phase_gap) / phase_gap);

	}



	void update_pos(float err_thres) { //Update pos for new images

		p = pos_r - pos_b;
		//p.y *= -1;
		theta = atan2(-p.y, p.x);

		pos.x = pos_b.x + q_norm * cos(phi + theta);
		pos.y = pos_b.y - q_norm * sin(phi + theta);

		//const_q.x = q_norm * static_cast<float>(cos(theta - phi));
		//const_q.y = q_norm * static_cast<float>(sin(theta - phi));
		//pos = pos_b + const_q;

		if (ManualControl == false) {
			error_pos = pos - goals[goal_ind];
			error_angle = theta;
			if (norm(error_pos) < err_thres) {
				mid_goal = 1;
				if (all_waypoints == true) {
					if (goal_ind < num_goals - 1) {

						goal_ind++;
						cout << "Updated goal index: " << goal_ind << " for Robot " << id << endl;
						mid_goal = 0;
						all_waypoints = false;
					}
					else {

						end_goal = 1;
					}
				}

			}
			update_zone();
		}
	}
	void print_data() {
		cout << "\nRobot ID: " << id << endl;
		cout << "Pos: " << pos << endl;
		cout << "Pos R: " << pos_r << endl;
		cout << "Pos B: " << pos_b << endl;
		cout << "Theta: " << theta * 180 / 3.141 << endl;
	}

	void set_goal(cv::Point2f goal_point) {
		goals.push_back(goal_point);
		//goals_angle.push_back(goal);
		//num_goals++;
		goals_phase_x.push_back(find_phase(0, goal_point));
		goals_phase_y.push_back(find_phase(1, goal_point));
	}

	void clear_goal() {
		goals.clear();
	}
	/*
	void update_goal_ind() {
		goal_ind++;
	}
	*/

	int find_phase(int axis, cv::Point2f way_point) {
		int phase;
		//0 for x axis, 1 for y axis
		switch (axis) {
		case 0: {
			phase = floor((way_point.x - phase_begin_x + 1.5*phase_gap) / phase_gap);
			return phase;
		}
		case 1: {
			phase = floor((way_point.y - phase_begin_y + 1.5*phase_gap) / phase_gap);
			return phase;
		}
		}
	}
};