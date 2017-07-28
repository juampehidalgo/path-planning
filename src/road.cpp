
#include "road.h"
#include "tools.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <cmath>

#ifdef VERBOSE
#include <iostream>
#endif

Road::Road(const std::string &map_file, const double max_s)
{
	this->max_s_ = max_s;
	int num_points_read = load_waypoints_file(map_file);
	if (num_points_read == 0)
	{
		throw std::runtime_error("Road::Road - reading the map file returned no entries");
	}
	initialize_splines(this->map_waypoints_x_, this->map_waypoints_y_, this->map_waypoints_s_, this->map_waypoints_dx_, this->map_waypoints_dy_);

#ifdef VERBOSE
	std::cout << this->map_waypoints_x_.size() << std::endl;
	std::cout << this->road_x_(0.0f) << std::endl;
#endif

}

Road::~Road()
{
}

int Road::load_waypoints_file(const std::string &waypoints_file)
{
	std::ifstream in_map_(waypoints_file.c_str(), std::ifstream::in);

	std::string line;

	int num_points_read = 0;

	while (getline(in_map_, line))
	{
		std::istringstream iss(line);

		double x, y, s, dx, dy;

		iss >> x;
		iss >> y;
		iss >> s;
		iss >> dx;
		iss >> dy;

		this->map_waypoints_x_.push_back(x);
		this->map_waypoints_y_.push_back(y);
		this->map_waypoints_s_.push_back(s);
		this->map_waypoints_dx_.push_back(dx);
		this->map_waypoints_dy_.push_back(dy);

		num_points_read++;
	}

	return num_points_read;
}

void Road::initialize_splines(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &s, const std::vector<double> &dx, const std::vector<double> &dy)
{
	this->road_x_.set_points(s, x);
	this->road_y_.set_points(s, y);
	this->road_dx_.set_points(s, dx);
	this->road_dy_.set_points(s, dy);
}

int Road::closest_waypoint(const double x, const double y)
{
	double closest_length = std::numeric_limits<double>::max();
	int closest_index = 0;

	for (int idx = 0; idx < this->map_waypoints_x_.size(); idx++)
	{
		double map_x = this->map_waypoints_x_[idx];
		double map_y = this->map_waypoints_y_[idx];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closest_length)
		{
			closest_length = dist;
			closest_index = idx;
		}
	}
	return closest_index;
}

int Road::next_waypoint(const double x, const double y, const double theta)
{
	int closest_waypoint_index = closest_waypoint(x, y);
	double map_x = this->map_waypoints_x_[closest_waypoint_index];
	double map_y = this->map_waypoints_y_[closest_waypoint_index];
	double heading = std::atan2(map_y - y, map_x - x);
	double angle = std::abs(theta - heading);

	if (angle > pi()/4)
	{
		closest_waypoint_index++;
	}
	return closest_waypoint_index;
}

std::vector<double> Road::get_frenet(const double x, const double y, const double theta)
{
	int next_wp = next_waypoint(x, y, theta);
	int prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = this->map_waypoints_x_.size() - 1;
	}

	double n_x = this->map_waypoints_x_[next_wp] - this->map_waypoints_x_[prev_wp];
	double n_y = this->map_waypoints_y_[next_wp] - this->map_waypoints_y_[prev_wp];
	double x_x = x - this->map_waypoints_x_[prev_wp];
	double x_y = y - this->map_waypoints_y_[prev_wp];
	// find projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);
	// check sign of d value by comparing it with center point
	double center_x = 1000 - this->map_waypoints_x_[prev_wp];
	double center_y = 2000 - this->map_waypoints_y_[prev_wp];
	double center_to_pos = distance(center_x, center_y, x_x, x_y);
	double center_to_ref = distance(center_x, center_y, proj_x, proj_y);
	if (center_to_pos <= center_to_ref)
	{
		frenet_d *= -1;
	}
	// now calculate the s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(this->map_waypoints_x_[i], this->map_waypoints_y_[i], this->map_waypoints_x_[i + 1], this->map_waypoints_y_[i + 1]);
	}
	frenet_s += distance(0, 0, proj_x, proj_y);
	return { frenet_s, frenet_d };
}

std::vector<double> Road::get_xy(const double s, const double d)
{
	int prev_wp = -1;
	while(s > this->map_waypoints_s_[prev_wp + 1] && prev_wp < (this->map_waypoints_s_.size() - 1))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp + 1) % this->map_waypoints_x_.size();
	double heading = std::atan2(this->map_waypoints_y_[wp2] - this->map_waypoints_y_[prev_wp], this->map_waypoints_x_[wp2] - this->map_waypoints_x_[prev_wp]);
	double seg_s = s - this->map_waypoints_s_[prev_wp];
	double seg_x = this->map_waypoints_x_[prev_wp] + seg_s * std::cos(heading);
	double seg_y = this->map_waypoints_y_[prev_wp] + seg_s * std::sin(heading);

	// basically I could use the splines till here, and I would get the base X,Y of the center of the road
	// then just project the d to obtain the final set of X, Y coordinates
	double perp_heading = heading - pi()/2;
	double x = seg_x + d * std::cos(perp_heading);
	double y = seg_y + d * std::sin(perp_heading);

	return { x, y };
}
