#ifndef ROAD_H_
#define ROAD_H_

#include <string>
#include <vector>
#include "spline.h"

class Road
{
public:
	Road(const std::string &map_file, const double max_s);
	virtual ~Road();
	std::vector<double> get_frenet(const double x, const double y, const double theta);
	std::vector<double> get_xy(const double s, const double d);
private:
	std::vector<double> map_waypoints_x_;
	std::vector<double> map_waypoints_y_;
	std::vector<double> map_waypoints_s_;
	std::vector<double> map_waypoints_dx_;
	std::vector<double> map_waypoints_dy_;

	double max_s_;

	tk::spline road_x_;
	tk::spline road_y_;
	tk::spline road_dx_;
	tk::spline road_dy_;

	int load_waypoints_file(const std::string &waypoints_file);
	void initialize_splines(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &s, const std::vector<double> &dx, const std::vector<double> &dy);
	int closest_waypoint(const double x, const double y);
	int next_waypoint(const double x, const double y, const double theta);
};

#endif
