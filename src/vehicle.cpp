#include "vehicle.h"
#include <cmath>
#include <iostream>
#include "tools.h"

Vehicle::Vehicle(int id)
{
	this->id_ = id;
	this->ever_updated_ = false;
}

Vehicle::~Vehicle()
{
}

void Vehicle::update_polar_velocity(const double x, const double y, const double s, const double d, const double yaw, const double speed)
{
	this->x_ = x;
	this->y_ = y;
	this->s_ = s;
	this->d_ = d;
	this->yaw_ = deg2rad(yaw);
	this->speed_ = mph2mps(speed);
	this->v_x_ = speed * std::cos(yaw);
	this->v_y_ = speed * std::sin(yaw);
	this->ever_updated_ = true;
}

void Vehicle::update_cartesian_velocity(const double x, const double y, const double s, const double d, const double v_x, const double v_y)
{
	this->x_ = x;
	this->y_ = y;
	this->s_ = s;
	this->d_ = d;
	this->v_x_ = v_x;
	this->v_y_ = v_y;
	this->speed_ = std::sqrt(std::pow(v_x, 2) + std::pow(v_y, 2));
	this->yaw_ = std::atan2(v_y, v_x);
	this->ever_updated_ = true;
}

void Vehicle::display(void)
{
	std::cout << "v_id:" << this->id_ << "\ts=" << this->s_ << "\td=" << this->d_ << "\tyaw:" << rad2deg(this->yaw_) << "\tspeed=" << mps2mph(this->speed_) << std::endl;
}
