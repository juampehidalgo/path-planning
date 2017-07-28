#ifndef VEHICLE_H_
#define VEHICLE_H_

class Vehicle
{
public:
	Vehicle(int id);
	virtual ~Vehicle();
	void update_polar_velocity(const double x, const double y, const double s, const double d, const double yaw, const double speed);
	void update_cartesian_velocity(const double x, const double y, const double s, const double d, const double v_x, const double v_y);
	void display(void);
private:
	double x_;
	double y_;
	double v_x_;
	double v_y_;
	double s_;
	double d_;
	double yaw_;
	double speed_;
	int id_;
	bool ever_updated_;
};

#endif
