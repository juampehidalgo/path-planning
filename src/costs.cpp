#include "costs.h"
#include <cmath>

double time_diff_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*> predictions)
{
	return logistic((double)(std::abs(traj_T - T))/T);
}

double s_diff_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double d_diff_const(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double collision_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double buffer_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double stays_on_road_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double exceeds_speed_limit_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double efficiency_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double max_accel_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double total_accel_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double max_jerk_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}

double total_jerk_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions)
{
	return 0.0f;
}


