#ifndef COSTS_H_
#define COSTS_H_

#include "tools.h"
#include "vehicle.h"
#include <vector>
#include <map>


double time_diff_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double s_diff_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double d_diff_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double collision_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double buffer_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double stays_on_road_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double exceeds_speed_limit_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double efficiency_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double max_accel_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const double T, const std::map<int, Vehicle*>& predictions);
double total_accel_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const std::map<int, Vehicle*>& predictions);
double max_jerk_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const std::map<int, Vehicle*>& predictions);
double total_jerk_cost(const std::vector<double>& traj_s, const std::vector<double>& traj_d, const double traj_T, const int v_id, const std::vector<double>& delta, const std::map<int, Vehicle*>& predictions);



#endif
