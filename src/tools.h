#ifndef TOOLS_H_
#define TOOLS_H_
#include <cmath>
#include <vector>
#include <cstddef>

constexpr double pi(void) { return M_PI; }
double deg2rad(const double x);
double rad2deg(const double x);
double mph2mps(const double x);
double mps2mph(const double x);
double distance(const double x1, const double y1, const double x2, const double y2);
std::vector<double> JMT(const std::vector<double>& start, const std::vector<double>& end, const double T);
// below is just an implementation entirely based on CppAD utility function Poly, just simplified to work only on doubles
double polyval(const size_t k, const std::vector<double>& a, const double z);
std::vector<double> polyder(const size_t k, const std::vector<double>& a);
double logistic(const double x);
#endif
