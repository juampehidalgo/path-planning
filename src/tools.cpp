#include "tools.h"
#include "Eigen-3.3/Eigen/Dense"
//#include <cmath>

//constexpr double pi()
//{
//	return M_PI;
//}

double deg2rad(const double x)
{
	return x * pi() / 180.0;
}

double rad2deg(const double x)
{
	return x * 180.0 / pi();
}

double mph2mps(const double x)
{
	return x * 0.44704;
}

double mps2mph(const double x)
{
	return x * 2.236936;
}

double distance(const double x1, const double y1, const double x2, const double y2)
{
	return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

std::vector<double> JMT(const std::vector<double>& start, const std::vector<double>& end, const double T)
{
	Eigen::VectorXd b(3);
	b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T * T),
             end[1] - (start[1] + start[2] * T),
	     end[2] - start[2];

	Eigen::MatrixXd A(3, 3);
	A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
	     3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
	     6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);

	Eigen::VectorXd result = A.colPivHouseholderQr().solve(b);

	return { start[0], start[1], 0.5 * start[2], result(0), result(1), result(2) };
}

double polyval(const size_t k, const std::vector<double>& a, const double z)
{
	// implementation just copied over from
	// https://www.coin-or.org/CppAD/Doc/poly.hpp.htm
	size_t i;
	size_t d = a.size() - 1;

	double tmp;

	// case where derivative order is greater than polynomial degree
	if (k > d)
	{
		tmp = 0.0f;
		return tmp;
	}
	// case on which we evaluate a derivative
	if (k > 0)
	{
		// initialize factor as (k-1)!
		size_t factor = 1;
		for (i = 2; i < k; i++)
		{
			factor *= i;
		}
		// set b to coefficient vector corresponding to derivative
		std::vector<double> b(d - k + 1);
		for (i = k; i <= d; i++)
		{
			factor *= i;
			tmp = (double)(factor);
			b[i - k] = a[i] * tmp;
			factor /= (i - k + 1);
		}
		// evaluate the derivative polynomial (it would appear that this is recursive, but it isn't, since the derivative order here is always 0)
		return polyval(0, b, z);
	}
	// case where we are evaluating the original polynomial
	double sum = a[d];
	i = d;
	while (i > 0)
	{
		sum *= z;
		sum += a[--i];
	}
	return sum;
}

std::vector<double> polyder(const size_t k, const std::vector<double>& a)
{
	size_t i;
	double tmp;
	size_t d = a.size() - 1;
	// case where derivative order is greater than polynomial degree
	if (k > d)
	{
		return { 0.0f };
	}

	// evaluate derivative
	if (k > 0)
	{
		// initialize factor as (k-1)!
		size_t factor = 1;
		for (i = 2; i < k; i++)
		{
			factor *= i;
		}
		// set b to coefficient vector as per derivative
		std::vector<double> b(d - k + 1);
		for (i = k; i <= d; i++)
		{
			factor *= i;
			tmp = (double)(factor);
			b[i - k] = a[i] * tmp;
			factor /= (i - k + 1);
		}
		return b;
	}
}

double logistic(const double x)
{
	return 2.0f / (1 + std::exp(-x)) - 1.0f;
}
