#pragma once
#include <functional>

// Type-generic Bezier cubic spline class. The type T must implement
// addition, subtraction and multiplication and division using operators
// +, -, *, and / respectively
template <typename T>
class BezierCubicSpline
{
public:
	BezierCubicSpline(const T& p0,
                      const T& p1,
                      const T& p2,
                      const T& p3,
                      std::function<double(const T&)> normFunc,
					  std::function<double(const T&, const T&)> dotFunc,
					  std::function<T(const T&, const T&)> crossFunc);

	// Given input t between 0 and 1, return the spline interpolation
	// between p0 and p3 respectively
	T Interpolate(double t);

	// Compute the arc length
	double ComputeArcLength(double tStep = 0.05);

	// Given input t between 0 and 1, return the tangent, normal, and
	// binormal vectors specified by t
	void ComputeTNB(double t, T& tangent, T& normal, T& binormal);

	// Given input t between 0 and 1, return the curvature at the point
	// specified by t
	double ComputeCurvature(double t);

protected:
	T p0, p1, p2, p3;
	std::function<double(const T&)> normFunc;
	std::function<double(const T&, const T&)> dotFunc;
	std::function<T(const T&, const T&)> crossFunc;
};

#include "../../src/bezier_cubic_spline.hpp"
