#pragma once
#include <functional>

// Type-generic Bezier cubic spline class. The type T must implement
// addition, subtraction and multiplication and division using operators
// +, -, *, and / respectively
template <typename T>
class BezierCubicSpline
{
public:
	BezierCubicSpline();

	// Sets up the spline instance with all of the data necessary to
	// interpolate and make other computations
	void SetData(const T& p0,
                 const T& p1,
                 const T& p2,
                 const T& p3,
                 double (*normFunc)(const T&),
                 double (*dotFunc)(const T&, const T&),
                 T (*crossFunc)(const T&, const T&));

	// Given input t between 0 and 1, return the spline interpolation
	// between p0 and p3 respectively
	T Interpolate(double t) const;

	// Compute the arc length
	double ComputeArcLength(double tStep = 0.05) const;

	// Given input t between 0 and 1, return the tangent, normal, and
	// binormal vectors specified by t
	void ComputeTNB(double t, T& tangent, T& normal, T& binormal) const;

	// Given input t between 0 and 1, return the curvature at the point
	// specified by t
	double ComputeCurvature(double t) const;

protected:
	T p0, p1, p2, p3;
	double (*normFunc)(const T&);
	double (*dotFunc)(const T&, const T&);
	T (*crossFunc)(const T&, const T&);
};

#include "../../src/bezier_cubic_spline.hpp"
