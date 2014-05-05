#pragma once

#include <Eigen/Dense>

// A class to generate a circle in the XY from points in 3D (Z ignored)
class Circle
{
public:
	Circle(const Eigen::Vector2d p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);
	~Circle();

	double GetRadius() { return radius; }
	double GetCenterX() { return center(0); }
	double GetCenterY() { return center(1); }

protected:
	Circle() {}

	// Calculate the radius and center point of a circle circumscribing three points
	void CalculateCircle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);

	// Check if the line p1,p2 or p2,p3 are axis-aligned
	bool IsOrthogonal(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

	double radius;
	Eigen::Vector2d center;
};
