#include <the_smooth_planner/circle.h>
#include <limits.h>

static double epsilon = 0.000000001;

Circle::Circle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
	if (!IsOrthogonal(p1, p2, p3))
		CalculateCircle(p1, p2, p3);
	else if (!IsOrthogonal(p1, p3, p2))
		CalculateCircle(p1, p3, p2);
	else if (!IsOrthogonal(p2, p1, p3))
		CalculateCircle(p2, p1, p3);
	else if (!IsOrthogonal(p2, p3, p1))
		CalculateCircle(p2, p3, p1);
	else if (!IsOrthogonal(p3, p1, p2))
		CalculateCircle(p3, p1, p2);
	else if (!IsOrthogonal(p3, p2, p1))
		CalculateCircle(p3, p2, p1);
	else
	{
		radius = std::numeric_limits<double>::quiet_NaN();
		center(0) = std::numeric_limits<double>::quiet_NaN();
		center(1) = std::numeric_limits<double>::quiet_NaN();
	}
}

Circle::~Circle()
{
}

bool Circle::CalculateCircle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
	double deltaY1 = p2(1) - p1(1);
	double deltaX1 = p2(0) - p1(0);
	double deltaY2 = p3(1) - p2(1);
	double deltaX2 = p3(0) - p2(0);

	if (fabs(deltaX1) <= epsilon && fabs(deltaY2) <= epsilon)
	{
		center(0) = 0.5*(p2(0) + p3(0));
		center(1) = 0.5*(p1(1) + p2(1));
		radius = (center-p1).norm();

		return true;
	}

	// Assume that deltas are non-zero
	double slopeA = deltaY1/deltaX1;
	double slopeB = deltaY2/deltaX2;
	if (fabs(slopeA - slopeB) <= epsilon)
	{
		return false;
	}

	// Calculate the center
	center(0) = (slopeA*slopeB*(p1(1) - p3(1)) + slopeB*(p1(0) + p2(0)) - slopeA*(p2(0) + p3(0))) / (2*(slopeB - slopeA));
	center(1) = -1.0*(center(0) - (p1(0) + p2(0))/2.0) / slopeA + (p1(1) + p2(1))/2.0;
	radius = (center - p1).norm();

	return true;
}

bool Circle::IsOrthogonal(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
	double deltaY1 = p2(1) - p1(1);
	double deltaX1 = p2(0) - p1(0);
	double deltaY2 = p3(1) - p2(1);
	double deltaX2 = p3(0) - p2(0);

	if (fabs(deltaX1) <= epsilon && fabs(deltaY2) <= epsilon)
	{
		return false;
	}
	if (fabs(deltaY1) <= epsilon)
	{
		return true;
	}
	else if (fabs(deltaY2) <= epsilon)
	{
		return true;
	}
	else if(fabs(deltaX1) <= epsilon)
	{
		return true;
	}
	else if (fabs(deltaX2) <= epsilon)
	{
		return true;
	}

	return false;
}
