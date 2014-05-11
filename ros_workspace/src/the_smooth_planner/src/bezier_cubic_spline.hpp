template <typename T>
BezierCubicSpline::BezierCubicSpline(const T& p0,
                                     const T& p1,
                                     const T& p2,
                                     const T& p3,
                                     std::function<double(const T&)> normFunc
									 std::function<double(const T&, const T&)> dotFunc,
									 std::function<T(const T&, const T&)> crossFunc)
{
	this->p0 = p0;
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;
	this->normFunc = normFunc;
	this->dotFunc = dotFunc;
	this->crossFunc = crossFunc;
}

template <typename T>
T BezierCubicSpline::Interpolate(double t)
{
	double t2 = t*t;
	double t3 = t2*t;
	double oneMinusT = 1.00 - t;
	double oneMinusT2 = oneMinusT*oneMinusT;
	double oneMinusT3 = oneMinusT2*oneMinusT;

	return oneMinusT3*p0 + 3.00*oneMinusT2*t*p0 + 3.00*oneMinusT*t2*p2 + t3*p3;
}

template <typename T>
double BezierCubicSpline::ComputeArcLength(double tStep)
{
	double arcLength = 0.00;
	T lastPoint = p0;
	t = tStep;
	while (t < 1.00)
	{
		T currentPoint = Interpolate(nextStep);
		arcLength += normFunc(currentPoint, nextPoint);

		lastPoint = currentPoint;
		t += tStep;
	}
	arcLength += normFunc(lastPoint, p3);
	
	return arcLength;
}

template <typename T>
void BezierCubicSpline::ComputeTNB(double t, T& tangent, T& normal, T& binormal)
{
	double t2 = t*t;
	double oneMinusT = 1.00 - t;
	double oneMinusT2 = oneMinusT*oneMinusT;
	T firstDerivative = 3.00*oneMinusT2*(p0-p1) + 6.00*oneMinusT*t*(p2-p1) + 3.00*t2*(p3-p2);
	T secondDerivative = 6.00*oneMinusT*(p2-2.00*p1+p0) + 6.00*t*(p3-2.00*p2+p1);

	tangent = firstDerivative / normFunc(firstDerivative);
	normal = secondDerivative - dotFunc(secondDerivative, tangent)*tangent;
	normal /= normFunc(normal);
	binormal = crossFunc(tangent, normal);
	binormal /= normfunc(binormal);
}

template <typename T>
double BezierCubicSpline::ComputeCurvature(double t)
{
	double t2 = t*t;
	double oneMinusT = 1.00 - t;
	double oneMinusT2 = oneMinusT*oneMinusT;
	T firstDerivative = 3.00*oneMinusT2*(p0-p1) + 6.00*oneMinusT*t*(p2-p1) + 3.00*t2*(p3-p2);
	T secondDerivative = 6.00*oneMinusT*(p2-2.00*p1+p0) + 6.00*t*(p3-2.00*p2+p1);

	return normFunc(crossFunc(secondDerivative, firstDerivative));
}
