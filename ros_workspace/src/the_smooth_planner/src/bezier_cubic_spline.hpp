template <typename T>
BezierCubicSpline<T>::BezierCubicSpline()
{
}

template <typename T>
void BezierCubicSpline<T>::SetData(const T& p0,
                                   const T& p1,
                                   const T& p2,
                                   const T& p3,
                                   double (*normFunc)(const T&),
                                   double (*dotFunc)(const T&, const T&),
                                   T (*crossFunc)(const T&, const T&))
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
T BezierCubicSpline<T>::Interpolate(double t) const
{
	double t2 = t*t;
	double t3 = t2*t;
	double oneMinusT = 1.00 - t;
	double oneMinusT2 = oneMinusT*oneMinusT;
	double oneMinusT3 = oneMinusT2*oneMinusT;

	return oneMinusT3*p0 + 3.00*oneMinusT2*t*p0 + 3.00*oneMinusT*t2*p2 + t3*p3;
}

template <typename T>
double BezierCubicSpline<T>::ComputeArcLength(double tStep) const
{
    double epsilon = 0.0001;
	double arcLength = 0.00;
	T previousPoint = p0;
	double t = tStep;
	while (t < 1.00 - epsilon)
	{
		T currentPoint = Interpolate(t);
		arcLength += normFunc(previousPoint - currentPoint);

		previousPoint = currentPoint;
		t += tStep;
	}
	arcLength += normFunc(previousPoint - p3);
	
	return arcLength;
}

template <typename T>
void BezierCubicSpline<T>::ComputeTNB(double t, T& tangent, T& normal, T& binormal) const
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
	binormal /= normFunc(binormal);
}

template <typename T>
double BezierCubicSpline<T>::ComputeCurvature(double t) const
{
    if(ComputeArcLength() < 0.00001)
    {
        return 0.0;;
    }
	double t2 = t*t;
	double oneMinusT = 1.00 - t;
	double oneMinusT2 = oneMinusT*oneMinusT;
	T firstDerivative = 3.00*oneMinusT2*(p1-p0) + 6.00*oneMinusT*t*(p2-p1) + 3.00*t2*(p3-p2);
	T secondDerivative = 6.00*oneMinusT*(p2-2.00*p1+p0) + 6.00*t*(p3-2.00*p2+p1);

	T tangent = firstDerivative / normFunc(firstDerivative);
	T normal = secondDerivative - dotFunc(secondDerivative, tangent)*tangent;
	T binormal = crossFunc(tangent, normal);
    double curvature = normFunc(crossFunc(secondDerivative, firstDerivative))/pow(normFunc(firstDerivative), 3);
    if (binormal(2) > 0)
        curvature = fabs(curvature);
    else
        curvature = -fabs(curvature);
	return  curvature;

	//double arcLength = ComputeArcLength();
	//return normFunc(firstDerivative/arcLength);
}
