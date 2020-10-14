#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen\Dense>

#pragma warning(disable:4018)
#pragma warning(disable:4244)


ASplineVec3::ASplineVec3() : mInterpolator(new ABernsteinInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
    if (mInterpolator) delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
    mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
    return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
    return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
    double fps = getFramerate();

	if (mInterpolator) { delete mInterpolator; }
    switch (type)
    {
	case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
	case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
	case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break;
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
	case LINEAR_EULER: mInterpolator = new AEulerLinearInterpolatorVec3(); break;
	case CUBIC_EULER: mInterpolator = new AEulerCubicInterpolatorVec3(); break;
    };
    
    mInterpolator->setFramerate(fps);
    computeControlPoints();
    cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
    return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys[keyID].second = value;
    computeControlPoints();
    cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0)
    {
        mStartPoint = value;
        computeControlPoints(false);
    }
    else if (ID == mCtrlPoints.size() + 1)
    {
        mEndPoint = value;
		computeControlPoints(false);
    }
    else mCtrlPoints[ID-1] = value;
    cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
    mKeys.push_back(Key(time, value));

    if (updateCurve)
    {
        computeControlPoints();
        cacheCurve();
    }
}

int ASplineVec3::insertKey(double time, const vec3& value, bool updateCurve)
{
	if (mKeys.size() == 0)
	{
		appendKey(time, value, updateCurve);
		return 0;
	}

	for (int i = 0; i < mKeys.size(); ++i)
	{
		assert(time != mKeys[i].first);
		if (time < mKeys[i].first)
		{
			mKeys.insert(mKeys.begin() + i, Key(time, value));
			if (updateCurve)
			{
				computeControlPoints();
				cacheCurve();
			}
			return i;
		}
	}

	// Append at the end of the curve
	appendKey(time, value, updateCurve);
	return mKeys.size() - 1;
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
    if (mKeys.size() == 0)
    {
        appendKey(0, value, updateCurve);
    }
    else
    {
        double lastT = mKeys[mKeys.size() - 1].first;
        appendKey(lastT + 1, value, updateCurve);
    }
}

void ASplineVec3::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys.erase(mKeys.begin() + keyID);
    computeControlPoints();
    cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID) const
{
    assert(keyID >= 0 && keyID < mKeys.size());
    return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
    return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID) const
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0) return mStartPoint;
    else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
    else return mCtrlPoints[ID-1];
}

int ASplineVec3::getNumControlPoints() const
{
    return mCtrlPoints.size() + 2; // include endpoints
}

void ASplineVec3::clear()
{
    mKeys.clear();
}

double ASplineVec3::getDuration() const 
{
    return mKeys.size() == 0 ? 0 : mKeys[mKeys.size()-1].first;
}

double ASplineVec3::getNormalizedTime(double t) const 
{
    return (t / getDuration());
}

double ASplineVec3::getKeyTime(int keyID) const
{
	assert(keyID >= 0 && keyID < mKeys.size());
	return mKeys[keyID].first;
}

vec3 ASplineVec3::getValue(double t) const
{
    if (mCachedCurve.size() == 0 || mKeys.size() == 0) return vec3();
	if (t < mKeys[0].first)
		return mCachedCurve[0];
	else
		t -= mKeys[0].first;

    double dt = mInterpolator->getDeltaTime();
    int rawi = (int)(t / dt); // assumes uniform spacing
    double frac = (t - rawi*dt) / dt;

	int i = mLooping? rawi % mCachedCurve.size() : std::min<int>(rawi, mCachedCurve.size() - 1);
	int inext = mLooping ? (i + 1) % mCachedCurve.size() : std::min<int>(i + 1, mCachedCurve.size() - 1);

    vec3 v1 = mCachedCurve[i];
    vec3 v2 = mCachedCurve[inext];
    vec3 v = v1*(1 - frac) + v2 * frac;
    return v;
}

void ASplineVec3::cacheCurve()
{
    mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints(bool updateEndPoints)
{
	if (mKeys.size() >= 2 && updateEndPoints)
	{
		int totalPoints = mKeys.size();

		//If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
		//They lie on the tangent of the first and last interpolation points.
		vec3 tmp = mKeys[0].second - mKeys[1].second;
		double n = tmp.Length();
		mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

		tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
		n = tmp.Length();
		mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
	}
    mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

vec3* ASplineVec3::getCachedCurveData()
{
	return mCachedCurve.data();
}

vec3 * ASplineVec3::getControlPointsData()
{
	return mCtrlPoints.data();
}

int ASplineVec3::getNumCurveSegments() const
{
    return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
    return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
    return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
    return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys, 
    const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0;

	curve.clear();

	int numSegments = keys.size() - 1;
	for (int segment = 0; segment < numSegments; segment++)
    {
        for (double t = keys[segment].first; t < keys[segment+1].first - FLT_EPSILON; t += mDt)
        {
			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
			// u = 0.0 when t = keys[segment-1].first  
			// u = 1.0 when t = keys[segment].first

			u = (t - keys[segment].first) / (keys[segment + 1].first - keys[segment].first);

            val = interpolateSegment(keys, ctrlPoints, segment, u);
            curve.push_back(val);
        }
    }
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments - 1, u);
		curve.push_back(val);
	}
}


// Interpolate p0 and p1 so that t = 0 returns p0 and t = 1 returns p1
vec3 ALinearInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;

	// TODO: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1
	curveValue = key0 * (1 - u) + key1 * (u);

	return curveValue;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0 = ctrlPoints[0 + (4 * segment)];
	vec3 b1 = ctrlPoints[1 + (4 * segment)];
	vec3 b2 = ctrlPoints[2 + (4 * segment)];
	vec3 b3 = ctrlPoints[3 + (4 * segment)];

	vec3 curveValue(0, 0, 0);
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	curveValue = (b0 * pow((1 - u), 3)) + (b1 * 3 * u * pow((1 - u), 2)) + (b2 * 3 * pow(u, 2) * (1 - u)) + (b3 * pow(u, 3));
	
	return curveValue;
}

vec3 lerp(vec3 a, vec3 b, double u) {
	return a * (1 - u) + b * u;
}

vec3 ACasteljauInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0 = ctrlPoints[0 + (4 * segment)];
	vec3 b1 = ctrlPoints[1 + (4 * segment)];
	vec3 b2 = ctrlPoints[2 + (4 * segment)];
	vec3 b3 = ctrlPoints[3 + (4 * segment)];
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  deCsteljau alogithm

	vec3 b01 = lerp(b0, b1, u);
	vec3 b11 = lerp(b1, b2, u);
	vec3 b21 = lerp(b2, b3, u);
	vec3 b02 = lerp(b01, b11, u);
	vec3 b12 = lerp(b11, b21, u);
	curveValue = lerp(b02, b12, u);
	
	return curveValue;
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0 = ctrlPoints[0 + (4 * segment)];
	vec3 b1 = ctrlPoints[1 + (4 * segment)];
	vec3 b2 = ctrlPoints[2 + (4 * segment)];
	vec3 b3 = ctrlPoints[3 + (4 * segment)];
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations

	Eigen::Matrix4d M;
	M << 1, -3, 3, -1,
			0, 3, -6, 3,
			0, 0, 3, -3,
			0, 0, 0, 1;
	Eigen::Vector4d U(1.0, u, pow(u, 2), pow(u, 3));

	Eigen::MatrixXd MU = M * U;

	curveValue = b0 * MU(0, 0) + b1 * MU(1, 0) + b2 * MU(2, 0) + b3 * MU(3, 0);

	return curveValue;
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 p0 = keys[segment].second;
	vec3 p1 = keys[1 + segment].second;
	vec3 q0 = ctrlPoints[segment];// slope at p0
	vec3 q1 = ctrlPoints[1 + segment]; // slope at p1
	vec3 curveValue(0, 0, 0);

	// TODO: Compute the interpolated value h(u) using a cubic Hermite polynomial 
	curveValue = p0 * (2 * pow(u, 3) - 3 * pow(u, 2) + 1) + p1 * (-2 * pow(u, 3)
		+ 3 * pow(u, 2)) + q0 * (pow(u, 3) - 2 * pow(u, 2) + u) + q1 * (pow(u, 3) - pow(u, 2));

	return curveValue;
}

vec3 ABSplineInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 curveValue(0, 0, 0);

	// Hint: Create a recursive helper function N(knots,n,j,t) to calculate BSpline basis function values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = curve interval on knot vector in which to interpolate
	//     t = time value	

	// Step 1: determine the index j
	// Step 2: compute the n nonzero Bspline Basis functions N given j
	// Step 3: get the corresponding control points from the ctrlPoints vector
	// Step 4: compute the Bspline curveValue at time t

	return curveValue;
}

void ACubicInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys, 
    std::vector<vec3>& ctrlPoints, 
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

	for (int i = 1; i < keys.size(); i++)
	{
		vec3 b0, b1, b2, b3;
		// TODO: compute b0, b1, b2, b3
		b0 = keys[i - 1].second;
		b3 = keys[i].second;

		// left side of curve
		if (i == 1) {
			b1 = b0 + (1.0 / 3.0) * ((keys[1].second - startPoint) / 2.0);
		}
		else {
			b1 = keys[i - 1].second + (1.0 / 3.0) * ((keys[i].second - keys[i - 2].second) / 2.0);
		}

		// right side of curve
		if (i == keys.size() - 1) {
			b2 = b3 - (1.0 / 3.0) * ((endPoint - keys[keys.size() - 1].second) / 2.0);
		}
		else {
			b2 = keys[i ].second - (1.0 / 3.0) * ((keys[i + 1].second - keys[i - 1].second) / 2.0);
		}

		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
}

void AHermiteInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints,
    vec3& startPoint, vec3& endPoint)
{
	ctrlPoints.clear();
	ctrlPoints.resize(keys.size(), vec3(0, 0, 0));
	if (keys.size() <= 1) return;

	// TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
	// Step 2: Initialize D
	// Step 3: Solve AC=D for C
	// Step 4: Save control points in ctrlPoints

	// Initialize A
	Eigen::MatrixXd A(keys.size(), keys.size());

	for (int row = 0; row < keys.size(); row++) {
		for (int col = 0; col < keys.size(); col++) {
			if (row == 0 && (col == 0 || col == 1)) {
				A(0, 0) = 2;
				A(0, 1) = 1;
			} 
			else if (row == keys.size() - 1 && (col == keys.size() - 2 || col == keys.size() - 1)) {
				A(keys.size() - 1, keys.size() - 2) = 1;
				A(keys.size() - 1, keys.size() - 1) = 2;
			}
			else if (row > 0 && col == row - 1) {
				A(row, col) = 1;
			}
			else if (col == row) {
				A(row, col) = 4;
			}
			else if (row < keys.size() - 1 && col == row + 1) {
				A(row, col) = 1;
			}
			else {
				A(row, col) = 0;
			}
		}
	}

	// Initialize D
	Eigen::MatrixXd D(keys.size(), 3);
	for (int row = 0; row < keys.size(); row++) {
		for (int col = 0; col < 3; col++) {
			// clamped endpoint conditions
			if (row == 0) {
				D(row, col) = 3 * keys[1].second[1] - keys[0].second[col];
			}
			else if (row == keys.size() - 1) {
				D(row, col) = 3 * keys[keys.size() - 1].second[col] - keys[keys.size() - 2].second[col];
			}
			else {
				D(row, col) = 3 * (keys[row + 1].second[col] - keys[row - 1].second[col]);
			}
		}
	}

	// Solve for C
	Eigen::MatrixXd C(keys.size(), 3);
	C = A.inverse() * D;

	for (int row = 0; row < keys.size() - 1; row++) {
		vec3 dp;

		for (int col = 0; col < 3; col++) {
			dp[col] = C(row, col);
		}
		ctrlPoints[row] = dp;
	}
}

void ABSplineInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints, 
    vec3& startPt, vec3& endPt)
{
    ctrlPoints.clear();
	ctrlPoints.resize(keys.size() + 2, vec3(0, 0, 0));
    if (keys.size() <= 1) return;

	// TODO:
	// Hints: 
	// 1. use Eigen::MatrixXd to calculate the control points by solving the system of equations AC=D for C

	// 2. Create a recursive helper function dN(knots,n,t,l) to calculate derivative BSpline values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = interval on knot vector in which to interpolate
	//     t = time value
	//     l = derivative (l = 1 => 1st derivative)

	// Step 1: Calculate knot vector using a uniform BSpline
	//         (assune knots are evenly spaced 1 apart and the start knot is at time = 0.0)

	// Step 2: Calculate A matrix  for a natural BSpline
	//         (Set 2nd derivative at t0 and tm to zero, where tm is the last point knot; m = #segments)

	// Step 3: Calculate  D matrix composed of our target points to interpolate

	// Step 4: Solve AC=D for C 

	// Step 5: save control points in ctrlPoints
}

std::vector<vec3> AInterpolatorVec3::convertAngles(vec3 key0, vec3 key1) {
	std::vector<vec3> angleVector;
	vec3 newKey0, newKey1;

	for (int i = 0; i < 3; i++) {
		if (key1[i] > 360) {
			key1[i] = fmod(key1[i], 360);
		}
		if (key0[i] > 360) {
			key0[i] = fmod(key0[i], 360);
		}
		if (key1[i] < -360) {
			key1[i] = fmod(key1[i], -360);
		}
		if (key0[i] < -360) {
			key0[i] = fmod(key0[i], -360);
		}

		float distance = fmod(key1[i] - key0[i], 360);
		if (distance < -180)
			distance += 360;
		else if (distance > 180)
			distance -= 360;

		newKey0[i] = key0[i];
		newKey1[i] = key0[i] + distance;
	}

	angleVector.push_back(newKey0);
	angleVector.push_back(newKey1);
	return angleVector;
}

vec3 AEulerLinearInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys, 
	const std::vector<vec3>& ctrlPoints, 
	int segment, double u)
{
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;

	// TODO:
	// Linear interpolate between key0 and key1
	// You should convert the angles to find the shortest path for interpolation

	key0 = convertAngles(key0, key1)[0];
	key1 = convertAngles(key0, key1)[1];

	curveValue = key0 * (1 - u) + key1 * (u);

	return curveValue;
}

vec3 AEulerCubicInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys, 
	const std::vector<vec3>& ctrlPoints, int segment, double t)
{
	vec3 b0 = ctrlPoints[0 + (4 * segment)];
	vec3 b1 = ctrlPoints[1 + (4 * segment)];
	vec3 b2 = ctrlPoints[2 + (4 * segment)];
	vec3 b3 = ctrlPoints[3 + (4 * segment)];
	vec3 curveValue(0, 0, 0);
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	// You should convert the angles to find the shortest path for interpolation

	b0 = convertAngles(b0, b1)[0];
	b1 = convertAngles(b0, b1)[1];
	b2 = convertAngles(b1, b2)[1];
	b3 = convertAngles(b2, b3)[1];

	curveValue = b0 * pow(1 - t, 3) + b1 * 3 * t * pow(1 - t, 2) + b2 * 3 * pow(t, 2) * (1 - t) + b3 * pow(t, 3);
	return curveValue;
}

void AEulerCubicInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys, 
	std::vector<vec3>& ctrlPoints, vec3 & startPoint, vec3 & endPoint)
{
	ctrlPoints.clear();
	if (keys.size() <= 1) return;

	// Hint: One naive way is to first convert the keys such that the differences of the x, y, z Euler angles 
	//		 between every two adjacent keys are less than 180 degrees respectively 

	for (int i = 1; i < keys.size(); i++)
	{
		vec3 b0, b1, b2, b3;
		b0 = convertAngles(keys[i - 1].second, keys[i].second)[0];
		b3 = convertAngles(keys[i - 1].second, keys[i].second)[1];

		// left side of curve
		if (i == 1) {
			vec3 sP_converted = convertAngles(startPoint, b0)[0];
			b1 = b0 + (1.0 / 3.0) * ((b3 - sP_converted) / 2.0);
		}
		else {
			vec3 b2_converted = convertAngles(keys[i - 2].second, b0)[0];
			b1 = b0 + (1.0 / 3.0) * ((b3 - b2_converted) / 2.0);
		}

		// right side of curve
		if (i == keys.size() - 1) {
			vec3 eP_converted = convertAngles(b3, endPoint)[1];
			b2 = b3 - (1.0 / 3.0) * ((eP_converted - b0) / 2.0);
		}
		else {
			vec3 b1_converted = convertAngles(b3, keys[i + 1].second)[1];
			b2 = b3 - (1.0 / 3.0) * ((b1_converted - b0) / 2.0);
		}

		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
}
