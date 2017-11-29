#pragma once

#include <tyga\Math.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <random>


/**
*	My Math Utils, developed by Joshua Stevenson (Q5120984), 2017
*	I'm putting all the math helper stuff I can think of in here for all parts of the ASP module, so there's going be an odd mix in here
*	Don't know much about licencing so I don't care what happens to this code, go crazy
*
*	TODO:
*	Finish Commenting
*	Extrapolate unrelated math functions to other namespaces (Ex. MathUtils.Core, MathUtils.Matrix)
*	Fix Sphereical coordinantes
*/
namespace MathUtils {
	
	//Bunch of functions to convert some tyga formats to string, used for debugging.

	/**
	* Turn Vector2 to string.
	*
	* @param v Vector
	* @return string output.
	*/
	std::string ToStringVector2(const tyga::Vector2 v);
	/**
	* Turn Vector3 to string.
	*
	* @param v Vector
	* @return string output.
	*/
	std::string ToStringVector3(const tyga::Vector3 v);
	/**
	* Turn Vector4 to string.
	*
	* @param v Vector
	* @return string output.
	*/
	std::string ToStringVector4(const tyga::Vector4 v);
	/**
	* Turn Matrix4x4 to string. (Some slightly confusing arguments here, maybe a refactor is needed?)
	*
	* @param matrix matrix
	* @param extendedStats extract and display translation, scale and rotation vectors?
	* @param useMatrix display rotation vector or matrix? (Used for debugging Matrix to Quart conversion)
	* @return string output.
	*/
	std::string ToStringMatrix4x4(const tyga::Matrix4x4 matrix, const bool extendedStats = true, const bool useMatrix = false);

	/**
	* Generic clamp function
	* NEEDS a comparable type, one that has greater or lesser than functions
	*
	* @param i Value to be clamped
	* @param min Min value
	* @param max Max value
	* @return i clamped between min and max.
	*/
	template<class T>
	T Clamp(const T i, const T min, const T max) {
		return i < min ? min : (i > max ? max : i);
	}

	/**
	* Generic lerp function
	* NEEDS a type that can be multiplied using the * operator.
	*
	* @param lhs Begin value
	* @param rhs End value
	* @param t Interpolation value
	* @return Value interpolated between lhs and rhs by t
	*/
	template<class T>
	T Lerp(const T lhs, const T rhs, const float t){
		return (1 - t)*lhs + t * rhs;
	}

	/**
	* Generic increment over time function
	* NEEDS a type that can be multiplied using the * operator and added through the + operator.
	*
	* @param initalValue Begin value
	* @param increment Incremental value per second
	* @param t Time passed
	* @return incremented initalValue
	*/

	template<class T>
	T IncOverTime(const T initalValue, const T increment, const float t)
	{
		return initalValue + increment * t;
	}

	/**
	* Distance Between two values
	* Currently is only specialised for Vector3 and Vector2
	*
	* @param lhs Begin value
	* @param rhs End value
	* @return Distance between
	*/
	template<class T>
	inline float DistanceBetween(T lhs, T rhs) {
		std::string s = std::string("Not Implimented For ");
		s += typeid(T).name();
		throw std::exception(s.c_str());
	}
	template<>
	inline float DistanceBetween<tyga::Vector3>(tyga::Vector3 lhs, tyga::Vector3 rhs) {
		return std::sqrt(
			(lhs.x - rhs.x) * (lhs.x - rhs.x) +
			(lhs.y - rhs.y) * (lhs.y - rhs.y) +
			(lhs.z - rhs.z) * (lhs.z - rhs.z)
		);
	}
	template<>
	inline float DistanceBetween<tyga::Vector2>(tyga::Vector2 lhs, tyga::Vector2 rhs) {
		return std::sqrt(
			(lhs.x - rhs.x) * (lhs.x - rhs.x) +
			(lhs.y - rhs.y) * (lhs.y - rhs.y)
		);
	}
	

	/**
	* Used to define planes using a point and a normal.
	* 
	*/
	struct Plane {
		tyga::Vector3 point;
		tyga::Vector3 normal;

		Plane(const tyga::Vector3 _point, const tyga::Vector3 _normal) : point(_point), normal(_normal) {}

	};

	/**
	* Used to define triangles with 3 points.
	*
	*/
	struct Triangle {
		tyga::Vector2 p0;
		tyga::Vector2 p1;
		tyga::Vector2 p2;

		Triangle(const tyga::Vector2 _p0, const tyga::Vector2 _p1, const tyga::Vector2 _p2) : p0(_p0),p1(_p1),p2(_p2) {}
	};

	/**
	* Generic Spline struct to make splines from 4 points.
	* NEEDS compariable type that can be multiplied and added together using relevant operators
	*/
	template<class T>
	struct Spline {

		std::vector<T> points;
		float arcLength;

		Spline(T _p0, T _p1, T _p2, T _p3) : points{_p0,_p1,_p2,_p3} {
			DeterminePoints();
		}

		Spline(std::vector<T> _points) : points{_points} {
			DeterminePoints();
		}

		~Spline() {
			
		}

		struct Result {
		public:
			std::vector<T> curve;
			float time;

			Result(std::vector<T> c, float t) : curve(c), time(t) {}

		};
		

		/**
		* Catmull-Rom Interpolation
		* NEEDS a type that can be multiplied using the * operator and added through the + operator.
		*
		* @param t value between 1 - 0
		* @return point along Catmull-Rom spline by t
		*/
		T GetCatmullRomInterpolatedPoint(float t) const {
			Result curve = GetCurve(t);
			InternalGetCatmullRomInterpolatedPoint(curve.time, curve.curve[0], curve.curve[1], curve.curve[2], curve.curve[3]);
		}

		/**
		* Cubic Beizer Interpolation
		* NEEDS a type that can be multiplied using the * operator and added through the + operator.
		*
		* @param t value between 1 - 0
		* @return point along Cubic Beizer spline by t
		*/
		T GetCubicInterpolatedPoint(float t) const {
			Result curve = GetCurve(t);
			return InternalGetCubicInterpolatedPoint(curve.time, curve.curve[0], curve.curve[1], curve.curve[2], curve.curve[3]);
		}

		T GetNormalisedCubicPoint(float u) const {
			float t = getArcTime(u);
			
			return GetCubicInterpolatedPoint(t);
		}

		T GetNormalisedCubicFirstDerivative(float u) const {
			float t = getArcTime(u);
			return GetCubicFirstDerivative(t);
		}

		T GetCubicFirstDerivative(float t) const {
			Result curve = GetCurve(t);
			return InternalGetCubicFirstDerivative(curve.time, curve.curve[0], curve.curve[1], curve.curve[2], curve.curve[3]);
		}

		T GetCubicSecondDerivative(float t) const {
			Result curve = GetCurve(t);
			return InternalGetCubicSecondDerivative(curve.time, curve.curve[0], curve.curve[1], curve.curve[2], curve.curve[3]);
		}

		// Arc Length Paramization solution, which was kinda like my old keyframe solution, but I've implimented a binary search which should make things faster

		float getArcTime(float u) const {
			float t;
			u = Clamp(u, 0.f, 1.f);
			int pointsLen = (int)preDeterminedPoints.size();
			float targetArcLength = u * preDeterminedPoints[pointsLen - 1];

			int low = 0, high = pointsLen, index = 0;
			while (low < high) {
				index = low + (((high - low) / 2) | 0);
				if (preDeterminedPoints[index] < targetArcLength) {
					low = index + 1;
				}
				else {
					high = index;
				}
			}
			if (preDeterminedPoints[index] > targetArcLength) {
				index--;
			}
			float lastLength = preDeterminedPoints[index];
			if (lastLength == targetArcLength) {
				t = (float)(index / pointsLen);
			}
			else {
				t = (index + (targetArcLength - lastLength) / (preDeterminedPoints[index + 1] - lastLength)) / pointsLen;
			}
			t = Clamp(t, 0.f, 1.f);
			return t;
		}

	private:

		void DeterminePoints() {
			preDeterminedPoints.clear();
			preDeterminedPoints.reserve(amountOfPoints);
			
			T previousPoint = GetCubicInterpolatedPoint(0);
			float sum = 0;

			for (float i = 0; i < amountOfPoints; i++)
			{
				T point = GetCubicInterpolatedPoint((1.f / amountOfPoints)*i);
				float distance = DistanceBetween(point, previousPoint);
				sum += distance;
				preDeterminedPoints.insert(preDeterminedPoints.begin() + (int)i, sum);
				previousPoint = point;

			}
		}
		int amountOfPoints = 100;
		std::vector<float> preDeterminedPoints;

		int GetCurveCount() const {
			return (((int)points.size() - 1) / 3);
		}

		Result GetCurve(float t) const {
			float time = t;
			if (points.size() > 3) {
				int i = 0;
				if (time >= 1) {
					time = 1;
					i = (int)points.size() - 4;
				}
				else {
					int count = GetCurveCount();
					time = Clamp(time, 0.f, 1.f) * count;
					i = (int)time;
					time -= i;
					i *= 3;
				
				}
				return Result(std::vector<T>{points[i], points[i + 1], points[i + 2], points[i + 3]}, time);
			}
			return Result(std::vector<T>{T(), T(), T(), T()}, time);
		}

		T InternalGetCatmullRomInterpolatedPoint(float t, T p0, T p1, T p2, T p3) const {
			t = Clamp<float>(t, 0, 1);
			return 0.5f * (
				(2 * p1) +
				(-p0 + p2) * t +
				(2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
				(-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t
				);
		}

		T InternalGetCubicInterpolatedPoint(float t, T p0, T p1, T p2, T p3) const {
			t = Clamp<float>(t, 0, 1);
			float nt = 1.f - t;
			return
				nt * nt * nt * p0 +
				3.f * nt * nt * t * p1 +
				3.f * nt * t * t * p2 +
				t * t * t * p3;
		}

		T InternalGetCubicFirstDerivative(float t, T p0, T p1, T p2, T p3) const {
			t = Clamp<float>(t, 0, 1);
			float nt = 1.f - t;
			return
				3.f * nt * nt * (p1 - p0) +
				6.f * nt * t * (p2 - p1) +
				3.f * t * t * (p3 - p2);
		}

		T InternalGetCubicSecondDerivative(float t, T p0, T p1, T p2, T p3) const {
			t = Clamp<float>(t, 0, 1);
			return
				2 * (p2 - (2 * p1) + p0);
		}
		

	};


	/**
	* Animation curve made from a Vector2 Spline structure with additional wrapper functions.
	* It also creates a vector of points along the curve, used for determining Y(x) functions
	* Mainly used for animating keyframes.
	* 
	* 
	*/
	struct AnimationCurve {
		

		AnimationCurve(const tyga::Vector2 _p1, const tyga::Vector2 _p2) : spline(Spline<tyga::Vector2>(tyga::Vector2(0,0),_p1,_p2,tyga::Vector2(1,1))) {}

		AnimationCurve(const tyga::Vector2 _p0, const tyga::Vector2 _p1, const tyga::Vector2 _p2, const tyga::Vector2 _p3) : spline(Spline<tyga::Vector2>(_p0, _p1, _p2, _p3)) {}

		AnimationCurve() : spline(Spline<tyga::Vector2>(tyga::Vector2(), tyga::Vector2(), tyga::Vector2(), tyga::Vector2())) {}

		/**
		* Catmull-Rom Interpolation
		*
		* @param t value between 1 - 0
		* @return point along Catmull-Rom spline by t
		*/
		tyga::Vector2 GetCatmullRomInterpolatedPoint(const float t) const {
			return spline.GetCatmullRomInterpolatedPoint(t);
		}

		/**
		* Cubic Beizer Interpolation
		*
		* @param t value between 1 - 0
		* @return point along Cubic Beizer spline by t
		*/
		tyga::Vector2 GetCubicInterpolatedPoint(const float t) const {
			return spline.GetCubicInterpolatedPoint(t);

		}

		tyga::Vector2 GetCubicFirstDerivative(float t) const {
			return spline.GetCubicFirstDerivative(t);
		}

		float GetCubicFirstDerivativeMagnitude(float t) const {
			return tyga::length(spline.GetCubicFirstDerivative(t));
		}

		/**
		* Animation Curve output, uses determined points to find Y for X along curve
		*
		* @param x amount along x axis, between 1 - 0
		* @return y value at given x axis, approximation
		*/
		float GetAnimationCurveOutput(const float x) const {
			tyga::Vector2 point = FindPointForX(x);
			float result = MathUtils::Clamp<float>(point.y, 0, 1);
			return result;
		}



	private:

		/**
		* Uses determined points to find Y for X along curve
		*
		* @param x amount along x axis, between 1 - 0
		* @return y value at given x axis, approximation
		*/
		tyga::Vector2 FindPointForX(float x) const {
			
			return spline.GetNormalisedCubicPoint(x);

			//return tyga::Vector2();
			/*x = Clamp(x, 0.f, 1.f);
			tyga::Vector2 prevPoint;
			tyga::Vector2 currentPoint;
			if (x < spline.points[0].x) {
				return spline.points[0];
			}
			for (int i = 0; i < amountOfPoints - 1; i++)
			{
				prevPoint = spline.points[i];
				currentPoint = spline.points[i + 1];
				if (x < currentPoint.x) {
					float px = (x - prevPoint.x) / (currentPoint.x + prevPoint.x);
					
					return Lerp(prevPoint, currentPoint, px);
				}
			}
			return spline.points[amountOfPoints-1];*/
		}

		/**
		* Create determined points along curve
		*/
		Spline<tyga::Vector2> spline;

	};

	/**
	* Generic lerp function
	* NEEDS a type that can be multiplied using the * operator.
	*
	* @param lhs Begin value
	* @param rhs End value
	* @param t Interpolation value
	* @param animationCurve AnimationCurve to affect t by, used to create eased lerps
	* @return Value interpolated between lhs and rhs by t
	*/
	template<class T>
	T Lerp(const T lhs, const T rhs, float t, const AnimationCurve animationCurve)
	{
		t = animationCurve.GetAnimationCurveOutput(t);
		return (1 - t)*lhs + t * rhs;
	}

	

	


	/**
	* PI constant, redundent.
	*/
	const float _pi = 3.14159265358979323846f;

	// typical curves used for animaion, all curves are cubic in nature.
	const AnimationCurve clampCurve = AnimationCurve();
	const AnimationCurve linearCurve = AnimationCurve(
		tyga::Vector2(0.f, 0.f),
		tyga::Vector2(1.f, 1.f)
		);
	const AnimationCurve easeInCurve = AnimationCurve(
		tyga::Vector2(0.55f, 0.055f),
		tyga::Vector2(0.675f, 0.19f)
		);
	const AnimationCurve easeOutCurve = AnimationCurve(
		tyga::Vector2(0.215f, 0.61f),
		tyga::Vector2(0.355f, 1.f)
		);
	const AnimationCurve smoothStepCurve = AnimationCurve(
		tyga::Vector2(0.645f, 0.045f),
		tyga::Vector2(0.355f, 1.f)
		);


	/**
	* Slerp function for Quaternions
	* TODO: make generic
	*
	* @param lhs Begin value
	* @param rhs End value
	* @param t Interpolation value
	* @param animationCurve AnimationCurve to affect t by, used to create eased slerps
	* @return Value interpolated between lhs and rhs by t
	*/
	tyga::Quaternion Slerp(const tyga::Quaternion lhs, const tyga::Quaternion rhs, float t);
	tyga::Quaternion Slerp(const tyga::Quaternion lhs, const tyga::Quaternion rhs, float t, const AnimationCurve animationCurve);
	

	/**
	* Degrees to Radians conversion.
	*
	* @param degrees Degrees
	* @return result in radians.
	*/
	template<class T>
	T DegreesToRadians(const T degrees) {
		return degrees * _pi / 180;
	}

	/**
	* Radians to Degrees conversion.
	*
	* @param radians Radians
	* @return result in degrees.
	*/
	template<class T>
	T RadiansToDegrees(const T radians) {
		return radians * 180 / _pi;
	}

	/**
	* MPH to meters-per-minute conversion.
	*
	* @param miles Miles Per Hour
	* @return Meters Per Minute.
	*/
	float MilesPerHourToMetersPerMinute(const float miles);
	
	/**
	* MPH to roations-per-minute conversion.
	*
	* @param miles Miles Per Hour
	* @param radius radius in meters
	* @return Rotations per minute.
	*/
	float MPHtoRotationsPerMinute(const float miles, const float radius);

	float QuatDotProduct(const tyga::Quaternion lhs, const tyga::Quaternion rhs);

	/**
	* Speherical Coords to Cartesian Coords (Might not work as intended)
	*
	* @param theta Spherical Theta
	* @param axion Spherical Axion
	* @param radius Radius Of Sphere
	* @return tyga::Vector3 of Coords.
	*/
	tyga::Vector3 SphericalPositionToVectorPosition(const float theta, const float axion, const float radius);

	//TODO Cartesian Coords to Speherical Coords (Might have some issues with world to local)

	float GetSpeedFromVelocity(const tyga::Vector3 velocity);

	float GetDistanceBetweenVectors(const tyga::Vector3 a, const tyga::Vector3 b);

	float GetAngleBetweenVectors(const tyga::Vector3 a, const tyga::Vector3 b);

	tyga::Vector3 GetAxisOfRotationBetweenVectors(const tyga::Vector3 a, const tyga::Vector3 b);

	float GetDistanceOfPointPerpendicularToPlane(const tyga::Vector3 point, const Plane plane);

	/**
	* Extract translation vector from Matrix
	*
	* @param m to extract from
	* @return extracted translation vector
	*/
	tyga::Vector3 GetTranslationVectorFromMatrix(const tyga::Matrix4x4 m);
	/**
	* Generate matrix from translation vector
	*
	* @param v translation vector
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromTranslationVector(const tyga::Vector3 v);


	/**
	* Extract scaling vector from Matrix
	*
	* @param m to extract from
	* @return extracted scaling vector
	*/
	tyga::Vector3 GetScaleVectorFromMatrix(const tyga::Matrix4x4 m);
	/**
	* Generate matrix from scaling vector
	*
	* @param v scaling vector
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromScaleVector(const tyga::Vector3 v);

	/**
	* Extract Rotation Matrix from Matrix
	*
	* @param m to extract from
	* @return extracted rotation matrix
	*/
	tyga::Matrix4x4 GetRotationMatrixFromMatrix(const tyga::Matrix4x4 m);

	/**
	* Extract eular from matrix
	*
	* @param m to extract from
	* @return extracted Eular
	*/
	tyga::Vector3 GetEularFromMatrix(const tyga::Matrix4x4 m);
	/**
	* Generate matrix from eular
	*
	* @param eular Eular Angles as vector
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromEular(const tyga::Vector3 eular);
	/**
	* Generate matrix from quaternion
	*
	* @param q Quaternion
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromQuat(const tyga::Quaternion q);

	/**
	* Generate quaternion from matrix
	*
	* @param m Matrix4x4
	* @return resulting quaternion
	*/
	tyga::Quaternion GetQuatFromMatrix(const tyga::Matrix4x4 m);

	/**
	* Generate quaternion from eular
	*
	* @param v Vector3 as eular angles
	* @return resulting quaternion
	*/
	tyga::Quaternion GetQuatFromEular(const tyga::Vector3 v);

	tyga::Vector3 RotateVectorByQuat(const tyga::Vector3 v, const tyga::Quaternion q);

	/**
	* NOT WORKING
	
	tyga::Quaternion getQuatFromDirection(const tyga::Vector3 v, const tyga::Vector3 up = tyga::Vector3(0, 0, 1));

	tyga::Matrix4x4 getMatrixFromDirection(const tyga::Vector3 v, const tyga::Vector3 up = tyga::Vector3(0,1,0));

	
	*/

	tyga::Vector3 GetForwardVectorFromMatrix(const tyga::Matrix4x4 m);
	tyga::Vector3 GetRightVectorFromMatrix(const tyga::Matrix4x4 m);
	tyga::Vector3 GetUpVectorFromMatrix(tyga::Matrix4x4 m);
	

	tyga::Matrix4x4 CombineMatrices(const tyga::Matrix4x4 first, const tyga::Matrix4x4 second);

	tyga::Vector3 MultiplyVector3(const tyga::Vector3 lhs, const tyga::Vector3 rhs);

	tyga::Matrix4x4 frenet(const tyga::Vector3 pos, const tyga::Vector3 dir, const tyga::Vector3 up = tyga::Vector3(0, 1, 0));

	bool IsInsideTriangle(tyga::Vector2 point, Triangle tri);

	/**
	* Generate Sine Wave and sample at time
	*
	* @param frequency Frequency of the Sine wave
	* @param amplitude Amplitude of the Sine wave
	* @param phase Phase difference along the Sine wave
	* @param inital Inital value of the Sine wave
	* @param time time sampled from the Sine wave
	* @return result from Sine Wave
	*/
	float SinWave(float frequency, float amplitude, float phase, float inital, float time);

	int RandomSeed(int seed = -1);

	float RandomRange(float min, float max);
	float RandomUnit();

	tyga::Vector3 RandomDirection();

}

