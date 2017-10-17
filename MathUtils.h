#pragma once

#include <tyga\Math.hpp>
#include <vector>
#include <string>
#include <iostream>


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
	std::string ToStringVector2(tyga::Vector2 v);
	/**
	* Turn Vector3 to string.
	*
	* @param v Vector
	* @return string output.
	*/
	std::string ToStringVector3(tyga::Vector3 v);
	/**
	* Turn Vector4 to string.
	*
	* @param v Vector
	* @return string output.
	*/
	std::string ToStringVector4(tyga::Vector4 v);
	/**
	* Turn Matrix4x4 to string. (Some slightly confusing arguments here, maybe a refactor is needed?)
	*
	* @param matrix matrix
	* @param extendedStats extract and display translation, scale and rotation vectors?
	* @param useMatrix display rotation vector or matrix? (Used for debugging Matrix to Quart conversion)
	* @return string output.
	*/
	std::string ToStringMatrix4x4(tyga::Matrix4x4 matrix, bool extendedStats = true, bool useMatrix = false);

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
	T Clamp(T i, T min, T max) {
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
	T Lerp(T lhs, T rhs, float t){
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
	T IncOverTime(T initalValue, T increment, float t)
	{
		return initalValue + increment * t;
	}

	

	/**
	* Used to define planes using a point and a normal.
	* 
	*/
	struct Plane {
		tyga::Vector3 point;
		tyga::Vector3 normal;

		Plane(tyga::Vector3 _point, tyga::Vector3 _normal) {
			point = _point;
			normal = _normal;
		}

	};

	/**
	* Used to define triangles with 3 points.
	*
	*/
	struct Triangle {
		tyga::Vector2 p0;
		tyga::Vector2 p1;
		tyga::Vector2 p2;

		Triangle(tyga::Vector2 _p0, tyga::Vector2 _p1, tyga::Vector2 _p2) {
			p0 = _p0;
			p1 = _p1;
			p2 = _p2;
		}
	};

	/**
	* Generic Spline struct to make splines from 4 points.
	* NEEDS compariable type that can be multiplied and added together using relevant operators
	*/
	template<class T>
	struct Spline {
		T p0, p1, p2, p3;

		Spline(T _p0, T _p1, T _p2, T _p3) : p0(_p0), p1(_p1), p2(_p2), p3(_p3) {
		
		}

		/**
		* Catmull-Rom Interpolation
		* NEEDS a type that can be multiplied using the * operator and added through the + operator.
		*
		* @param t value between 1 - 0
		* @return point along Catmull-Rom spline by t
		*/
		T GetCatmullRomInterpolatedPoint(float t) {
			t = Clamp<float>(t, 0, 1);
			return 0.5f * (
				(2 * p1) +
				(-p0 + p2) * t +
				(2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
				(-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t
				);
		}

		/**
		* Cubic Beizer Interpolation
		* NEEDS a type that can be multiplied using the * operator and added through the + operator.
		*
		* @param t value between 1 - 0
		* @return point along Cubic Beizer spline by t
		*/
		T GetCubicInterpolatedPoint(float t) {
			t = Clamp<float>(t, 0, 1);
			float nt = 1.f - t;
			return
			nt * nt * nt * p0 +
			3.f * nt * nt * t * p1 +
			3.f * nt * t * t * p2 +
			t * t * t * p3;
		}

		T GetCubicFirstDerivative(float t) {
			t = Clamp<float>(t, 0, 1);
			float nt = 1.f - t;
			return
				3.f * nt * nt * (p1 - p0) +
				6.f * nt * t * (p2 - p1) +
				3.f * t * t * (p3 - p2);
		}

		T GetCubicSecondDerivative(float t) {
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
		

		AnimationCurve(tyga::Vector2 _p1, tyga::Vector2 _p2) : spline(Spline<tyga::Vector2>(tyga::Vector2(0,0),_p1,_p2,tyga::Vector2(1,1))) {
			DeterminePoints();
		}

		AnimationCurve(tyga::Vector2 _p0, tyga::Vector2 _p1, tyga::Vector2 _p2, tyga::Vector2 _p3) : spline(Spline<tyga::Vector2>(_p0, _p1, _p2, _p3)) {
			DeterminePoints();
		}

		AnimationCurve() : spline(Spline<tyga::Vector2>(tyga::Vector2(), tyga::Vector2(), tyga::Vector2(), tyga::Vector2())) {
			DeterminePoints();
		}

		/**
		* Catmull-Rom Interpolation
		*
		* @param t value between 1 - 0
		* @return point along Catmull-Rom spline by t
		*/
		tyga::Vector2 GetCatmullRomInterpolatedPoint(float t) {
			return spline.GetCatmullRomInterpolatedPoint(t);
		}

		/**
		* Cubic Beizer Interpolation
		*
		* @param t value between 1 - 0
		* @return point along Cubic Beizer spline by t
		*/
		tyga::Vector2 GetCubicInterpolatedPoint(float t) {
			return spline.GetCubicInterpolatedPoint(t);

		}
		/**
		* Animation Curve output, uses determined points to find Y for X along curve
		*
		* @param x amount along x axis, between 1 - 0
		* @return y value at given x axis, approximation
		*/
		float GetAnimationCurveOutput(float x) {
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
		tyga::Vector2 FindPointForX(float x) {
			
			x = Clamp(x, 0.f, 1.f);
			tyga::Vector2 prevPoint;
			tyga::Vector2 currentPoint;
			if (x < points[0].x) {
				return points[0];
			}
			for (int i = 0; i < amountOfPoints - 1; i++)
			{
				prevPoint = points[i];
				currentPoint = points[i + 1];
				if (x < currentPoint.x) {
					float px = (x - prevPoint.x) / (currentPoint.x + prevPoint.x);
					
					return Lerp(prevPoint, currentPoint, px);
				}
			}
			return points[amountOfPoints-1];
		}

		/**
		* Create determined points along curve
		*/
		void DeterminePoints() {
			points.clear();
			points.resize(amountOfPoints);
			for (float i = 0; i < amountOfPoints; i++)
			{
				tyga::Vector2 point = GetCubicInterpolatedPoint((1.f / amountOfPoints)*i);
				points.insert(points.begin() + (int)i,point);

			}
		}
		int amountOfPoints = 1000;
		std::vector<tyga::Vector2> points;
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
	T Lerp(T lhs, T rhs, float t, AnimationCurve animationCurve)
	{
		t = animationCurve.GetAnimationCurveOutput(t);
		return (1 - t)*lhs + t * rhs;
	}


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
	tyga::Quaternion Slerp(tyga::Quaternion lhs, tyga::Quaternion rhs, float t, AnimationCurve animationCurve);

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
	* Degrees to Radians conversion.
	*
	* @param degrees Degrees
	* @return result in radians.
	*/
	float DegreesToRadians(float degrees);

	/**
	* Radians to Degrees conversion.
	*
	* @param radians Radians
	* @return result in degrees.
	*/
	float RadiansToDegrees(float radians);

	/**
	* MPH to meters-per-minute conversion.
	*
	* @param miles Miles Per Hour
	* @return Meters Per Minute.
	*/
	float MilesPerHourToMetersPerMinute(float miles);
	
	/**
	* MPH to roations-per-minute conversion.
	*
	* @param miles Miles Per Hour
	* @param radius radius in meters
	* @return Rotations per minute.
	*/
	float MPHtoRotationsPerMinute(float miles, float radius);

	float QuatDotProduct(tyga::Quaternion lhs, tyga::Quaternion rhs);

	/**
	* Speherical Coords to Cartesian Coords (Might not work as intended)
	*
	* @param theta Spherical Theta
	* @param axion Spherical Axion
	* @param radius Radius Of Sphere
	* @return tyga::Vector3 of Coords.
	*/
	tyga::Vector3 SphericalPositionToVectorPosition(float theta, float axion, float radius);

	//TODO Cartesian Coords to Speherical Coords (Might have some issues with world to local)

	float GetSpeedFromVelocity(tyga::Vector3 velocity);

	float GetDistanceBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	float GetAngleBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	tyga::Vector3 GetAxisOfRotationBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	float GetDistanceOfPointPerpendicularToPlane(tyga::Vector3 point, Plane plane);

	/**
	* Extract translation vector from Matrix
	*
	* @param m to extract from
	* @return extracted translation vector
	*/
	tyga::Vector3 GetTranslationVectorFromMatrix(tyga::Matrix4x4 m);
	/**
	* Generate matrix from translation vector
	*
	* @param v translation vector
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromTranslationVector(tyga::Vector3 v);


	/**
	* Extract scaling vector from Matrix
	*
	* @param m to extract from
	* @return extracted scaling vector
	*/
	tyga::Vector3 GetScaleVectorFromMatrix(tyga::Matrix4x4 m);
	/**
	* Generate matrix from scaling vector
	*
	* @param v scaling vector
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromScaleVector(tyga::Vector3 v);

	/**
	* Extract Rotation Matrix from Matrix
	*
	* @param m to extract from
	* @return extracted rotation matrix
	*/
	tyga::Matrix4x4 GetRotationMatrixFromMatrix(tyga::Matrix4x4 m);

	/**
	* Extract eular from matrix
	*
	* @param m to extract from
	* @return extracted Eular
	*/
	tyga::Vector3 GetEularFromMatrix(tyga::Matrix4x4 m);
	/**
	* Generate matrix from eular
	*
	* @param eular Eular Angles as vector
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromEular(tyga::Vector3 eular);
	/**
	* Generate matrix from quaternion
	*
	* @param q Quaternion
	* @return resulting matrix
	*/
	tyga::Matrix4x4 GetMatrixFromQuat(tyga::Quaternion q);

	tyga::Quaternion GetQuatFromMatrix(tyga::Matrix4x4 m);

	tyga::Quaternion GetQuatFromEular(tyga::Vector3 v);

	tyga::Quaternion getQuatFromDirection(tyga::Vector3 v, tyga::Vector3 up = tyga::Vector3(0, 0, 1));

	tyga::Matrix4x4 getMatrixFromDirection(tyga::Vector3 v, tyga::Vector3 up = tyga::Vector3(0,1,0));

	tyga::Vector3 RotateVectorByQuat(tyga::Vector3 v, tyga::Quaternion q);

	tyga::Vector3 GetForwardVectorFromMatrix(tyga::Matrix4x4 m);
	tyga::Vector3 GetRightVectorFromMatrix(tyga::Matrix4x4 m);
	tyga::Vector3 GetUpVectorFromMatrix(tyga::Matrix4x4 m);

	tyga::Matrix4x4 CombineMatrices(tyga::Matrix4x4 first, tyga::Matrix4x4 second);
	tyga::Vector3 MultiplyVectors(tyga::Vector3 lhs, tyga::Vector3 rhs);

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
}

