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
	
	std::string ToStringVector2(tyga::Vector2 v);
	std::string ToStringVector3(tyga::Vector3 v);
	std::string ToStringVector4(tyga::Vector4 v);
	std::string ToStringMatrix4x4(tyga::Matrix4x4 matrix, bool extendedStats = true, bool useMatrix = false);

	template<class T>
	T Clamp(T i, T min, T max) {
		return i < min ? min : (i > max ? max : i);
	}

	template<class T>
	T Lerp(T lhs, T rhs, float t){
		return (1 - t)*lhs + t * rhs;
	}

	template<class T>
	T IncOverTime(T initalValue, T increment, float time)
	{
		return initalValue + increment * time;
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

	template<class T>
	struct Spline {
		T p0, p1, p2, p3;

		Spline(T _p0, T _p1, T _p2, T _p3) : p0(_p0), p1(_p1), p2(_p2), p3(_p3) {
		
		}

		T GetCatmullRomInterpolatedPoint(float t) {
			t = Clamp<float>(t, 0, 1);
			return 0.5f * (
				(2 * p1) +
				(-p0 + p2) * t +
				(2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
				(-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t
				);
		}
		
		bool IsLinear() {
			return (p0.x == p1.x && p2.x == p3.x && p0.y == p1.y && p2.y == p3.y);
		}

		T GetCubicInterpolatedPoint(float t) {
			t = Clamp<float>(t, 0, 1);
			float nt = 1.f - t;
			return
			nt * nt * nt * p0 +
			3.f * nt * nt * t * p1 +
			3.f * nt * t * t * p2 +
			t * t * t * p3;
			/*
			T a0 = Lerp<T>(p0, p1, t);
			T a1 = Lerp<T>(p1, p2, t);
			T a2 = Lerp<T>(p2, p3, t);

			T b0 = Lerp<T>(a0, a1, t);
			T b1 = Lerp<T>(a1, a2, t);

			return Lerp<T>(b0, b1, t);*/
		}

	};

	struct AnimationCurve {
		
		

		AnimationCurve(tyga::Vector2 _p1, tyga::Vector2 _p2) : spline(Spline<tyga::Vector2>(tyga::Vector2(0,0),_p1,_p2,tyga::Vector2(1,1))) {
			DeterminePoints();
		}

		tyga::Vector2 GetCatmullRomInterpolatedPoint(float t) {
			return spline.GetCatmullRomInterpolatedPoint(t);
		}

		tyga::Vector2 GetCubicInterpolatedPoint(float t) {
			return spline.GetCubicInterpolatedPoint(t);

		}

		float GetAnimationCurveOutput(float x) {
			tyga::Vector2 point = FindPointForX(x);
			float result = MathUtils::Clamp<float>(point.y, 0, 1);
			return result;
		}



	private:

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

		void DeterminePoints() {
			points.clear();
			points.resize(amountOfPoints);
			for (float i = 0; i < amountOfPoints; i++)
			{
				tyga::Vector2 point = GetCubicInterpolatedPoint((1.f / amountOfPoints)*i);
				points.insert(points.begin() + i,point);

			}
		}
		int amountOfPoints = 100;
		std::vector<tyga::Vector2> points;
		Spline<tyga::Vector2> spline;

	};

	/*template<class T>
	T Lerp(T lhs, T rhs, float t, EaseType type = EaseType::Linear)
	{
		return (1 - EaseTime(type, t))*lhs + EaseTime(type, t) * rhs;
	}*/

	template<class T>
	T Lerp(T lhs, T rhs, float t, MathUtils::AnimationCurve animationCurve)
	{
		t = animationCurve.GetAnimationCurveOutput(t);
		return (1 - t)*lhs + t * rhs;
	}

	/**
	* PI constant, redundent.
	*/
	const double _pi = 3.14159265358979323846;

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
	double DegreesToRadians(double degrees);

	/**
	* Radians to Degrees conversion.
	*
	* @param radians Radians
	* @return result in degrees.
	*/
	double RadiansToDegrees(double radians);

	/**
	* MPH to meters-per-minute conversion.
	*
	* @param miles Miles Per Hour
	* @return Meters Per Minute.
	*/
	double MilesPerHourToMetersPerMinute(double miles);
	
	/**
	* MPH to roations-per-minute conversion.
	*
	* @param miles Miles Per Hour
	* @param radius radius in meters
	* @return Rotations per minute.
	*/
	double MPHtoRotationsPerMinute(double miles, double radius);

	double QuatDotProduct(tyga::Quaternion lhs, tyga::Quaternion rhs);

	/**
	* Speherical Coords to Cartesian Coords (Might not work as intended)
	*
	* @param theta Spherical Theta
	* @param axion Spherical Axion
	* @param radius Radius Of Sphere
	* @return tyga::Vector3 of Coords.
	*/
	tyga::Vector3 SphericalPositionToVectorPosition(double theta, double axion, double radius);

	//TODO Cartesian Coords to Speherical Coords (Might have some issues with world to local)

	double GetSpeedFromVelocity(tyga::Vector3 velocity);

	double GetDistanceBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	double GetAngleBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	tyga::Vector3 GetAxisOfRotationBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	double GetDistanceOfPointPerpendicularToPlane(tyga::Vector3 point, Plane plane);

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

	tyga::Matrix4x4 GetMatrixFromQuat(tyga::Quaternion q);

	tyga::Matrix4x4 CombineMatrices(tyga::Matrix4x4 first, tyga::Matrix4x4 second);

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
	double SinWave(double frequency, double amplitude, double phase, double inital, double time);

	tyga::Quaternion Slerp(tyga::Quaternion lhs, tyga::Quaternion rhs, float t, MathUtils::AnimationCurve animationCurve);

	template<class T>
	T IncOverTime(T initalValue, T increment, float time);
	

	//This is all kinda legacy now, will remove once I find all dependancies
	/*
	float EaseTime(EaseType type, float time);

	float LinearTime(float t);
	float SmoothStepTime(float t);
	float EaseOutTime(float t);
	float EaseInTime(float t);
	*/
	
	

}

