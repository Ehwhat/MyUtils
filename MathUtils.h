#pragma once

#include <tyga\Math.hpp>
#include <string>


namespace MathUtils {

	enum EaseType {
		Linear,
		EaseIn,
		EaseOut,
		Smooth
	};

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

	const double _pi = 3.14159265358979323846;

	double DegreesToRadians(double degrees);

	double RadiansToDegrees(double radians);

	double MilesPerHourToMetersPerMinute(double miles);
	
	double MPHtoRotationsPerMinute(double miles, double radius);

	double QuatDotProduct(tyga::Quaternion lhs, tyga::Quaternion rhs);

	tyga::Vector3 SphericalPositionToVectorPosition(double theta, double axion, double radius);

	double GetSpeedFromVelocity(tyga::Vector3 velocity);

	double GetDistanceBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	double GetAngleBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	tyga::Vector3 GetAxisOfRotationBetweenVectors(tyga::Vector3 a, tyga::Vector3 b);

	double GetDistanceOfPointPerpendicularToPlane(tyga::Vector3 point, Plane plane);

	tyga::Vector3 GetTranslationVectorFromMatrix(tyga::Matrix4x4 m);
	tyga::Matrix4x4 GetMatrixFromTranslationVector(tyga::Vector3 v);

	tyga::Vector3 GetScaleVectorFromMatrix(tyga::Matrix4x4 m);
	tyga::Matrix4x4 GetMatrixFromScaleVector(tyga::Vector3 v);

	tyga::Matrix4x4 GetRotationMatrixFromMatrix(tyga::Matrix4x4 m);

	tyga::Vector3 GetEularFromMatrix(tyga::Matrix4x4 m);
	tyga::Matrix4x4 GetMatrixFromEular(tyga::Vector3 eular);

	tyga::Matrix4x4 GetMatrixFromQuat(tyga::Quaternion q);

	tyga::Matrix4x4 CombineMatrices(tyga::Matrix4x4 first, tyga::Matrix4x4 second);

	bool IsInsideTriangle(tyga::Vector2 point, Triangle tri);

	template<class T>
	T Lerp(T lhs, T rhs, float t);

	tyga::Quaternion Slerp(tyga::Quaternion lhs, tyga::Quaternion rhs, float t, EaseType type = EaseType::Linear);

	template<class T>
	T IncOverTime(T initalValue, T increment, float time);
	

	std::string ToStringVector3(tyga::Vector3 v);
	std::string ToStringVector4(tyga::Vector4 v);
	std::string ToStringMatrix4x4(tyga::Matrix4x4 matrix, bool extendedStats = true, bool useMatrix = false);

	float EaseTime(EaseType type, float time);

	template<class T>
	T Lerp(T lhs, T rhs, float t, EaseType type = EaseType::Linear)
	{
		return (1 - t)*lhs + EaseTime(type,t) * rhs;
	}

	float LinearTime(float t);

	float SmoothStepTime(float t);

	float EaseOutTime(float t);
	float EaseInTime(float t);
	template<class T>
	T IncOverTime(T initalValue, T increment, float time)
	{
		return initalValue + increment * time;
	}
	

}

