#include "MathUtils.h"
#include <cmath>
#include <algorithm>

namespace MathUtils {

	double DegreesToRadians(double degrees) {
		return degrees * _pi / 180;
	}

	double RadiansToDegrees(double radians) {
		return radians * 180 / _pi;
	}

	double MilesPerHourToMetersPerMinute(double miles)
	{
		return miles * 1609;
	}

	double MPHtoRotationsPerMinute(double miles, double radius)
	{
		return MilesPerHourToMetersPerMinute(miles) / (radius*M_PI * 2);
	}

	double QuatDotProduct(tyga::Quaternion lhs, tyga::Quaternion rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
	}

	tyga::Vector3 SphericalPositionToVectorPosition(double theta, double axion, double radius)
	{
		return tyga::Vector3(
			radius * sin(theta)*cos(axion),
			radius * sin(theta)*sin(axion),
			radius * cos(theta)
		);
	}

	double GetSpeedFromVelocity(tyga::Vector3 velocity){
		return tyga::length(velocity);
	}
	double GetDistanceBetweenVectors(tyga::Vector3 a, tyga::Vector3 b)
	{
		return tyga::length(a - b);
	}
	double GetAngleBetweenVectors(tyga::Vector3 a, tyga::Vector3 b)
	{
		double dot = tyga::dot(a, b)/(tyga::length(a) * tyga::length(b));
		return std::acos(dot);
	}

	tyga::Vector3 GetAxisOfRotationBetweenVectors(tyga::Vector3 a, tyga::Vector3 b) {
		return tyga::cross(a, b);
	}

	double GetDistanceOfPointPerpendicularToPlane(tyga::Vector3 point, Plane plane)
	{
		return tyga::dot(plane.normal, (point - plane.point));
	}

	tyga::Vector3 GetTranslationVectorFromMatrix(tyga::Matrix4x4 m)
	{
		return tyga::Vector3(m._30,m._31,m._32);
	}

	tyga::Matrix4x4 GetMatrixFromTranslationVector(tyga::Vector3 v) {
		return tyga::Matrix4x4(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			v.x, v.y, v.z, 1
		);
	}

	tyga::Matrix4x4 GetMatrixFromPoint(tyga::Vector3 v) {
		return tyga::Matrix4x4(
			1, 0, 0, v.x,
			0, 1, 0, v.y,
			0, 0, 1, v.z,
			0, 0, 0, 1
		);
	}

	tyga::Vector3 GetScaleVectorFromMatrix(tyga::Matrix4x4 m)
	{
		double sx = tyga::length(tyga::Vector3(m._00, m._10, m._20));
		double sy = tyga::length(tyga::Vector3(m._01, m._11, m._21));
		double sz = tyga::length(tyga::Vector3(m._02, m._12, m._22));
		return tyga::Vector3(sx, sy, sz);
	}

	tyga::Matrix4x4 GetMatrixFromScaleVector(tyga::Vector3 v)
	{
		return tyga::Matrix4x4(
			v.x, 0, 0, 0,
			0, v.y, 0, 0,
			0, 0, v.z, 0,
			0, 0, 0, 1
		);
	}

	tyga::Matrix4x4 GetRotationMatrixFromMatrix(tyga::Matrix4x4 m)
	{
		tyga::Vector3 s = GetScaleVectorFromMatrix(m);
		return tyga::Matrix4x4(
			m._00 / s.x, m._01 / s.y, m._02 / s.z, 0,
			m._10 / s.x, m._11 / s.y, m._12 / s.z, 0,
			m._20 / s.x, m._21 / s.y, m._22 / s.z, 0,
			0,0,0,1
		);
	}

	tyga::Vector3 GetEularFromMatrix(tyga::Matrix4x4 m)
	{
		m = GetRotationMatrixFromMatrix(m);
		double sy = std::sqrt(m._00*m._00 + m._10 * m._10);
		bool isSingular = sy <  1e-6;
		double x, y, z;
		if (isSingular) {
			x = std::atan2(-m._12, m._11);
			y = std::atan2(-m._20, sy);
			z = 0;
		}
		else {
			x = std::atan2(m._21, m._22);
			y = std::atan2(-m._20, sy);
			z = std::atan2(m._10, m._00);
		}
		return tyga::Vector3(x, y, z);
	}

	tyga::Matrix4x4 GetMatrixFromEular(tyga::Vector3 eular)
	{
		eular *= M_PI / 180;
		tyga::Matrix4x4 x = tyga::Matrix4x4(
			1, 0, 0, 0,
			0, cosf(eular.x), -sinf(eular.x), 0,
			0, sinf(eular.x), cosf(eular.x), 0,
			0, 0, 0, 1

		);
		tyga::Matrix4x4 y = tyga::Matrix4x4(
			cosf(eular.y), 0, sinf(eular.y), 0,
			0, 1, 0, 0,
			-sinf(eular.y), 0, cosf(eular.y), 0,
			0, 0, 0, 1
		);
		tyga::Matrix4x4 z = tyga::Matrix4x4(
			cosf(eular.z), -sinf(eular.z), 0, 0,
			sinf(eular.z), cosf(eular.z), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1
		);
		return z*y*x;
	}

	tyga::Matrix4x4 GetMatrixFromQuat(tyga::Quaternion q)
	{
		return tyga::Matrix4x4(1 - 2 * (q.y*q.y) - 2 * (q.z*q.z), 2 * q.x*q.y + 2 * q.z*q.w, 2 * q.x*q.z - 2 * q.y*q.w, 0,
			2 * q.x*q.y - 2 * q.z*q.w, 1 - 2 * (q.x*q.x) - 2 * (q.z*q.z), 2 * q.y*q.z + 2 * q.x*q.w, 0,
			2 * q.x*q.z + 2 * q.y*q.w, 2 * q.y*q.z - 2 * q.x*q.w, 1 - 2 * (q.x*q.x) - 2 * (q.y*q.y), 0,
			0, 0, 0, 1);
	}

	tyga::Matrix4x4 CombineMatrices(tyga::Matrix4x4 first, tyga::Matrix4x4 second)
	{
		return first * second;
	}

	bool IsInsideTriangle(tyga::Vector2 point, Triangle tri)
	{
		double area = 0.5 *(-tri.p1.y*tri.p2.x + tri.p0.y*(-tri.p1.x + tri.p2.x) + tri.p0.x*(tri.p1.y - tri.p2.y) + tri.p1.x*tri.p2.y);
		double s = 1 / (2 * area)*(tri.p0.y*tri.p2.x - tri.p0.x*tri.p2.y + (tri.p2.y - tri.p0.y)*point.x + (tri.p0.x - tri.p2.x)*point.y);
		double t = 1 / (2 * area)*(tri.p0.x*tri.p1.y - tri.p0.y*tri.p1.x + (tri.p0.y - tri.p1.y)*point.x + (tri.p1.x - tri.p0.x)*point.y);
		return (s > 0 && t > 0 && (1 - s - t) > 0);
	}

	tyga::Quaternion Slerp(tyga::Quaternion lhs, tyga::Quaternion rhs, float t, EaseType type)
	{
		lhs = tyga::unit(lhs);
		rhs = tyga::unit(rhs);

		tyga::Quaternion result;

		t = EaseTime(type, t);

		float dotProduct = QuatDotProduct(lhs, rhs);
		//const double theshold = 0.9995; // If the results look weird from being to close
		//if (fabs(dotProduct) > theshold) {
		//	result = lhs + t*(rhs - lhs);
		//	tyga::unit(result);
		//	return result;
		//}
		if (dotProduct < 0.0) {
			rhs = -rhs;
			dotProduct = -dotProduct;
		}

		dotProduct = std::max(-1.f, std::min(dotProduct, 1.f));
		double angleBetweenInputs = acosf(dotProduct);
		double angleToResult = angleBetweenInputs*t;

		result = rhs - lhs*dotProduct;
		tyga::unit(result);

		return lhs*cosf(angleToResult) + result*sinf(angleToResult);

	}

	

	



	std::string ToStringVector3(tyga::Vector3 v) {
		return "(X: " + std::to_string(v.x) + " Y: " + std::to_string(v.y) + " Z: " + std::to_string(v.z) + ")";
	}

	std::string ToStringVector4(tyga::Vector4 v) {
		return "(X: " + std::to_string(v.x) + " Y: " + std::to_string(v.y) + " Z: " + std::to_string(v.z) + " W: " + std::to_string(v.w) + ")";
	}

	std::string ToStringMatrix4x4(tyga::Matrix4x4 matrix, bool extendedStats, bool useMatrix)
	{
		std::string output =
			"| " + std::to_string(matrix._00) + ", " + std::to_string(matrix._01) + ", " + std::to_string(matrix._02) + ", " + std::to_string(matrix._03) + " |\n" +
			"| " + std::to_string(matrix._10) + ", " + std::to_string(matrix._11) + ", " + std::to_string(matrix._12) + ", " + std::to_string(matrix._13) + " |\n" +
			"| " + std::to_string(matrix._20) + ", " + std::to_string(matrix._21) + ", " + std::to_string(matrix._22) + ", " + std::to_string(matrix._23) + " |\n" +
			"| " + std::to_string(matrix._30) + ", " + std::to_string(matrix._31) + ", " + std::to_string(matrix._32) + ", " + std::to_string(matrix._33) + " |\n";
		if (extendedStats) {
			output +=
				"| T : " + ToStringVector3(GetTranslationVectorFromMatrix(matrix)) + "\n" +
				"| S : " + ToStringVector3(GetScaleVectorFromMatrix(matrix)) + "\n";
			if (useMatrix) {
				output += "| Rotation Matrix : \n" + ToStringMatrix4x4(GetRotationMatrixFromMatrix(matrix), false) + "\n";
			}
			else {
				output += "| R : " + ToStringVector3(GetEularFromMatrix(matrix)) + "\n";
			}
		}
		return output;
	}

	float LinearTime(float t)
	{
		return t;
	}

	float SmoothStepTime(float t)
	{
		return t*t*t * (t * (6.f*t - 15.f) + 10.f);
	}

	float EaseOutTime(float t) {
		return sinf(t*M_PI*0.5f);
	}

	float EaseInTime(float t) {
		return 1.f - cosf(t*M_PI*0.5f);
	}

	float EaseTime(EaseType type, float time) {
		switch (type)
		{
		case Linear:
			return LinearTime(time);
			break;
		case EaseIn:
			return EaseInTime(time);
			break;
		case EaseOut:
			return EaseOutTime(time);
			break;
		case Smooth:
			return SmoothStepTime(time);
			break;
		default:
			return LinearTime(time);
			break;
		}
	}

	


}