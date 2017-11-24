#include "MathUtils.h"
#include <cmath>
#include <algorithm>

namespace MathUtils {

	float MilesPerHourToMetersPerMinute(float miles)
	{
		return miles * 1609; // Super magic number is conversion rate, don't worry about it
	}

	float MPHtoRotationsPerMinute(float miles, float radius)
	{
		return MilesPerHourToMetersPerMinute(miles) / (radius* _pi * 2);
	}

	float QuatDotProduct(tyga::Quaternion lhs, tyga::Quaternion rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
	}

	tyga::Vector3 SphericalPositionToVectorPosition(float theta, float axion, float radius)
	{
		return tyga::Vector3(
			radius * sin(theta)*cos(axion),
			radius * sin(theta)*sin(axion),
			radius * cos(theta)
		);
	}

	float GetSpeedFromVelocity(tyga::Vector3 velocity){
		return tyga::length(velocity);
	}
	float GetDistanceBetweenVectors(tyga::Vector3 a, tyga::Vector3 b)
	{
		return tyga::length(a - b);
	}
	float GetAngleBetweenVectors(tyga::Vector3 a, tyga::Vector3 b)
	{
		float dot = tyga::dot(a, b)/(tyga::length(a) * tyga::length(b));
		return std::acosf(dot);
	}

	tyga::Vector3 GetAxisOfRotationBetweenVectors(tyga::Vector3 a, tyga::Vector3 b) {
		return tyga::cross(a, b);
	}

	float GetDistanceOfPointPerpendicularToPlane(tyga::Vector3 point, Plane plane)
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
		float sx = tyga::length(tyga::Vector3(m._00, m._10, m._20));
		float sy = tyga::length(tyga::Vector3(m._01, m._11, m._21));
		float sz = tyga::length(tyga::Vector3(m._02, m._12, m._22));
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
		float sy = std::sqrt(m._00*m._00 + m._10 * m._10);
		bool isSingular = sy <  1e-6;
		float x, y, z;
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
		eular *= _pi / 180; // Degrees to radians
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
		return z*x*y;
	}

	tyga::Matrix4x4 GetMatrixFromQuat(tyga::Quaternion q)
	{
		return tyga::Matrix4x4(1 - 2 * (q.y*q.y) - 2 * (q.z*q.z), 2 * q.x*q.y + 2 * q.z*q.w, 2 * q.x*q.z - 2 * q.y*q.w, 0,
			2 * q.x*q.y - 2 * q.z*q.w, 1 - 2 * (q.x*q.x) - 2 * (q.z*q.z), 2 * q.y*q.z + 2 * q.x*q.w, 0,
			2 * q.x*q.z + 2 * q.y*q.w, 2 * q.y*q.z - 2 * q.x*q.w, 1 - 2 * (q.x*q.x) - 2 * (q.y*q.y), 0,
			0, 0, 0, 1);
	}

	tyga::Quaternion GetQuatFromMatrix(tyga::Matrix4x4 m) //Here we go..
	{
		tyga::Quaternion result;
		float diagonal = m._00 = m._11 + m._22; //Need to test so we can avoid division errors
		if (diagonal > 0) {
			float s = 0.5f / sqrtf(diagonal + 1.0f);
			result.w = 0.25f / s;
			result.x = (m._21 - m._12) * s;
			result.y = (m._02 - m._20) * s;
			result.z = (m._10 - m._01) * s;
		}
		else {
			if (m._00 > m._11 && m._00 > m._22) {
				float s = 2.0f * sqrtf(1.0f + m._00 - m._11 - m._22);
				result.w = (m._21 - m._12) / s;
				result.x = 0.25f * s;
				result.y = (m._01 + m._10) / s;
				result.z = (m._02 + m._20) / s;
			}
			else if (m._11 > m._22) {
				float s = 2.0f * sqrtf(1.0f + m._11 - m._00 - m._22);
				result.w = (m._02 - m._20) / s;
				result.x = (m._01 + m._10) / s;
				result.y = 0.25f * s;
				result.z = (m._12 + m._21) / s;
			}
			else {
				float s = 2.0f * sqrtf(1.0f + m._22 - m._00 - m._11);
				result.w = (m._10 - m._01) / s;
				result.x = (m._02 + m._20) / s;
				result.y = (m._12 + m._21) / s;
				result.z = 0.25f * s;
			}
		}
		return result;
	}

	tyga::Quaternion GetQuatFromEular(tyga::Vector3 v)
	{
		tyga::Quaternion result;

		v *= _pi / 180;

		float cz = cosf(v.z * 0.5f);
		float sz = sinf(v.z * 0.5f);
		float cy = cosf(v.y * 0.5f);
		float sy = sinf(v.y * 0.5f);
		float cx = cosf(v.x * 0.5f);
		float sx = sinf(v.x * 0.5f);

		result.w = cz * cx * cy + sz * sx * sy;
		result.x = cz * sx * cy + sz * cx * sy;
		result.y = cz * cx * sy + sz * sx * cy;
		result.z = sz * cx * cy + cz * sx * sy;

		return result;

	}

	//tyga::Quaternion getQuatFromDirection(tyga::Vector3 v, tyga::Vector3 up)
	//{
	//	return tyga::Quaternion();
	//}

	tyga::Quaternion getQuatFromDirection(tyga::Vector3 v, tyga::Vector3 up)
	{
		float d = tyga::dot(v, up);
		tyga::Vector3 axis = tyga::cross(v, up);
		float qw = sqrtf((tyga::length(v)*tyga::length(v))*(tyga::length(up)*tyga::length(up))) + d;
		if (qw < 0.0001) { // vectors are 180 degrees apart
			return tyga::unit(tyga::Quaternion(0, -v.z, v.y, v.x));
		}
		return tyga::unit(tyga::Quaternion(qw, axis.x, axis.y, axis.z));
	}

	tyga::Matrix4x4 getMatrixFromDirection(tyga::Vector3 v, tyga::Vector3 up)
	{
		const tyga::Vector3 W = tyga::unit(v);
		const tyga::Vector3 U = tyga::cross(up, W);
		const tyga::Vector3 V = tyga::cross(U, W);
		return tyga::Matrix4x4(U.x, U.y, U.z, 0,
			V.x, V.y, V.z, 0,
			W.x, W.y, W.z, 0,
			0, 0, 0, 1);

	}

	tyga::Vector3 RotateVectorByQuat(tyga::Vector3 v, tyga::Quaternion q)
	{
		tyga::Vector3 qv = tyga::Vector3(q.x, q.y, q.z);
		float scalar = q.w;

		return 2.0f * tyga::dot(qv, v) * qv + (scalar*scalar - tyga::dot(qv, qv)) * v + 2.0f * scalar * tyga::cross(qv, v);
	}

	tyga::Vector3 GetForwardVectorFromMatrix(tyga::Matrix4x4 m)
	{
		return -tyga::Vector3(m._02,m._12,m._22);
	}

	tyga::Vector3 GetRightVectorFromMatrix(tyga::Matrix4x4 m)
	{
		return tyga::Vector3(m._00, m._10, m._20);
	}

	tyga::Vector3 GetUpVectorFromMatrix(tyga::Matrix4x4 m)
	{
		return tyga::Vector3(m._01, m._11, m._21);
	}

	tyga::Matrix4x4 CombineMatrices(tyga::Matrix4x4 first, tyga::Matrix4x4 second)
	{
		return first * second;
	}

	tyga::Vector3 MultiplyVector3(tyga::Vector3 lhs, tyga::Vector3 rhs)
	{
		return tyga::Vector3(lhs.x*rhs.x, lhs.y*rhs.y, lhs.z*lhs.z);
	}

	tyga::Matrix4x4 frenet(const tyga::Vector3 pos, const tyga::Vector3 dir, const tyga::Vector3 up)
	{
		const tyga::Vector3 W = tyga::unit(dir);
		const tyga::Vector3 U = tyga::cross(up, W);
		const tyga::Vector3 V = tyga::cross(W, U);
		return tyga::Matrix4x4(U.x, U.y, U.z, 0,
			V.x, V.y, V.z, 0,
			W.x, W.y, W.z, 0,
			pos.x, pos.y, pos.z, 1);
	}

	bool IsInsideTriangle(tyga::Vector2 point, Triangle tri)
	{
		float area = 0.5f *(-tri.p1.y*tri.p2.x + tri.p0.y*(-tri.p1.x + tri.p2.x) + tri.p0.x*(tri.p1.y - tri.p2.y) + tri.p1.x*tri.p2.y);
		float s = 1 / (2 * area)*(tri.p0.y*tri.p2.x - tri.p0.x*tri.p2.y + (tri.p2.y - tri.p0.y)*point.x + (tri.p0.x - tri.p2.x)*point.y);
		float t = 1 / (2 * area)*(tri.p0.x*tri.p1.y - tri.p0.y*tri.p1.x + (tri.p0.y - tri.p1.y)*point.x + (tri.p1.x - tri.p0.x)*point.y);
		return (s > 0 && t > 0 && (1 - s - t) > 0);
	}

	float Dot(tyga::Quaternion lhs, tyga::Quaternion rhs) {
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
	}

	tyga::Quaternion Slerp(tyga::Quaternion lhs, tyga::Quaternion rhs, float t, MathUtils::AnimationCurve animationCurve) {
		t = animationCurve.GetAnimationCurveOutput(t);
		return Slerp(lhs, rhs, t);
	}

	tyga::Quaternion Slerp(tyga::Quaternion lhs, tyga::Quaternion rhs, float t)
	{
		tyga::Quaternion result;
		float dot = Dot(lhs, rhs);

		if (dot < 0)
		{
			dot = -dot;
			result = -rhs;
		}
		else {
			result = rhs;
		}

		if (dot < 0.95f)
		{
			float angle = acosf(dot);
			return (lhs*sinf(angle*(1 - t)) + result*sinf(angle*t)) / sinf(angle);
		}
		else
		{ // if the angle is small, use linear interpolation								
			return Lerp(lhs, result, t);
		}
	}
	
	float SinWave(float frequency, float amplitude, float phase, float inital, float time)
	{
		return inital + sinf(2*_pi*frequency*time + phase)*amplitude;
	}
	

	std::string ToStringVector2(tyga::Vector2 v)
	{
		return "(X: " + std::to_string(v.x) + " Y: " + std::to_string(v.y) + ")";
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
}