#pragma once

#include <tyga\Math.hpp>
#include <vector>
#include <string>

enum KeyFrameEasing {
	Linear,
	EaseIn,
	EaseOut,
	Smooth,
	Clamp
};

struct Keyframe {
public:

	tyga::Vector3 position;
	tyga::Quaternion rotation;
	tyga::Vector3 scale;
	KeyFrameEasing ease;

	float time;

	Keyframe() {

	}

	Keyframe(float _time, tyga::Vector3 _position = tyga::Vector3(), tyga::Quaternion _rotation = tyga::Quaternion(), tyga::Vector3 _scale = tyga::Vector3(1,1,1), KeyFrameEasing _ease = KeyFrameEasing::Smooth) {
		time = _time;
		position = _position;
		rotation = _rotation;
		scale = _scale;
		ease = _ease;
	}

	Keyframe CombineKeyframes(Keyframe rhs) {
		Keyframe result = Keyframe(
			time,
			position + rhs.position,
			rotation + rhs.rotation,
			scale + rhs.scale
		);
		return result;
	}

private:
};

class KeyframeAnimation
{
public:

	int keyAmount = 0;
	float endTime = 0;

	KeyframeAnimation();
	~KeyframeAnimation();

	Keyframe AddKeyframe(float _time, tyga::Vector3 _position = tyga::Vector3(), tyga::Quaternion _rotation = tyga::Quaternion(), tyga::Vector3 _scale = tyga::Vector3(), KeyFrameEasing _ease = KeyFrameEasing::Smooth);
	Keyframe AddKeyframe(Keyframe keyframe);

	Keyframe GetInterpolatedKeyframe(float time);
	

	std::string ToString();

private:

	enum KeyComparisonResult {
		Before,
		After,
		SameTime
	};

	std::vector<Keyframe> keyframes;

	void SortAddKeyframe(Keyframe keyframe);
	KeyframeAnimation::KeyComparisonResult isKeyAfterCurrentKey(Keyframe keyframe, Keyframe checkKey);
	void UpdateAnimationDuration();
	bool IsTimeBetweenKeyframes(float time, Keyframe first, Keyframe second);
	float GetPercentageBetweenKeyframes(float time, Keyframe first, Keyframe second);
	Keyframe GenerateInterpolatedKeyframe(float time, Keyframe first, Keyframe second, KeyFrameEasing ease = KeyFrameEasing::Linear);

};

