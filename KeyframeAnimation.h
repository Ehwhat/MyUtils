#pragma once

#include "MathUtils.h"
#include <tyga\Math.hpp>
#include <vector>
#include <string>
#include <functional>

/**
*	Keyframe Animation system, developed by Joshua Stevenson (Q5120984), 2017
*	Allows for animation of scalar values over a variable amount of keyframes using AnimationCurves for easing
*	Can also use clamping styles for looping or 'PingPong' animations
*	Can use basic event system for more complex animations (Still working on this so don't expect any wonders yet)
*
*	TODO:
*	Develop event system to be more workable
*	Maybe rewrite curve calulation to store each keyframes in and out tangents and calulate curves from that?
*/

enum AnimationClampType {
	Single,
	Loop,
	PingPong
};
/**
*	Keyframe class, stores default scalars, the animation curve assoisated with it and animation events
*/
class Keyframe {
public:

	tyga::Vector3 position;
	tyga::Quaternion rotation;
	tyga::Vector3 scale;
	MathUtils::AnimationCurve animationCurve = MathUtils::linearCurve;
	
	//KeyFrameEasing ease;

	float time;

	Keyframe() {
	}

	Keyframe(float _time, MathUtils::AnimationCurve curve = MathUtils::linearCurve, tyga::Vector3 _position = tyga::Vector3(), tyga::Quaternion _rotation = tyga::Quaternion(), tyga::Vector3 _scale = tyga::Vector3(1,1,1)) {
		time = _time;
		position = _position;
		rotation = _rotation;
		scale = _scale;
		animationCurve = curve;
	}

	void AddSetKeyframeFunction(std::function<void(float)> function) {
		keyframeFunction = function;
	}

	void KeyframeFunction(float t) {
		if (keyframeFunction) {
			keyframeFunction(t);
		}
	}

	Keyframe CombineKeyframes(Keyframe rhs) {
		Keyframe result = Keyframe(
			time,
			animationCurve = this->animationCurve,
			position + rhs.position,
			rotation + rhs.rotation,
			scale + rhs.scale
		);
		return result;
	}

private:

	std::function<void(float time)> keyframeFunction;

};

class KeyframeAnimation
{
public:

	int keyAmount = 0;
	float endTime = 0;

	KeyframeAnimation(AnimationClampType clamp = AnimationClampType::Single);
	~KeyframeAnimation();

	Keyframe AddKeyframe(float _time, MathUtils::AnimationCurve _curve = MathUtils::linearCurve, tyga::Vector3 _position = tyga::Vector3(), tyga::Quaternion _rotation = tyga::Quaternion(), tyga::Vector3 _scale = tyga::Vector3());
	Keyframe AddKeyframe(Keyframe keyframe, std::function<void(float time)> keyFunction = nullptr);

	Keyframe GetInterpolatedKeyframe(float time);
	
	void SetClampType(AnimationClampType type);

	std::string ToString();

private:

	AnimationClampType clampType;

	enum KeyComparisonResult {
		Before,
		After,
		SameTime
	};

	std::vector<Keyframe> keyframes;

	float GetTimeFromClamp(float time);
	void SortAddKeyframe(Keyframe keyframe);
	KeyframeAnimation::KeyComparisonResult isKeyAfterCurrentKey(Keyframe keyframe, Keyframe checkKey);
	void UpdateAnimationDuration();
	bool IsTimeBetweenKeyframes(float time, Keyframe first, Keyframe second);
	float GetPercentageBetweenKeyframes(float time, Keyframe first, Keyframe second);
	Keyframe GenerateInterpolatedKeyframe(float time, Keyframe first, Keyframe second, MathUtils::AnimationCurve curve);
	//Keyframe GenerateInterpolatedKeyframe(float time, Keyframe first, Keyframe second, KeyFrameEasing ease = KeyFrameEasing::Linear);

};

