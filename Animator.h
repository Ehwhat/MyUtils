#pragma once

#include "MathUtils.h"
#include <tyga\Math.hpp>
#include <vector>
#include <string>
#include <functional>

#ifndef ANIMATOR_H
#define ANIMATOR_H

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
template<class T>
class Keyframe {
public:

	float startTime;
	float endTime;
	std::function<void(T object,float time)> keyframeFunction;

	inline Keyframe() {
	}

	inline Keyframe(float _startTime, float _endTime, std::function<void(T object,float time)> function) {
		startTime = _startTime;
		endTime = _endTime;
		keyframeFunction = function;
	}

	inline Keyframe(float _startTime, std::function<void(T object, float time)> function) {
		startTime = _startTime;
		keyframeFunction = function;
	}

	inline void Execute(T object,float t) {
		if (keyframeFunction) {
			keyframeFunction(object, t);
		}
	}

private:


};

template<class OwnerType>
class Animator
{
public:

	int keyAmount = 0;
	float endTime = 0;

	inline Animator(AnimationClampType clamp = AnimationClampType::Single)
	{
		SetClampType(clamp);
	}

	inline ~Animator() {}

	inline Keyframe<OwnerType> AddKeyframe(Keyframe<OwnerType> keyframe)
	{
		SortAddKeyframe(keyframe);
		return keyframe;
	}

	inline Keyframe<OwnerType> AddKeyframe(float startTime, float endTime, std::function<void(OwnerType object, float time)> function, bool isLength = true)
	{
		float end = endTime;
		if (isLength) {
			end = startTime + endTime;
		}
		Keyframe<OwnerType> keyframe = Keyframe<OwnerType>(startTime, end, function);
		SortAddKeyframe(keyframe);
		return keyframe;
	}

	inline Keyframe<OwnerType> AddKeyframe(float startTime, std::function<void(OwnerType object, float time)> function)
	{
		Keyframe<OwnerType> keyframe = Keyframe<OwnerType>(startTime, function);
		SortAddKeyframe(keyframe);
		return keyframe;
	}

	inline void ExecuteAtTime(OwnerType object ,float time)
	{
		if (keyAmount == 0) {
			return;
		}
		time = GetTimeFromClamp(time);

		for (int i = 0; i < keyAmount; i++)
		{
			Keyframe<OwnerType> currentKeyFrame = keyframes[i];
			if (IsTimeWithinKeyframe(time, currentKeyFrame)) {

				float t = GetPercentageWithinKeyframe(time, currentKeyFrame);
				currentKeyFrame.Execute(object,t);
				t = MathUtils::Clamp<float>(t, 0, 1);
			}
		}
	}
	
	inline void SetClampType(AnimationClampType type)
	{
		clampType = type;
	}

	inline std::string ToString()
	{
		std::string result;
		if (keyAmount > 0) {
			for (int i = 0; i < keyAmount; i++)
			{
				Keyframe<OwnerType> currentKeyFrame = keyframes[i];
				result += "ST: " + std::to_string(currentKeyFrame.startTime) + " ET:" + std::to_string(currentKeyFrame.endTime) + "\n";
			}
		}
		else {
			result = "No Keys In Keyframe Animation";
		}
		return result;
	}

private:

	AnimationClampType clampType;

	enum KeyComparisonResult {
		Before,
		After,
		SameTime
	};

	std::vector<Keyframe<OwnerType>> keyframes;

	inline float GetTimeFromClamp(float time)
	{
		switch (clampType)
		{
		case AnimationClampType::Single:
			return MathUtils::Clamp<float>(time, 0, endTime);
		case AnimationClampType::Loop:
			return MathUtils::Clamp<float>(std::fmodf(time, endTime), 0, endTime);
		case AnimationClampType::PingPong:
			time = std::fmodf(time, endTime * 2);
			if (time > endTime) {
				time -= (time - endTime) * 2;
			}
			return MathUtils::Clamp<float>(time, 0, endTime);
		default:
			return MathUtils::Clamp<float>(time, 0, endTime);
		}
	}

	inline void SortAddKeyframe(Keyframe<OwnerType> keyframe)
	{
		keyAmount++;
		keyframes.resize(keyAmount);
		if (keyAmount == 1) {
			keyframes[0] = keyframe;
		}
		else {
			for (int i = 0; i < keyAmount; i++)
			{
				Keyframe<OwnerType> currentKeyFrame = keyframes[i];
				KeyComparisonResult result = isKeyAfterCurrentKey(keyframe, currentKeyFrame);
				if (result != KeyComparisonResult::After) {
					if (result == KeyComparisonResult::Before && keyframe.endTime == 0) {
						keyframe.endTime = currentKeyFrame.startTime;
					}
					else if (keyframe.endTime == 0) {
						keyframe.endTime = keyframe.startTime + 5;
					}
					keyframes.insert(keyframes.begin() + i, keyframe);
					return;
				}
			}
			keyframes[keyAmount - 1] = keyframe;
		}
		endTime = keyframe.endTime;
	}

	inline KeyComparisonResult isKeyAfterCurrentKey(Keyframe<OwnerType> keyframe, Keyframe<OwnerType> checkKey)
	{
		if (keyframe.startTime > checkKey.startTime) {
			return KeyComparisonResult::After;
		}
		else if (keyframe.startTime == checkKey.startTime) {
			return KeyComparisonResult::SameTime;
		}
		return KeyComparisonResult::Before;
	}

	inline bool IsTimeWithinKeyframe(float time, Keyframe<OwnerType> keyframe)
	{
		return (time > keyframe.startTime) && (time <= keyframe.endTime);
	}

	inline float GetPercentageWithinKeyframe(float time, Keyframe<OwnerType> keyframe)
	{
		return (time - keyframe.startTime) / (keyframe.endTime - keyframe.startTime);
	}
	//Keyframe GenerateInterpolatedKeyframe(float time, Keyframe first, Keyframe second, KeyFrameEasing ease = KeyFrameEasing::Linear);

};





#endif
