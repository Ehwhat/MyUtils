#include "KeyframeAnimation.h"
#include "MathUtils.h"


KeyframeAnimation::KeyframeAnimation()
{
	keyframes.resize(1);
	keyframes[0] = Keyframe();
	keyAmount = 1;
}


KeyframeAnimation::~KeyframeAnimation()
{
}

Keyframe KeyframeAnimation::AddKeyframe(float _time, tyga::Vector3 _position, tyga::Quaternion _rotation, tyga::Vector3 _scale, KeyFrameEasing _ease)
{
	Keyframe keyframe = Keyframe(_time, _position, _rotation, _scale, _ease);
	SortAddKeyframe(keyframe);
	return keyframe;
}

Keyframe KeyframeAnimation::AddKeyframe(Keyframe keyframe)
{
	SortAddKeyframe(keyframe);
	return keyframe;
}

Keyframe KeyframeAnimation::GetInterpolatedKeyframe(float time)
{
	if (keyAmount == 0) {
		return Keyframe();
	}
	if (keyAmount < 2 || time < 0) {
		return keyframes[0];
	}
	if (time > endTime) {
		return keyframes[keyAmount-1];
	}
	for (int i = 0; i < keyAmount-1; i++)
	{
		Keyframe currentKeyFrame = keyframes[i];
		Keyframe nextKeyFrame = keyframes[i + 1];
		if (IsTimeBetweenKeyframes(time,currentKeyFrame, nextKeyFrame)) {
			float t = GetPercentageBetweenKeyframes(time, currentKeyFrame, nextKeyFrame);
			Keyframe result = GenerateInterpolatedKeyframe(t, currentKeyFrame, nextKeyFrame, currentKeyFrame.ease);
			return result;
		}
	}
	return Keyframe();
}

std::string KeyframeAnimation::ToString()
{
	std::string result;
	if (keyAmount > 0) {
		for (int i = 0; i < keyAmount; i++)
		{
			Keyframe currentKeyFrame = keyframes[i];
			result += "T: " + std::to_string(currentKeyFrame.time) + " P: " + MathUtils::ToStringVector3(currentKeyFrame.position) + " R: " + " S: " + MathUtils::ToStringVector3(currentKeyFrame.scale) + "\n";
		}
	}
	else {
		result = "No Keys In Keyframe Animation";
	}
	return result;
}

void KeyframeAnimation::SortAddKeyframe(Keyframe keyframe)
{
	keyAmount++;
	keyframes.resize(keyAmount);
	if (keyAmount == 1) {
		keyframes[0] = keyframe;
	}
	else {
		for (int i = 0; i < keyAmount; i++)
		{
			Keyframe currentKeyFrame = keyframes[i];
			KeyComparisonResult result = isKeyAfterCurrentKey(keyframe, currentKeyFrame);
			if (result == KeyComparisonResult::SameTime) {
				keyframes[i].CombineKeyframes(keyframe);
			}else if(result == KeyComparisonResult::Before){
				keyframes.insert(keyframes.begin()+i, keyframe);
				return;
			}
		}
		keyframes[keyAmount-1] = keyframe;
		endTime = keyframe.time;
	}
}

KeyframeAnimation::KeyComparisonResult KeyframeAnimation::isKeyAfterCurrentKey(Keyframe keyframe, Keyframe checkKey)
{
	if (keyframe.time > checkKey.time) {
		return KeyComparisonResult::After;
	}
	else if (keyframe.time == checkKey.time) {
		return KeyComparisonResult::SameTime;
	}
	return KeyComparisonResult::Before;
}

void KeyframeAnimation::UpdateAnimationDuration()
{
}

bool KeyframeAnimation::IsTimeBetweenKeyframes(float time, Keyframe first, Keyframe second)
{
	return (time > first.time) && (time < second.time);
}

float KeyframeAnimation::GetPercentageBetweenKeyframes(float time, Keyframe first, Keyframe second)
{
	return (time-first.time)/(second.time-first.time);
}

Keyframe KeyframeAnimation::GenerateInterpolatedKeyframe(float time, Keyframe first, Keyframe second, KeyFrameEasing ease)
{
	Keyframe result;
	switch (ease)
	{
	case Linear:
		result = Keyframe(
			time,
			MathUtils::Lerp<tyga::Vector3>(first.position, second.position, time, MathUtils::EaseType::Linear),
			MathUtils::Slerp(first.rotation, second.rotation, time, MathUtils::EaseType::Linear),
			MathUtils::Lerp<tyga::Vector3>(first.scale, second.scale, time, MathUtils::EaseType::Linear)
		);
		break;
	case EaseIn:
		result = Keyframe(
			time,
			MathUtils::Lerp<tyga::Vector3>(first.position, second.position, time, MathUtils::EaseType::EaseIn),
			MathUtils::Slerp(first.rotation, second.rotation, time, MathUtils::EaseType::EaseIn),
			MathUtils::Lerp<tyga::Vector3>(first.scale, second.scale, time, MathUtils::EaseType::EaseIn)
		);
		break;
	case EaseOut:
		result = Keyframe(
			time,
			MathUtils::Lerp<tyga::Vector3>(first.position, second.position, time, MathUtils::EaseType::EaseOut),
			MathUtils::Slerp(first.rotation, second.rotation, time, MathUtils::EaseType::EaseOut),
			MathUtils::Lerp<tyga::Vector3>(first.scale, second.scale, time, MathUtils::EaseType::EaseOut)
		);
		break;
	case Smooth:
		result = Keyframe(
			time,
			MathUtils::Lerp<tyga::Vector3>(first.position, second.position, time, MathUtils::EaseType::Smooth),
			MathUtils::Slerp(first.rotation, second.rotation, time, MathUtils::EaseType::Smooth),
			MathUtils::Lerp<tyga::Vector3>(first.scale, second.scale, time, MathUtils::EaseType::Smooth)
		);
		break;
	case Clamp:
		result = Keyframe(
		time,
		first.position,
		first.rotation,
		first.scale
		);
		break;
	default:
		result = Keyframe(
			time,
			MathUtils::Lerp<tyga::Vector3>(first.position, second.position, time, MathUtils::EaseType::Linear),
			MathUtils::Slerp(first.rotation, second.rotation, time, MathUtils::EaseType::Linear),
			MathUtils::Lerp<tyga::Vector3>(first.scale, second.scale, time, MathUtils::EaseType::Linear)
		);
		break;
	}
	return result;

}


