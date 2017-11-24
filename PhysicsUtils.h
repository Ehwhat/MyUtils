#pragma once
namespace PhysicsUtils
{
	template<class T>
	void Eular(T &currentValue, T derivative, float h) {
		currentValue = currentValue + (h*derivative);
	}
};

