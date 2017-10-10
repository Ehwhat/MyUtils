#include "HActor.h"
#include <tyga/ActorWorld.hpp>
#include <tyga/Actor.hpp>

namespace tyga {

	HActor::HActor(bool _isRoot) : isRoot(_isRoot) {
		
	}
	HActor::~HActor(){}

	void tyga::HActor::actorClockTick(std::shared_ptr<tyga::Actor> actor) {
		if (isRoot) {
			hActorClockTick(_this);
			Matrix4x4 resultingTransfom = GetResultingTransform();
			this->Actor()->setTransformation(resultingTransfom);
		}
		Matrix4x4 resultingTransfom = GetResultingTransform();
		applyTransformToChildren(resultingTransfom);
	}

	void tyga::HActor::actorDidEnterWorld(std::shared_ptr<tyga::Actor> actor)
	{
		_this = std::shared_ptr<tyga::HActor>(this);
		hActorDidEnterWorld(_this);
	}

	void tyga::HActor::actorWillLeaveWorld(std::shared_ptr<tyga::Actor> actor)
	{
		hActorWillLeaveWorld(_this);
	}

	void tyga::HActor::AddChild(std::shared_ptr<tyga::HActor> actor) {
		children.push_back(actor);
	}

	tyga::Matrix4x4 HActor::GetResultingTransform()
	{
		return localTransform*offsetTransform*globalTransform;
	}

	void tyga::HActor::SetTransform(Matrix4x4 transform)
	{
		localTransform = transform;
	}

	void HActor::SetOffsetTransform(Matrix4x4 transform)
	{
		offsetTransform = transform;
	}

	void HActor::AttachComponent(std::shared_ptr<ActorComponent> c)
	{
		_this->Actor()->attachComponent(c);
	}

	void HActor::DetachComponent(std::shared_ptr<ActorComponent> c)
	{
		_this->Actor()->detachComponent(c);
	}

	void tyga::HActor::applyTransformToChildren(Matrix4x4 transform) {
		for (std::shared_ptr<tyga::HActor> child : children) {
			child->globalTransform = transform;
			child->hActorClockTick(child);
			tyga::Matrix4x4 result = child->GetResultingTransform();
			child->Actor()->setTransformation(result);
		}
	}

}