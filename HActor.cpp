#include "HActor.h"
#include <tyga/ActorWorld.hpp>
#include <tyga/Actor.hpp>
#include <string>

namespace tyga {

	HActor::HActor(bool _isRoot) : isRoot(_isRoot) {
		_this = std::shared_ptr<tyga::HActor>(this, [](tyga::HActor* p) {}); // Honestly this is a travisty, and needs a major revision to not allow this to happen.
	}
	HActor::~HActor(){}

	void tyga::HActor::actorClockTick(std::shared_ptr<tyga::Actor> actor) {
		if (isRoot) {
			hActorClockTick(_this);
			Matrix4x4 resultingTransfom = GetResultingTransform();
			this->Actor()->setTransformation(resultingTransfom);
			
			hActorLateClockTick(_this);
			applyTransformToChildren(resultingTransfom);
		}
		//Matrix4x4 resultingTransfom = GetResultingTransform();
		
	}

	void tyga::HActor::actorDidEnterWorld(std::shared_ptr<tyga::Actor> actor)
	{
		hActorDidEnterWorld(_this);
	}

	void tyga::HActor::actorWillLeaveWorld(std::shared_ptr<tyga::Actor> actor)
	{
		hActorWillLeaveWorld(_this);
	}

	void HActor::SetName(std::string _name)
	{
		name = _name;
	}

	void tyga::HActor::AddChild(std::shared_ptr<tyga::HActor> actor) {
		actor->_parent = _this;
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

	std::string HActor::PrintChildren()
	{
		return PrintChildren(_this, 0);
	}

	std::string HActor::PrintChildren(std::shared_ptr<tyga::HActor> actor, int indent)
	{
		std::string output = std::string(indent*3, '-') + "| " + actor->name + "\n";
		for (auto child : actor->children)
		{
			output += PrintChildren(child, indent + 1);
		}
		return output;
	}


	void tyga::HActor::applyTransformToChildren(Matrix4x4 transform) {
		for (std::shared_ptr<tyga::HActor> child : children) {
			child->globalTransform = transform;
			child->hActorClockTick(child);
			tyga::Matrix4x4 result = child->GetResultingTransform();
			child->Actor()->setTransformation(result);
			child->applyTransformToChildren(result);
		}
	}

}