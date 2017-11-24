#pragma once

#include <tyga\ActorDelegate.hpp>
#include <tyga\Math.hpp>
#include <vector>
#include <unordered_set>
#include <tyga\Actor.hpp>

/**
*	HActor, extending the ActorDelegate class included in tyga, exteneded by Joshua Stevenson (Q5120984), 2017
*	HActor is short for hierarchical actor, and as such includes the process of automatically transforming a parents children and making all their transformations relative to their parents 
*/

namespace tyga {

	class HActor : public tyga::ActorDelegate
	{
	public:
		bool isRoot = false;

		tyga::Matrix4x4 localTransform = tyga::Matrix4x4();
		tyga::Matrix4x4 globalTransform = tyga::Matrix4x4();
		tyga::Matrix4x4 offsetTransform = tyga::Matrix4x4(); 
		// Having an offset transformation along with a local transformation is a little bit odd as it's a weird line in the sand to draw, expecially after my implimentation of layers in the AnimationStateMachine
		// Maybe having a vector of transformations to allow for flexibllity would be better?

		std::vector<std::shared_ptr<tyga::HActor>> children;
		std::string name = "Unnamed HActor";

		HActor(bool _isRoot = false);
		~HActor();

		//Custom wrapper functions for ActorDelegate functions, which allow the HActor to do operations before and after custom behaviours 

		virtual void hActorDidEnterWorld(std::shared_ptr<tyga::HActor> actor) {}

		virtual void hActorWillLeaveWorld(std::shared_ptr<tyga::HActor> actor) {}

		virtual void hActorClockTick(std::shared_ptr<tyga::HActor> actor) {}

		virtual void hActorLateClockTick(std::shared_ptr<tyga::HActor> actor){}


		void SetName(std::string _name);

		void AddChild(std::shared_ptr<tyga::HActor> hActor);

		void RemoveChild(std::shared_ptr<tyga::HActor> hActor);

		tyga::Matrix4x4 GetResultingTransform();

		void SetTransform(Matrix4x4 transform);

		void SetOffsetTransform(Matrix4x4 transform);

		void AttachComponent(std::shared_ptr<ActorComponent> c);

		void DetachComponent(std::shared_ptr<ActorComponent> c);

		std::string PrintChildren(); // returns a string of a representation of all children, I really should rename this
		

	private:

		std::shared_ptr<tyga::HActor> _parent;
		std::shared_ptr<tyga::HActor> _this;

		virtual void actorClockTick(std::shared_ptr<tyga::Actor> actor) override;

		virtual void actorDidEnterWorld(std::shared_ptr<tyga::Actor> actor) override;

		virtual void actorWillLeaveWorld(std::shared_ptr<tyga::Actor> actor) override;

		void applyTransformToChildren(Matrix4x4 transform);

		std::string PrintChildren(std::shared_ptr<tyga::HActor> actor, int indent);

	};
}

