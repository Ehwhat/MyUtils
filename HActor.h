#pragma once

#include <tyga\ActorDelegate.hpp>
#include <tyga\Math.hpp>
#include <vector>
#include <unordered_set>
#include <tyga\Actor.hpp>

namespace tyga {

	class HActor : public tyga::ActorDelegate 
	{
	public:

		bool isRoot = false;
		tyga::Matrix4x4 localTransform = tyga::Matrix4x4();
		tyga::Matrix4x4 globalTransform = tyga::Matrix4x4();
		tyga::Matrix4x4 offsetTransform = tyga::Matrix4x4();

		std::vector<std::shared_ptr<tyga::HActor>> children;

		HActor(bool _isRoot = false);
		~HActor();

		virtual void hActorDidEnterWorld(std::shared_ptr<tyga::HActor> actor) {}

		virtual void hActorWillLeaveWorld(std::shared_ptr<tyga::HActor> actor) {}

		virtual void hActorClockTick(std::shared_ptr<tyga::HActor> actor) {}

		void AddChild(std::shared_ptr<tyga::HActor> hActor);

		void RemoveChild(std::shared_ptr<tyga::HActor> hActor);

		tyga::Matrix4x4 GetResultingTransform();

		void SetTransform(Matrix4x4 transform);

		void SetOffsetTransform(Matrix4x4 transform);

		void AttachComponent(std::shared_ptr<ActorComponent> c);

		void DetachComponent(std::shared_ptr<ActorComponent> c);

	private:


		std::shared_ptr<tyga::HActor> _this;

		virtual void actorClockTick(std::shared_ptr<tyga::Actor> actor) override;

		virtual void actorDidEnterWorld(std::shared_ptr<tyga::Actor> actor) override;

		virtual void actorWillLeaveWorld(std::shared_ptr<tyga::Actor> actor) override;

		void applyTransform();

		void applyTransformToChildren(Matrix4x4 transform);

	};
}

