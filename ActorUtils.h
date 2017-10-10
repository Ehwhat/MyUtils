#pragma once
#include <tyga/ActorDelegate.hpp>
#include <tyga/GraphicsCentre.hpp>
#include "HActor.h"
#include <tyga/Math.hpp>
#include <string>

namespace ActorUtils
{
	void AddModelToActor(std::shared_ptr<tyga::ActorDelegate> actor ,std::string modelIdentifier, tyga::Vector3 colour);
	void AddModelToActor(std::shared_ptr<tyga::Actor> actor, std::string modelIdentifier, tyga::Vector3 colour);
	void AddModelToActor(std::shared_ptr<tyga::Actor> actor, std::string modelIdentifier, std::shared_ptr<tyga::GraphicsMaterial> material);
	std::shared_ptr<tyga::Actor> AddActorToWorld();
	std::shared_ptr<tyga::HActor> AddHActorToWorld(bool isRoot = false);

	template <class T>
	std::shared_ptr<T> AddHActorToWorld(bool isRoot);

	std::shared_ptr<tyga::HActor> AddHActorToWorld(std::shared_ptr<tyga::HActor> actor, bool isRoot);

	std::shared_ptr<tyga::HActor> AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, std::string meshName, tyga::Vector3 colour, bool isRoot);

	std::shared_ptr<tyga::HActor> AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, std::string meshName, std::shared_ptr<tyga::GraphicsMaterial> material, bool isRoot);

	std::shared_ptr<tyga::HActor> AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, bool isRoot);

};

