#include "ActorUtils.h"
#include "MathUtils.h"
#include <tyga/ActorWorld.hpp>
#include <tyga/GraphicsCentre.hpp>
#include <tyga/Actor.hpp>
#include <tyga/BasicWorldClock.hpp>

void ActorUtils::AddModelToActor(std::shared_ptr<tyga::ActorDelegate> actor ,std::string modelIdentifier, tyga::Vector3 colour)
{
	auto graphics = tyga::GraphicsCentre::defaultCentre();

	auto material = graphics->newMaterial();
	material->colour = colour;

	auto mesh = graphics->newMeshWithIdentifier(modelIdentifier);

	auto model = graphics->newModel();
	model->material = material;
	model->mesh = mesh;

	actor->Actor()->attachComponent(model);

}

void ActorUtils::AddModelToActor(std::shared_ptr<tyga::Actor> actor, std::string modelIdentifier, tyga::Vector3 colour)
{
	auto graphics = tyga::GraphicsCentre::defaultCentre();

	auto material = graphics->newMaterial();
	material->colour = colour;

	auto mesh = graphics->newMeshWithIdentifier(modelIdentifier);

	auto model = graphics->newModel();
	model->material = material;
	model->mesh = mesh;

	actor->attachComponent(model);

}

void ActorUtils::AddModelToActor(std::shared_ptr<tyga::Actor> actor, std::string modelIdentifier, std::shared_ptr<tyga::GraphicsMaterial> material)
{
	auto graphics = tyga::GraphicsCentre::defaultCentre();

	auto mesh = graphics->newMeshWithIdentifier(modelIdentifier);

	auto model = graphics->newModel();
	model->material = material;
	model->mesh = mesh;

	actor->attachComponent(model);
}

std::shared_ptr<tyga::Actor> ActorUtils::AddActorToWorld()
{
	auto world = tyga::ActorWorld::defaultWorld();
	auto actor = std::make_shared<tyga::Actor>();
	world->addActor(actor);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(bool isRoot)
{
	auto world = tyga::ActorWorld::defaultWorld();
	auto actor = std::make_shared<tyga::HActor>();
	actor->isRoot = isRoot;
	actor->addToWorld(world);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(std::shared_ptr<tyga::HActor> actor,bool isRoot)
{
	auto world = tyga::ActorWorld::defaultWorld();
	actor->isRoot = isRoot;
	actor->addToWorld(world);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, std::string meshName, tyga::Vector3 colour, bool isRoot)
{
	auto actor = std::make_shared<tyga::HActor>(tyga::HActor());
	actor->SetOffsetTransform(MathUtils::GetMatrixFromEular(rot)*MathUtils::GetMatrixFromTranslationVector(pos));
	AddHActorToWorld(actor, isRoot);
	AddModelToActor(actor->Actor(), meshName, colour);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, std::string meshName, std::shared_ptr<tyga::GraphicsMaterial> material, bool isRoot)
{
	auto actor = std::make_shared<tyga::HActor>(tyga::HActor());
	actor->SetOffsetTransform(MathUtils::GetMatrixFromEular(rot)*MathUtils::GetMatrixFromTranslationVector(pos));
	AddHActorToWorld(actor, isRoot);
	AddModelToActor(actor->Actor(), meshName, material);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, bool isRoot)
{
	auto actor = std::make_shared<tyga::HActor>(tyga::HActor());
	actor->SetOffsetTransform(MathUtils::GetMatrixFromEular(rot)*MathUtils::GetMatrixFromTranslationVector(pos));
	AddHActorToWorld(actor, isRoot);
	return actor;
}
