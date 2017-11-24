#include "ActorUtils.h"
#include "MathUtils.h"
#include <tyga/ActorWorld.hpp>
#include <tyga/Actor.hpp>
#include <tyga/BasicWorldClock.hpp>
#include <iostream>

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

void ActorUtils::AddModelToActor(std::shared_ptr<tyga::Actor> actor, std::shared_ptr<tyga::GraphicsMesh> mesh, std::shared_ptr<tyga::GraphicsMaterial> material)
{
	auto graphics = tyga::GraphicsCentre::defaultCentre();
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

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, std::string meshName, tyga::Vector3 colour, bool isRoot, std::string name)
{
	auto actor = std::make_shared<tyga::HActor>(tyga::HActor());
	actor->SetOffsetTransform(MathUtils::GetMatrixFromEular(rot)*MathUtils::GetMatrixFromTranslationVector(pos));
	actor->SetName(name);
	AddHActorToWorld(actor, isRoot);
	AddModelToActor(actor->Actor(), meshName, colour);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, std::string meshName, std::shared_ptr<tyga::GraphicsMaterial> material, bool isRoot, std::string name)
{
	auto actor = std::make_shared<tyga::HActor>(tyga::HActor());
	actor->SetOffsetTransform(MathUtils::GetMatrixFromEular(rot)*MathUtils::GetMatrixFromTranslationVector(pos));
	actor->SetName(name);
	AddHActorToWorld(actor, isRoot);
	AddModelToActor(actor->Actor(), meshName, material);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, std::shared_ptr<tyga::GraphicsMesh> mesh, std::shared_ptr<tyga::GraphicsMaterial> material, bool isRoot, std::string name)
{
	auto actor = std::make_shared<tyga::HActor>(tyga::HActor());
	actor->SetOffsetTransform(MathUtils::GetMatrixFromEular(rot)*MathUtils::GetMatrixFromTranslationVector(pos));
	actor->SetName(name);
	AddHActorToWorld(actor, isRoot);
	AddModelToActor(actor->Actor(), mesh, material);
	return actor;
}

std::shared_ptr<tyga::HActor> ActorUtils::AddHActorToWorld(tyga::Vector3 pos, tyga::Vector3 rot, bool isRoot, std::string name)
{
	auto actor = std::make_shared<tyga::HActor>(tyga::HActor());
	actor->SetOffsetTransform(MathUtils::GetMatrixFromEular(rot)*MathUtils::GetMatrixFromTranslationVector(pos));
	actor->SetName(name);
	AddHActorToWorld(actor, isRoot);
	return actor;
}

void ActorUtils::AddModelToHActorAccordingToName(std::string prefix, std::shared_ptr<tyga::HActor> actor)
{
	auto graphics = tyga::GraphicsCentre::defaultCentre();
	auto model = graphics->newModel();
	model->material = graphics->newMaterial();
	model->material->texture = actor->name;
	model->mesh = graphics->newMeshWithIdentifier(prefix + actor->name);

	actor->Actor()->attachComponent(model);

}

std::shared_ptr<tyga::GraphicsMesh> ActorUtils::GraphicsShare::GetMeshFromIdentifier(std::string name)
{
	auto search = _sharedMeshes.find(name);
	if (search == _sharedMeshes.end()) {
		//Not found a mesh
		auto graphics = tyga::GraphicsCentre::defaultCentre();
		std::shared_ptr<tyga::GraphicsMesh> mesh = graphics->newMeshWithIdentifier(name);
		_sharedMeshes.emplace(name, mesh);
		return mesh;
	}
	return search->second;
}

std::shared_ptr<tyga::GraphicsMaterial> ActorUtils::GraphicsShare::GetMaterial(std::string name, tyga::Vector3 colour)
{
	auto search = _sharedMaterials.find(name);
	if (search == _sharedMaterials.end()) {
		//Not found a material
		auto graphics = tyga::GraphicsCentre::defaultCentre();
		std::shared_ptr<tyga::GraphicsMaterial> material = graphics->newMaterial();
		material->colour = colour;
		_sharedMaterials.emplace(name, material);
		return material;
	}
	return search->second;
}

std::shared_ptr<tyga::GraphicsMaterial> ActorUtils::GraphicsShare::GetMaterial(std::string name, std::string materialName)
{
	auto search = _sharedMaterials.find(name);
	if (search == _sharedMaterials.end()) {
		//Not found a material
		auto graphics = tyga::GraphicsCentre::defaultCentre();
		std::shared_ptr<tyga::GraphicsMaterial> material = graphics->newMaterial();
		material->texture = materialName;
		_sharedMaterials.emplace(name, material);
		return material;
	}
	return search->second;
	return std::shared_ptr<tyga::GraphicsMaterial>();
}
