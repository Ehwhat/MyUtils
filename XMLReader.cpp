#include "XMLReader.h"
#include "ActorUtils.h"
#include "MathUtils.h"
#include <iostream>
#include <sstream>

XMLReader::ActorResult XMLReader::ReadAndProduceActor(std::string file, std::string pose) {
	ActorResult actorResult;
	actorResult.rootResult = std::make_shared<tyga::HActor>();
	actorResult.rootResult->name = "Root";

	pugi::xml_document doc;
	actorResult.parseResult = doc.load_file(file.c_str());
	if (actorResult.parseResult) {
		std::cout << "File Found" << std::endl;
	}
	else {
		std::cout << actorResult.parseResult << std::endl;
		return actorResult;
	}

	std::string query = "/entity/pose[@name='" + pose + "']/actor";
	pugi::xpath_node_set actors = doc.select_nodes(query.c_str());
	for (pugi::xpath_node actor : actors) {
		pugi::xml_node actorNode = actor.node();
		auto child = std::make_shared<tyga::HActor>();
		child->SetName(actorNode.attribute("name").value());
		RecursiveLookup(actorNode, child);
		actorResult.rootResult->AddChild(child);
	}

	return actorResult;

}

void XMLReader::RecursiveLookup(pugi::xml_node node, std::shared_ptr<tyga::HActor> parentActor) // We're using a recursive approach to look through the XML document and produce actos based on the hirerarcies described.
{
	auto children = node.children("actor");
	for (auto child : children) {
		auto childActor = std::make_shared<tyga::HActor>();
		childActor->SetName(child.attribute("name").value());
		RecursiveLookup(child, childActor);
		parentActor->AddChild(childActor);
	}
}

XMLReader::PoseResult<XMLReader::PoseData> XMLReader::ReadAndProducePosedActor(std::string file) {
	XMLReader::PoseResult<XMLReader::PoseData> poseResult;
	poseResult.actorResult.rootResult = std::make_shared<tyga::HActor>();
	poseResult.actorResult.rootResult->name = "Root";

	pugi::xml_document doc;
	poseResult.actorResult.parseResult = doc.load_file(file.c_str());
	if (poseResult.actorResult.parseResult) {
		std::cout << "File Found" << std::endl;
	}
	else {
		std::cout << poseResult.actorResult.parseResult << std::endl;
		return poseResult;
	}
	
	bool isBuilt = false;
	auto poses = doc.first_child().children("pose");

	for (auto pose : poses)
	{
		auto actors = pose.children("actor");
		for (auto actor : actors) {
			auto child = std::shared_ptr<tyga::HActor>();
			if (!isBuilt){
				child = std::make_shared<tyga::HActor>();
				child->SetName(actor.attribute("name").value());
				poseResult.actorResult.rootResult->AddChild(child);
			}
			std::string poseName = pose.attribute("name").value();
			auto childPose = GetPoseDataFromActor(actor);
			if (childPose.first) {
				poseResult.poses[poseName].poseData[actor.attribute("name").value()] = childPose.second;
			}
			RecursivePoseLookup(poseResult.poses[poseName], actor, child, isBuilt);
			
		}
		isBuilt = true;
	}

	return poseResult;

}

void XMLReader::RecursivePoseLookup(XMLReader::PoseMap<XMLReader::PoseData>& map, pugi::xml_node node, std::shared_ptr<tyga::HActor> parentActor, bool isBuilt)
{
	auto children = node.children("actor");
	auto childActor = parentActor;
	for (auto child : children) {
		if (!isBuilt) {
			childActor = std::make_shared<tyga::HActor>();
			childActor->SetName(child.attribute("name").value());
			parentActor->AddChild(childActor);
		}
		bool isTransformed = false;
		XMLReader::PoseData data;
		auto childPose = GetPoseDataFromActor(child);
		if (childPose.first) {
			map.poseData[child.attribute("name").value()] = childPose.second;
		}
		RecursivePoseLookup(map, child, childActor, isBuilt);
		
	}
}

std::pair<bool, XMLReader::PoseData> XMLReader::GetPoseDataFromActor(pugi::xml_node actor) {

	std::pair<bool, XMLReader::PoseData> result;

	auto translate = actor.child("translate").text();
	if (translate) {
		result.first = true;
		std::stringstream ss(translate.as_string());
		tyga::Vector3 v;
		ss >> v.x >> v.y >> v.z;
		if (ss) {
			result.second.translate = v;
		}
	}

	auto euler = actor.child("euler").text();
	if (euler) {
		result.first = true;
		std::stringstream ss(euler.as_string());
		tyga::Vector3 v;
		ss >> v.x >> v.y >> v.z;
		if (ss) {
			result.second.eular = v;
		}
	}
	return result;


}

void XMLReader::AddPosedActorToWorldInPose(std::shared_ptr<tyga::HActor> root, std::string pose, std::string modelPrefix, XMLReader::PoseResult<XMLReader::PoseData> poseResult, bool isRoot)
{
	ActorUtils::AddHActorToWorld(root, isRoot);
	auto poseArray = poseResult.poses.find(pose);
	if(poseArray != poseResult.poses.end()){
		auto poseData = poseResult.poses[pose].poseData.find(root->name);
		if (poseData != poseResult.poses[pose].poseData.end()) {
			PositionActorAccordingToPose(root, poseResult.poses[pose].poseData[root->name]);
		}
		ActorUtils::AddModelToHActorAccordingToName(modelPrefix,root);
		for (auto child : root->children)
		{
			AddPosedActorToWorldInPose(child, pose, modelPrefix, poseResult, false);
		}
	}
	
}

void XMLReader::PositionActorAccordingToPose(std::shared_ptr<tyga::HActor> actor, XMLReader::PoseData poseData)
{
	actor->localTransform = MathUtils::GetMatrixFromQuat(MathUtils::GetQuatFromEular(poseData.eular)) * MathUtils::GetMatrixFromTranslationVector(poseData.translate);
}

void XMLReader::LerpActorPoses(std::shared_ptr<tyga::HActor> root, XMLReader::PoseResult<XMLReader::PoseData> poseResult , std::string poseA, std::string poseB, float t) {
	auto poseAArray = poseResult.poses.find(poseA);
	auto poseBArray = poseResult.poses.find(poseB);
	if (poseAArray != poseResult.poses.end() && poseBArray != poseResult.poses.end()) {
		auto poseAData = poseResult.poses[poseA].poseData.find(root->name);
		auto poseBData = poseResult.poses[poseB].poseData.find(root->name);
		PoseData poseAResult, poseBResult;
		if (poseAData != poseResult.poses[poseA].poseData.end()) {
			poseAResult = poseAData->second;
		}
		else {
			poseAResult = PoseData();
		}
		if (poseBData != poseResult.poses[poseB].poseData.end()) {
			poseBResult = poseBData->second;
		}
		else {
			poseBResult = PoseData();
		}

		if (poseBData != poseResult.poses[poseB].poseData.end() && poseAData != poseResult.poses[poseA].poseData.end()) {
			root->localTransform = (MathUtils::GetMatrixFromQuat(
				MathUtils::Slerp(MathUtils::GetQuatFromEular(poseAResult.eular), MathUtils::GetQuatFromEular(poseBResult.eular), t)) *
				MathUtils::GetMatrixFromTranslationVector(MathUtils::Lerp(poseAResult.translate, poseBResult.translate, t)));
		}
		for (auto child : root->children)
		{
			LerpActorPoses(child, poseResult, poseA, poseB, t);
		}
	}
}

void XMLReader::LerpActorPoses(std::shared_ptr<tyga::HActor> root, XMLReader::PoseMap<XMLReader::PoseData> poseMapA, XMLReader::PoseMap<XMLReader::PoseData>  poseMapB, float t) {
	auto poseAData = poseMapA.poseData.find(root->name);
	auto poseBData = poseMapB.poseData.find(root->name);
	PoseData poseAResult, poseBResult;
	if (poseAData != poseMapA.poseData.end()) {
		poseAResult = poseAData->second;
	}
	if (poseBData != poseMapB.poseData.end()) {
		poseBResult = poseBData->second;
	}
	root->localTransform = MathUtils::GetMatrixFromQuat(MathUtils::Slerp(MathUtils::GetQuatFromEular(poseAResult.eular), MathUtils::GetQuatFromEular(poseBResult.eular), t)) *
		MathUtils::GetMatrixFromTranslationVector(MathUtils::Lerp(poseAResult.translate, poseBResult.translate, t));

	for (auto child : root->children)
	{
		LerpActorPoses(child, poseMapA, poseMapB, t);
	}
}

