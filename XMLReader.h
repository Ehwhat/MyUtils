#pragma once
#include "HActor.h"

#include <pugixml\pugixml.hpp>
#include <string>
#include <unordered_map>

namespace XMLReader
{
	struct PoseData {
	public:
		tyga::Vector3 translate;
		tyga::Vector3 eular;
		
		PoseData() {}

	};

	template<typename T>
	struct PoseMap{
	public:
		std::unordered_map<std::string, T> poseData;
	};

	struct ActorResult {
	public:
		pugi::xml_parse_result parseResult;
		std::shared_ptr<tyga::HActor> rootResult;

	};

	template<typename T>
	struct PoseResult {
	public:
		ActorResult actorResult;
		std::unordered_map<std::string, PoseMap<T>> poses;
	};

	ActorResult ReadAndProduceActor(std::string file, std::string pose);

	PoseResult<PoseData> ReadAndProducePosedActor(std::string file);

	void RecursivePoseLookup(XMLReader::PoseMap<XMLReader::PoseData>& map, pugi::xml_node node, std::shared_ptr<tyga::HActor> parentActor, bool isBuilt);

	void RecursiveLookup(pugi::xml_node node, std::shared_ptr<tyga::HActor> parentActor);

	std::pair<bool, XMLReader::PoseData> GetPoseDataFromActor(pugi::xml_node);

	void AddPosedActorToWorldInPose(std::shared_ptr<tyga::HActor> root, std::string pose, std::string modelPrefix, XMLReader::PoseResult<XMLReader::PoseData> poseResult, bool isRoot = true);

	//void AddPosedActorToWorldInPose(std::shared_ptr<tyga::HActor> root, std::string pose, XMLReader::PoseResult<XMLReader::PoseData> poseData, bool isRoot = true);

	void PositionActorAccordingToPose(std::shared_ptr<tyga::HActor> actor, PoseData poseData);

	void LerpActorPoses(std::shared_ptr<tyga::HActor> root, XMLReader::PoseResult<XMLReader::PoseData> poseResult, std::string poseA, std::string poseB, float t);

	void LerpActorPoses(std::shared_ptr<tyga::HActor> root, XMLReader::PoseMap<XMLReader::PoseData> poseMapA, XMLReader::PoseMap<XMLReader::PoseData> poseMapB, float t);


};