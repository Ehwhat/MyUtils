#pragma once

#include "MathUtils.h"
#include <memory>
#include <functional>

namespace Animation {

	enum AnimationStateStatus
	{
		NotStarted,
		CurrentlyRunning,
		Complete
	};

	template<class OwnerType>
	class AnimationState;

	template<class OwnerType>
	class AnimationConnection {

	public:

		AnimationConnection(){}

		AnimationConnection(std::shared_ptr<AnimationState<OwnerType>> _connectedState, std::function<bool(AnimationStateStatus ownerStateStatus)> _verifyFunction) {
			connectedState = _connectedState;
			verifyFunction = _verifyFunction;
		}

		std::shared_ptr<AnimationState<OwnerType>> connectedState;
		std::pair<bool, std::shared_ptr<AnimationState<OwnerType>>> TryRunState(AnimationStateStatus ownerStateStatus) {
			if (connectedState != nullptr) {
				if (verifyFunction && verifyFunction(ownerStateStatus)) {
					return std::make_pair(true, connectedState);
				}
			}
			return std::make_pair(false, nullptr);
		}

	private:
		std::function<bool(AnimationStateStatus ownerStateStatus)> verifyFunction = [](AnimationStateStatus ownerStateStatus) {return true; };
	};

	enum AnimationStateClampType {
		NoLength,
		RunOnce,
		Single,
		Loop,
		PingPong
	};

	template<class OwnerType>
	class AnimationState {

	public:

		AnimationState() {

		}

		AnimationState(float _stateDuration, AnimationStateClampType _clamp, std::function<void(OwnerType object, float time)> _stateFunction) {
			stateFunction = _stateFunction;
			stateDuration = _stateDuration;
			clamp = _clamp;
		}

		float stateDuration = 5;
		AnimationStateClampType clamp = AnimationStateClampType::Single;
		AnimationStateStatus status = AnimationStateStatus::NotStarted;
		std::vector<AnimationConnection<OwnerType>> connections;
		std::function<void(OwnerType object, float time)> stateFunction;

		void Init() {
			currentTime = 0;
			status = AnimationStateStatus::NotStarted;
		}

		void End() {
			status = AnimationStateStatus::Complete;
		}

		void Execute(OwnerType object,float deltaTime) {
			if (status != AnimationStateStatus::Complete && clamp != AnimationStateClampType::NoLength) {
				status = AnimationStateStatus::CurrentlyRunning;
				if (stateFunction) {
					stateFunction(object, (currentTime/stateDuration) );
				}
				if (clamp == AnimationStateClampType::RunOnce) {
					status = AnimationStateStatus::Complete;
					return;
				}
				HandleTime(deltaTime);
			}
		}

		std::pair<bool, std::shared_ptr<AnimationState<OwnerType>>> HandleConnections() {
			for (int i = 0; i < connections.size(); i++)
			{
				auto result = connections[i].TryRunState(status);
				if (result.first) {
					return result;
				}
			}
			return std::make_pair(false, nullptr);
		}

		std::shared_ptr<AnimationState<OwnerType>> AddConnection(float _stateDuration, AnimationStateClampType _clamp, std::function<void(OwnerType object, float time)> _stateFunction, std::function<bool(AnimationStateStatus ownerStateStatus)> verifyFunction) {
			std::shared_ptr<AnimationState<OwnerType>> connectedState = std::make_shared<AnimationState<OwnerType>>(_stateDuration,_clamp, _stateFunction);
			connections.push_back(AnimationConnection<OwnerType>(connectedState, verifyFunction));
			return connectedState;
		}

		std::shared_ptr<AnimationState<OwnerType>> AddConnection(std::function<bool(AnimationStateStatus ownerStateStatus)> verifyFunction) {
			std::shared_ptr<AnimationState<OwnerType>> connectedState;
			connections.push_back(AnimationConnection<OwnerType>(connectedState, verifyFunction));
			return connectedState;
		}

		std::shared_ptr<AnimationState<OwnerType>> AddConnection(std::shared_ptr<AnimationState<OwnerType>> connectedState, std::function<bool(AnimationStateStatus ownerStateStatus)> verifyFunction) {
			connections.push_back(AnimationConnection<OwnerType>(connectedState, verifyFunction));
			return connectedState;
		}

		std::shared_ptr<AnimationState<OwnerType>> AddConnection(std::shared_ptr<AnimationState<OwnerType>> connectedState) {
			connections.push_back(AnimationConnection<OwnerType>(connectedState, [](AnimationStateStatus ownerStateStatus) {return true; }));
			return connectedState;
		}

		std::shared_ptr<AnimationState<OwnerType>> AddConnection(AnimationState<OwnerType> connectedState, std::function<bool(AnimationStateStatus ownerStateStatus)> verifyFunction) {
			std::shared_ptr<AnimationState<OwnerType>> state = std::make_shared(connectedState);
			connections.push_back(AnimationConnection<OwnerType>(state, verifyFunction));
			return state;
		}


	private:

		void HandleTime(float deltaTime) {
			currentTime += deltaTime;
			actualTime += deltaTime;
			switch (clamp)
			{
			case AnimationStateClampType::Single:
				if (currentTime >= stateDuration-0.01f) {
					status = AnimationStateStatus::Complete;
					currentTime = stateDuration;

				}
				break;
			case AnimationStateClampType::Loop:
				if (currentTime > stateDuration) {
					currentTime = 0;
				}
				break;
			case AnimationStateClampType::PingPong:
				currentTime = abs(fmodf((actualTime + stateDuration), (stateDuration * 2)) - stateDuration);
				break;
			default:
				break;
			}
			
		}


		float currentTime = 0;
		float actualTime = 0;

	};

	template<class OwnerType>
	class AnimationLayer {
	public:
		std::shared_ptr<AnimationState<OwnerType>> rootState;
		std::shared_ptr<AnimationState<OwnerType>> currentState;

		AnimationLayer() {
			rootState = std::make_shared<AnimationState<OwnerType>>();
			currentState = rootState;
		}

		void Execute(OwnerType object, float deltaTime) {
			if (currentState == nullptr) {
				currentState = rootState;
			}
			currentState->Execute(object, deltaTime);
			std::shared_ptr<AnimationState<OwnerType>> outputState;
			auto result = currentState->HandleConnections();
			if (result.first) {
				currentState->End();
				currentState = result.second;
				currentState->Init();
			}
		}

		std::shared_ptr<AnimationState<OwnerType>> GetRootState(){
			return rootState;
		}

	};

	template<class OwnerType>
	class AnimationStateMachine {
	public:
		std::vector<std::shared_ptr<AnimationLayer<OwnerType>>> animationLayers;

		AnimationStateMachine() {
			AddLayer();
		}

		std::shared_ptr<AnimationLayer<OwnerType>> GetLayer(int layerIndex = 0) {
			if (animationLayers.size() <= layerIndex) {
				for (int i = animationLayers.size(); i <= layerIndex; i++)
				{
					AddLayer();
				}
			}
			return animationLayers[layerIndex];
		}

		void Execute(OwnerType object, float deltaTime) {
			for (int i = 0; i < animationLayers.size(); i++)
			{
				animationLayers[i]->Execute(object, deltaTime);
			}
		}

	private:

		void AddLayer() {
			animationLayers.push_back(std::make_shared<AnimationLayer<OwnerType>>());
		}

	};
}

