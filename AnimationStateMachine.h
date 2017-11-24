#pragma once

#include "MathUtils.h"
#include <memory>
#include <functional>

/**
*	My Animation State Machine, developed by Joshua Stevenson (Q5120984), 2017
*	This is a more flexible solution to my previous keyframing system, allowing for more complicated animation sequences.
*	Don't know much about licencing so I don't care what happens to this code, go crazy
*
*   OwnerType, as the main generic class, is the class of the object that can be given as a parameter in the state functions.
*
*	TODO:
*	Add Remove connections so animations can change themselves
*   Problem with general archtecture, All states are owned by their parents, which is not exactly the model I want, what if I need to move a state from one connection to another?
*/

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
	class AnimationConnection { // AnimationConnection is a representation of a connection from a parent state to a child state, containing a veification function to see if the connection is valid.

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

	
	// An animation state contains data defining the animation, such as the duration and lambda function, along with a vector of all the connections to other states it harbours.

	template<class OwnerType>
	class AnimationState { 

	public:

		AnimationState() {
			stateDuration = 0;
			clamp = AnimationStateClampType::NoLength;
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

		void Execute(OwnerType object,float deltaTime) { // Execute our current function
			if (clamp != AnimationStateClampType::NoLength) {
				if (status != AnimationStateStatus::Complete) {
					status = AnimationStateStatus::CurrentlyRunning;
					if (stateFunction) {
						stateFunction(object, (currentTime / stateDuration));
					}
					if (clamp == AnimationStateClampType::RunOnce) {
						status = AnimationStateStatus::Complete;
						return;
					}
					HandleTime(deltaTime);
				}
				else {
					stateFunction(object, 1);
				}
			}
		}

		std::pair<bool, std::shared_ptr<AnimationState<OwnerType>>> HandleConnections() { // Check all current connections to see if one is valid, if so, return true and the valid state connected
			for (int i = 0; i < connections.size(); i++)
			{
				auto result = connections[i].TryRunState(status);
				if (result.first) {
					return result;
				}
			}
			return std::make_pair(false, nullptr);
		}

		// Bunch of Add connection functions as to allow for flexibility

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

		void HandleTime(float deltaTime) { // How is reaching the end of the duration handled?
			currentTime += deltaTime;
			actualTime += deltaTime;
			switch (clamp)
			{
			case AnimationStateClampType::Single: // If it's single, then we just stop running the state function
				if (currentTime >= stateDuration) {
					status = AnimationStateStatus::Complete;
					currentTime = stateDuration;

				}
				break;
			case AnimationStateClampType::Loop: // if it's loop, we keep running the state function and reset the current time of the duration back to 0
				if (currentTime > stateDuration) {
					currentTime = 0;
				}
				break;
			case AnimationStateClampType::PingPong: // if it's ping-pong, we keep running, and go backwards from the duration to 0, and then back again, etc.
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
	class AnimationLayer { // Animation layers contain animation states, along with a root state, the animation layer's job is to execute the correct state, and hold the current tree of states.
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
			currentState->Execute(object, deltaTime); // Execute current state function
			std::shared_ptr<AnimationState<OwnerType>> outputState;
			auto result = currentState->HandleConnections(); // Have current state check all it's connections for any valid ones
			if (result.first) { // If there was a valid one, then we're gonna want to end the current state, change the current state, then begin executing the new current state
				currentState->End();
				currentState = result.second;
				currentState->Init();
				currentState->Execute(object, 0);
			}
		}

		std::shared_ptr<AnimationState<OwnerType>> GetRootState(){
			return rootState; // the root state is used for an inital starting point for a layer, which contains no actual function just serves as a solid anchor 
		}

	};

	template<class OwnerType>
	class AnimationStateMachine { // The base animationmachine contains all the animationlayers, which allow for layered animations at the same time.
	public:
		std::vector<std::shared_ptr<AnimationLayer<OwnerType>>> animationLayers;

		AnimationStateMachine() {
			AddLayer();
		}

		std::shared_ptr<AnimationLayer<OwnerType>> GetLayer(int layerIndex = 0) {
			if (animationLayers.size() <= layerIndex) {
				for (int i = (int)animationLayers.size(); i <= layerIndex; i++)
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

