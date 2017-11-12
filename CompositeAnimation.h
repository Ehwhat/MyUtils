#pragma once

#include <unordered_map>
#include <string>
#include "HActor.h"
#include <functional>
#include <iostream>

namespace Animator {

		template<class T>
		class CompositeAnimation
		{
		public:
			CompositeAnimation() {

			}

			~CompositeAnimation() {

			}

			void RegisterActorToComposite(std::string key, T actor) {
				auto result = compositeActors.find(key);
				if (result == compositeActors.end()) {

				}
				compositeActors[key] = actor;
			}

			void RegisterState(std::string name, std::function<void(std::unordered_map<std::string, T>* compositeActors, float deltaTime)> function) {
				compositeActors[name] = function;
			}

			void SetState() {
				auto result = compositeStates.find(key);
				if (result != compositeStates.end()) {
					currentState = compositeStates[key];
				} else{
					printf("State not found: " + key);
				}
			}

			void Execute() {

			}

		private:
			std::unordered_map<std::string, T> compositeActors;
			std::unordered_map<std::string, std::function<void(std::unordered_map<std::string, T>* compositeActors, float deltaTime)>> compositeStates;

			std::function<void(std::unordered_map<std::string, T>* compositeActors, float deltaTime)> currentState;

		};
}


