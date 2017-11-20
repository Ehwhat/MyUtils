#pragma once

#include <unordered_map>
#include <string>
#include "HActor.h"
#include "MathUtils.h"
#include <functional>
#include <iostream>

namespace Animator {

		template<class T, typename PoseData>
		class CompositeAnimationActor {
		public:
			T actor;
			std::unordered_map<std::string, PoseData> poses;
		};

		template<class T, typename PoseData>
		class CompositeAnimation
		{
		public:
			CompositeAnimation(std::function<void(CompositeAnimationActor<T, PoseData> actor)> _poseFunction) : poseFunction(_poseFunction)  {}

			~CompositeAnimation() {

			}

			void RegisterActorToComposite(std::string key, T actor) {
				auto result = compositeActors.find(key);
				if (result == compositeActors.end()) {

				}
				compositeActors[key] = actor;
			}

		private:
			std::unordered_map<std::string, CompositeAnimationActor<T,PoseData>> compositeActors;

			std::function<void(CompositeAnimationActor<T, PoseData> actor)> poseFunction;

		};
		namespace Pose {
			struct PoseData {
				tyga::Vector3 translate;
				tyga::Vector3 eular;
			};

			class Pose : public CompositeAnimation<std::shared_ptr<tyga::HActor>, PoseData> {
			public:
				Pose() : CompositeAnimation(void(poseFunction)()) {

				}
				~Pose() {

				}

			private:
				void poseFunction(CompositeAnimationActor<std::shared_ptr<tyga::HActor>, PoseData> actor) {
					
				}

			};
		}

}


