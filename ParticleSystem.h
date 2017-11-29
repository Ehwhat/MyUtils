#pragma once
#include <tyga\GraphicsCentre.hpp>
#include <tyga\Math.hpp>
#include <functional>
#include <tyga\RunloopTaskProtocol.hpp>
#include <tyga\Application.hpp>
#include <tyga\BasicWorldClock.hpp>

namespace ParticleSystem {

#pragma region ObjectPool

	

#pragma endregion

#pragma region Particle

	class Particle: std::enable_shared_from_this<Particle> {
		friend class ParticleEmitter;
	public:
		tyga::Vector3 position = tyga::Vector3();
		tyga::Vector3 velocity = tyga::Vector3();
		tyga::Vector3 colour = tyga::Vector3(1, 1, 1);
		float size = 1;
		float alpha = 1;

		bool Animate(float dT);

		void Reset();

		void Activate();
		void Deactivate();

		Particle(float _maxLifetime, std::function<void(float t, Particle* particle)> _lifetimeFunction);

	private:

		std::function<void(float t, Particle*)> lifetimeFunction;

		float currentLifetime = 0;
		float maxLifetime = 5;

		std::shared_ptr<Particle> next;
		bool isActive = false;
	};

#pragma endregion

#pragma region Emitter


	class Emitter {
	public:

		tyga::Vector3 position = tyga::Vector3();

		Emitter(tyga::Vector3 pos = tyga::Vector3()) : position(pos) {}

		virtual void Emit(std::shared_ptr<Particle> particle, float speed = 1) = 0;
	};

	class PointEmitter : public Emitter {
	public:
		tyga::Vector3 emitDirection = tyga::Vector3(0, 1, 0);
		bool randomDirection = false;

		PointEmitter(tyga::Vector3 pos, tyga::Vector3 dir, bool randDir);

		virtual void Emit(std::shared_ptr<Particle> particle, float speed = 1) override;
	};

	class PlaneEmitter : public Emitter {
	public:
		tyga::Vector2 planeSize = tyga::Vector2(1, 1);
		tyga::Vector3 planeNormal = tyga::Vector3(0, 1, 0);
		tyga::Vector3 emitDirection = tyga::Vector3(0, 1, 0);
		bool randomDirection = false;

		PlaneEmitter(tyga::Vector3 pos, tyga::Vector3 dir, tyga::Vector2 size, bool randDir);

		virtual void Emit(std::shared_ptr<Particle> particle, float speed = 1) override;
	};


#pragma endregion

#pragma region ParticleEmitter

	class ParticleEmitter : public tyga::GraphicsSpriteDelegate,
							public std::enable_shared_from_this<ParticleEmitter>
	{
	private:

		std::shared_ptr<Emitter> emitter = std::make_shared<PointEmitter>(tyga::Vector3(), tyga::Vector3(0,1,0), false);

		std::shared_ptr<Particle> firstAvaliable;
		std::vector<std::shared_ptr<Particle>> particles;

		int maxParticles;
		float particleSpawnRate = 0;
		float lastParticleTime = 0;
		
		float emitterTotalTime = 0;

		
		void Create(int amount);

		std::string
			graphicsSpriteTexture() const override;

		int
			graphicsSpriteVertexCount() const override;

		void
			graphicsSpriteGenerate(
				tyga::GraphicsSpriteVertex vertex_array[]) const override;

		void Animate(float deltaTime);

		friend void Animate(float deltaTime, std::shared_ptr<ParticleEmitter> emitter);

	public:

		float particlesPerSecond = 100;
		float particleLifetime = 10;
		std::string texture;

		ParticleEmitter(std::string _texture, int _maxParticles = 100, float _particlesPerSecond = 10, float _particleLifetime = 10, std::shared_ptr<Emitter> _emitter = std::make_shared<PointEmitter>(tyga::Vector3(), tyga::Vector3(0, 1, 0), false), std::function<void(float t, Particle* particle)> _lifetimeFunction = [](float t, Particle* particle) {});
		~ParticleEmitter();

		void UpdatePosition(tyga::Vector3 pos);

	};



#pragma endregion

#pragma region ParticleManager
	
	class ParticleManager {
	public:
		static std::list<std::shared_ptr<ParticleEmitter>> particleEmitters;
		static bool isRunning;

		static void Init();

		static void RegisterParticleEmitter(std::shared_ptr<ParticleEmitter> emitter);

		static void DeregisterParticleEmitter(std::shared_ptr<ParticleEmitter> emitter);

		static int isParticleEmitterRegistered(std::shared_ptr<ParticleEmitter> emitter);

	};

	class ParticleEmitterTask : public tyga::RunloopTaskProtocol {
	public:
		std::function<void()> runFunction;

		ParticleEmitterTask(std::function<void()> func) : runFunction(func) {
		}

		virtual void
			runloopWillBegin() override; // do private processing before entity interaction

		virtual void
			runloopExecuteTask() override;

		virtual void
			runloopDidEnd() override; // do private processing before entity interaction
	};

#pragma endregion


	

}

