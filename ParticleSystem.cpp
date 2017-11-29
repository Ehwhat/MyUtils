#include "ParticleSystem.h"
#include "MathUtils.h"

namespace ParticleSystem {

#pragma region Particle


	bool Particle::Animate(float dT)
	{
		
		if (isActive) {
			float t = currentLifetime / maxLifetime;
			currentLifetime += dT;

			if (lifetimeFunction) {
				lifetimeFunction(t, this);
			}

			position += velocity * dT;

			if (currentLifetime >= maxLifetime) {
				return true;
			}
		}
		return false;
	}

	void Particle::Reset()
	{
		currentLifetime = 0;
	}

	void Particle::Activate()
	{
		isActive = true;
		Reset();

	}

	void Particle::Deactivate()
	{
		isActive = false;
	}

	Particle::Particle(float _maxLifetime, std::function<void(float t, Particle* particle)> _lifetimeFunction) : maxLifetime(_maxLifetime), lifetimeFunction(_lifetimeFunction)
	{
	}



#pragma endregion

#pragma region Emitter

	PointEmitter::PointEmitter(tyga::Vector3 pos, tyga::Vector3 dir, bool randDir) : Emitter(pos), emitDirection(dir), randomDirection(randDir)
	{

	}

	void PointEmitter::Emit(std::shared_ptr<Particle> particle, float speed)
	{
		particle->position = position;
		if (randomDirection) {
			emitDirection = MathUtils::RandomDirection();
		}
		particle->velocity = tyga::unit(emitDirection) * speed;
	}


	PlaneEmitter::PlaneEmitter(tyga::Vector3 pos, tyga::Vector3 dir, tyga::Vector2 size, bool randDir) : Emitter(pos), emitDirection(dir), randomDirection(randDir), planeSize(size), planeNormal(dir)
	{
	}

	void PlaneEmitter::Emit(std::shared_ptr<Particle> particle, float speed)
	{
		tyga::Vector3 randPos = tyga::Vector3(MathUtils::RandomRange(-planeSize.x / 2, planeSize.x / 2), 0, MathUtils::RandomRange(-planeSize.y / 2, planeSize.y / 2));
		tyga::Vector3 planePos = tyga::cross(planeNormal, randPos);
		particle->position = position + planePos;
		if (randomDirection) {
			emitDirection = MathUtils::RandomDirection();
		}

		particle->velocity = tyga::unit(emitDirection) * speed;
	}

#pragma endregion

#pragma region ParticleEmitter

	ParticleEmitter::ParticleEmitter(std::string _texture, int _maxParticles, float _particlesPerSecond, float _particleLifetime, std::shared_ptr<Emitter> _emitter, std::function<void(float t, Particle*particle)> _lifetimeFunction) : texture(_texture), emitter(_emitter), maxParticles(_maxParticles), particlesPerSecond(_particlesPerSecond), particleLifetime(_particleLifetime){

		if (!ParticleManager::isRunning) {
			ParticleManager::Init();
		}

		ParticleManager::RegisterParticleEmitter(std::shared_ptr<ParticleEmitter>(this));

		particleSpawnRate = 1 / particlesPerSecond;
		emitterTotalTime = 0;

		particles.resize(maxParticles);
		firstAvaliable = std::make_shared<Particle>(particleLifetime, _lifetimeFunction);
		std::shared_ptr<Particle> current = firstAvaliable;
		for (int i = 0; i < maxParticles-1; i++)
		{
			particles[i] = current;
			std::shared_ptr<Particle> next = std::make_shared<Particle>(particleLifetime, _lifetimeFunction);
			current->next = next;
			current = next;
		}
		particles[maxParticles - 1] = current;
		current->next = NULL;
	}

	ParticleEmitter::~ParticleEmitter()
	{
		ParticleManager::DeregisterParticleEmitter(std::shared_ptr<ParticleEmitter>(this));
	}

	void ParticleEmitter::Animate(float deltaTime)
	{
		int numOfParticles = 1;
		if (particleSpawnRate < deltaTime) {
			numOfParticles = (int)floorf(deltaTime / particleSpawnRate);
		}
		if (emitterTotalTime - lastParticleTime >= particleSpawnRate) {
			Create(numOfParticles);
			lastParticleTime = emitterTotalTime;
		}
		for (int j = 0; j < maxParticles - j; j+=numOfParticles)
		{
			if (particles[j]->Animate(deltaTime)) {
				particles[j]->Deactivate();
				particles[j]->next = firstAvaliable;
				firstAvaliable = particles[j];
			}
		}
		emitterTotalTime += deltaTime;
	}

	void ParticleEmitter::UpdatePosition(tyga::Vector3 pos)
	{
		emitter->position = pos;
	}

	void ParticleEmitter::Create(int amount)
	{
		for (int i = 0; i < amount; i++)
		{
			if (firstAvaliable != NULL) {
				std::shared_ptr<Particle> particle = firstAvaliable;
				particle->Activate();
				emitter->Emit(particle);
				firstAvaliable = particle->next;
			}
		}
	}

	std::string ParticleEmitter::graphicsSpriteTexture() const
	{
		return texture;
	}

	int ParticleEmitter::graphicsSpriteVertexCount() const
	{
		return maxParticles;
	}

	void ParticleEmitter::graphicsSpriteGenerate(tyga::GraphicsSpriteVertex vertex_array[]) const
	{
		for (int i = 0; i < maxParticles - 1; i++)
		{
			if (particles[i]->isActive) {
				vertex_array[i].position = particles[i]->position;
				vertex_array[i].size = particles[i]->size;
				vertex_array[i].colour = particles[i]->colour;
				vertex_array[i].alpha = particles[i]->alpha;
				vertex_array[i].rotation = 0.f; // NB: has no effect in basic renderer
			}
			else {
				vertex_array[i].position = tyga::Vector3();
				vertex_array[i].size = 0.f;
				vertex_array[i].colour = tyga::Vector3(1, 1, 1);
				vertex_array[i].alpha = 0.f;
				vertex_array[i].rotation = 0.f; // NB: has no effect in basic renderer
			}
		}
	}



#pragma endregion

	std::list<std::shared_ptr<ParticleEmitter>> ParticleManager::particleEmitters;
	bool ParticleManager::isRunning = false;

	void Animate(float deltaTime, std::shared_ptr<ParticleEmitter> emitter)
	{
		emitter->Animate(deltaTime);
	}
	
	void ParticleManager::Init()
	{
		isRunning = true;
		tyga::Application::addRunloopTask(std::make_shared<ParticleEmitterTask>([=]() {
			float dT = tyga::BasicWorldClock::CurrentTickInterval();
			for each (std::shared_ptr<ParticleEmitter> emitter in particleEmitters)
			{
				
				Animate(dT, emitter);
			}
		}));
	}

	void ParticleManager::RegisterParticleEmitter(std::shared_ptr<ParticleEmitter> emitter)
	{
		if (isParticleEmitterRegistered(emitter) == -1) {
			particleEmitters.push_back(emitter);
		}
	}

	void ParticleManager::DeregisterParticleEmitter(std::shared_ptr<ParticleEmitter> emitter)
	{
		int index = isParticleEmitterRegistered(emitter);

		if (index != -1) {
			auto i = particleEmitters.begin();
			std::advance(i, index);
			particleEmitters.erase(i);
		}
	}

	int ParticleManager::isParticleEmitterRegistered(std::shared_ptr<ParticleEmitter> emitter)
	{
		auto result = std::find(particleEmitters.begin(), particleEmitters.end(), emitter);
		if (result != particleEmitters.end()) {
			return std::distance(particleEmitters.begin(), result);
		}
		return -1;
	}

	void ParticleEmitterTask::runloopWillBegin()
	{
	}

	void ParticleEmitterTask::runloopExecuteTask()
	{
		runFunction();
	}

	void ParticleEmitterTask::runloopDidEnd()
	{
	}

}