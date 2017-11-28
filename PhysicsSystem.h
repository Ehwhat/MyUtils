#pragma once
#include <tyga/Actor.hpp>
#include <tyga/Math.hpp>
#include <tyga/RunloopTaskProtocol.hpp>
#include <tyga/Application.hpp>
#include <list>
#include <algorithm>
#include <memory>
#include <functional>
#include <math.h>

#include "MathUtils.h"

namespace PhysicsSystem {
	
	

	class RigidBody;

	struct CollisionData {
		std::shared_ptr<RigidBody> bodyA;
		std::shared_ptr<RigidBody> bodyB;
		float penetration;
		tyga::Vector3 normal;

		CollisionData(std::shared_ptr<RigidBody> a, std::shared_ptr<RigidBody> b) :bodyA(a), bodyB(b) {}

	};

	enum ColliderType {
		Box,
		Sphere,
		Plane,
		NONE
	};

	class Collider {
	public:
		
		Collider();
		virtual ~Collider();

		virtual ColliderType GetColliderType();

		virtual bool TestCollision(std::shared_ptr<CollisionData> hitData);

	};

	class RigidBody : public tyga::ActorComponent {
	public:

		enum ForceMode {
			Acceleration,
			VelocityChange,
			Impulse
		};

		struct AppliedForce {
		public:
			ForceMode mode;
			tyga::Vector3 force;

			AppliedForce(tyga::Vector3 _force, ForceMode _mode) : force(_force), mode(_mode) {}

		};

		struct RigidBodyData {
		public:
			tyga::Vector3 velocity;
			tyga::Vector3 position;
			float mass;

			const float dT;
			const std::vector<AppliedForce> forces;

			RigidBodyData(tyga::Vector3 vel, tyga::Vector3 pos, float m, float _dT, std::vector<AppliedForce> _forces) : velocity(vel), position(pos), mass(m), dT(_dT), forces(_forces) {}

		};

		tyga::Vector3 position = tyga::Vector3(0, 0, 0);
		tyga::Vector3 velocity = tyga::Vector3(0, 0, 0);

		bool isKinematic = false;
		float mass = 1;
		float infMass = 0;

		std::shared_ptr<Collider> collider;

		RigidBody(std::shared_ptr<Collider> c);

		~RigidBody();

		void Update(float deltaTime);

		void AddForce(tyga::Vector3 force, ForceMode mode = ForceMode::Acceleration);

		void SetVelocity(tyga::Vector3 vel);

	private:

		void ApplyForces(float dT);

		std::vector<AppliedForce> forcesToBeApplied;

	};

	class RigidBodyForceTask : public tyga::RunloopTaskProtocol {
	public:

		RigidBodyForceTask(std::shared_ptr<std::list<std::shared_ptr<RigidBody>>> bodiesPointer);

		virtual void
			runloopWillBegin() override; // do private processing before entity interaction

		virtual void
			runloopExecuteTask() override;

		virtual void
			runloopDidEnd() override; // do private processing before entity interaction

	private:

		int IsRigidBodyAttached(std::shared_ptr<RigidBody> r);

		void ApplyForcesToBodies(float dT);

		void TestColliders();

		void ResolveCollision(std::shared_ptr<CollisionData> collisionData);

		void FloatCorrection(std::shared_ptr<CollisionData> collisionData);

		std::shared_ptr<RigidBody> GetRigidBodyAt(int i);

		std::shared_ptr<std::list<std::shared_ptr<RigidBody>>> rigidBodies;

	};

	class BoxCollider : public Collider {
	public:
		tyga::Vector3 halfSize;

		BoxCollider(tyga::Vector3 size) : Collider(), halfSize(size) {}

		virtual ColliderType GetColliderType() override;
		virtual bool TestCollision(std::shared_ptr<CollisionData> hitData) override;

	private:
		bool TestAABBvsAABB(std::shared_ptr<CollisionData> hitData);
		bool TestAABBvsSphere(std::shared_ptr<CollisionData> hitData);

	};

	class SphereCollider : public Collider {
	public:
		float radius;

		SphereCollider(float radius) : Collider(), radius(radius) {}

		virtual ColliderType GetColliderType() override;
		virtual bool TestCollision(std::shared_ptr<CollisionData> hitData) override;

	private:
		bool TestSphereVsSphere(std::shared_ptr<CollisionData> hitData);

	};

	
	class PhysicsManager {
	public:
		static std::list<std::shared_ptr<RigidBody>> rigidBodies;

		static tyga::Vector3 gravity;

		static float physicsDT;

		static void Initalise();

		static void Update(float deltaTime);

		static void AttachRigidBody(std::shared_ptr<RigidBody> r);

		static void DetachRigidBody(std::shared_ptr<RigidBody> r);

	private:

		static int IsRigidBodyAttached(std::shared_ptr<RigidBody> r);

		static void ApplyForcesToBodies(float dT);

		static void TestColliders();

		static void ResolveCollision(std::shared_ptr<CollisionData> collisionData);

		static void FloatCorrection(std::shared_ptr<CollisionData> collisionData);

		static std::shared_ptr<RigidBody> GetRigidBodyAt(int i);

	};

}
