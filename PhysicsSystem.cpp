#include "PhysicsSystem.h"


namespace PhysicsSystem {

#pragma region PhysicsManager

		std::list<std::shared_ptr<RigidBody>> PhysicsManager::rigidBodies;

		tyga::Vector3 PhysicsManager::gravity = tyga::Vector3(0,0,-9.81f);
		
		float PhysicsManager::physicsDT = 0;

		void PhysicsManager::Initalise()
		{
			tyga::Application::addRunloopTask(std::make_shared<RigidBodyForceTask>(RigidBodyForceTask(std::shared_ptr<std::list<std::shared_ptr<RigidBody>>>(&rigidBodies))));
		}

		void PhysicsManager::Update(float deltaTime)
		{
			//TestColliders();
			physicsDT = deltaTime;
			//ApplyForcesToBodies(deltaTime);
		}

		void PhysicsManager::AttachRigidBody(std::shared_ptr<RigidBody> r) {
			if (IsRigidBodyAttached(r) == -1) {
				rigidBodies.push_back(r);
			}
		}

		void PhysicsManager::DetachRigidBody(std::shared_ptr<RigidBody> r) {
			int index = IsRigidBodyAttached(r);

			if (index != -1) {
				auto i = rigidBodies.begin();
				std::advance(i, index);
				rigidBodies.erase(i);
			}
		}

		int PhysicsManager::IsRigidBodyAttached(std::shared_ptr<RigidBody> r) {
			auto result = std::find(rigidBodies.begin(), rigidBodies.end(), r);
			if (result != rigidBodies.end()) {
				return std::distance(rigidBodies.begin(), result);
			}
			return -1;
		}
		void PhysicsManager::ApplyForcesToBodies(float dT)
		{
			for each (std::shared_ptr<RigidBody> body in rigidBodies)
			{
				if (!body->isKinematic) {
					body->AddForce(gravity);
					body->Update(dT);
				}
			}
		}
		void PhysicsManager::TestColliders()
		{
			for (int i = 0; i < rigidBodies.size(); i++)
			{
				auto A = GetRigidBodyAt(i);
				for (int j = 0; j < rigidBodies.size(); j++)
				{
					if (i == j) {
						continue;
					}
					auto B = GetRigidBodyAt(j);
					std::shared_ptr<CollisionData> hitData = std::make_shared<CollisionData>(A, B);

					bool wasHit = false;

					if (std::dynamic_pointer_cast<BoxCollider>(A->collider) != 0) {
						auto c = std::dynamic_pointer_cast<BoxCollider>(A->collider);
						wasHit = c->TestCollision(hitData);
					}
					if (std::dynamic_pointer_cast<SphereCollider>(A->collider) != 0) {
						auto c = std::dynamic_pointer_cast<SphereCollider>(A->collider);
						wasHit = c->TestCollision(hitData);
					}

					if (wasHit) {
						ResolveCollision(hitData);
					}
				}
			}
		}
		void PhysicsManager::ResolveCollision(std::shared_ptr<CollisionData> collisionData)
		{
			std::shared_ptr<RigidBody> A = collisionData->bodyA;
			std::shared_ptr<RigidBody> B = collisionData->bodyB;

			tyga::Vector3 rv = B->velocity - A->velocity;
			float magOnNormal = tyga::dot(rv, collisionData->normal);
			if (magOnNormal > 0) {
				return;
			}

			float k = 0.25f;

			float j = -(1 + k)*magOnNormal;
			j /= (A->infMass) + (B->infMass);

			tyga::Vector3 impulse = j * collisionData->normal;
			if (!A->isKinematic) {
				A->velocity += (A->infMass) * impulse;
			}
			if (!B->isKinematic) {
				B->velocity += (B->infMass) * impulse;
			}
			if (!B->isKinematic || !A->isKinematic) {
				FloatCorrection(collisionData);
			}

		}
		void PhysicsManager::FloatCorrection(std::shared_ptr<CollisionData> collisionData)
		{
			const float percent = 1.f;
			const float slop = 0.02f;
			tyga::Vector3 correction = std::max(collisionData->penetration - slop, 0.f) / (collisionData->bodyA->infMass + collisionData->bodyB->infMass) * percent * collisionData->normal;
			collisionData->bodyA->position -= collisionData->bodyA->infMass * correction;
			collisionData->bodyB->position += collisionData->bodyB->infMass * correction;
		}
		std::shared_ptr<RigidBody> PhysicsManager::GetRigidBodyAt(int i)
		{
			if (rigidBodies.size() > i) {
				std::shared_ptr<RigidBody> it = *std::next(rigidBodies.begin(), i);
				return it;
			}
			return nullptr;
		}
#pragma endregion

#pragma region RigidBody

		RigidBody::RigidBody(std::shared_ptr<Collider> c): collider(c)
		{
			PhysicsManager::AttachRigidBody(std::shared_ptr<RigidBody>(this));
		}

		RigidBody::~RigidBody()
		{
			PhysicsManager::DetachRigidBody(std::shared_ptr<RigidBody>(this));
		}

		void RigidBody::Update(float deltaTime) {
			infMass = (mass == 0) ? 0 : 1 / mass;
			ApplyForces(deltaTime);
			position += velocity*deltaTime;
			Actor()->setTransformation(MathUtils::GetMatrixFromTranslationVector(position));
		}

		void RigidBody::AddForce(tyga::Vector3 force, ForceMode mode) {
			forcesToBeApplied.push_back(AppliedForce(force, mode));
		}

		void RigidBody::SetVelocity(tyga::Vector3 vel) {
			velocity = vel;
		}

		void RigidBody::ApplyForces(float dT) {
			for each (AppliedForce force in forcesToBeApplied)
			{
				switch (force.mode)
				{
				case ForceMode::Acceleration:
					velocity += force.force * (dT*dT);
					break;
				case ForceMode::Impulse:
					velocity += mass*force.force * dT;
					break;
				default:
					velocity += force.force * dT;
					break;
				}
			}
		}

#pragma endregion

#pragma region Colliders

		Collider::Collider()
		{
		}

		Collider::~Collider()
		{
		}

		PhysicsSystem::ColliderType Collider::GetColliderType()
		{
			return PhysicsSystem::ColliderType::NONE;
		}

		bool Collider::TestCollision(std::shared_ptr<CollisionData> hitData)
		{
			return false;
		}

		PhysicsSystem::ColliderType BoxCollider::GetColliderType()
		{
			return PhysicsSystem::ColliderType::Box;
		}

		bool BoxCollider::TestCollision(std::shared_ptr<CollisionData> hitData)
		{
			auto col = hitData->bodyB->collider;
			switch (col->GetColliderType())
			{
			case  PhysicsSystem::ColliderType::Box:
				return TestAABBvsAABB(hitData);
			case PhysicsSystem::ColliderType::Sphere:
				//return false;
				return TestAABBvsSphere(hitData);
			}
			return false;
		}
		bool BoxCollider::TestAABBvsAABB(std::shared_ptr<CollisionData> hitData)
		{
			std::shared_ptr<RigidBody> A = hitData->bodyA;
			std::shared_ptr<RigidBody> B = hitData->bodyB;

			tyga::Vector3 dis = B->position - A->position;

			std::shared_ptr<BoxCollider> aCol = std::dynamic_pointer_cast<BoxCollider>(A->collider);
			std::shared_ptr<BoxCollider> bCol = std::dynamic_pointer_cast<BoxCollider>(B->collider);

			float xOver = bCol->halfSize.x + aCol->halfSize.x - abs(dis.x);

			if (xOver > 0) {
				float yOver = bCol->halfSize.y + aCol->halfSize.y - abs(dis.y);
				if (yOver > 0) {
					float zOver = bCol->halfSize.z + aCol->halfSize.z - abs(dis.z);
					if (zOver > 0) {
						if (xOver > yOver) {
							if (xOver > zOver) {
								if (dis.x < 0) {
									hitData->normal = tyga::Vector3(0, 0, -1);
								}
								else {
									hitData->normal = tyga::Vector3(0, 0, 1);
								}
								hitData->penetration = zOver;
								return true;
							}
							else {
								if (dis.z < 0) {
									hitData->normal = tyga::Vector3(0, 0, -1);
								}
								else {
									hitData->normal = tyga::Vector3(0, 0, 1);
								}
								hitData->penetration = zOver;
								return true;
							}
						}
						else {
							if (yOver > zOver) {
								if (dis.y < 0) {
									hitData->normal = tyga::Vector3(0, -1, 0);
								}
								else {
									hitData->normal = tyga::Vector3(0, 1, 0);
								}
								hitData->penetration = yOver;
								return true;
							}
							else {
								if (dis.z < 0) {
									hitData->normal = tyga::Vector3(0, 0, -1);
								}
								else {
									hitData->normal = tyga::Vector3(0, 0, 1);
								}
								hitData->penetration = zOver;
								return true;
							}
						}
					}
				}
			}
			

			return false;
		}

		bool BoxCollider::TestAABBvsSphere(std::shared_ptr<CollisionData> hitData)
		{
			std::shared_ptr<RigidBody> A = hitData->bodyA;
			std::shared_ptr<RigidBody> B = hitData->bodyB;

			tyga::Vector3 dis = B->position - A->position;

			std::shared_ptr<BoxCollider> aCol = std::dynamic_pointer_cast<BoxCollider>(A->collider);
			std::shared_ptr<SphereCollider> bCol = std::dynamic_pointer_cast<SphereCollider>(B->collider);

			tyga::Vector3 closestPoint = dis;
			closestPoint.x = MathUtils::Clamp(closestPoint.x, -aCol->halfSize.x, aCol->halfSize.x);
			closestPoint.y = MathUtils::Clamp(closestPoint.y, -aCol->halfSize.y, aCol->halfSize.y);
			closestPoint.z = MathUtils::Clamp(closestPoint.z, -aCol->halfSize.z, aCol->halfSize.z);

			bool inside = false;

			if (MathUtils::EqualVector3(dis, closestPoint)) {
				inside = true;
				if (abs(dis.x) > abs(dis.y)) {
					if (abs(dis.x) > abs(dis.z)) {
						if (closestPoint.x > 0) {
							closestPoint.x = aCol->halfSize.x;
						}
						else {
							closestPoint.x = -aCol->halfSize.x;
						}
					}
					else {
						if (closestPoint.z > 0) {
							closestPoint.z = aCol->halfSize.z;
						}
						else {
							closestPoint.z = -aCol->halfSize.z;
						}
					}
				}
				else {
					if (abs(dis.y) > abs(dis.z)) {
						if (closestPoint.y > 0) {
							closestPoint.y = aCol->halfSize.y;
						}
						else {
							closestPoint.y = -aCol->halfSize.y;
						}
					}
					else {
						if (closestPoint.z > 0) {
							closestPoint.z = aCol->halfSize.z;
						}
						else {
							closestPoint.z = -aCol->halfSize.z;
						}
					}
				}
			}

			tyga::Vector3 normal = dis - closestPoint;
			float colDis = tyga::length(MathUtils::MultiplyVector3(normal, normal));
			float radius = bCol->radius;

			if (colDis > radius && !inside) {
				return false;
			}

			colDis = sqrtf(colDis);

			if (inside) {
				hitData->normal = tyga::unit(-normal);
				hitData->penetration = radius - colDis;
			}
			else {
				hitData->normal = tyga::unit(normal);
				hitData->penetration = radius - colDis;
			}

			return true;


		}

		ColliderType SphereCollider::GetColliderType()
		{
			return PhysicsSystem::ColliderType::Sphere;
		}

		bool SphereCollider::TestCollision(std::shared_ptr<CollisionData> hitData)
		{
			auto col = hitData->bodyB->collider;
			switch (col->GetColliderType())
			{
			case  PhysicsSystem::ColliderType::Sphere:
				return TestSphereVsSphere(hitData);
			}
			return false;
		}

		bool SphereCollider::TestSphereVsSphere(std::shared_ptr<CollisionData> hitData)
		{
			std::shared_ptr<RigidBody> A = hitData->bodyA;
			std::shared_ptr<RigidBody> B = hitData->bodyB;

			tyga::Vector3 dis = B->position - A->position;

			std::shared_ptr<SphereCollider> aCol = std::dynamic_pointer_cast<SphereCollider>(A->collider);
			std::shared_ptr<SphereCollider> bCol = std::dynamic_pointer_cast<SphereCollider>(B->collider);

			float r = aCol->radius + bCol->radius;
			r *= r;

			if(tyga::length(MathUtils::MultiplyVector3(dis,dis)) > r){
				return false;
			}

			float colDis = tyga::length(dis);

			if (colDis != 0) {
				hitData->penetration = r - colDis;
				hitData->normal = dis / colDis;
				return true;
			}
			hitData->penetration = aCol->radius;
			hitData->normal = tyga::Vector3(0,0,1);
			return true;

		}

#pragma endregion

		RigidBodyForceTask::RigidBodyForceTask(std::shared_ptr<std::list<std::shared_ptr<RigidBody>>> bodiesPointer) : rigidBodies(bodiesPointer){}

		void RigidBodyForceTask::runloopWillBegin()
		{
		}

		void RigidBodyForceTask::runloopExecuteTask()
		{
			TestColliders();
			ApplyForcesToBodies(PhysicsManager::physicsDT);
		}

		void RigidBodyForceTask::runloopDidEnd()
		{
		}

		std::shared_ptr<RigidBody> RigidBodyForceTask::GetRigidBodyAt(int i)
		{
			if (rigidBodies->size() > i) {
				std::shared_ptr<RigidBody> it = *std::next(rigidBodies->begin(), i);
				return it;
			}
			return nullptr;
		}

		void RigidBodyForceTask::ApplyForcesToBodies(float dT)
		{
			for each (std::shared_ptr<RigidBody> body in *rigidBodies)
			{
				if (!body->isKinematic) {
					body->AddForce(PhysicsManager::gravity);
					body->Update(dT);
				}
			}
		}
		void RigidBodyForceTask::TestColliders()
		{
			for (int i = 0; i < rigidBodies->size(); i++)
			{
				auto A = GetRigidBodyAt(i);
				for (int j = 0; j < rigidBodies->size(); j++)
				{
					if (i == j) {
						continue;
					}
					auto B = GetRigidBodyAt(j);
					std::shared_ptr<CollisionData> hitData = std::make_shared<CollisionData>(A, B);

					bool wasHit = false;

					if (std::dynamic_pointer_cast<BoxCollider>(A->collider) != 0) {
						auto c = std::dynamic_pointer_cast<BoxCollider>(A->collider);
						wasHit = c->TestCollision(hitData);
					}
					if (std::dynamic_pointer_cast<SphereCollider>(A->collider) != 0) {
						auto c = std::dynamic_pointer_cast<SphereCollider>(A->collider);
						wasHit = c->TestCollision(hitData);
					}

					if (wasHit) {
						ResolveCollision(hitData);
					}
				}
			}
		}

		void RigidBodyForceTask::ResolveCollision(std::shared_ptr<CollisionData> collisionData)
		{
			std::shared_ptr<RigidBody> A = collisionData->bodyA;
			std::shared_ptr<RigidBody> B = collisionData->bodyB;

			tyga::Vector3 rv = B->velocity - A->velocity;
			float magOnNormal = tyga::dot(rv, collisionData->normal);
			if (magOnNormal > 0) {
				return;
			}

			float k = 0.25f;

			float j = -(1 + k)*magOnNormal;
			j /= (A->infMass) + (B->infMass);

			tyga::Vector3 impulse = j * collisionData->normal;
			if (!A->isKinematic) {
				A->velocity += (A->infMass) * impulse;
			}
			if (!B->isKinematic) {
				B->velocity += (B->infMass) * impulse;
			}
			if (!B->isKinematic || !A->isKinematic) {
				FloatCorrection(collisionData);
			}

		}
		void RigidBodyForceTask::FloatCorrection(std::shared_ptr<CollisionData> collisionData)
		{
			const float percent = 1.f;
			const float slop = 0.02f;
			tyga::Vector3 correction = std::max(collisionData->penetration - slop, 0.f) / (collisionData->bodyA->infMass + collisionData->bodyB->infMass) * percent * collisionData->normal;
			collisionData->bodyA->position -= collisionData->bodyA->infMass * correction;
			collisionData->bodyB->position += collisionData->bodyB->infMass * correction;
		}

}
