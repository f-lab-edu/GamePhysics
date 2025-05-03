//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for (int i = 0; i < m_bodies.size(); i++) {
		delete m_bodies[i].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for (int i = 0; i < m_bodies.size(); i++) {
		delete m_bodies[i].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;
	
	// Dynamic Bodies
	for (int x = 0; x < 6; ++x) {
		for (int y = 0; y < 6; ++y) {
			float radius = 0.5f;
			float xx = float(x - 1) * radius * 1.5f;
			float yy = float(y - 1) * radius * 1.5f;
			body.m_position = Vec3(xx, yy, 10.0f);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}

	
	// Static "floor"
	for (int x = 0; x < 3; ++x) {
		for (int y = 0; y < 3; ++y) {
			float radius = 80.0f;
			float xx = float(x - 1) * radius * 0.25f;
			float yy = float(y - 1) * radius * 0.25f;
			body.m_position = Vec3(xx, yy, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float deltaSecond) {
	// Gravity impulse
	for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex) {
		Body* currentBody = &m_bodies[currentBodyIndex];

		float mass = 1.0f / currentBody->m_invMass;
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * deltaSecond;
		currentBody->ApplyImpulseLinear(impulseGravity);
	}

	// Broadphase
	std::vector<collisionPair_t> collisionPairs;
	BroadPhase(m_bodies.data(), static_cast<int>(m_bodies.size()), collisionPairs, deltaSecond);

	// Narrowphase
	int numContacts = 0;
	const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t* contacts = reinterpret_cast<contact_t*>(alloca(sizeof(contact_t) * maxContacts));

	// check for collisions with other bodies
	// now using collision pairs
	for (int currentPairIndex = 0; currentPairIndex < collisionPairs.size(); ++currentPairIndex) {
		const collisionPair_t& currentPair = collisionPairs[currentPairIndex];
		Body* bodyA = &m_bodies[currentPair.a];
		Body* bodyB = &m_bodies[currentPair.b];

		// skip body pairs with infinite mass
		if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
			continue;

		contact_t contact;
		if (Intersect(bodyA, bodyB, deltaSecond, contact)) {
			contacts[numContacts] = contact;
			++numContacts;
		}
	}

	// sort TOI from earliest to latest
	if (numContacts > 1)
		qsort(contacts, numContacts, sizeof(contact_t), CompareContacts);

	// resolve collisions
	// note that there’s no recalculation of earlier collisions for later ones to improve performance.
	// thus, while the first collision is handled correctly, later collisions may be processed improperly if they are related to the earlier collisions.
	float accumulatedTime = 0.0f;
	for (int currentContactIndex = 0; currentContactIndex < numContacts; ++currentContactIndex) {
		contact_t& contact = contacts[currentContactIndex];
		const float deltaTime = contact.timeOfImpact - accumulatedTime;

		// position update
		for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex)
			m_bodies[currentBodyIndex].Update(deltaTime);

		ResolveContact(contact);
		accumulatedTime += deltaTime;
	}

	// update the positions for the rest of this frame's time
	const float timeRemaining = deltaSecond - accumulatedTime;
	if (timeRemaining > 0.0f) {
		for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex)
			m_bodies[currentBodyIndex].Update(timeRemaining);
	}
}
