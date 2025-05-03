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
	body.m_position = Vec3(-3, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(10, 0, 0);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.0f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(1.0f);
	m_bodies.push_back(body);

	body.m_position = Vec3(0, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.0f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(0.5f);
	m_bodies.push_back(body);

	// Add a ground sphere that won't fall under the influence of gravity
	body.m_position = Vec3(0, 0, -1000);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_invMass = 0.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(1000.0f);
	m_bodies.push_back(body);
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

	int numContacts = 0;
	const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t* contacts = reinterpret_cast<contact_t*>(alloca(sizeof(contact_t) * maxContacts));

	// check for collisions with other bodies
	for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex) {
		for (int currentBodyIndexNested = currentBodyIndex + 1; currentBodyIndexNested < m_bodies.size(); ++currentBodyIndexNested) {
			Body* bodyA = &m_bodies[currentBodyIndex];
			Body* bodyB = &m_bodies[currentBodyIndexNested];

			// skip body pairs with infinite mass
			if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
				continue;

			contact_t contact;
			if (Intersect(bodyA, bodyB, deltaSecond, contact)) {
				contacts[numContacts] = contact;
				++numContacts;
			}
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

		Body* bodyA = contact.bodyA;
		Body* bodyB = contact.bodyB;

		// skip body pairs with infinite mass
		if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
			continue;

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
