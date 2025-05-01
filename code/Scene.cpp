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
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
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
	body.m_position = Vec3(0, 0, 10);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_shape = new ShapeSphere(1.0f);
	m_bodies.push_back(body);

	// Add a ground sphere that won't fall under the influence of gravity
	body.m_position = Vec3(0, 0, -1000);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_invMass = 0.0f;
	body.m_elasticity = 1.0f;
	body.m_shape = new ShapeSphere(1000.0f);
	m_bodies.push_back(body);
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float deltaSecond ) {
	for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex) {
		Body* currentBody = &m_bodies[currentBodyIndex];

		float mass = 1.0f / currentBody->m_invMass;
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * deltaSecond;
		currentBody->ApplyImpulseLinear(impulseGravity);
	}

	// check for collisions with other bodies
	for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex) {
		for (int currentBodyIndexNested = currentBodyIndex + 1; currentBodyIndexNested < m_bodies.size(); ++currentBodyIndexNested) {
			Body* bodyA = &m_bodies[currentBodyIndex];
			Body* bodyB = &m_bodies[currentBodyIndexNested];
		
			// skip body pairs with infinite mass
			if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
				continue;

			contact_t contact;
			if (Intersect(bodyA, bodyB, contact))
				ResolveContact(contact);
		}
	}

	for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex) 
		m_bodies[currentBodyIndex].m_position += m_bodies[currentBodyIndex].m_linearVelocity * deltaSecond;
}
