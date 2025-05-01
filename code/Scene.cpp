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
	body.m_position = Vec3( 0, 0, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 1.0f );
	m_bodies.push_back( body );

	body.m_position = Vec3( 0, 0, -101 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 100.0f );
	m_bodies.push_back( body );

	// TODO: Add code
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

	for (int currentBodyIndex = 0; currentBodyIndex < m_bodies.size(); ++currentBodyIndex) {
		m_bodies[currentBodyIndex].m_position += m_bodies[currentBodyIndex].m_linearVelocity * deltaSecond;
}
