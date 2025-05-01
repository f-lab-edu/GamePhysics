//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact( contact_t & contact ) {
	Body* bodyA = contact.bodyA;
	Body* bodyB = contact.bodyB;

	bodyA->m_linearVelocity.Zero();
	bodyB->m_linearVelocity.Zero();

	// let's also move our colliding objects to just outside of each other
	const float proportionOfA = bodyA->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);
	const float proportionOfB = bodyB->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);

	const Vec3 vectorBtwTwoContactPoints = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
	bodyA->m_position += vectorBtwTwoContactPoints * proportionOfA;
	bodyB->m_position -= vectorBtwTwoContactPoints * proportionOfB;
}
