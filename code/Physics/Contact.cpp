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

	const float invMassA = bodyA->m_invMass;
	const float invMassB = bodyB->m_invMass;
	
	// calculate the collision impulse
	const Vec3& normalizedVector = contact.normal;
	const Vec3& velocityAminusB = bodyA->m_linearVelocity - bodyB->m_linearVelocity;
	const float impulseJ = -2.0f * velocityAminusB.Dot(normalizedVector) / (invMassA + invMassB);
	const Vec3 vectorImpulseJ = normalizedVector * impulseJ;

	bodyA->ApplyImpulseLinear(vectorImpulseJ * 1.0f);
	bodyB->ApplyImpulseLinear(vectorImpulseJ * -1.0f);


	// let's also move our colliding objects to just outside of each other
	const float proportionOfA = bodyA->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);
	const float proportionOfB = bodyB->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);

	const Vec3 vectorBtwTwoContactPoints = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
	bodyA->m_position += vectorBtwTwoContactPoints * proportionOfA;
	bodyB->m_position -= vectorBtwTwoContactPoints * proportionOfB;
}
