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

	const Vec3 pointOnA = contact.ptOnA_WorldSpace;
	const Vec3 pointOnB = contact.ptOnB_WorldSpace;

	const float elasticityA = bodyA->m_elasticity;
	const float elasticityB = bodyB->m_elasticity;
	const float elasticity = elasticityA * elasticityB;	// [0,1]

	const float invMassA = bodyA->m_invMass;
	const float invMassB = bodyB->m_invMass;

	const Mat3 invWorldInertiaA = bodyA->GetInverseInertiaTensorWorldSpace();
	const Mat3 invWorldInertiaB = bodyA->GetInverseInertiaTensorWorldSpace();

	const Vec3 normalizedVector = contact.normal;

	const Vec3 comToPointA = pointOnA - bodyA->GetCenterOfMassWorldSpace();
	const Vec3 comToPointB = pointOnB - bodyB->GetCenterOfMassWorldSpace();

	const Vec3 angularJA = (invWorldInertiaA * comToPointA.Cross(normalizedVector)).Cross(comToPointA);
	const Vec3 angularJB = (invWorldInertiaB * comToPointB.Cross(normalizedVector)).Cross(comToPointB);
	const float angularFactor = (angularJA + angularJB).Dot(normalizedVector);

	// get the world space velocity of the motion and rotation
	const Vec3 totalVelocityA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross(comToPointA);
	const Vec3 totalVelocityB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross(comToPointB);

	// apply collision impulses
	const Vec3 totalRelativeVelocity = totalVelocityA - totalVelocityB;
	const float totalImpulseScalar = (1.0f + elasticity) * totalRelativeVelocity.Dot(normalizedVector) / (invMassA + invMassB + angularFactor);
	const Vec3 totalImpulseVector = normalizedVector * totalImpulseScalar;
	bodyA->ApplyImpulse(pointOnA, totalImpulseVector * -1.0f);
	bodyB->ApplyImpulse(pointOnB, totalImpulseVector * 1.0f);



	// The adjustedRelativeVelocity is a vector with the direction of normalizedVector 
	// but with a magnitude equal to the projection of totalRelativeVelocity onto normalizedVector.
	const Vec3 adjustedRelativeVelocity = normalizedVector * normalizedVector.Dot(totalRelativeVelocity);
	// Find the tangential vector, which has a direction parallel to the contact surface 
	// and a magnitude representing the component of the totalRelativeVelocity in that tangential direction
	const Vec3 tagentialVector = totalRelativeVelocity - adjustedRelativeVelocity;
	// Get a unit vector representing only the tangential direction.
	Vec3 unitTangentialVector = tagentialVector;
	unitTangentialVector.Normalize();

	const Vec3 inertiaA = (invWorldInertiaA * ra.Cross(unitTangentialVector)).Cross(ra);
	const Vec3 inertiaB = (invWorldInertiaB * rb.Cross(unitTangentialVector)).Cross(rb);
	const float invInertia = (inertiaA + inertiaB ).Dot(unitTangentialVector);
	// calculate the tangential impulse for friction
	const float reducedMass = 1.0f / (bodyA->m_invMass + bodyB->m_invMass + invInertia);
	const Vec3 impulseFriction = tagentialVector * reducedMass * friction ;
	// apply kinetic friction impulses
	bodyA->ApplyImpulse(ptOnA, impulseFriction * -1.0f);
	bodyB->ApplyImpulse(ptOnB, impulseFriction * 1.0f);



	// let's also move our colliding objects to just outside of each other
	const float proportionOfA = bodyA->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);
	const float proportionOfB = bodyB->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);

	const Vec3 vectorBtwTwoContactPoints = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
	bodyA->m_position += vectorBtwTwoContactPoints * proportionOfA;
	bodyB->m_position -= vectorBtwTwoContactPoints * proportionOfB;
}
