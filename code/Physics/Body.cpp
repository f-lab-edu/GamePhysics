//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
	m_position(0.0f),
	m_orientation(0.0f, 0.0f, 0.0f, 1.0f),
	m_linearVelocity(0.0f),
	m_shape( NULL ) {
}

Vec3 Body::GetCenterOfMassWorldSpace() const {
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}
Vec3 Body::GetCenterOfMassModelSpace() const {
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}
Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const {
	Vec3 tmp = worldPt - GetCenterOfMassWorldSpace();
	Quat inverseOrient = m_orientation.Inverse();
	Vec3 bodySpace = inverseOrient.RotatePoint(tmp);
	return bodySpace;
}
Vec3 Body::BodySpaceToWorldSpace(const Vec3& worldPt) const {
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(worldPt);
	return worldSpace;
}


Mat3 Body::GetInverseInertiaTensorBodySpace() const {
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	return invInertiaTensor;
}
Mat3 Body::GetInverseInertiaTensorWorldSpace() const {
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	Mat3 orient = m_orientation.ToMat3();
	invInertiaTensor = orient * invInertiaTensor * orient.Transpose();
	return invInertiaTensor;
}


void Body::ApplyImpulseLinear(const Vec3& linearImpulse) {
	if (0.0f == m_invMass)
		return;

	m_linearVelocity += linearImpulse * m_invMass;
}

void Body::ApplyImpulseAngular(const Vec3& angularImpulse) {
	if (0.0f == m_invMass)
		return;

	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * angularImpulse;

	// if the angular velocity is too high, modify it to the arbitrary limit
	const float maxAngularSpeed = 30.0f;
	if (m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed) {
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}
