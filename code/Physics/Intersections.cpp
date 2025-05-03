//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"




/*
====================================================
Continuous Collision Checking
====================================================
*/
bool RaySphere(const Vec3& rayStart, const Vec3& rayDirection, const Vec3& sphereCenter, const float sphereRadius, float& time1, float& time2) {
	const Vec3 vectorRayOriginToSphereCenter = sphereCenter - rayStart;
	const float a = rayDirection.Dot(rayDirection);
	// The standard derivation suggests b should be -1.0f * rayDirection.Dot(vectorRayOriginToSphereCenter) for the quadratic form a*t^2 + 2b*t + c = 0.
	// However, this code uses b = rayDirection.Dot(vectorRayOriginToSphereCenter) (positive) and adjusts the solution formula accordingly.
	// This avoids the negation operation.
	const float b = rayDirection.Dot(vectorRayOriginToSphereCenter);
	const float c = vectorRayOriginToSphereCenter.Dot(vectorRayOriginToSphereCenter) - sphereRadius * sphereRadius;

	// because b^2 is equal to (-b)^2, the operation below makes sense
	const float discriminantSqaured = b * b - a * c;
	const float invA = 1.0f / a;

	if (discriminantSqaured < 0)
		return false;		// they don't collide

	const float discriminant = sqrtf(discriminantSqaured);
	// Since b is caluclated without negation, the solution below makes sense although b is not negated
	time1 = invA * (b - discriminant);
	time2 = invA * (b + discriminant);
	return true;
}
bool SphereSphereDynamic(const ShapeSphere* shapeA, const ShapeSphere* shapeB, const Vec3& posA, const Vec3& posB, const Vec3& velA, const Vec3& velB,
	const float deltaTime, Vec3& pointOnA, Vec3& pointOnB, float timeOfImpact) {
	const Vec3 relativeVelocity = velA - velB;

	const Vec3 startPointA = posA;
	const Vec3 endPointA = posA + relativeVelocity * deltaTime;
	const Vec3 rayDirection = endPointA - startPointA;

	float time1 = 0;	// - discriminant
	float time2 = 0;	// + discriminant
	if (rayDirection.GetLengthSqr() < 0.001f * 0.001f) {
		// Ray is too short, just check if already intersecting
		Vec3 vectorAtoB = posB - posA;
		float radiusPlusRadius = shapeA->m_radius + shapeB->m_radius + 0.001f;
		if (vectorAtoB.GetLengthSqr() > radiusPlusRadius * radiusPlusRadius)
			return false;
	}
	else if (!RaySphere(posA, rayDirection, posB, shapeA->m_radius + shapeB->m_radius, time1, time2))
		return false;

	// If the control flow reaches this point, it means that the ray representing 
	// the future movement of sphereA is not currently intersecting with sphereB.
	// However, sphereA will collide with sphereB after some time (time1 or time2).

	// change from [0,1] range to [0,dt] range
	time1 *= deltaTime;
	time2 *= deltaTime;

	// if the collision is only in the past, then there's no future collision in this frame
	if (time2 < 0.0f)
		return false;

	// get the earliest positive time of impact
	timeOfImpact = (time1 < 0.0f) ? 0.0f : time1;

	// if the earliest collision is too far in the future, then there's no collision in this frame
	if (timeOfImpact > deltaTime)
		return false;
	
	// at this point, timeOfImpact can be zero or positive
	// if it's zero, then it means that the two spheres are currently intersecting
	// otherwise, they will collide soon
	
	// get the points on the respective points of collision and return true
	Vec3 newPosA = posA + velA * timeOfImpact;
	Vec3 newPosB = posB + velB * timeOfImpact;
	Vec3 vectorAtoB = newPosB - newPosA;
	vectorAtoB.Normalize();

	pointOnA = newPosA + vectorAtoB * shapeA->m_radius;
	pointOnB = newPosB - vectorAtoB * shapeB->m_radius;
	return true;
}




/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body *bodyA, Body *bodyB, contact_t &contact ) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	const Vec3 vectorAB = bodyB->m_position - bodyA->m_position;
	contact.normal = vectorAB;
	contact.normal.Normalize();

	const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
	const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

	contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphereA->m_radius;
	contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * sphereB->m_radius;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float lengthSquared = vectorAB.GetLengthSqr();
	if (lengthSquared <= (radiusAB * radiusAB))
		return true;

	return false;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float deltaTime, contact_t & contact ) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE) {
		const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
		const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

		Vec3 posA = bodyA->m_position;
		Vec3 posB = bodyB->m_position;

		Vec3 velA = bodyA->m_linearVelocity;
		Vec3 velB = bodyB->m_linearVelocity;

		if (SphereSphereDynamic(sphereA, sphereB, posA, posB, velA, velB, deltaTime, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact.timeOfImpact)) {
			// step bodies forward to get local space collision points
			bodyA->Update(contact.timeOfImpact);
			bodyB->Update(contact.timeOfImpact);
			
			// convert world space contacts to local space
			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			contact.normal = bodyA->m_position - bodyB->m_position;
			contact.normal.Normalize();

			// unwind time step
			bodyA->Update(-contact.timeOfImpact);
			bodyB->Update(-contact.timeOfImpact);
			
			// calculate the separation distance
			Vec3 vectorAtoB = bodyB->m_position - bodyA->m_position;
			contact.separationDistance = vectorAtoB.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
			return true;
		}
	}
	return false;
}
