//
//	Intersections.h
//
#pragma once
#include "Contact.h"

bool RaySphere(const Vec3& rayStart, const Vec3& rayDirection, const Vec3& sphereCenter, const float sphereRadius, float& time1, float& time2);
bool SphereSphereDynamic(const ShapeSphere* shapeA, const ShapeSphere* shapeB, const Vec3& positionA, const Vec3& positionB, const Vec3& velocityA, const Vec3& velocityB,
						 const float deltaTime, Vec3& pointOnA, Vec3& pointOnB, float timeOfImpact);
bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact );
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact );
