//
//  Broadphase.cpp
//
#include "Broadphase.h"



int CompareSAP(const void* lhs, const void* rhs) {
	const psuedoBody_t* left = reinterpret_cast<const psuedoBody_t*>(lhs);
	const psuedoBody_t* right = reinterpret_cast<const psuedoBody_t*>(rhs);

	if (left->value < right->value)
		return -1;
	return 1;
}
void SortBodiesBounds(const Body* bodies, const int numBodies, psuedoBody_t* sortedArray, const float deltaSecond) {
	Vec3 axis = Vec3(1, 1, 1);  // target axis
	axis.Normalize();

	for (int currentBodyIndex = 0; currentBodyIndex < numBodies; ++currentBodyIndex) {
		const Body& body = bodies[currentBodyIndex];
		Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

		// expand the bounds by the linear velocity
		bounds.Expand(bounds.mins + body.m_linearVelocity * deltaSecond);
		bounds.Expand(bounds.maxs + body.m_linearVelocity * deltaSecond);

		const float epsilon = 0.01f;
		bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * epsilon);
		bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * epsilon);
		
		sortedArray[currentBodyIndex * 2 + 0].id = currentBodyIndex;
		sortedArray[currentBodyIndex * 2 + 0].value = axis.Dot(bounds.mins);
		sortedArray[currentBodyIndex * 2 + 0].isMin = true;

		sortedArray[currentBodyIndex * 2 + 1].id = currentBodyIndex;
		sortedArray[currentBodyIndex * 2 + 1].value = axis.Dot(bounds.maxs);
		sortedArray[currentBodyIndex * 2 + 1].isMin = false;
	}

	qsort(sortedArray, numBodies * 2, sizeof(psuedoBody_t), CompareSAP);
}
void BuildPairs(std::vector<collisionPair_t>& collisionPairs, const psuedoBody_t* sortedBodies, const int numBodies) {
	collisionPairs.clear();

	// now that the bodies are sorted, build the collision pairs
	const int doubleNumBodies = numBodies * 2;
	for (int targetBodyIndex = 0; targetBodyIndex < doubleNumBodies; ++targetBodyIndex) {
		const psuedoBody_t& targetBody = sortedBodies[targetBodyIndex];
		if (targetBody.isMin == false)
			continue;

		collisionPair_t pair;
		pair.a = targetBody.id;

		for (int currentBodyIndex = targetBodyIndex + 1; currentBodyIndex < doubleNumBodies; ++currentBodyIndex) {
			const psuedoBody_t& currentBody = sortedBodies[currentBodyIndex];
			if (currentBody.id == targetBody.id)
				break;
			if (currentBody.isMin == false)
				continue;

			// create a new pair
			pair.b = currentBody.id;
			collisionPairs.push_back(pair);
		}
	}
}
void SweepAndPrune1D(const Body* bodies, const int numBodies, std::vector<collisionPair_t>& finalPairs, const float deltaSecond) {
	psuedoBody_t* sortedBodies = reinterpret_cast<psuedoBody_t*>(alloca(sizeof(psuedoBody_t) * numBodies * 2));

	SortBodiesBounds(bodies, numBodies, sortedBodies, deltaSecond);
	BuildPairs(finalPairs, sortedBodies, numBodies);
}


/*
====================================================
BroadPhase
====================================================
*/
void BroadPhase( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float deltaSecond ) {
	finalPairs.clear();
	SweepAndPrune1D(bodies, num, finalPairs, deltaSecond);
}
