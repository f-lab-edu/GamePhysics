//
//	Broadphase.h
//
#pragma once
#include "Body.h"
#include <vector>


struct collisionPair_t {
	int a;
	int b;

	bool operator == ( const collisionPair_t & rhs ) const {
		return ( ( ( a == rhs.a ) && ( b == rhs.b ) ) || ( ( a == rhs.b ) && ( b == rhs.a ) ) );
	}
	bool operator != ( const collisionPair_t & rhs ) const {
		return !( *this == rhs );
	}
};

struct psuedoBody_t {
	int id;
	float value;
	bool isMin;
};

int CompareSAP(const void* lhs, const void* rhs);
void SortBodiesBounds(const Body* boides, const int num, psuedoBody_t* sortedArray, const float deltaSecond);
void BulidPairs(std::vector<collisionPair_t>& collisionPairs, const psuedoBody_t* sortedBodies, const int num);
void SweepAndPrune1D(const Body* bides, const int num, std::vector<collisionPair_t>& finalPairs, const float deltaSecond);
void BroadPhase( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float dt_sec );
