// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: featherstone.inl
// Core Featherstone's Articulated Body Algorithm implementation file

#ifndef FEATHERSTONE_INL
#define FEATHERSTONE_INL

#include <Eigen/Core>
#include "../math/spatialmath.inl"
#include "../math/spatialmathutils.inl"
#include "../rig/rig.inl"

#define EIGEN_DEFAULT_TO_ROW_MAJOR

using namespace std;
using namespace Eigen;
using namespace spatialmathCore;

namespace physicsCore {
//====================================
// Function Declarations
//====================================

extern inline void featherstoneABA(rigCore::rig& r, const evecX& jointStateVector, 
								   const evecX& jointVelocityVector, evecX& jointAccelerationVector, 
								   const evecX& torques);
extern inline void jcalc(rigCore::rig& r, const int& jointID, stransform6& jointTransform, 
						 svec6& motionSubspace, svec6& jointVelocity, svec6& jointAcceleration, 
						 const float& state, const float& velocity);

//====================================
// Function Implementations
//====================================

void jcalc(rigCore::rig& r, const int& jointID, stransform6& jointTransform, svec6& motionSubspace, 
		   svec6& jointVelocity, svec6& jointAcceleration, const float& state, const float& velocity){
	//assign some things to names that match the book for easier coding
	svec6 S = motionSubspace;
	svec6 cj = jointAcceleration;
	svec6 vJ = jointVelocity;
	stransform6 Xj = jointTransform;
	rigCore::joint j = r.joints[jointID];
	float q = state;
	float qdot = velocity;

	//set joint axis and joint acceleration for rhenomic joints (RBDA pg. 55)
	S = j.axis0;
	cj = svec6::Zero();

	//calculate joint transforms
	if(j.type==rigCore::jointPrismatic){
		evec3 trans = evec3(j.axis0[3]*q, j.axis0[4]*q, j.axis0[5]*q);
		Xj = createSpatialTranslate(trans);
	}else if(j.type==rigCore::jointRevolute){
		evec3 rotate = evec3(j.axis0[0], j.axis0[1], j.axis0[2]);
		Xj = createSpatialRotate(q, rotate);
	}
	vJ = S*qdot;

	//assign things back to original names
	motionSubspace = S;
	jointAcceleration = cj;
	jointTransform = Xj;
	jointVelocity = vJ;
}

void featherstoneABA(rigCore::rig& r, const evecX& jointStateVector, const evecX& jointVelocityVector, 
					 evecX& jointAccelerationVector, const evecX& jointTorqueVector){
	//assign some things to names that match the book for easier coding
	evecX Q = jointStateVector;
	evecX Qdot = jointVelocityVector;
	evecX Qdotdot = jointAccelerationVector;
	evecX tau = jointTorqueVector;
	vector<svec6> v = r.spatialVelocities;

	svec6 sgravity(0.0f, 0.0f, 0.0f, r.rootForce[0], r.rootForce[1], r.rootForce[2]);
	v[0] = v[0].Zero();

	int numberOfBodies = r.bodies.size();

	//First loop: calculate velocity-product accelerations and rigid-body bias forces
	for(int i=0; i<numberOfBodies; i++){
		stransform6 Xj;
		svec6 vj;
		svec6 cj;
		int lambda = r.parentIDs[i];



	}



	//assign things back to original names
	jointAccelerationVector = Qdotdot;
	r.spatialVelocities = v;
}
}

#endif
