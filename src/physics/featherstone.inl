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
	svec6 vj = jointVelocity;
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
	vj = S*qdot;

	std::cout << "jcalc " << jointID << std::endl;
	std::cout << vj.transpose() << std::endl;
	std::cout << Xj.translation.transpose() << std::endl;
	std::cout << Xj.rotation << std::endl;

	//assign things back to original names
	motionSubspace = S;
	jointAcceleration = cj;
	jointTransform = Xj;
	jointVelocity = vj;
}

void featherstoneABA(rigCore::rig& r, const evecX& jointStateVector, const evecX& jointVelocityVector, 
					 evecX& jointAccelerationVector, const evecX& jointTorqueVector){
	//assign some things to names that match the book for easier coding
	evecX Q = jointStateVector;
	evecX Qdot = jointVelocityVector;
	evecX Qdotdot = jointAccelerationVector;
	evecX tau = jointTorqueVector;
	vector<svec6> v = r.spatialVelocities;
	vector<stransform6> Xlambda = r.parentToCurrentTransform;
	vector<stransform6> Xt = r.parentToJointTransforms;
	vector<stransform6> Xbase = r.baseToRBFrameTransform;
	vector<svec6> c = r.velocityDrivenSpatialAccelerations;
	vector<smat6> IA = r.rbSpatialInertias;
	vector<svec6> pA = r.spatialBiasForces;
	vector<svec6> U = r.tempUi;
	evecX d = r.tempdi;
	evecX u = r.tempu;
	vector<svec6> S = r.jointAxes;

	svec6 sgravity(0.0f, 0.0f, 0.0f, r.rootForce[0], r.rootForce[1], r.rootForce[2]);
	v[0] = v[0].Zero();

	int numberOfBodies = r.bodies.size();

	//First loop: calculate velocity-product accelerations and rigid-body bias forces
	for(int i=1; i<numberOfBodies; i++){
		stransform6 Xj;
		svec6 vj;
		svec6 cj;
		int lambda = r.parentIDs[i];

		jcalc(r, i, Xj, r.jointAxes[i], vj, cj, Q[i-1], Qdot[i-1]);
		Xlambda[i] = Xj * Xt[i];

		if(lambda!=0){
			Xbase[i] = Xlambda[i] * Xbase[lambda];
		}else{
			Xbase[i] = Xlambda[i];
		}

		v[i] = applySpatialTransformToSpatialVector(v[lambda], Xlambda[i]) + vj;

		cout << v[i].transpose() << endl;

		c[i] = cj + crossm(v[i],vj);
		IA[i] = r.bodies[i].spatialInertia;
		pA[i] = crossf(v[i], IA[i]*v[i]);

		cout << c[i].transpose() << endl;
		cout << IA[i] << endl;
		cout << pA[i].transpose() << endl;
	}

	//Second loop: calculate articulated-body inertias and bias forces
	for(int i=numberOfBodies-1; i>0; i--){
		U[i] = IA[i] * S[i];
		d[i] = S[i].dot(U[i]);
		u[i] = tau[i-1] - S[i].dot(pA[i]);

		int lambda = r.parentIDs[i];
		if(lambda!=0){
			smat6 tempIA = IA[i] - U[i] * (U[i] / d[i]).transpose();
			svec6 temppA = pA[i] + tempIA * c[i] + U[i] * u[i] / d[i];
			IA[lambda].noalias() += spatialTransformToSpatialMatrixTranspose(Xlambda[i]) * tempIA * 
									spatialTransformToSpatialMatrix(Xlambda[i]);
			pA[lambda].noalias() += applySpatialTransformToTranspose(temppA, Xlambda[i]);

			cout << IA[lambda] << endl;
			cout << pA[lambda].transpose() << endl;
		}
	}

	//assign things back to original names
	jointAccelerationVector = Qdotdot;
	r.spatialVelocities = v;
	r.parentToCurrentTransform = Xlambda;
	r.parentToJointTransforms = Xt;
	r.baseToRBFrameTransform = Xbase;
	r.velocityDrivenSpatialAccelerations = c;
	r.rbSpatialInertias = IA;
	r.spatialBiasForces = pA;
	r.tempUi = U;
	r.tempdi = d;
	r.tempu = u;
	r.jointAxes = S;
}
}

#endif
