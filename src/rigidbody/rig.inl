// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: rig.inl
// Implements data structures and functions rigs built of joints and rbds

#ifndef RIG_INL
#define RIG_INL

#include <Eigen/Core>
#include "../math/spatialmath.inl"
#include "rigidbody.inl"
#include "joint.inl"
#include "../utilities/utilities.h"

#define EIGEN_DEFAULT_TO_ROW_MAJOR

using namespace std;
using namespace Eigen;
using namespace spatialmathCore;

namespace rigidbodyCore {
//====================================
// Struct and Function Declarations
//====================================

//Defines a rig as a series of joints and rbds
struct rig {
	//Rig structure data
	vector<int> parentIDs; //lambda
	vector< vector<int> > childrenIDs; //mu
	int numberOfDegreesOfFreedom;
	int mostRecentBodyID;	//used to track previous added vody for appending
	vec3 rootGravity;
	vector<svec6> spatialVelocities;
	vector<svec6> spatialAccelerations;
	//Joint data
	vector<joint> joints;
	vector<svec6> jointAxes; //S
	vector<stransform6> parentToJointTransforms; //X_T
	vector<int> fixedJointCounts;
	//RBD data
	vector<svec6> velocityDrivenSpatialAccelerations; //c
	vector<smat6> spatialInertias; //IA
	vector<svec6> spatialViasForces; //pA
	vector<svec6> tempUi; //U_i
	vecx tempdi; //D_i
	vecx tempu; //u
	vector<sinertia6> spatialInertiaPerBodies;

};


}

#endif
