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
	//rig structure data
	vector<int> parentIDs; //lambda
	vector< vector<int> > childrenIDs; //mu
	int numberOfDegreesOfFreedom;
	vec3 rootForce;
	vector<svec6> spatialVelocities; //v
	vector<svec6> spatialAccelerations; //a
	//joint data
	vector<joint> joints;
	vector<svec6> jointAxes; //S
	vector<stransform6> parentToJointTransforms; //X_T
	vector<int> fixedJointCounts;
	//dynamics data
	vector<svec6> velocityDrivenSpatialAccelerations; //c
	vector<smat6> rbSpatialInertias; //IA
	vector<svec6> spatialBiasForces; //pA
	vector<svec6> tempUi; //U_i
	vecX tempdi; //D_i
	vecX tempu; //u
	vector<svec6> rbInternalForces; //f
	vector<sinertia6> spatialInertiasPerRb; //Ic
	//rb data
	vector<stransform6> parentToCurrentTransform; //X_lambda
	vector<stransform6> baseToRBFrameTransform; //X_base
	vector<rigidBody> bodies;
};

extern inline rig createRig();
extern inline int addBodyToRig(rig& r, const int& parentID, const stransform6& jointFrame,
							   const joint& j, const rigidBody& rb);
extern inline int addBodyFixedJointToRig(rig& r, const int& parentID, const stransform6& jointFrame,
							   			 const rigidBody& rb);
extern inline int addBodyMultiJointToRig(rig& r, const int& parentID, const stransform6& jointFrame,
							   			 const joint& j, const rigidBody& rb);

//====================================
// Function Implementations
//====================================

//Create a rig with a default root node and joint
rig createRig(){
	rig r;

	//set up a bunch of zeroed out crap for root node/joint
	r.parentIDs.push_back(0);
	r.childrenIDs.push_back(vector<int>());
	r.numberOfDegreesOfFreedom = 0;

	//set some initial velocities and whatnot
	r.rootForce = vec3(0.0f, -9.8f, 0.0f); //Earth gravity!
	r.spatialVelocities.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.spatialAccelerations.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));

	//create zeroed out root joint
	r.joints.push_back(createJoint());
	r.jointAxes.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.parentToJointTransforms.push_back(stransform6());

	//dynamics and temp stuff
	r.velocityDrivenSpatialAccelerations.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.rbSpatialInertias.push_back(smat6::Identity());
	r.spatialBiasForces.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.tempUi.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.tempdi = vecX::Zero(1);
	r.tempu = vecX::Zero(1);
	r.rbInternalForces.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	sinertia6 rootSI(0.0f, vec3(0.0f, 0.0f, 0.0f), mat3::Zero());
	r.spatialInertiasPerRb.push_back(rootSI);

	//create root rb
	r.parentToCurrentTransform.push_back(stransform6());
	r.baseToRBFrameTransform.push_back(stransform6());
	r.bodies.push_back(createRigidBody());

	return r;
}

int addBodyFixedJointToRig(rig& r, const int& parentID, const stransform6& jointFrame, const rigidBody& rb){
	rigidBody fixedBody = rb;
	fixedBody.fixed = true;
	fixedBody.parentID = parentID;
	fixedBody.parentTransform = jointFrame;
	//if the new node's parent is also fixed, make the new node's parentID the parent's parent
	if(r.bodies[parentID].fixed==true){
		fixedBody.parentID = r.bodies[parentID].parentID;
		fixedBody.parentTransform = r.bodies[parentID].parentTransform;
	}
	//merge node with parent node and update rig's rigidbody vector
	rigidBody newRb = r.bodies[fixedBody.parentID];
	newRb = joinRigidBodies(newRb, rb, fixedBody.parentTransform);
	r.bodies[fixedBody.parentID] = newRb;
	r.bodies.push_back(fixedBody);
	return r.bodies.size()-1;
}

int addBodyMultiJointToRig(rig& r, const int& parentID, const stransform6& jointFrame, const joint& j, 
						   const rigidBody& rb){
	//create ghost node for later use
	rigidBody ghostRb = createRigidBody(0.0f, vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 0.0f), false, 
										parentID);
	ghostRb.ghost = true;
	//for each dof, add a ghost node
	int dofCount = j.degreesOfFreedom;
	int ghostParent = parentID;
	joint ghostJoint;
	stransform6 jointFrameTransform;
	for(int i=0; i<dofCount; i++){
		svec6 a = getJointAxis(j, i);
		vec3 rotation(a[0], a[1], a[2]);
		vec3 translation(a[3], a[4], a[5]);	
		//determine whether joint is prismatic or revolute or non of the above
		if(vec3(rotation-vec3(0,0,0)).norm()<=EPSILON){
			ghostJoint = createJoint(translation, jointPrismatic);
		}else if(vec3(translation-vec3(0,0,0)).norm()<=EPSILON){
			ghostJoint = createJoint(rotation, jointRevolute);
		}	
		//joint 0 is transformed by jointFrame, all other joints have zeroed transforms
		if(i==0){
			jointFrameTransform = jointFrame;
		}else{
			jointFrameTransform = stransform6();
		}
		//at last joint, add the real node. else, add a ghost node.
		if(i!=dofCount-1){
			ghostParent = addBodyToRig(r, ghostParent, jointFrameTransform, ghostJoint, ghostRb);
		}
	}
	return addBodyToRig(r, ghostParent, jointFrameTransform, ghostJoint, rb);
}

//Adds node to node with given parent ID, attached with the given joint. Note to self: JointFrame is the 
//transformation from the parent frame to the origin of the joint frame, aka the X_T John was wondering about
int addBodyToRig(rig& r, const int& parentID, const stransform6& jointFrame, const joint& j, 
				 const rigidBody& rb){
	if(j.type==jointFixed){
		return addBodyFixedJointToRig(r, parentID, jointFrame, rb);
	}else if(j.type!=jointPrismatic && j.type!=jointRevolute){
		return addBodyMultiJointToRig(r, parentID, jointFrame, j, rb);
	}
	//make sure joint is defined
	if(j.type==undefined){
		return -100;
	}
	//make sure rb is attached to fixed body's movable parent if parent is fixed
	int movableParentID = parentID;
	stransform6 moveableParentTransform;
	if(r.bodies[parentID].fixed==true){
		movableParentID = r.bodies[parentID].parentID;
		moveableParentTransform = r.bodies[parentID].parentTransform;
	}
	//push back a ton of stuff for rb
	r.parentIDs.push_back(movableParentID);
	r.childrenIDs.push_back(vector<int>());
	r.childrenIDs.at(movableParentID).push_back(r.bodies.size());
	r.parentToCurrentTransform.push_back(stransform6());
	r.baseToRBFrameTransform.push_back(stransform6());
	r.bodies.push_back(rb);
	r.numberOfDegreesOfFreedom = r.numberOfDegreesOfFreedom+1;
	//push back velocities and joints and stuff
	r.spatialVelocities.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.spatialAccelerations.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.joints.push_back(j);
	r.jointAxes.push_back(j.axis0);
	//invert transform to child node's perspective
	r.parentToJointTransforms.push_back(jointFrame*moveableParentTransform); 
	//dynamics stuff
	r.velocityDrivenSpatialAccelerations.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.rbSpatialInertias.push_back(rb.spatialInertia);
	r.spatialBiasForces.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.tempUi.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	r.tempdi = vecX::Zero(r.bodies.size());
	r.tempu = vecX::Zero(r.bodies.size());
	r.rbInternalForces.push_back(svec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	sinertia6 rbSI(rb.mass, rb.centerOfMass, rb.inertia);
	r.spatialInertiasPerRb.push_back(rbSI);
	return r.bodies.size()-1;;
}
}

#endif
