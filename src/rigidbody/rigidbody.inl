// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: rigidbody.inl
// Implements data structures and functions for rigid body objects

#ifndef RIGIDBODY_INL
#define RIGIDBODY_INL

#include <Eigen/Core>
#include "../math/eigenmathutils.inl"
#include "../math/spatialmath.inl"
#include "../utilities/utilities.h"

#define EIGEN_DEFAULT_TO_ROW_MAJOR

using namespace std;
using namespace Eigen;
using namespace spatialmathCore;

namespace rigidbodyCore {
//====================================
// Struct and Function Declarations
//====================================

//Defines a single rigid body and associated properties
struct rigidBody {
	float mass;
	vec3 centerOfMass;
	mat3 inertia;
	smat6 spatialInertia;
	bool fixed;
};

//Forward declarations for externed inlineable methods
extern inline rigidBody createRigidBody(const float& mass, const vec3& centerOfMass, 
							  			const vec3& gyrationRadii, bool fixed);
extern inline rigidBody createRigidBody(const float& mass, const vec3& centerOfMass, 
							  			const mat3& inertia, bool fixed);
extern inline rigidBody joinRigidBodies(const rigidBody& rb1, const rigidBody& rb2, 
										const stransform6& transform);

//====================================
// Function Implementations
//====================================

//Create rigid body from mass, center of mass (in body coords), and the radii of gyration at 
//the center of mass
rigidBody createRigidBody(const float& mass, const vec3& centerOfMass, const vec3& gyrationRadii, 	
						  bool fixed){
	mat3 inertia = createMat3(gyrationRadii[0], 0.0f, 0.0f,
						  	  0.0f, gyrationRadii[1], 0.0f,
						  	  0.0f, 0.0f, gyrationRadii[2]);
	return createRigidBody(mass, centerOfMass, inertia, fixed);
}

//Create rigid body from mass, center of mass, and inertia matrix
rigidBody createRigidBody(const float& mass, const vec3& centerOfMass, const mat3& inertia, bool fixed){
	rigidBody rb;
	rb.fixed = fixed;
	rb.mass = mass;
	rb.centerOfMass = centerOfMass;
	rb.inertia = inertia;
	mat3 centerOfMassCrossed = createMat3(0.0f, -centerOfMass[2], centerOfMass[1],
	         					 		  centerOfMass[2], 0.0f, -centerOfMass[0],
	         						 	  -centerOfMass[1], centerOfMass[0], 0.0f);	
	mat3 massCOMC = mass * centerOfMassCrossed;
	mat3 massCOMCT = massCOMC.transpose();
	mat3 parallelAxis = massCOMC * centerOfMassCrossed.transpose();
	mat3 inertiaPA = inertia + parallelAxis;
	rb.spatialInertia = smat6(inertiaPA(0,0), inertiaPA(0,1), inertiaPA(0,2), 
						 	  massCOMC(0, 0), massCOMC(0, 1), massCOMC(0, 2),
						 	  inertiaPA(1,0), inertiaPA(1,1), inertiaPA(1,2), 
						 	  massCOMC(1, 0), massCOMC(1, 1), massCOMC(1, 2),
						 	  inertiaPA(2,0), inertiaPA(2,1), inertiaPA(2,2), 
						 	  massCOMC(2, 0), massCOMC(2, 1), massCOMC(2, 2),
						 	  massCOMCT(0, 0), massCOMCT(0, 1), massCOMCT(0, 2), mass, 0.0f, 0.0f,
						 	  massCOMCT(1, 0), massCOMCT(1, 1), massCOMCT(1, 2), 0.0f, mass, 0.0f,
						 	  massCOMCT(2, 0), massCOMCT(2, 1), massCOMCT(2, 2), 0.0f, 0.0f, mass);
	return rb;
}

//Creates a single body that behaves the same as if the two given bodies were connected by a fixed joint
//Transform is frame transformation from rb2's origin to rb1's origin
rigidBody joinRigidBodies(const rigidBody& rb1, const rigidBody& rb2, const stransform6& transform){
	float newMass = rb1.mass + rb2.mass;
	vec3 rb2CenterOfMass = (transform.rotation.transpose() * rb2.centerOfMass) + transform.translation;
	vec3 newCenterOfMass = (1.0f / newMass ) * (rb1.mass * rb1.centerOfMass + rb2.mass * rb2CenterOfMass);
	mat3 rb2Inertia = rb2.spatialInertia.block<3,3>(0,0);
	//Transform inertia from rb2 origin to rb2 centerOfMAss
	mat3 rb2COMCrossed = vectorCrossMatrix(rb2.centerOfMass);
	mat3 rb2InertiaCOM = rb2Inertia - rb2.mass * rb2COMCrossed * rb2COMCrossed.transpose();
	//Rotate rb2 inertia to align with frame of rb1, then transform rb2 inertia to origin frame of rb1
	mat3 rb2InertiaCOMRotated = transform.rotation.transpose() * rb2InertiaCOM * transform.rotation;
	mat3 rb2InertiaCOMRotatedToRb1 = parallelAxis(rb2InertiaCOMRotated, rb2.mass, rb2CenterOfMass);
	//Sum inertias and transform result to new center of mass
	mat3 sumOfIntertias = rb1.spatialInertia.block<3,3>(0,0) + rb2InertiaCOMRotatedToRb1;
	mat3 newCOMCrossed = vectorCrossMatrix(newCenterOfMass);
	mat3 newInertia = sumOfIntertias - (newMass * newCOMCrossed * newCOMCrossed.transpose()); 
	return createRigidBody(newMass, newCenterOfMass, newInertia, rb1.fixed);
}
}

#endif