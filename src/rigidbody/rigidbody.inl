// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: rigidbody.inl
// Implements data structures and functions for rigid body objects

#ifndef RIGIDBODY_INL
#define RIGIDBODY_INL

#include <Eigen/Core>
#include "../math/eigenutils.inl"
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
	Vector3f centerOfMass;
	Matrix3f inertia;
	SpatialMatrix6f spatialInertia;
	bool fixed;
};

//Forward declarations for externed inlineable methods
extern inline rigidBody createRigidBody(const float& mass, const Vector3f& centerOfMass, 
							  			const Vector3f& gyrationRadii, bool fixed);
extern inline rigidBody createRigidBody(const float& mass, const Vector3f& centerOfMass, 
							  			const Matrix3f& inertia, bool fixed);
extern inline rigidBody joinRigidBodies(const rigidBody& rb1, const rigidBody& rb2);

//====================================
// Function Implementations
//====================================

//Create rigid body from mass, center of mass (in body coords), and the radii of gyration at 
//the center of mass
rigidBody createRigidBody(const float& mass, const Vector3f& centerOfMass, const Vector3f& gyrationRadii, 	
						  bool fixed){
	Matrix3f inertia = createMatrix3f(gyrationRadii[0], 0.0f, 0.0f,
						  	 		  0.0f, gyrationRadii[1], 0.0f,
						  	 		  0.0f, 0.0f, gyrationRadii[2]);
	return createRigidBody(mass, centerOfMass, inertia, fixed);
}

//Create rigid body from mass, center of mass, and inertia matrix
rigidBody createRigidBody(const float& mass, const Vector3f& centerOfMass, const Matrix3f& inertia, 	
						  bool fixed){
	rigidBody rb;
	rb.fixed = fixed;
	rb.mass = mass;
	rb.centerOfMass = centerOfMass;
	rb.inertia = inertia;
	Matrix3f centerOfMassCrossed = createMatrix3f(0.0f, -centerOfMass[2], centerOfMass[1],
	         					 				  centerOfMass[2], 0.0f, -centerOfMass[0],
	         						 			  -centerOfMass[1], centerOfMass[0], 0.0f);	
	Matrix3f massCOMC = mass * centerOfMassCrossed;
	Matrix3f massCOMCT = massCOMC.transpose();
	Matrix3f parallelAxis = massCOMC * centerOfMassCrossed.transpose();
	Matrix3f inertiaPA = inertia + parallelAxis;
	rb.spatialInertia = SpatialMatrix6f(
								inertiaPA(0,0), inertiaPA(0,1), inertiaPA(0,2), 
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
rigidBody joinRigidBodies(const rigidBody& rb1, const rigidBody& rb2){

}

}

#endif