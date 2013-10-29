// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: rigidbody.inl
// Implements data structures and functions for joints

#ifndef JOINT_INL
#define JOINT_INL

#include <Eigen/Core>
#include <vector>
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

//Multijoints are implemented as single joints with zero mass bodies chained on. Cheat-ey, but it works.
enum jointType {
	jointRevolute,
	jointPrismatic,
	jointFixed,
	joint1DoF,
	joint2DoF,
	joint3DoF,
	joint4DoF,
	joint5DoF,
	joint6DoF,
	undefined //not sure if this is actually needed...
};

//Defines a single joint and associated properties
struct joint{
	svec6 axis0;
	svec6 axis1;
	svec6 axis2;
	svec6 axis3;
	svec6 axis4;
	svec6 axis5;
	jointType type;
	int degreesOfFreedom;
};

//Forward declarations for externed inlineable methods
extern inline joint createJoint();
extern inline joint createJoint(const vector<svec6>& axes, const jointType& type);
extern inline joint createJoint(const vec3& axis, const jointType& type);
extern inline joint createJoint(const svec6& axis0);
extern inline joint createJoint(const svec6& axis0, const svec6& axis1);
extern inline joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2);
extern inline joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2, 
								const svec6& axis3);
extern inline joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2, 
								const svec6& axis3, const svec6& axis4);
extern inline joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2, 
								const svec6& axis3, const svec6& axis4, const svec6& axis5);
extern inline svec6 getJointAxis(const joint& j, const int& axisNumber);
inline void blankEmptyAxes(joint& j);

//====================================
// Function Implementations
//====================================

joint createJoint(){
	joint j;
	j.degreesOfFreedom = 0;
	j.type = undefined;
	blankEmptyAxes(j);
	return j;
}

joint createJoint(const vec3& axis, const jointType& type){
	joint j;
	j.degreesOfFreedom = 1;
	svec6 newAxis;
	if(type==jointRevolute){
		newAxis = svec6(axis[0], axis[1], axis[2], 0.0f, 0.0f, 0.0f);
	}else if(type==jointPrismatic){
		newAxis = svec6(0.0f, 0.0f, 0.0f, axis[0], axis[1], axis[2]);
	}
	j.axis0 = newAxis;
	blankEmptyAxes(j);
	return j;
}

void blankEmptyAxes(joint& j){
	for(int i=j.degreesOfFreedom; i<6; i++){
		if(i==0){ j.axis0 = svec6(-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f); }
		else if(i==1){ j.axis1 = svec6(-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f); }
		else if(i==2){ j.axis2 = svec6(-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f); }
		else if(i==3){ j.axis3 = svec6(-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f); }
		else if(i==4){ j.axis4 = svec6(-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f); }
		else if(i==5){ j.axis5 = svec6(-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f); }
	}
}

svec6 getJointAxis(const joint& j, const int& axisNumber){
	int i = max(min(axisNumber,5), 0);
	if(i==0){ return j.axis0; }
	else if(i==1){ return j.axis1; }
	else if(i==2){ return j.axis2; }
	else if(i==3){ return j.axis3; }
	else if(i==4){ return j.axis4; }
	else if(i==5){ return j.axis5; }
}

joint createJoint(const svec6& axis0){
	joint j;
	j.degreesOfFreedom = 1;
	j.type = joint1DoF;
	j.axis0 = axis0;
	blankEmptyAxes(j);
	return j;
}

joint createJoint(const svec6& axis0, const svec6& axis1){
	joint j;
	j.degreesOfFreedom = 2;
	j.type = joint2DoF;
	j.axis0 = axis0; j.axis1 = axis1;
	blankEmptyAxes(j);
	return j;
}

joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2){
	joint j;
	j.degreesOfFreedom = 3;
	j.type = joint3DoF;
	j.axis0 = axis0; j.axis1 = axis1; j.axis2 = axis2;
	blankEmptyAxes(j);
	return j;
}

joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2, const svec6& axis3){
	joint j;
	j.degreesOfFreedom = 4;
	j.type = joint4DoF;
	j.axis0 = axis0; j.axis1 = axis1; j.axis2 = axis2;
	j.axis3 = axis3;
	blankEmptyAxes(j);
	return j;
}

joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2, const svec6& axis3, 
				  const svec6& axis4){
	joint j;
	j.degreesOfFreedom = 5;
	j.type = joint5DoF;
	j.axis0 = axis0; j.axis1 = axis1; j.axis2 = axis2;
	j.axis3 = axis3; j.axis4 = axis4;
	blankEmptyAxes(j);
	return j;
}

joint createJoint(const svec6& axis0, const svec6& axis1, const svec6& axis2, const svec6& axis3, 
				  const svec6& axis4, const svec6& axis5){
	joint j;
	j.degreesOfFreedom = 6;
	j.type = joint6DoF;
	j.axis0 = axis0; j.axis1 = axis1; j.axis2 = axis2;
	j.axis3 = axis3; j.axis4 = axis4; j.axis5 = axis5;
	blankEmptyAxes(j);
	return j;
}

joint createJoint(const vector<svec6>& axes, const jointType& type){
	joint j;
	j.degreesOfFreedom = axes.size();
	for(int i=0; i<j.degreesOfFreedom; i++){
		if(i==0){ j.axis0 = axes[0]; }else if(i==1){ j.axis1 = axes[1]; }
		else if(i==2){ j.axis2 = axes[2]; }else if(i==3){ j.axis3 = axes[3]; }
		else if(i==4){ j.axis4 = axes[4]; }else if(i==5){ j.axis5 = axes[5]; }
	}
	blankEmptyAxes(j);
	j.type = type;
	return j;
}
}

//====================================
// Eigen Specialization Stuff
//====================================

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(rigidbodyCore::joint);

#endif
