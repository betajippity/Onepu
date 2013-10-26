// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: mathutils.inl
// A thin wrapper for Eigen that makes some matrix and vector operations easier to deal with

#ifndef EIGENUTILS_INL
#define EIGENUTILS_INL

#include <Eigen/Core>
#include "../utilities/utilities.h"

#define EIGEN_DEFAULT_TO_ROW_MAJOR

using namespace std;
using namespace Eigen;

//====================================
// Struct and Function Declarations
//====================================

extern inline Matrix3f createMatrix3f(const float& v00, const float& v01, const float& v02,
									  const float& v10, const float& v11, const float& v12,
									  const float& v20, const float& v21, const float& v22);
extern inline Matrix3f createMatrix3f(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2);

//====================================
// Function Implementations
//====================================

Matrix3f createMatrix3f(const float& v00, const float& v01, const float& v02,
					    const float& v10, const float& v11, const float& v12,
					    const float& v20, const float& v21, const float& v22){
	Matrix3f m;
	m(0,0) = v00; m(0,1) = v01; m(0,2) = v02;
	m(1,0) = v10; m(1,1) = v11; m(1,2) = v12;
	m(2,0) = v20; m(2,1) = v21; m(2,2) = v22;
	return m;
}

Matrix3f createMatrix3f(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2){
	Matrix3f m;
	m(0,0) = v0[0]; m(0,1) = v0[1]; m(0,2) = v0[2];
	m(1,0) = v1[0]; m(1,1) = v1[1]; m(1,2) = v1[2];
	m(2,0) = v2[0]; m(2,1) = v2[1]; m(2,2) = v2[2];
	return m;
}

#endif