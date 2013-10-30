// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: eigenmathutils.inl
// A thin wrapper for Eigen that makes some matrix and vector operations easier to deal with

#ifndef EIGENMATHUTILS_INL
#define EIGENMATHUTILS_INL

#include <Eigen/Core>

#define EIGEN_DEFAULT_TO_ROW_MAJOR

using namespace std;
using namespace Eigen;

namespace spatialmathCore {

typedef VectorXf evecX;
typedef Vector3f evec3;
typedef Vector2f evec2;
typedef Vector4f evec4;
typedef Matrix3f emat3;
typedef Matrix4f emat4;

//====================================
// Struct and Function Declarations
//====================================

extern inline emat3 createEmat3(const float& v00, const float& v01, const float& v02,
							    const float& v10, const float& v11, const float& v12,
							    const float& v20, const float& v21, const float& v22);
extern inline emat3 createEmat3(const evec3& v0, const evec3& v1, const evec3& v2);
extern inline emat3 vectorCrossMatrix(const evec3& v);
extern inline emat3 parallelAxis(const emat3& inertia, const float& mass, const evec3& centerOfMass);

//====================================
// Function Implementations
//====================================

emat3 parallelAxis(const emat3& inertia, const float& mass, const evec3& centerOfMass){
	emat3 centerOfMassCrossed = vectorCrossMatrix(centerOfMass);
	return inertia + (mass * centerOfMassCrossed * centerOfMassCrossed.transpose());
}

emat3 vectorCrossMatrix(const evec3& v){
	return createEmat3(0.0f, -v[2], v[1],
					   v[2], 0.0f, -v[0],
					   -v[1], v[0], 0.0f);
}

emat3 createEmat3(const float& v00, const float& v01, const float& v02,
				  const float& v10, const float& v11, const float& v12,
				  const float& v20, const float& v21, const float& v22){
	emat3 m;
	m(0,0) = v00; m(0,1) = v01; m(0,2) = v02;
	m(1,0) = v10; m(1,1) = v11; m(1,2) = v12;
	m(2,0) = v20; m(2,1) = v21; m(2,2) = v22;
	return m;
}

emat3 createEmat3(const evec3& v0, const evec3& v1, const evec3& v2){
	emat3 m;
	m(0,0) = v0[0]; m(0,1) = v0[1]; m(0,2) = v0[2];
	m(1,0) = v1[0]; m(1,1) = v1[1]; m(1,2) = v1[2];
	m(2,0) = v2[0]; m(2,1) = v2[1]; m(2,2) = v2[2];
	return m;
}
}

#endif