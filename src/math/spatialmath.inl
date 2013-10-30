// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: spatialmath.inl
// Implements spatial matrices and spatial vectors
// This file is based on Martin Felis's RBDL spatial matrix class (http://rbdl.bitbucket.org/)

#ifndef SPATIALMATH_INL
#define SPATIALMATH_INL

#include <Eigen/Core>
#include <Eigen/StdVector>
#include "eigenmathutils.inl"

#define EIGEN_DEFAULT_TO_ROW_MAJOR

using namespace std;
using namespace Eigen;

namespace spatialmathCore {
//====================================
// Class and Function Declarations
//====================================
class svec6;
class smat6;
class sinertia6;
class stransform6;

extern inline sinertia6 spatialMatrixToSpatialInertia(const smat6& m);
extern inline smat6 spatialInertiaToSpatialMatrix(const sinertia6& I);
extern inline svec6 applySpatialTransformToSpatialVector(const svec6& v, const stransform6& t);
extern inline sinertia6 applySpatialTransformToSpatialInertia(const sinertia6& I, const stransform6& t);
extern inline svec6 applySpatialTransformToTranspose(const svec6& v, const stransform6& t);
extern inline svec6 applySpatialTransformToAdjoint(const svec6& v, const stransform6& t);
extern inline smat6 spatialTransformToSpatialMatrix(const stransform6& t);
extern inline smat6 spatialTransformToSpatialMatrixAdjoint(const stransform6& t);
extern inline smat6 spatialTransformToSpatialMatrixTranspose(const stransform6& t);

//====================================
// Class Implementations
//====================================

//6 element vector as described in Featherstone's algorithm. Extended from an Eigen VectorXf.
class svec6 : public Eigen::Matrix<float, 6, 1>{
	public:
		typedef Eigen::Matrix<float, 6, 1> Base;

		template<typename OtherDerived>
			svec6(const Eigen::MatrixBase<OtherDerived>& other): Eigen::Matrix<float, 6, 1>(other){}

		template<typename OtherDerived> svec6& operator= (const Eigen::MatrixBase<OtherDerived>& other){
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE svec6(){}

		EIGEN_STRONG_INLINE svec6(const float& v0, const float& v1, const float& v2,
								  const float& v3, const float& v4, const float& v5){
			Base::_check_template_params();
			(*this) << v0, v1, v2, v3, v4, v5;
		}
};

//6x6 spatial matrix as described in Featherstone's algorithm. Extended from an Eigen MatrixXf.
class smat6 : public Eigen::Matrix<float, 6, 6>{
	public:
		typedef Eigen::Matrix<float, 6, 6> Base;

		template<typename OtherDerived>
			smat6(const Eigen::MatrixBase<OtherDerived>& other): Eigen::Matrix<float, 6, 6>(other){}

		template<typename OtherDerived> smat6& operator= (const Eigen::MatrixBase<OtherDerived>& other){
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE smat6(){}

		EIGEN_STRONG_INLINE smat6(const Scalar& m00, const Scalar& m01, const Scalar& m02, 
								  const Scalar& m03, const Scalar& m04, const Scalar& m05,
								  const Scalar& m10, const Scalar& m11, const Scalar& m12, 
								  const Scalar& m13, const Scalar& m14, const Scalar& m15,
								  const Scalar& m20, const Scalar& m21, const Scalar& m22, 
								  const Scalar& m23, const Scalar& m24, const Scalar& m25,
								  const Scalar& m30, const Scalar& m31, const Scalar& m32, 
								  const Scalar& m33, const Scalar& m34, const Scalar& m35,
								  const Scalar& m40, const Scalar& m41, const Scalar& m42, 
								  const Scalar& m43, const Scalar& m44, const Scalar& m45,
								  const Scalar& m50, const Scalar& m51, const Scalar& m52, 
								  const Scalar& m53, const Scalar& m54, const Scalar& m55){
			Base::_check_template_params();
			(*this)
				<< m00, m01, m02, m03, m04, m05,
				   m10, m11, m12, m13, m14, m15,
				   m20, m21, m22, m23, m24, m25,
				   m30, m31, m32, m33, m34, m35,
				   m40, m41, m42, m43, m44, m45,
				   m50, m51, m52, m53, m54, m55;
		}
};

//Wrapper around intertial information in a format easy to translate to smat6
class sinertia6{
	public:
		//Data
		float mass;
		evec3 centerOfMass;
		emat3 inertia;

		//Operators and functions
		sinertia6(){
			mass = 0.0f;
			centerOfMass = evec3::Zero();
			inertia = emat3::Zero();
		}

		sinertia6(const float& m, const evec3& com, const emat3& i){
			mass = m;
			centerOfMass = com;
			inertia = i;
		}

		svec6 operator* (const svec6 &v) {
			evec3 mv_upper(v[0], v[1], v[2]);
			evec3 mv_lower(v[3], v[4], v[5]);
			evec3 res_upper = inertia * evec3(v[0], v[1], v[2]) + centerOfMass.cross(mv_lower);
			evec3 res_lower = mass * mv_lower - centerOfMass.cross(mv_upper);
			return svec6(res_upper[0], res_upper[1], res_upper[2],
						 res_lower[0], res_lower[1], res_lower[2]);
		}

		sinertia6 operator+ (const sinertia6 &rbi) {
			return sinertia6(mass + rbi.mass, (centerOfMass + rbi.centerOfMass) / (mass + rbi.mass),
							 inertia + rbi.inertia);
		}
};

//Compact version of a 6x6 spatial matrix used to store transformations in a 3x3 matrix and a evec3. 
//Matrix is for rotation, vector is for translation. We can do this since scale isn't needed.
//This idea is borrowed directly from Martin Felis's implementation.
class stransform6{
	public:
		//Data
		emat3 rotation;
		evec3 translation;

		//Operators and functions
		stransform6(){
			rotation = emat3::Identity();
			translation = evec3::Zero();
		}

		stransform6(const emat3& newRotation, const evec3& newTranslation){
			rotation = newRotation;
			translation = newTranslation;
		}

		stransform6 operator* (const stransform6& st) const {
			return stransform6(rotation * st.rotation, 
				 			   st.translation + st.rotation.transpose() * translation);
		}

		void operator*= (const stransform6& st) {
			translation = st.translation + st.rotation.transpose() * translation;
			rotation *= st.rotation;
		}
};

//====================================
// Function Implementations
//====================================

sinertia6 spatialMatrixToSpatialInertia(const smat6& m){
	float mass = m(3,3);
	evec3 centerOfMass = evec3(-m(1,5), m(0,5), -m(0,4));
	emat3 inertia = m.block<3,3>(0,0);
	return sinertia6(mass, centerOfMass, inertia);
}

smat6 spatialInertiaToSpatialMatrix(const sinertia6& I){
	smat6 m;
	m.block<3,3>(0,0) = I.inertia;
	m.block<3,3>(0,3) = vectorCrossMatrix(I.centerOfMass);
	m.block<3,3>(3,0) = -vectorCrossMatrix(I.centerOfMass);
	m.block<3,3>(3,3) = emat3::Identity() * I.mass;
	return m;
}

svec6 applySpatialTransformToSpatialVector(const svec6& v, const stransform6& t){
	evec3 v_translated(v[3] - (t.translation[1] * v[2]) + (t.translation[2] * v[1]),
					   v[4] - (t.translation[2] * v[0]) + (t.translation[0] * v[2]),
					   v[5] - (t.translation[0] * v[1]) + (t.translation[1] * v[0]));
	svec6 v_rotated;
	v_rotated[0] = (t.rotation(0,0) * v[0]) + (t.rotation(0,1) * v[1]) + (t.rotation(0,2) * v[2]);
	v_rotated[1] = (t.rotation(1,0) * v[0]) + (t.rotation(1,1) * v[1]) + (t.rotation(1,2) * v[2]);
	v_rotated[2] = (t.rotation(2,0) * v[0]) + (t.rotation(2,1) * v[1]) + (t.rotation(2,2) * v[2]);
	v_rotated[3] = (t.rotation(0,0) * v_translated[0]) + (t.rotation(0,1) * v_translated[1]) + 
				   (t.rotation(0,2) * v_translated[2]);
	v_rotated[4] = (t.rotation(1,0) * v_translated[0]) + (t.rotation(1,1) * v_translated[1]) + 
				   (t.rotation(1,2) * v_translated[2]);
	v_rotated[5] = (t.rotation(2,0) * v_translated[0]) + (t.rotation(2,1) * v_translated[1]) + 
				   (t.rotation(2,2) * v_translated[2]);
	return v_rotated;
}

//sinertia6(const float& m, const evec3& com, const emat3& i){
sinertia6 applySpatialTransformToSpatialInertia(const sinertia6& I, 
													   const stransform6& t){
	float mass = I.mass;
	evec3 centerOfMass = t.rotation.transpose() * (I.centerOfMass / I.mass) + t.translation;
	emat3 inertia = t.rotation.transpose() * I.inertia * t.rotation - 
				    vectorCrossMatrix(t.translation) * 
				    vectorCrossMatrix(t.rotation.transpose() * I.centerOfMass) -
				    vectorCrossMatrix(t.rotation.transpose() * I.centerOfMass + t.translation * I.mass) * 
				    vectorCrossMatrix(t.translation);
	return sinertia6(mass, centerOfMass, inertia);
}

svec6 applySpatialTransformToTranspose(const svec6& v, const stransform6& t){
	evec3 rotationTransposedV;
	rotationTransposedV[0] = t.rotation(0,0) * v[3] + t.rotation(1,0) * v[4] + t.rotation(2,0) * v[5];
	rotationTransposedV[1] = t.rotation(0,1) * v[3] + t.rotation(1,1) * v[4] + t.rotation(2,1) * v[5];
	rotationTransposedV[2] = t.rotation(0,2) * v[3] + t.rotation(1,2) * v[4] + t.rotation(2,2) * v[5];
	svec6 transpose;
	transpose[0] = (t.rotation(0,0) * v[0]) + (t.rotation(1,0) * v[1]) + (t.rotation(2,0) * v[2]) - 
				   (t.translation[2] * rotationTransposedV[1]) + (t.translation[1] * rotationTransposedV[2]);
	transpose[1] = (t.rotation(0,1) * v[0]) + (t.rotation(1,1) * v[1]) + (t.rotation(2,1) * v[2]) + 
				   (t.translation[2] * rotationTransposedV[0]) - (t.translation[0] * rotationTransposedV[2]);
	transpose[2] = (t.rotation(0,2) * v[0]) + (t.rotation(1,2) * v[1]) + (t.rotation(2,2) * v[2]) - 
				   (t.translation[1] * rotationTransposedV[0]) + (t.translation[0] * rotationTransposedV[1]);
	transpose[3] = rotationTransposedV[0];
	transpose[4] = rotationTransposedV[1];
	transpose[5] = rotationTransposedV[2];
	return transpose;
}

svec6 applySpatialTransformToAdjoint(const svec6& v, const stransform6& t){
	evec3 transformedAdjoint = (t.rotation * (evec3(v[0], v[1], v[2])) - 
					  		   (t.translation.cross(evec3(v[3], v[4], v[5]))));	
	svec6 adjoint;
	adjoint[0] = transformedAdjoint[0];
	adjoint[1] = transformedAdjoint[1];
	adjoint[2] = transformedAdjoint[2];
	adjoint[3] = t.rotation(0,0) * v[3] + t.rotation(0,1) * v[4] + t.rotation(0,2) * v[5];
	adjoint[4] = t.rotation(1,0) * v[3] + t.rotation(1,1) * v[4] + t.rotation(1,2) * v[5];
	adjoint[5] = t.rotation(2,0) * v[3] + t.rotation(2,1) * v[4] + t.rotation(2,2) * v[5];
	return adjoint;
}

smat6 spatialTransformToSpatialMatrix(const stransform6& t){
	emat3 Erx = t.rotation * createEmat3(0.0f, -t.translation[2], t.translation[1],
				   					     t.translation[2], 0.0f, -t.translation[0],
				   					     -t.translation[1], t.translation[0], 0.0f);
	smat6 m;
	m.block<3,3>(0,0) = t.rotation;
	m.block<3,3>(0,3) = emat3::Zero();
	m.block<3,3>(3,0) = -Erx;
	m.block<3,3>(3,3) = t.rotation;
	return m;	
}

smat6 spatialTransformToSpatialMatrixAdjoint(const stransform6& t){
	emat3 Erx = t.rotation * createEmat3(0.0f, -t.translation[2], t.translation[1],
				   					     t.translation[2], 0.0f, -t.translation[0],
				   					     -t.translation[1], t.translation[0], 0.0f);
	smat6 m;
	m.block<3,3>(0,0) = t.rotation;
	m.block<3,3>(0,3) = -Erx;
	m.block<3,3>(3,0) = emat3::Zero();
	m.block<3,3>(3,3) = t.rotation;
	return m;	
}

smat6 spatialTransformToSpatialMatrixTranspose(const stransform6& t){
	emat3 Erx = t.rotation * createEmat3(0.0f, -t.translation[2], t.translation[1],
				   					     t.translation[2], 0.0f, -t.translation[0],
				   					     -t.translation[1], t.translation[0], 0.0f);
	smat6 m;
	m.block<3,3>(0,0) = t.rotation.transpose();
	m.block<3,3>(0,3) = -Erx.transpose();
	m.block<3,3>(3,0) = emat3::Zero();
	m.block<3,3>(3,3) = t.rotation.transpose();
	return m;	
}
}

//====================================
// Eigen Specialization Stuff
//====================================

//Specialize STL's vectors for Eigen3 based spatial data types to deal with Eigen3 alignment issues
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::svec6)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::smat6)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::stransform6)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::sinertia6)

#endif
