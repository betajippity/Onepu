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
#include "eigenutils.inl"
#include "../utilities/utilities.h"

#define EIGEN_DEFAULT_TO_ROW_MAJOR

using namespace std;
using namespace Eigen;

namespace mathCore {
//====================================
// Class and Function Declarations
//====================================
class SpatialVector6f;
class SpatialMatrix6f;
class SpatialInertia6f;
class SpatialTransform6f;

extern inline SpatialInertia6f spatialMatrixToSpatialInertia(const SpatialMatrix6f& m);
extern inline SpatialMatrix6f spatialInertiaToSpatialMatrix(const SpatialInertia6f& I);
extern inline SpatialVector6f applySpatialTransformToSpatialVector(const SpatialVector6f& v, 
																   const SpatialTransform6f& t);
extern inline SpatialInertia6f applySpatialTransformToSpatialInertia(const SpatialInertia6f& I,
																	 const SpatialTransform6f& t);
extern inline SpatialVector6f applySpatialTransformToTranspose(const SpatialVector6f& v, 
															   const SpatialTransform6f& t);
extern inline SpatialVector6f applySpatialTransformToAdjoint(const SpatialVector6f& v, 
															 const SpatialTransform6f& t);
extern inline SpatialMatrix6f spatialTransformToSpatialMatrix(const SpatialTransform6f& t);
extern inline SpatialMatrix6f spatialTransformToSpatialMatrixAdjoint(const SpatialTransform6f& t);
extern inline SpatialMatrix6f spatialTransformToSpatialMatrixTranspose(const SpatialTransform6f& t);

//====================================
// Class Implementations
//====================================

//6 element vector as described in Featherstone's algorithm. Extended from an Eigen VectorXf.
class SpatialVector6f : public Eigen::Matrix<float, 6, 1>{
	public:
		typedef Eigen::Matrix<float, 6, 1> Base;

		template<typename OtherDerived>
			SpatialVector6f(const Eigen::MatrixBase<OtherDerived>& other)
			: Eigen::Matrix<float, 6, 1>(other){}

		template<typename OtherDerived> SpatialVector6f& operator=
										(const Eigen::MatrixBase<OtherDerived>& other){
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE SpatialVector6f(){}

		EIGEN_STRONG_INLINE SpatialVector6f(const float& v0, const float& v1, const float& v2,
											const float& v3, const float& v4, const float& v5){
			Base::_check_template_params();
			(*this) << v0, v1, v2, v3, v4, v5;
		}
};

//6x6 spatial matrix as described in Featherstone's algorithm. Extended from an Eigen MatrixXf.
class SpatialMatrix6f : public Eigen::Matrix<float, 6, 6>{
	public:
		typedef Eigen::Matrix<float, 6, 6> Base;

		template<typename OtherDerived>
			SpatialMatrix6f(const Eigen::MatrixBase<OtherDerived>& other)
			: Eigen::Matrix<float, 6, 6>(other){}

		template<typename OtherDerived> SpatialMatrix6f& operator=
										(const Eigen::MatrixBase<OtherDerived>& other){
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE SpatialMatrix6f(){}

		EIGEN_STRONG_INLINE SpatialMatrix6f(const Scalar& m00, const Scalar& m01, const Scalar& m02, 
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

//Wrapper around intertial information in a format easy to translate to SpatialMatrix6f
class SpatialInertia6f{
	public:
		//Data
		float mass;
		Vector3f centerOfMass;
		Matrix3f inertia;

		//Operators and functions
		SpatialInertia6f(){
			mass = 0.0f;
			centerOfMass = Vector3f::Zero();
			inertia = Matrix3f::Zero();
		}

		SpatialInertia6f(const float& m, const Vector3f& com, const Matrix3f& i){
			mass = m;
			centerOfMass = com;
			inertia = i;
		}

		SpatialVector6f operator* (const SpatialVector6f &v) {
			Vector3f mv_upper(v[0], v[1], v[2]);
			Vector3f mv_lower(v[3], v[4], v[5]);
			Vector3f res_upper = inertia * Vector3f(v[0], v[1], v[2]) + centerOfMass.cross(mv_lower);
			Vector3f res_lower = mass * mv_lower - centerOfMass.cross(mv_upper);
			return SpatialVector6f(res_upper[0], res_upper[1], res_upper[2],
								   res_lower[0], res_lower[1], res_lower[2]);
		}

		SpatialInertia6f operator+ (const SpatialInertia6f &rbi) {
			return SpatialInertia6f(mass + rbi.mass, (centerOfMass + rbi.centerOfMass) / (mass + rbi.mass),
									inertia + rbi.inertia);
		}
};

//Compact version of a 6x6 spatial matrix used to store transformations in a 3x3 matrix and a Vector3f. 
//Matrix is for rotation, vector is for translation. We can do this since scale isn't needed.
//This idea is borrowed directly from Martin Felis's implementation.
class SpatialTransform6f{
	public:
		//Data
		Matrix3f rotation;
		Vector3f translation;

		//Operators and functions
		SpatialTransform6f(){
			rotation = Matrix3f::Identity();
			translation = Vector3f::Zero();
		}

		SpatialTransform6f(const Matrix3f& newRotation, const Vector3f& newTranslation){
			rotation = newRotation;
			translation = newTranslation;
		}

		SpatialTransform6f operator* (const SpatialTransform6f& st) const {
			return SpatialTransform6f(rotation * st.rotation, 
									  st.translation + st.rotation.transpose() * translation);
		}

		void operator*= (const SpatialTransform6f& st) {
			translation = st.translation + st.rotation.transpose() * translation;
			rotation *= st.rotation;
		}
};

//====================================
// Function Implementations
//====================================

SpatialInertia6f spatialMatrixToSpatialInertia(const SpatialMatrix6f& m){
	float mass = m(3,3);
	Vector3f centerOfMass = Vector3f(-m(1,5), m(0,5), -m(0,4));
	Matrix3f inertia = m.block<3,3>(0,0);
	return SpatialInertia6f(mass, centerOfMass, inertia);
}

SpatialMatrix6f spatialInertiaToSpatialMatrix(const SpatialInertia6f& I){
	SpatialMatrix6f m;
	m.block<3,3>(0,0) = I.inertia;
	m.block<3,3>(0,3) = vectorCrossMatrix(I.centerOfMass);
	m.block<3,3>(3,0) = -vectorCrossMatrix(I.centerOfMass);
	m.block<3,3>(3,3) = Matrix3f::Identity() * I.mass;
	return m;
}

SpatialVector6f applySpatialTransformToSpatialVector(const SpatialVector6f& v, const SpatialTransform6f& t){
	Vector3f v_translated(v[3] - (t.translation[1] * v[2]) + (t.translation[2] * v[1]),
						  v[4] - (t.translation[2] * v[0]) + (t.translation[0] * v[2]),
						  v[5] - (t.translation[0] * v[1]) + (t.translation[1] * v[0]));
	SpatialVector6f v_rotated;
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

//SpatialInertia6f(const float& m, const Vector3f& com, const Matrix3f& i){
SpatialInertia6f applySpatialTransformToSpatialInertia(const SpatialInertia6f& I, 
													   const SpatialTransform6f& t){
	float mass = I.mass;
	Vector3f centerOfMass = t.rotation.transpose() * (I.centerOfMass / I.mass) + t.translation;
	Matrix3f inertia = t.rotation.transpose() * I.inertia * t.rotation - 
					   vectorCrossMatrix(t.translation) * 
					   vectorCrossMatrix(t.rotation.transpose() * I.centerOfMass) -
					   vectorCrossMatrix(t.rotation.transpose() * I.centerOfMass + t.translation * I.mass) * 
					   vectorCrossMatrix(t.translation);
	return SpatialInertia6f(mass, centerOfMass, inertia);
}

SpatialVector6f applySpatialTransformToTranspose(const SpatialVector6f& v, const SpatialTransform6f& t){
	Vector3f rotationTransposedV;
	rotationTransposedV[0] = t.rotation(0,0) * v[3] + t.rotation(1,0) * v[4] + t.rotation(2,0) * v[5];
	rotationTransposedV[1] = t.rotation(0,1) * v[3] + t.rotation(1,1) * v[4] + t.rotation(2,1) * v[5];
	rotationTransposedV[2] = t.rotation(0,2) * v[3] + t.rotation(1,2) * v[4] + t.rotation(2,2) * v[5];
	SpatialVector6f transpose;
	transpose[0] = (t.rotation(0,0) * v[0]) + (t.rotation(1,0) * v[1]) + (t.rotation(2,0) * v[2]) - 
				   (t.translation[2] * rotationTransposedV[1]) + 
				   (t.translation[1] * rotationTransposedV[2]);
	transpose[1] = (t.rotation(0,1) * v[0]) + (t.rotation(1,1) * v[1]) + (t.rotation(2,1) * v[2]) + 
				   (t.translation[2] * rotationTransposedV[0]) - 
				   (t.translation[0] * rotationTransposedV[2]);
	transpose[2] = (t.rotation(0,2) * v[0]) + (t.rotation(1,2) * v[1]) + (t.rotation(2,2) * v[2]) - 
				   (t.translation[1] * rotationTransposedV[0]) + 
				   (t.translation[0] * rotationTransposedV[1]);
	transpose[3] = rotationTransposedV[0];
	transpose[4] = rotationTransposedV[1];
	transpose[5] = rotationTransposedV[2];
	return transpose;
}

SpatialVector6f applySpatialTransformToAdjoint(const SpatialVector6f& v, const SpatialTransform6f& t){
	Vector3f transformedAdjoint = (t.rotation * (Vector3f(v[0], v[1], v[2])) - 
					  			  (t.translation.cross(Vector3f(v[3], v[4], v[5]))));	
	SpatialVector6f adjoint;
	adjoint[0] = transformedAdjoint[0];
	adjoint[1] = transformedAdjoint[1];
	adjoint[2] = transformedAdjoint[2];
	adjoint[3] = t.rotation(0,0) * v[3] + t.rotation(0,1) * v[4] + t.rotation(0,2) * v[5];
	adjoint[4] = t.rotation(1,0) * v[3] + t.rotation(1,1) * v[4] + t.rotation(1,2) * v[5];
	adjoint[5] = t.rotation(2,0) * v[3] + t.rotation(2,1) * v[4] + t.rotation(2,2) * v[5];
	return adjoint;
}

SpatialMatrix6f spatialTransformToSpatialMatrix(const SpatialTransform6f& t){
	Matrix3f Erx = t.rotation * createMatrix3f(0.0f, -t.translation[2], t.translation[1],
				   							   t.translation[2], 0.0f, -t.translation[0],
				   							   -t.translation[1], t.translation[0], 0.0f);
	SpatialMatrix6f m;
	m.block<3,3>(0,0) = t.rotation;
	m.block<3,3>(0,3) = Matrix3f::Zero();
	m.block<3,3>(3,0) = -Erx;
	m.block<3,3>(3,3) = t.rotation;
	return m;	
}

SpatialMatrix6f spatialTransformToSpatialMatrixAdjoint(const SpatialTransform6f& t){
	Matrix3f Erx = t.rotation * createMatrix3f(0.0f, -t.translation[2], t.translation[1],
				   							   t.translation[2], 0.0f, -t.translation[0],
				   							   -t.translation[1], t.translation[0], 0.0f);
	SpatialMatrix6f m;
	m.block<3,3>(0,0) = t.rotation;
	m.block<3,3>(0,3) = -Erx;
	m.block<3,3>(3,0) = Matrix3f::Zero();
	m.block<3,3>(3,3) = t.rotation;
	return m;	
}

SpatialMatrix6f spatialTransformToSpatialMatrixTranspose(const SpatialTransform6f& t){
	Matrix3f Erx = t.rotation * createMatrix3f(0.0f, -t.translation[2], t.translation[1],
				   							   t.translation[2], 0.0f, -t.translation[0],
				   							   -t.translation[1], t.translation[0], 0.0f);
	SpatialMatrix6f m;
	m.block<3,3>(0,0) = t.rotation.transpose();
	m.block<3,3>(0,3) = -Erx.transpose();
	m.block<3,3>(3,0) = Matrix3f::Zero();
	m.block<3,3>(3,3) = t.rotation.transpose();
	return m;	
}
}

//====================================
// Eigen Specialization Stuff
//====================================

//Specialize STL's vectors for Eigen3 based spatial data types to deal with Eigen3 alignment issues
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(mathCore::SpatialVector6f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(mathCore::SpatialMatrix6f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(mathCore::SpatialTransform6f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(mathCore::SpatialInertia6f)

#endif
