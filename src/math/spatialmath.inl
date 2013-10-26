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

namespace spatialmathCore {
//====================================
// Class and Function Declarations
//====================================
class SpatialVector6f;
class SpatialMatrix6f;
class SpatialInertia6f;
class SpatialTransform6f;

//====================================
// Class Implementations
//====================================

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

class SpatialInertia6f{
	public:
		float mass;
		Vector3f centerOfMass;
		Matrix3f inertia;

		SpatialInertia6f(){
			mass = 0.0f;
			centerOfMass = Vector3f::Zero();
			inertia = Matrix3f::Zero();
		};

		SpatialInertia6f(const float& m, const Vector3f& com, const Matrix3f& i){
			mass = m;
			centerOfMass = com;
			inertia = i;
		};

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

//====================================
// Function Implementations
//====================================

}

//====================================
// Eigen Specialization Stuff
//====================================

//Specialize STL's vectors for Eigen3 based spatial data types to deal with Eigen3 alignment issues
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::SpatialVector6f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::SpatialMatrix6f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::SpatialTransform6f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(spatialmathCore::SpatialInertia6f)

#endif
