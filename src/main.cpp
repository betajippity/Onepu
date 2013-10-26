// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: main.cpp
// Entry point for ONEPU

#include "utilities/utilities.h"
#include <Eigen/Core>
#include "math/spatialmath.inl"
#include "rigidbody/rigidbody.inl"

using namespace std;
using namespace Eigen;
using namespace spatialmathCore;

int main(int argc, char** argv){

	cout << "===================================================" << endl;
	cout << "Onepu: Featherstone's Articulated Body Algorithm" << endl;
	cout << "Version 0.1.13.43a" << endl;
	cout << "Copyright (C) Yining Karl Li. All rights reserved." << endl;
	cout << "===================================================" << endl;
	cout << "" << endl;


	// Matrix3f m3;
	// m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	// Matrix4f m4 = Matrix4f::Identity();
	// VectorXf v6(6);
	// std::cout << "m3\n" << m3 << "\nm4:\n"
	// << m4 << "\nv4:\n" << v6 << std::endl;

	SpatialVector6f sv(1,2,3,4,5,6);
	sv = sv*4.0f;
	cout << sv << endl;
}