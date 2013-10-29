// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: main.cpp
// Entry point for ONEPU

#include "utilities/utilities.h"
#include <Eigen/Core>
#include "math/spatialmath.inl"
#include "rigidbody/rig.inl"

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

	rigidbodyCore::rig test = rigidbodyCore::createRig();

}
