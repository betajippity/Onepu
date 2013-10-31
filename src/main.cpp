// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: main.cpp
// Entry point for ONEPU

#include <Eigen/Core>
#include "math/spatialmath.inl"
#include "math/spatialmathutils.inl"
#include "rig/rig.inl"
#include "viewer/rigviewer.hpp"

using namespace std;
using namespace Eigen;
using namespace spatialmathCore;
using namespace rigCore;

int main(int argc, char** argv){

	cout << "===================================================" << endl;
	cout << "Onepu: Featherstone's Articulated Body Algorithm" << endl;
	cout << "Version 0.1.13.44a" << endl;
	cout << "Copyright (C) Yining Karl Li. All rights reserved." << endl;
	cout << "===================================================" << endl;
	cout << "" << endl;

	// rigidbodyCore::rig test = rigidbodyCore::createRig();
	rigCore::rig* test = rigCore::createRig();
	rigidBody rb1 = createRigidBody(1.0f, evec3(0.5f, 0.0f, 0.0f), evec3(1,1,1), false, 0);
	joint j1 = createJoint(evec3(0,0,1), jointRevolute);
	int rb1ID = addBodyToRig(*test, 0, createSpatialRotate(17,2), j1, rb1);

	rigidBody rb2 = createRigidBody(1.0f, evec3(0.0f, 0.5f, 0.0f), evec3(1,1,1), false, rb1ID);
	joint j2 = createJoint(evec3(0,0,1), jointRevolute);
	int rb2ID = addBodyToRig(*test, rb1ID, createSpatialTranslate(evec3(1,0,0)), j2, rb2);

	rigidBody rb3 = createRigidBody(0.0f, evec3(0.5f, 0.0f, 0.0f), evec3(1,1,1), false, rb2ID);
	joint j3 = createJoint(evec3(0,0,1), jointRevolute);
	int rb3ID = addBodyToRig(*test, rb2ID, createSpatialTranslate(evec3(0,1,0)), j3, rb3);

	for(int i=0; i<test->parentToJointTransforms.size(); i++){
		cout << "node " << i << endl;
		cout << test->parentToJointTransforms[i].translation[0] << " " <<
				test->parentToJointTransforms[i].translation[1] << " " <<
				test->parentToJointTransforms[i].translation[2] << endl;;
	}
	//rigidBody rb3 = createRigidBody(dd);
	 
	viewerCore::rigviewer* viewer = new viewerCore::rigviewer(test); 
	viewer->launch();
}
