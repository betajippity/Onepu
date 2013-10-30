// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: rigviewer.hpp
// OpenGL viewer for rigs

#ifndef RIGVIEWER_HPP
#define RIGVIEWER_HPP

#define GLEW_STATIC

#include <omp.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>
#include "../math/eigenmathutils.inl"
#include "../rigidbody/rig.inl"

enum vbotype{QUADS, TRIANGLES, LINES};

using namespace std;
using namespace Eigen;
using namespace spatialmathCore;

namespace viewerCore {

struct vboData{
	GLuint vboID;
	int size;
	vbotype type;
	evec3 color;
};

//Used just for tracking OpenGL viewport camera position/keeping in sync with rendercam
struct glCamera{
	evec2 mouseOld;
	evec3 rotate;
	evec3 translate;
	int currentKey;
	int currentMouseClick;
	float rotateSpeed;
	float zoomSpeed;
	float panSpeed;

	//Initializer
	glCamera(): mouseOld(evec2(0.0f,0.0f)), 
				rotate(evec3(0.0f,0.0f,0.0f)), 
				translate(evec3(0.0f,0.0f,0.0f)),
				currentKey(0),
				currentMouseClick(0),
				rotateSpeed(0.2f),
				zoomSpeed(0.1f),
				panSpeed(0.1f){};
};

//====================================
// Class Declarations
//====================================

class rigviewer {
	public:
		rigviewer(rigidbodyCore::rig* newr);
		~rigviewer();

		void launch();
	private:
		//Initialize stuff
		void init();

		//Main draw functions
		void mainLoop();
		void updateInputs();

		//VBO stuff
		vboData createVBO(vboData data, float* vertices, int numberOfVertices, vbotype type);

		//OMP Threading Stuff
		void threadManager();
		void consoleThread();
		void glThread();

		//Interface callbacks
		static void errorCallback(int error, const char* description);		
		static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

		//Cached data
		evec2 resolution;
		GLFWwindow* window;
		vector<vboData> vbos;
		glCamera cam;
		rigidbodyCore::rig* r;
};
}

#endif
