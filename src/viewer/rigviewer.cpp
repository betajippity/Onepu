// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: rigviewer.cpp
// Implements rigviewer.hpp

#include "rigviewer.hpp"
#include "../utilities/utilities.h"

using namespace viewerCore;
using namespace spatialmathCore;

rigviewer::rigviewer(){
    resolution = vec2(1000, 1000);
}

rigviewer::~rigviewer(){

}

void rigviewer::launch(){
    threadManager();
}

void rigviewer::consoleThread(){
    while(true){                
        char choice;
        // cout << "Console test, hit enter: \n";
        // cin >> choice;
        // cout << choice << endl;
    }   
}

vboData rigviewer::createVBO(vboData data, float* vertices, int numberOfVertices, vbotype type){
    data.size = numberOfVertices;
    glGenBuffers(1, &data.vboID);
    glBindBuffer(GL_ARRAY_BUFFER, data.vboID);
    glBufferData(GL_ARRAY_BUFFER, data.size*sizeof(float), vertices, GL_STATIC_DRAW);
    data.type = type;
    return data;
}

void rigviewer::updateInputs(){
    double x; double y;
    glfwGetCursorPos(window, &x, &y);
    vec2 d;
    d[0] = float(x-cam.mouseOld[0]);
    d[1] = float(y-cam.mouseOld[1]);
    cam.mouseOld[0] = x;
    cam.mouseOld[1] = y;
    if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == 1 || 
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == 1 ||
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == 1){

        bool doCamera = false;

        if(glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS || 
           glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS){
            doCamera = true;
        }
        if(doCamera==true){
            if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == 1){
                cam.rotate[0] += d[1] * cam.rotateSpeed;
                cam.rotate[1] += d[0] * cam.rotateSpeed;
            }
            if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == 1){
                cam.translate[2] += d[1] * cam.zoomSpeed;
            }
            if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == 1){
                cam.translate[0] += d[0] * cam.panSpeed;
                cam.translate[1] -= d[1] * cam.panSpeed;
            }  
        }
    }
}

void rigviewer::mainLoop(){
    while (!glfwWindowShouldClose(window)){
        glClearColor(0.125, 0.125, 0.125, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glTranslatef(cam.translate[0], cam.translate[1], cam.translate[2]);
        // apply the current rotation
        glRotatef(cam.rotate[0], 1, 0, 0);
        glRotatef(cam.rotate[1], 0, 1, 0);
        glRotatef(cam.rotate[2], 0, 0, 1);
        
        for(int i=0; i<vbos.size(); i++){
            glBindBuffer(GL_ARRAY_BUFFER, vbos[i].vboID);
            glVertexPointer(3, GL_FLOAT, 0, NULL);
            glEnableClientState(GL_VERTEX_ARRAY);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glColor3f(vbos[i].color[0], vbos[i].color[1], vbos[i].color[2]);
            if(vbos[i].type==QUADS){
                glDrawArrays(GL_QUADS, 0, vbos[i].size/3);
            }else if(vbos[i].type==TRIANGLES){
                glDrawArrays(GL_TRIANGLES, 0, vbos[i].size/3);
            }else if(vbos[i].type==LINES){
                glDrawArrays(GL_LINES, 0, vbos[i].size/3);
            }
            glDisableClientState(GL_VERTEX_ARRAY);
        }
       // drawRays();
        
        glfwSwapBuffers(window);
        glfwPollEvents();
        updateInputs();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
}

void rigviewer::glThread(){
    init();
    mainLoop();
    exit(EXIT_SUCCESS);
}

void rigviewer::threadManager(){
    #pragma omp parallel
    {
        #pragma omp master
        {
            glThread();
        }
        #pragma omp single
        {
            consoleThread();
        }
    }
}

void rigviewer::init(){
    //Camera setup stuff
    vec2 fov = vec2(45.0f, 45.0f);

    //Window setup stuff
    glfwSetErrorCallback(errorCallback);
    if (!glfwInit()){
        exit(EXIT_FAILURE);
    }
    window = glfwCreateWindow(resolution[0], resolution[1], "ONEPU Rig Viewer", NULL, NULL);
    if (!window){
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);   
    glewExperimental = GL_TRUE;
    if(glewInit()!=GLEW_OK){
        exit(EXIT_FAILURE);   
    }

    //camera stuff
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    vec2 xBounds;
    vec2 yBounds;
    utilityCore::fovToPerspective(fov[0], 1, 1, xBounds, yBounds); 
    glFrustum(xBounds[0], xBounds[1], yBounds[0], yBounds[1], 1, 500);
    glMatrixMode(GL_MODELVIEW);

}

void rigviewer::errorCallback(int error, const char* description){
    fputs(description, stderr);
}

void rigviewer::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods){
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS){
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}
