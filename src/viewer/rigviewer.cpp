// Onepu: Featherstone's Articulated Body Algorithm implementation
// Written by Yining Karl Li
//
// File: rigviewer.cpp
// Implements rigviewer.hpp

#include "rigviewer.hpp"

using namespace viewerCore;
using namespace spatialmathCore;

rigviewer::rigviewer(rigCore::rig* newr){
    resolution = vec2(1000, 1000);
    r = newr;

    timestep = .01f;
    Q = evecX::Zero(r->numberOfDegreesOfFreedom);
    QDot = evecX::Zero(r->numberOfDegreesOfFreedom);
    Tau = evecX::Zero(r->numberOfDegreesOfFreedom);
    QDDot = evecX::Zero(r->numberOfDegreesOfFreedom);
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
    d.x = float(x-cam.mouseOld.x);
    d.y = float(y-cam.mouseOld.y);
    cam.mouseOld.x = x;
    cam.mouseOld.y = y;
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
                cam.rotate.x += d.y * cam.rotateSpeed;
                cam.rotate.y += d.x * cam.rotateSpeed;
            }
            if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == 1){
                cam.translate.z += d.y * cam.zoomSpeed;
            }
            if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == 1){
                cam.translate.x += d.x * cam.panSpeed;
                cam.translate.y -= d.y * cam.panSpeed;
            }  
        }
    }
}

void rigviewer::mainLoop(){
    // physicsCore::featherstoneABA(*r, Q, QDot, QDDot, Tau);
    // physicsCore::integrateVelocities(*r, Q, QDot, QDDot, timestep);

   // cout << QDDot << endl;

    // physicsCore::featherstoneABA(*r, Q, QDot, QDDot, Tau);
    // physicsCore::integrateVelocities(*r, Q, QDot, QDDot, timestep);

    //cout << QDDot << endl;

    while (!glfwWindowShouldClose(window)){

       physicsCore::featherstoneABA(*r, Q, QDot, QDDot, Tau);
        physicsCore::integrateVelocities(*r, Q, QDot, QDDot, timestep);
 
        // cout << Q[0] << " " << Q[1] << " " << Q[2] << endl;

        glClearColor(0.125, 0.125, 0.125, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glTranslatef(cam.translate.x, cam.translate.y, cam.translate.z-10.0f);
        // apply the current rotation
        glRotatef(cam.rotate.x, 1, 0, 0);
        glRotatef(cam.rotate.y, 0, 1, 0);
        glRotatef(cam.rotate.z, 0, 0, 1);
        
        // for(int i=0; i<vbos.size(); i++){
        //     glBindBuffer(GL_ARRAY_BUFFER, vbos[i].vboID);
        //     glVertexPointer(3, GL_FLOAT, 0, NULL);
        //     glEnableClientState(GL_VERTEX_ARRAY);
        //     glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        //     glColor3f(vbos[i].color[0], vbos[i].color[1], vbos[i].color[2]);
        //     if(vbos[i].type==QUADS){
        //         glDrawArrays(GL_QUADS, 0, vbos[i].size/3);
        //     }else if(vbos[i].type==TRIANGLES){
        //         glDrawArrays(GL_TRIANGLES, 0, vbos[i].size/3);
        //     }else if(vbos[i].type==LINES){
        //         glDrawArrays(GL_LINES, 0, vbos[i].size/3);
        //     }
        //     glDisableClientState(GL_VERTEX_ARRAY);
        // }        
        GLUquadric* sphere;
        sphere = gluNewQuadric();

        for(int i=0; i<r->stackedTransforms.size(); i++){
            
            gluQuadricDrawStyle(sphere, GLU_SILHOUETTE);
            glPushMatrix();
                evec4 trans = r->stackedTransforms[i] * evec4(0,0,0,1);
                glTranslatef(trans[0], trans[1], trans[2]);
                glColor3f(1,1,1);
                if(i==0){
                    glColor3f(1,0,0);
                    gluSphere(sphere, .25, 20, 20);
                }else{
                    gluSphere(sphere, .2, 20, 20);
                }
            glPopMatrix();

            if(i>0){
             evec4 trans2 = r->stackedTransforms[i-1] * evec4(0,0,0,1);

                    glLineWidth(2.5); 
                    glColor3f(1.0, 0.0, 0.0);
                    glBegin(GL_LINES);
                    glVertex3f(trans[0], trans[1], trans[2]);
                    glVertex3f(trans2[0], trans2[1], trans2[2]);
                    glEnd();
                }
        }

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
    window = glfwCreateWindow(resolution.x, resolution.y, "ONEPU Rig Viewer", NULL, NULL);
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
    utilityCore::fovToPerspective(fov.x, 1, 1, xBounds, yBounds); 
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
