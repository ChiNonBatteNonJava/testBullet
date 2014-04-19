//
//  main.cpp
//  TestBullet
//
//  Created by Marco Bedulli on 4/4/14.
//  Copyright (c) 2014 Marco Bedulli. All rights reserved.
//

#include <iostream>

#include <map>
#include<GLUT/GLUT.h>
#include<OpenGL/gl.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>

#include "main.h"
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include "world.h"
#include "worldManager.h"
#include "vehicle.h"

using namespace std;
worldManager* asd;
/*
int l=0;




void addRigidBody(btRigidBody* myRigid,string name){
    myMap[name]=myRigid;
}

void initWorld(){
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
    
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,-10,0));
    
    
}

btRigidBody* addCube(btVector3 size,btScalar mass,btVector3 position ,btQuaternion rotation){
    
    btCollisionShape* fallShape = new btBoxShape(size);
    btDefaultMotionState* fallMotionState =new btDefaultMotionState(btTransform(rotation,position));
    
    btVector3 fallInertia(0,0,0);
    
    fallShape->calculateLocalInertia(mass,fallInertia);
    
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);
    fallRigidBodyCI.m_restitution=0.1f;
    fallRigidBodyCI.m_friction = 0.0f;
    
    fallRigidBodyCI.m_linearDamping =  .5f;  // Think of this as resistance to translational motion.
    fallRigidBodyCI.m_angularDamping = 0.2f;
    //fallRigidBodyCI.m_
    btRigidBody* myRigidBody = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(myRigidBody);
    l=myRigidBody->getUserIndex();
    myRigidBody->setUserIndex(4);
    myRigidBody->setCollisionFlags(myRigidBody->getCollisionFlags() |
                             btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
    
    
    return myRigidBody;
    
}

btTriangleMesh * editTriangleMeshes(btVector3 arrayVector[],int length){
    btTriangleMesh *mTriMesh = new btTriangleMesh();
    for (int i=0; i < length; i+=3) {
        mTriMesh->addTriangle(arrayVector[i+0],arrayVector[i+1],arrayVector[i+2]);
    }
    return mTriMesh;
}




btRigidBody* meshCollisionShape(btTriangleMesh *mTriMesh,btScalar mass,btVector3 position ,btQuaternion rotation){
    
   
  
    
    btCollisionShape *mTriMeshShape = new btConvexTriangleMeshShape(mTriMesh);
    
    btConvexShape *tmpshape = new btConvexTriangleMeshShape(mTriMesh);
    btShapeHull *hull = new btShapeHull(tmpshape);
    btScalar margin = mTriMeshShape->getMargin();
    hull->buildHull(margin);
    mTriMeshShape->setUserPointer(hull);
    
    btDefaultMotionState* fallMotionStateTriangle =
    new btDefaultMotionState(btTransform(rotation,position));
    
    btVector3 fallInertiaT(0,1,0);
    mTriMeshShape->calculateLocalInertia(mass,fallInertiaT);
    
    
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIT(mass,fallMotionStateTriangle,tmpshape,fallInertiaT);
    fallRigidBodyCIT.m_restitution=1.0f;
    fallRigidBodyCIT.m_friction = 0.0f;
    
    btRigidBody* myNewBody = new btRigidBody(fallRigidBodyCIT);
    dynamicsWorld->addRigidBody(myNewBody);
    return myNewBody;
    
}

void addBoxCollision(string name,btVector3 size,btScalar mass,btVector3 position ,btQuaternion rotation){
    addRigidBody(addCube(size, mass, position , rotation),name);
}
void addMeshTriangleShape(string name,btVector3 arrayVector[],btScalar mass,btVector3 position ,btQuaternion rotation,int numVertex){
    
    btTriangleMesh *mTriMesh=editTriangleMeshes(arrayVector,numVertex);
    addRigidBody(meshCollisionShape(mTriMesh, mass, position , rotation),name);
    
}
bool callbackFunc(btManifoldPoint& cp,
                  const btCollisionObjectWrapper* colObj0,
                  int partId0,
                  int index0,
                  const btCollisionObjectWrapper* colObj1,
                  int partId1,
                  int index1)
{
    
    int asd=colObj0->getCollisionObject()->getUserIndex();
    int asd2= colObj1->getCollisionObject()->getUserIndex();
    
    return false;
}
*/
float angle=0;
void display(void)
{
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90,1, 0.1f,100);
    
    gluLookAt(0,25, 0, 1,0,0, 0,1,0);
    glMatrixMode(GL_MODELVIEW);
    
     btScalar m[16];
    glPopMatrix();
    glLoadIdentity();
    asd->UpdateWorld("first", 1/60.f);
  asd->getOpenGlMatrix("first", "first", m);
    
   // asd->setLinearVelocity("first", "first", btVector3(2.0f*sinf(angle),currentSpeed.y(),2.0f*cosf(angle)));
    
    
    glColor3f(1, 0,0);
    glPushMatrix();
    glMultMatrixf((GLfloat*)m);
    
     glPushMatrix();
    glutSolidCube(1);
    
    
    glPopMatrix();
    glLoadIdentity();
   
    /*
    dynamicsWorld->stepSimulation(1/60.f,1);
    btTransform trans;

    glColor3f(1, 0,0);
    
    //myMap["cubo"]->getMotionState()->getWorldTransform(trans);
  
    int number = 123;
    string text = to_string(number);
   
    
 
    
    myMap["cubo"]->getMotionState()->getWorldTransform(trans);
    glPopMatrix();
    glLoadIdentity();
    
    btScalar m[16];
    trans.getOpenGLMatrix(m);
    glPushMatrix();
    glMultMatrixf((GLfloat*)m);
    
    glPushMatrix();
    glutSolidCube(1);
    
  
    glPopMatrix();
    glLoadIdentity(); */
    

    glPopMatrix();
    glLoadIdentity();
    
    asd->getOpenGlMatrix("first", "triangle", m);

  
    glPushMatrix();
    glMultMatrixf((GLfloat*)m);
   glPushMatrix();
    
     glColor3f(1, 1,0);
    glBegin(GL_TRIANGLES);
   glVertex3f(400,0,400);
    glVertex3f(-400,0,400);
   glVertex3f (400,0,-400);
   glVertex3f (-400,0,400);
   glVertex3f (-400,0,-400);
   glVertex3f (400,0,-400);
    
    
    glEnd();
   
    glutSwapBuffers();
}

void reshape(int width, int height)
{
    glViewport(0, 0, width, height);
    // glFrustum(0.0f, width, height, 0.0f, 32.0f, 192.0f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90,width/height, 0.1f,100);
    
    gluLookAt(3, 9, 0, 1,0,0, 0,1,0);
    glMatrixMode(GL_MODELVIEW);
}


void idle(void)
{
    glutPostRedisplay();
}


void callBac(){

}

void keyPressed(unsigned char key, int x, int y){

    curva(key);

}
int main(int argc, char** argv)
{
    

    asd=new worldManager();
    asd->CreateNewWorld("first");
    asd->addBoxCollision("first", "first", btVector3(1,1,1),10,btVector3(5,1,0),btQuaternion(0,0,0,1));
    btVector3 triangles[6];
    triangles[0]=btVector3(400,0,400);
    triangles[1]=btVector3(-400,0,400);
    triangles[2]=btVector3(400,0,-400);
    triangles[3]=btVector3(-400,0,400);
    triangles[4]=btVector3(-400,0,-400);
    triangles[5]=btVector3(400,0,-400);
    asd->addMeshTriangleShape("first","triangle",triangles,0,btVector3(0,0,0),btQuaternion(0,0,0,1),6);
    
    triangles[0]=btVector3(4,0,4);
    triangles[1]=btVector3(-4,2,4);
    triangles[2]=btVector3(4,2,-4);
    triangles[3]=btVector3(-4,2,4);
    triangles[4]=btVector3(-4,0,-4);
    triangles[5]=btVector3(4,2,-4);
    
    
    //asd->addMeshTriangleShape("first","triangle",triangles,0,btVector3(0,-10,0),btQuaternion(0,0,0,1),6);
    
    glutInit(&argc, argv);
    
/*
    
    initWorld();
    
    for (int i=0;i<10;++i){
        cout<<i;
    
    }
        gContactAddedCallback = callbackFunc;

        addBoxCollision("cubo",btVector3(1,1,1),2,btVector3(0,16,1),btQuaternion(0,0,0,1));
        btVector3 triangles[6];
        triangles[0]=btVector3(4,0,4);
        triangles[1]=btVector3(4,0,-4);
        triangles[2]=btVector3(-4,0,-4);
        triangles[3]=btVector3(4,0,4);
        triangles[4]=btVector3(-4,1,-4);
        triangles[5]=btVector3(4,1,-4);
    
        addMeshTriangleShape("triangle",triangles,0,btVector3(0,1,1),btQuaternion(0,0,0,1),6);
    
    
    
    //addBoxCollision("cubo1",btVector3(100,1,100),0,btVector3(0,6,1),btQuaternion(0,0,0,1));

    
    
    
    btQuaternion a;
    float a1=3.4f;
    btScalar s=a1;
    //a.setRotation(<#const btVector3 &axis#>, <#const btScalar &_angle#>)
  
    
    
    */
    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    
    glutCreateWindow("GLUT Program");
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyPressed);
    
    glutMainLoop();
    


    
   // btQuaternion* asd=new btQuaternion();
    delete dynamicsWorld;
    
    return EXIT_SUCCESS;
}







