//
//  world.h
//  testBullet
//
//  Created by Marco Bedulli on 4/17/14.
//  Copyright (c) 2014 Marco Bedulli. All rights reserved.
//

#ifndef __testBullet__world__
#define __testBullet__world__
#include <JavaVM/jni.h>
#include <iostream>
#include <map>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>




using namespace std;

class world{
public:
    
    void CreateWorld();
    void UpdateWorld(float t);
    void addBoxCollision(string name, btVector3 size, btScalar mass,btVector3 position, btQuaternion rotation);
    void addMeshTriangleShape(string name, btVector3 arrayVector[], btScalar mass,btVector3 position, btQuaternion rotation,int numVertex);
    float* getOpenGlMatrix(string name,float matrix[]) ;
    void ApplyForce(string name,btVector3 force,btVector3 rel_pos);
    void ApplyTorque(string name, btVector3 force);
	void ApplyTorqueInpulse(string name, btVector3 torque);
    void ApplyCentralImpulse(string name, btVector3 impulse);
    
	void ClearForces(string name);
    
	void setLinearVelocity(string name,btVector3 lin_vel);
	void setAngularVelocity(string name,btVector3 ang_vel);
    
	btVector3 getLinearVelocity(string name);
    
    btVector3 getAngularVelocity(string name);
    
    void translate(string name,btVector3 position);
	
	void setMotionState(string name,btMotionState *motionState);
    
    void addCallBack(ContactAddedCallback ptFun);
    
    //TO ADD
    
    void setFriction(string name,float f);
    void setRestitution(string name,float f);
    void setUserId(string name, int idS);


private:
    
    
    btDiscreteDynamicsWorld* dynamicsWorld;
    
    map<string, btRigidBody*> myMap;
    
    btRigidBody* addCube(btVector3 size, btScalar mass, btVector3 position,btQuaternion rotation);
    
    btTriangleMesh * editTriangleMeshes(btVector3 arrayVector[], int length);
    
    btRigidBody* meshCollisionShape(btTriangleMesh *mTriMesh, btScalar mass,btVector3 position, btQuaternion rotation);
    
    void addRigidBody(btRigidBody* myRigid, string name);
    
    void call(btManifoldPoint& cp,
              const btCollisionObjectWrapper* colObj0,
              int partId0,
              int index0,
              const btCollisionObjectWrapper* colObj1,
              int partId1,
              int index1);
  
    
 

};

#endif


