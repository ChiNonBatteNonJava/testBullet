//
//  world.cpp
//  testBullet
//
//  Created by Marco Bedulli on 4/17/14.
//  Copyright (c) 2014 Marco Bedulli. All rights reserved.
//

#include "world.h"

#include "vehicle.h"



template<class inType,class outType>
outType toArray(int size,inType array[],JNIEnv * env){
};
template<>
jfloatArray toArray(int size,float array[],JNIEnv * env){
	jfloatArray result;
    result = env->NewFloatArray(size);
    if (result == NULL)
        return NULL;
    env->SetFloatArrayRegion(result, 0, size, array);
    return result;
    
};
template<>
jintArray toArray(int size,int array[],JNIEnv * env){
	jintArray result;
    result = env->NewIntArray(size);
    if (result == NULL)
        return NULL;
    env->SetIntArrayRegion(result, 0, size, array);
    return result;
};




void world::addRigidBody(btRigidBody* myRigid, string name) {
     myMap[name] = myRigid;
}
void world::CreateWorld() {
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration =
    new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(
                                                                  collisionConfiguration);
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
    
	btSequentialImpulseConstraintSolver* solver =
    new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver,
                                                collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -10, 0));
   
    initVehicle(dynamicsWorld);
    
}



void world::UpdateWorld(float t){
    dynamicsWorld->stepSimulation(t,1.0f);
    clientMoveAndDisplay();
    
}
btRigidBody* world::addCube(btVector3 size, btScalar mass, btVector3 position,
                     btQuaternion rotation) {
    
	btCollisionShape* fallShape = new btBoxShape(size);
	btDefaultMotionState* fallMotionState = new btDefaultMotionState(
                                                                     btTransform(rotation, position));
    bool isDynamic = (mass != 0.f);
    
	btVector3 fallInertia(0, 0, 0);
	if (isDynamic)
        fallShape->calculateLocalInertia(mass, fallInertia);
    
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,
                                                             fallMotionState, fallShape, fallInertia);
	fallRigidBodyCI.m_restitution = 0.f;
	fallRigidBodyCI.m_friction = 0.f;
	fallRigidBodyCI.m_linearDamping = 1.0f; // Think of this as resistance to translational motion.
	fallRigidBodyCI.m_angularDamping = 0.2f;
	//fallRigidBodyCI.m_
	btRigidBody* myRigidBody = new btRigidBody(fallRigidBodyCI);
   
	dynamicsWorld->addRigidBody(myRigidBody);
	return myRigidBody;
    
}

btTriangleMesh * world::editTriangleMeshes(btVector3 arrayVector[], int length) {
	btTriangleMesh *mTriMesh = new btTriangleMesh();
    
	for (int i = 0; i < length; i+=3) {
		mTriMesh->addTriangle(arrayVector[i + 0], arrayVector[i + 1],
                              arrayVector[i + 2]);

	//	__android_log_print(ANDROID_LOG_INFO, "sometag", "test int = %f", arrayVector[length*i + 0].x());
	}
	return mTriMesh;
}


btRigidBody* world::meshCollisionShape(btTriangleMesh *mTriMesh, btScalar mass,
                                btVector3 position, btQuaternion rotation) {
    
  
    
	btCollisionShape *mTriMeshShape = new btConvexTriangleMeshShape(mTriMesh);
    
	btConvexShape *tmpshape = new btConvexTriangleMeshShape(mTriMesh);
	btShapeHull *hull = new btShapeHull(tmpshape);
	btScalar margin = mTriMeshShape->getMargin();
	hull->buildHull(margin);
	mTriMeshShape->setUserPointer(hull);
    
	btDefaultMotionState* fallMotionStateTriangle = new btDefaultMotionState(
                                                                             btTransform(rotation, position));
    
    
	btVector3 fallInertiaT(0, 1, 0);
	mTriMeshShape->calculateLocalInertia(mass, fallInertiaT);
    
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCIT(mass,
                                                              fallMotionStateTriangle, tmpshape, fallInertiaT);
 
    fallRigidBodyCIT.m_restitution = 1.0f;
	fallRigidBodyCIT.m_friction = 0.5f;
	fallRigidBodyCIT.m_linearDamping = 1.0f; // Think of this as resistance to translational motion.
	fallRigidBodyCIT.m_angularDamping = 0.2f;
	btRigidBody* myNewBody = new btRigidBody(fallRigidBodyCIT);
	dynamicsWorld->addRigidBody(myNewBody);
    
    
	return myNewBody;
    
}

void world::addBoxCollision(string name, btVector3 size, btScalar mass,
                     btVector3 position, btQuaternion rotation) {
	addRigidBody(addCube(size, mass, position, rotation), name);
}
void world::addMeshTriangleShape(string name, btVector3 arrayVector[], btScalar mass,btVector3 position, btQuaternion rotation, int numVertex) {
    
	btTriangleMesh *mTriMesh = editTriangleMeshes(arrayVector, numVertex);
	addRigidBody(meshCollisionShape(mTriMesh, mass, position, rotation), name);
    
}

float* world::getOpenGlMatrix(string name,float matrix[]){

    btTransform trans;
    myMap[name]->getMotionState()->getWorldTransform(trans);
    
    btScalar m[16];
    trans.getOpenGLMatrix(matrix);
 

    return matrix;

}

void world::ApplyForce(string name,btVector3 force,btVector3 rel_pos){
    myMap[name]->applyForce(force, rel_pos);
}

void world::ApplyTorque(string name, btVector3 force){
    myMap[name]->applyTorque(force);
}
void world::ApplyTorqueInpulse(string name, btVector3 torque){
    myMap[name]->applyTorqueImpulse(torque);
}
void world::ApplyCentralImpulse(string name, btVector3 impulse){
    myMap[name]->applyCentralImpulse(impulse);
}
void world::ClearForces(string name){
    myMap[name]->clearForces();
}
void world::setLinearVelocity(string name,btVector3 lin_vel){
    myMap[name]->setLinearVelocity(lin_vel);

}
void world::setAngularVelocity(string name,btVector3 ang_vel){
    myMap[name]->setAngularVelocity(ang_vel);
}

btVector3 world::getLinearVelocity(string name){
    return  myMap[name]->getLinearVelocity();
}

btVector3 world::getAngularVelocity(string name){
      return  myMap[name]->getAngularVelocity();
}

void world::translate(string name,btVector3 position){
    myMap[name]->translate(position);

}

void world::setMotionState(string name,btMotionState *motionState){
    myMap[name]->setMotionState(motionState);

}

void world::setFriction(string name,float f){
    myMap[name]->btCollisionObject::setFriction(f);

}

void world::setRestitution(string name,float f){
    myMap[name]->btCollisionObject::setRestitution(f);
    
}

void world::setUserId(string name, int idS){
    myMap[name]->btCollisionObject::setUserIndex(idS);
}





