//
//  worldManager.cpp
//  testBullet
//
//  Created by Marco Bedulli on 4/18/14.
//  Copyright (c) 2014 Marco Bedulli. All rights reserved.
//

#include "worldManager.h"

void worldManager::CreateNewWorld(string name){
    
    myMap[name]=new world();
    myMap[name]->CreateWorld();

}

void worldManager::UpdateWorld(string name,float t){
    myMap[name]->UpdateWorld(t);
    
}

void worldManager::addBoxCollision(string world,string name,btVector3 size, btScalar mass,btVector3 position, btQuaternion rotation){
    myMap[world]->addBoxCollision(name,  size,  mass,  position,  rotation);
    
}
void worldManager::addMeshTriangleShape(string world,string name, btVector3 arrayVector[], btScalar mass,btVector3 position, btQuaternion rotation,int numVertex){
    myMap[world]->addMeshTriangleShape( name,  arrayVector,  mass,  position,  rotation,  numVertex);
}


float* worldManager::getOpenGlMatrix(string world,string name,float matrix[]) {
    myMap[world]->getOpenGlMatrix(name, matrix);
    return matrix;
    
}
void worldManager::ApplyForce(string world,string name,btVector3 force,btVector3 rel_pos){
    myMap[world]->ApplyForce( name,  force,  rel_pos);
    
}
void worldManager::ApplyTorque(string world, string name, btVector3 force){
    myMap[world]->ApplyTorque( name,  force);
    
}
void worldManager::ApplyTorqueInpulse(string world, string name, btVector3 torque){
    myMap[world]->ApplyTorqueInpulse(name,torque);
}
void worldManager::ApplyCentralImpulse(string world, string name, btVector3 impulse){
    myMap[world]->ApplyCentralImpulse(name,impulse);
}

void worldManager::ClearForces(string world, string name){
    myMap[world]->ClearForces(name);
}

void worldManager::setLinearVelocity(string world, string name,btVector3 lin_vel){
    myMap[world]->setLinearVelocity(name,lin_vel);
    
}
void worldManager::setAngularVelocity(string world, string name,btVector3 ang_vel){
    myMap[world]->setAngularVelocity( name,  ang_vel);
}

btVector3 worldManager::getLinearVelocity(string world, string name){
    return myMap[world]->getLinearVelocity(name);
    
}

btVector3 worldManager::getAngularVelocity(string world, string name){
     return myMap[world]->getAngularVelocity(name);
    
}

void worldManager::translate(string world,string name,btVector3 position){
    myMap[world]->translate( name,  position);
}

void worldManager::setMotionState(string world,string name,btMotionState *motionState){
    myMap[world]->setMotionState(name,motionState);
    
}
void worldManager::setFriction(string world,string name,float f){
    myMap[world]->setFriction( name,  f);
}
void worldManager::setRestitution(string world,string name,float f){
    myMap[world]->setRestitution( name,  f);
}
void worldManager::setUserId(string world,string name, int idS){
    myMap[world]->setUserId(name,idS);
}


//void addCallBack(string world);