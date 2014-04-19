//
//  vehicle.h
//  testBullet
//
//  Created by Marco Bedulli on 4/18/14.
//  Copyright (c) 2014 Marco Bedulli. All rights reserved.
//

#ifndef __testBullet__vehicle__
#define __testBullet__vehicle__

#include <iostream>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include<GLUT/GLUT.h>
#include<OpenGL/gl.h>

void initVehicle(btDiscreteDynamicsWorld *world);
void clientMoveAndDisplay();
btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, btDynamicsWorld *world);
void curva(char key);
#endif /* defined(__testBullet__vehicle__) */
