//
//  main.h
//  testBullet
//
//  Created by Marco Bedulli on 4/9/14.
//  Copyright (c) 2014 Marco Bedulli. All rights reserved.
//

#include <iostream>

#include <map>
#include<GLUT/GLUT.h>
#include<OpenGL/gl.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>


#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#ifndef testBullet_main_h
#define testBullet_main_h



#endif

btDiscreteDynamicsWorld* dynamicsWorld;
std::map<std::string, btRigidBody*> myMap;
