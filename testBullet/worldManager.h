
#ifndef __testBullet__worldManager__
#define __testBullet__worldManager__
#include <JavaVM/jni.h>
#include <iostream>
#include <map>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include "world.h"




using namespace std;

class worldManager{
    
public:
    
    void CreateNewWorld(string name);
    void UpdateWorld(string name,float t);
    
    void addBoxCollision(string world,string name,btVector3 size, btScalar mass,btVector3 position, btQuaternion rotation);
    void addMeshTriangleShape(string world,string name, btVector3 arrayVector[], btScalar mass,btVector3 position, btQuaternion rotation,int numVertex);
    
    
    float* getOpenGlMatrix(string world,string name,float matrix[]) ;
    void ApplyForce(string world,string name,btVector3 force,btVector3 rel_pos);
    void ApplyTorque(string world, string name, btVector3 force);
	void ApplyTorqueInpulse(string world, string name, btVector3 torque);
    void ApplyCentralImpulse(string world, string name, btVector3 impulse);
    
	void ClearForces(string world, string name);
    
	void setLinearVelocity(string world, string name,btVector3 lin_vel);
	void setAngularVelocity(string world, string name,btVector3 ang_vel);
    
	btVector3 getLinearVelocity(string world, string name);
    
    btVector3 getAngularVelocity(string world, string name);
    
    void translate(string world,string name,btVector3 position);
	
	void setMotionState(string world,string name,btMotionState *motionState);
    
    
    void addCallBack(string world);
    
    void setFriction(string world,string name,float f);
    void setRestitution(string world,string name,float f);
    void setUserId(string world,string name, int idS);
    
private:
    map<string,world*> myMap;
    
    
};
#endif

extern "C"{
    
	JNIEXPORT void JNICALL Java_com_project_physics_Update(jfloat t);
    JNIEXPORT void JNICALL Java_com_project_physics_wCreateNewWorld(jstring name);
    JNIEXPORT void JNICALL Java_com_project_physics_wUpdateWorld(jstring name);
    
    JNIEXPORT void JNICALL Java_com_project_physics_waddBoxCollision(jstring world,jobject size, btScalar mass,jobject position, jlong rotation);
    JNIEXPORT void JNICALL Java_com_project_physics_waddMeshTriangleShape(jstring world,jstring name, jobject arrayVector[], jfloat mass,btVector3 position, jlong rotation,jint numVertex);
    
    
    JNIEXPORT jfloatArray JNICALL Java_com_project_physics_wgetOpenGlMatrix(jstring world,jstring name) ;
    JNIEXPORT void JNICALL Java_com_project_physics_wApplyForce(jstring world,jstring name,jobject force,jobject rel_pos);
    JNIEXPORT void JNICALL Java_com_project_physics_wApplyTorque(jstring world, jstring name, jobject force);
	JNIEXPORT void JNICALL Java_com_project_physics_wApplyTorqueInpulse(jstring world, jstring name, jobject torque);
    JNIEXPORT void JNICALL Java_com_project_physics_wApplyCentralImpulse(jstring world, jstring name, jobject impulse);
    
	JNIEXPORT void JNICALL Java_com_project_physics_wClearForces(jstring world, jstring name);
    
	JNIEXPORT void JNICALL Java_com_project_physics_wsetLinearVelocity(jstring world, jstring name,jobject lin_vel);
	JNIEXPORT void JNICALL Java_com_project_physics_wsetAngularVelocity(jstring world, jstring name,jobject ang_vel);
    
    JNIEXPORT void JNICALL Java_com_project_physics_wgetLinearVelocity(jstring world, jstring name,jobject result);
    
    JNIEXPORT void JNICALL Java_com_project_physics_wgetAngularVelocity(jstring world, jstring name,jobject result);
    
    JNIEXPORT void JNICALL Java_com_project_physics_wtranslate(jstring world,jstring name,jobject position);
	
	JNIEXPORT void JNICALL Java_com_project_physics_wsetMotionState(jstring world,jstring name,jobject motionState);
    
    
}
