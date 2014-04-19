//
//  vehicle.cpp
//  testBullet
//
//  Created by Marco Bedulli on 4/18/14.
//  Copyright (c) 2014 Marco Bedulli. All rights reserved.
//

#include "vehicle.h"

int rightIndex = 0;
int upIndex = 1;
int forwardIndex = 2;
btVector3 wheelDirectionCS0(0,-1,0);
btVector3 wheelAxleCS(-1,0,0);

const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
float	gEngineForce = 0.f;
float	gBreakingForce = 0.f;

float	maxEngineForce = 1000.f;//this should be engine/velocity dependent
float	maxBreakingForce = 100.f;

float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.4f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 1.0f;
float	wheelFriction = 1000;//BT_LARGE_FLOAT;
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.4f;//1.0f;


btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1

btRigidBody* m_carChassis;

btRaycastVehicle::btVehicleTuning	m_tuning;
btVehicleRaycaster*	m_vehicleRayCaster;
btRaycastVehicle*	m_vehicle;
btCollisionShape*	m_wheelShape;
btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
void initVehicle(btDiscreteDynamicsWorld *world){
    
    btTransform tr;
    tr.setIdentity();
    
    btCollisionShape* chassisShape = new btBoxShape(btVector3(1,0.5f,1));
	//m_collisionShapes.push_back(chassisShape);
    
	btCompoundShape* compound = new btCompoundShape();
    
	//m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,0,0));

    
	compound->addChildShape(localTrans,chassisShape);
    compound->setMargin(0);

	tr.setOrigin(btVector3(0,4,0));
    
	m_carChassis = localCreateRigidBody(800,tr,compound,world);//chassisShape);
	//m_carChassis->setDamping(0.2,0.2);
	
	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	

    
	/// create vehicle
	{
		
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(world);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);
		
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);
        
		world->addVehicle(m_vehicle);
        
		float connectionHeight = 0.1f;
        
        
		bool isFrontWheel=true;
        
		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);
        

		btVector3 connectionPointCS0(0,connectionHeight,2);

        
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

		connectionPointCS0 = btVector3(1,connectionHeight,2);

        
		//m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

		connectionPointCS0 = btVector3(-1,connectionHeight,-2);
		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		connectionPointCS0 = btVector3(1,connectionHeight,-2);

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}
    
	
    


}
void clientMoveAndDisplay()
{
    
	gEngineForce=500.0f;
    gBreakingForce=0;
	
	{
        
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 1;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);
        
        
		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
        m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		wheelIndex = 3;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
        
        
        btScalar m[16];
        for (int i=0;i<m_vehicle->getNumWheels();i++)
        {
            //synchronize the wheels with the (interpolated) chassis worldtransform
            m_vehicle->updateWheelTransform(i,true);
            //draw wheels (cylinders)
            m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
            glPopMatrix();
            glLoadIdentity();
            
            glColor3f(1, 0,0);
            glMultMatrixf((GLfloat*)m);
            
            glPushMatrix();
            glutWireCube(wheelRadius);
           //glutSolidSphere(wheelRadius,100,100);
           // glutSolidCube(wheelRadius);
            
            glPopMatrix();
            glLoadIdentity();

        }
        
        btTransform mamma;
        m_carChassis->getMotionState()->getWorldTransform(mamma);
        mamma.getOpenGLMatrix(m);
        
        glColor3f(1, 0,0);
        glPushMatrix();
        glMultMatrixf((GLfloat*)m);
        
        glPushMatrix();
        glutSolidCube(1);
       
        
        glPopMatrix();
        glLoadIdentity();
        
        
	}
}
btScalar m_defaultContactProcessingThreshold(BT_LARGE_FLOAT);
btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, btDynamicsWorld *world)
{

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);
    
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);
    
	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);
    
	btRigidBody* body = new btRigidBody(cInfo);
	body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);
    

	body->setWorldTransform(startTransform);

    
	world->addRigidBody(body);
    
	return body;
}


void curva(char key){
    if (key == 'a') {
    if(gVehicleSteering<1)
        gVehicleSteering+=steeringIncrement;
    }else{
        if(gVehicleSteering>-1)
            gVehicleSteering-=steeringIncrement;
    }
}
