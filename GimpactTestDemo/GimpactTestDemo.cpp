/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include <iostream>
#include <string>
#include "btBulletDynamicsCommon.h"
#include "GimpactTestDemo.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

//#include "stdafx.h"
#include <cstdlib>
#include <math.h>
#include <cmath>
#include "cpp/src/optimization.h"
#define SHOW_NUM_DEEP_PENETRATIONS

#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"
#include "GLDebugFont.h"
/// Including GIMPACT here



#include "GLDebugDrawer.h"

#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

/// Include Torus Mesh here
#include "TorusMesh.h"
#include "BunnyMesh.h"

#ifdef SHOW_NUM_DEEP_PENETRATIONS 
extern int gNumDeepPenetrationChecks;
extern int gNumSplitImpulseRecoveries;
extern int gNumGjkChecks;
#endif //

using namespace alglib;

//Real			dts = 0.000001f;
Real			dts = 1.0 / 60.0;

const double simulation_precision = 1 / 60.0;
///**************************************************************************************
///	GIMPACT Test Demo made by DevO
///
///**************************************************************************************
void  function1_fvec(const real_1d_array &x, real_1d_array &fi, void *ptr);
void  function2_fvec(const real_1d_array &x, real_1d_array &fi, void *ptr);
void  nlcfunc2_jac(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac, void *ptr);
struct Collision_Information;

GimpactConcaveDemo::~GimpactConcaveDemo()
{

	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	delete m_dynamicsWorld;

	delete m_indexVertexArrays;
	delete m_trimeshShape;

	delete m_indexVertexArrays2;
	delete m_trimeshShape2;

	for (i = 0; i<m_collisionShapes.size(); i++)
	{
		btCollisionShape* shape = m_collisionShapes[i];
		delete shape;
	}


	delete m_gimpactCollisionCreateFunc;

	delete m_collisionConfiguration;
	delete m_dispatcher;
	delete m_broadphase;
	delete m_constraintSolver;

}




//------------------------------------------------------------------------------
void GimpactConcaveDemo::renderme(int collision_world)
{
	updateCamera();


	btScalar m[16];

	if (m_dynamicsWorld)
	{
		btVector3	worldBoundsMin, worldBoundsMax;
		getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);


		int numObjects = m_dynamicsWorld->getNumCollisionObjects();
		btVector3 wireColor(1, 0, 0);
		for (int i = 0; i<numObjects; i++)
		{
			btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(colObj);

			if (body && body->getMotionState())
			{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
			}
			else
			{
				colObj->getWorldTransform().getOpenGLMatrix(m);
			}

			btVector3 wireColor(1.f, 1.0f, 0.5f); //wants deactivation
			if (i & 1)
			{
				wireColor = btVector3(0.f, 0.0f, 1.f);
			}
			///color differently for active, sleeping, wantsdeactivation states
			if (colObj->getActivationState() == 1) //active
			{
				if (i & 1)
				{
					wireColor += btVector3(0.8f, 0.1f, 0.1f);
				}
				else
				{
					wireColor += btVector3(0.5f, 0.f, 0.f);
				}
			}
			/*if (colObj->getActivationState() == 2) //ISLAND_SLEEPING
			{
			if (i & 1)
			{
			wireColor += btVector3 (0.5f,0.8f, 0.5f);
			} else
			{
			wireColor += btVector3 (0.f,0.5f,0.f);
			}
			}*/
			if (i == 0) {
				wireColor.setZ(1);
				wireColor.setY(1);
				wireColor.setX(1);
			}
			m_shapeDrawer->drawOpenGL(m, colObj->getCollisionShape(), wireColor, getDebugMode(), worldBoundsMin, worldBoundsMax/*,collision_world*/);
		}
		{ // Draw CorrdSystem 
			m_shapeDrawer->drawCoordSystem();
		}
		float xOffset = 10.f;
		float yStart = 20.f;
		float yIncr = 20.f;
		char buf[124];

		glColor3f(0, 0, 0);

		setOrthographicProjection();

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "mouse to interact");
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		/*	glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"space to reset");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		*/
		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "cursor keys and z,x to navigate");
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "i to toggle simulation, s single step");
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "q to quit");
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, ". to shoot TRIMESH (dot)");
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		// not yet hooked up again after refactoring...

		/*			glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"d to toggle deactivation");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		*/

		/*
		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"a to draw temporal AABBs");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		*/

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "h to toggle help text");
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		//bool useBulletLCP = !(getDebugMode() & btIDebugDraw::DBG_DisableBulletLCP);

		bool useCCD = ((getDebugMode() & btIDebugDraw::DBG_EnableCCD) != 0);

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "1 CCD mode (adhoc) = %i", useCCD);
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "+- shooting speed = %10.2f", m_ShootBoxInitialSpeed);
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

#ifdef SHOW_NUM_DEEP_PENETRATIONS

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "gNumDeepPenetrationChecks = %d", gNumDeepPenetrationChecks);
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "gNumSplitImpulseRecoveries= %d", gNumSplitImpulseRecoveries);
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;




		glRasterPos3f(xOffset, yStart, 0);
		sprintf(buf, "gNumGjkChecks= %d", gNumGjkChecks);
		GLDebugDrawString(xOffset, yStart, buf);
		yStart += yIncr;

#endif //SHOW_NUM_DEEP_PENETRATIONS


		resetPerspectiveProjection();


	}

}

//------------------------------------------------------------------------------
void	GimpactConcaveDemo::initGImpactCollision()
{
	/// Create Torus Shape
	{
		m_indexVertexArrays = new btTriangleIndexVertexArray
		(NUM_TRIANGLES,
			&gIndices[0][0],
			3 * sizeof(int),
			NUM_VERTICES,
			(Real*)&gVertices[0], sizeof(Real) * 3);


#ifdef BULLET_GIMPACT
#ifdef BULLET_GIMPACT_CONVEX_DECOMPOSITION
		btGImpactConvexDecompositionShape * trimesh = new
			btGImpactConvexDecompositionShape(
				m_indexVertexArrays, btVector3(1.f, 1.f, 1.f), btScalar(0.01));
		trimesh->setMargin(0.07);
		trimesh->updateBound();


#else
		btGImpactMeshShape * trimesh = new btGImpactMeshShape(m_indexVertexArrays);
		trimesh->setLocalScaling(btVector3(1.f, 1.f, 1.f));
#ifdef BULLET_TRIANGLE_COLLISION 
		trimesh->setMargin(0.07f); ///?????
#else
		trimesh->setMargin(0.0f);
#endif
		trimesh->updateBound();
#endif

		m_trimeshShape = trimesh;

#else
		m_trimeshShape = new btGIMPACTMeshData(m_indexVertexArrays);
#endif

	}

	/// Create Bunny Shape
	{
		m_indexVertexArrays2 = new btTriangleIndexVertexArray
		(BUNNY_NUM_TRIANGLES,
			&gIndicesBunny[0][0],
			3 * sizeof(int),
			BUNNY_NUM_VERTICES,
			(Real*)&gVerticesBunny[0], sizeof(Real) * 3);
#ifdef BULLET_GIMPACT

#ifdef BULLET_GIMPACT_CONVEX_DECOMPOSITION
		btGImpactConvexDecompositionShape * trimesh2 = new
			btGImpactConvexDecompositionShape(
				m_indexVertexArrays2, btVector3(4.f, 4.f, 4.f), btScalar(0.01));
		trimesh2->setMargin(0.07);
		trimesh2->updateBound();
#else
		btGImpactMeshShape * trimesh2 = new btGImpactMeshShape(m_indexVertexArrays2);
		trimesh2->setLocalScaling(btVector3(4.f, 4.f, 4.f));
#ifdef BULLET_TRIANGLE_COLLISION 
		trimesh2->setMargin(0.07f); ///?????
#else
		trimesh2->setMargin(0.0f);
#endif
		trimesh2->updateBound();
#endif



		m_trimeshShape2 = trimesh2;
#else
		m_trimeshShape2 = new btGIMPACTMeshData(m_indexVertexArrays2);

#endif

	}


	///register GIMPACT algorithm
	btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld->getDispatcher());

#ifdef BULLET_GIMPACT
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
#else
	btConcaveConcaveCollisionAlgorithm::registerAlgorithm(dispatcher);
#endif


}

#ifndef BULLET_GIMPACT
btCollisionShape * GimpactConcaveDemo::createTorusShape()
{
	btGIMPACTMeshShape * newtrimeshShape = new btGIMPACTMeshShape(m_trimeshShape);
	newtrimeshShape->setLocalScaling(btVector3(1.f, 1.f, 1.f));
	return newtrimeshShape;
}
btCollisionShape * GimpactConcaveDemo::createBunnyShape()
{
	btGIMPACTMeshShape * newtrimeshShape = new btGIMPACTMeshShape(m_trimeshShape2);
	newtrimeshShape->setLocalScaling(btVector3(4.f, 4.f, 4.f));
	return newtrimeshShape;
}
#endif
//------------------------------------------------------------------------------
void	GimpactConcaveDemo::initPhysics()
{
	setTexturing(true);
	setShadows(false);

	setCameraDistance(1.f);


	/// Init Bullet
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	//btOverlappingPairCache* broadphase = new btSimpleBroadphase();
	//m_broadphase = new btSimpleBroadphase();

	int  maxProxies = 1024;
	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	m_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

	//create trimesh model and shape
	initGImpactCollision();



	/// Create Scene
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();


	btCollisionShape* staticboxShape1 = new btBoxShape(btVector3(200, 1, 200));//floor
	m_collisionShapes.push_back(staticboxShape1);
	startTransform.setOrigin(btVector3(0, -1, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape1);

	btCollisionShape* staticboxShape2 = new btBoxShape(btVector3(1, 50, 200));//left wall
	m_collisionShapes.push_back(staticboxShape2);
	startTransform.setOrigin(btVector3(-200, 15, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape2);

	btCollisionShape* staticboxShape3 = new btBoxShape(btVector3(1, 50, 200));//right wall
	m_collisionShapes.push_back(staticboxShape3);
	startTransform.setOrigin(btVector3(200, 15, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape3);

	btCollisionShape* staticboxShape4 = new btBoxShape(btVector3(200, 50, 1));//front wall
	m_collisionShapes.push_back(staticboxShape4);
	startTransform.setOrigin(btVector3(0, 15, 200));
	localCreateRigidBody(mass, startTransform, staticboxShape4);

	btCollisionShape* staticboxShape5 = new btBoxShape(btVector3(200, 50, 1));//back wall
	m_collisionShapes.push_back(staticboxShape5);
	startTransform.setOrigin(btVector3(0, 15, -200));
	localCreateRigidBody(mass, startTransform, staticboxShape5);


	//static plane
	/*
	btVector3 normal(-0.5,0.5,0.0);
	normal.normalize();
	btCollisionShape* staticplaneShape6 = new btStaticPlaneShape(normal,0.0);// A plane
	m_collisionShapes.push_back(staticplaneShape6);
	startTransform.setOrigin(btVector3(0,-9,0));

	btRigidBody* staticBody2 = localCreateRigidBody(mass, startTransform,staticplaneShape6 );

	//another static plane

	normal.setValue(0.5,0.7,0.0);
	//normal.normalize();
	btCollisionShape* staticplaneShape7 = new btStaticPlaneShape(normal,0.0);// A plane
	m_collisionShapes.push_back(staticplaneShape7);
	startTransform.setOrigin(btVector3(0,-10,0));

	staticBody2 = localCreateRigidBody(mass, startTransform,staticplaneShape7 );
	*/
	/// Create Static Torus


#ifdef BULLET_GIMPACT

	TorusInit();

#else
	kinematicTorus = localCreateRigidBody(0.0, startTransform, createTorusShape());

#endif	//kinematicTorus->setCollisionFlags(kinematicTorus->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	//kinematicTorus->setActivationState(ISLAND_SLEEPING);

	//kinematicTorus->setCollisionFlags( kinematicTorus->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//kinematicTorus->setActivationState(DISABLE_DEACTIVATION);

	/// Kinematic
	kinTorusTran = btVector3(-0.1, 0, 0);
	kinTorusRot = btQuaternion(0, 3.14159265*0.01, 0);




#ifdef TEST_GIMPACT_TORUS

#ifdef BULLET_GIMPACT
	/// Create dynamic Torus
	/*
	for (int i=0;i<6;i++)
	{
	height -= step;
	startTransform.setOrigin(btVector3(0,height,-5));
	startTransform.setRotation(btQuaternion(0,0,3.14159265*0.5));

	btRigidBody* bodyA;
	bodyA= localCreateRigidBody(massT, startTransform,m_trimeshShape);

	height -= step;
	startTransform.setOrigin(btVector3(0,height,-5));
	startTransform.setRotation(btQuaternion(3.14159265*0.5,0,3.14159265*0.5));
	btRigidBody* bodyB;
	bodyB= localCreateRigidBody(massT, startTransform,m_trimeshShape);

	}
	*/
#else

	/// Create dynamic Torus
	for (int i = 0; i<6; i++)
	{
		height -= step;
		startTransform.setOrigin(btVector3(0, height, -5));
		startTransform.setRotation(btQuaternion(0, 0, 3.14159265*0.5));

		btRigidBody* bodyA = localCreateRigidBody(massT, startTransform, createTorusShape());

		height -= step;
		startTransform.setOrigin(btVector3(0, height, -5));
		startTransform.setRotation(btQuaternion(3.14159265*0.5, 0, 3.14159265*0.5));
		btRigidBody* bodyB = localCreateRigidBody(massT, startTransform, createTorusShape());

	}
#endif //no BULLET_GIMPACT
#endif

	startTransform.setIdentity();

	/*
	/// Create Dynamic Boxes
	{
	for (int i=0;i<8;i++)
	{
	btCollisionShape* boxShape = new btBoxShape(btVector3(1,1,1));
	m_collisionShapes.push_back(boxShape);

	startTransform.setOrigin(btVector3(2*i-5,2,-3));
	localCreateRigidBody(1, startTransform,boxShape);
	}
	}
	*/

	//m_debugMode |= btIDebugDraw::DBG_DrawWireframe;

}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::shootTrimesh(const btVector3& destination)
{


	/*{
	float mass = 4.f;
	btTransform startTransform;
	startTransform.setIdentity();
	btVector3 camPos = getCameraPosition();
	startTransform.setOrigin(camPos);
	#ifdef BULLET_GIMPACT
	btRigidBody* body = this->localCreateRigidBody(mass, startTransform,m_trimeshShape2);
	#else
	btRigidBody* body = this->localCreateRigidBody(mass, startTransform,createBunnyShape());
	#endif
	btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
	linVel.normalize();
	linVel*=m_ShootBoxInitialSpeed*0.25;

	body->getWorldTransform().setOrigin(camPos);
	body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
	body->setLinearVelocity(linVel);
	body->setAngularVelocity(btVector3(0,0,0));
	}*/
	/*TODO Shooting */
	if (m_dynamicsWorld)
	{
		btTransform startTransform;
		btVector3 camPos = getCameraPosition();
		float  height = 28;
		float step = 2.5;
		float massT = 1.0;

		//startTransform.setOrigin(btVector3(0, height, -5));
		startTransform.setOrigin(camPos);
		startTransform.setRotation(btQuaternion(3.14159265*0.5, 0, 3.14159265*0.5));

		kinematicTorus = localCreateRigidBody(1.0, startTransform, m_trimeshShape);


		//btVector3 linVel(destination[0] - camPos[0], destination[1] - camPos[1], destination[2] - camPos[2]);
		//linVel.normalize();
		//linVel *= 1 ;
		btVector3 linVel(-camPos[0], -camPos[1], -camPos[2]);
		linVel.normalize();
		kinematicTorus->getWorldTransform().setOrigin(camPos);
		kinematicTorus->getWorldTransform().setRotation(btQuaternion(0, 0, 0, 1));
		kinematicTorus->setLinearVelocity(linVel);
		kinematicTorus->setAngularVelocity(btVector3(0, 0, 0));
	}
}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::clientMoveAndDisplay()
{



	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#define USE_KINEMATIC_GROUND
#ifdef USE_KINEMATIC_GROUND
	//kinTorusTran = btVector3(-0.05,0,0);
	//kinTorusRot  = btQuaternion(0,3.14159265*0.1,0);

	//kinematic object
	btCollisionObject* colObj = kinematicTorus;
	//is this a rigidbody with a motionstate? then use the motionstate to update positions!
	if (colObj && btRigidBody::upcast(colObj) && btRigidBody::upcast(colObj)->getMotionState())
	{
		btTransform newTrans;
		btRigidBody::upcast(colObj)->getMotionState()->getWorldTransform(newTrans);

		newTrans.getOrigin() += kinTorusTran;
		newTrans.getBasis() = newTrans.getBasis() * btMatrix3x3(kinTorusRot);
		if (newTrans.getOrigin().getX() > 6.0) {
			newTrans.getOrigin().setX(6.0);
			kinTorusTran = -kinTorusTran;
		}
		if (newTrans.getOrigin().getX() < -6.0) {
			newTrans.getOrigin().setX(-6.0);
			kinTorusTran = -kinTorusTran;
		}

		//btRigidBody::upcast(colObj)->getMotionState()->setWorldTransform(newTrans);
	}
	else
	{
		/*
		btTransform &newTrans =  m_dynamicsWorld->getCollisionObjectArray()[0]->getWorldTransform();
		newTrans.getOrigin() += kinTorusTran;
		if(newTrans.getOrigin().getX() > 0.1) kinTorusTran = -kinTorusTran;
		if(newTrans.getOrigin().getX() < 0.1) kinTorusTran = -kinTorusTran;
		*/
	}

#endif //USE_KINEMATIC_GROUND


	unsigned long int time = getDeltaTimeMicroseconds() / btScalar(1000);


	//#ifdef BULLET_GIMPACT
	//	printf("%i time %.1f ms \n",m_steps_done,btGImpactCollisionAlgorithm::getAverageTreeCollisionTime());
	//#else
	//	printf("%i time %.1f ms \n",m_steps_done,btConcaveConcaveCollisionAlgorithm::getAverageTreeCollisionTime());
	//#endif

	//float dt = float(m_clock.getTimeMicroseconds()) * dts; //0.000001f;

	/*TODO Update function*/
	TorusDemo();
	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[6];

	btRigidBody* body = btRigidBody::upcast(obj);
	btTransform trans = obj->getWorldTransform();

	//std::cout << "timer is:" << time << std::endl;
	//std::cout << "CENTER xyz is" << center.getX() << "," << center.getY() << "," << center.getZ() << std::endl;
	//btVector3 vv = body->getLinearVelocity();
	//std::cout << "xyz is" << vv.getX() << "," << vv.getY() << "," << vv.getZ() << std::endl;
	//btVector3 vv = trans.getOrigin();
	//std::cout << "xyz is" << vv.getX() << "," << vv.getY() << "," << vv.getZ() << std::endl;

	//AABBcollision();
	float dt = simulation_precision;

	double d_titi = (time * m_steps_done) * pow(10, -3);
	//std::cout << "d_tti is" << d_titi << std::endl;
	// 200 : old : 1.92089

	/*
	TestingOutputState(double time_now, double T_C);
	*/

	// First test is 4.544 sec
	//if (shooting_condition(m_steps_done, time*pow(10,-3)) ) //1.11 just test this function //Caution :: Integer ?
	//std::cout << "d_tti is" << d_titi << std::endl;


	btTransform predict;
	body->predictIntegratedTransform(d_titi, predict);
	//std::cout << "predict result is";
	//btVector3Output(predict.getOrigin(), std::string(""));



	m_dynamicsWorld->stepSimulation(dt);
	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();

	m_steps_done++;

	//if (m_steps_done >= 100)
	//body->applyCentralForce(btVector3(-100, 0, 0));
	//btVector3Output(trans.getOrigin(), std::string("simulation origin is:"));


	renderme();

	glFlush();
	swapBuffers();


}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::clientResetScene()
{
	m_steps_done = 0;
	DemoApplication::clientResetScene();
}

#define KEY_ESCAPE     0x1B


//------------------------------------------------------------------------------
void GimpactConcaveDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '.':
	{
		shootTrimesh(getCameraTargetPosition());
		break;
	}

	case '2':
	{
		dts += 0.000001f;
		break;
	}
	case '3':
	{
		dts -= 0.000001f; if (dts<0.000001f) dts = 0.000001f;
		break;
	}

	default:
		DemoApplication::keyboardCallback(key, x, y);
	}
}


void GimpactConcaveDemo::TorusDemo() {

}

void GimpactConcaveDemo::TorusInit(User_fix_information* debug) {

	//==============================================================================
	/* Create Rod*/
	btTransform startTransform;
	startTransform.setRotation(btQuaternion(0.0, 0, 0));
	// half extent
	btCollisionShape* staticboxShape6 = new btBoxShape(btVector3(0.6, 20, 0.6));//back wall
	m_collisionShapes.push_back(staticboxShape6);
	startTransform.setOrigin(btVector3(0, 0, 0));
	double theta = 0;
	startTransform.setRotation(btQuaternion(0.0, 0, theta));
	double rod_x = 20 * sin(theta);
	double rod_y = 20 * cos(theta);
	std::cout << "destination are:" << -rod_x << "," << rod_y << std::endl;
	btVector3 destination(-rod_x, rod_y, 0);
	localCreateRigidBody(0, startTransform, staticboxShape6);
	//==============================================================================



	//==============================================================================
	/*Torus Property */
	float step = 2.5;
	float massT = 1.0;
	// Trail and error (Because torus is created by the code author) 
	const double torus_width_radius = 2.5;
	const double torus_height_radius = 0.476;
	//==============================================================================
	/* User Setting
	x, y, angle-- user_set
	*/

	// Test  user_define_height > 20  or user_define_height < 10 , don't test user_define_height ~= 20



	double dimension[6] = { 25,-5,25,1,0,0 };
	//debug = NULL;
	if (debug != NULL) {
		dimension[1] = debug->torus_init_point.getX();
		dimension[0] = debug->torus_init_point.getY();
		dimension[2] = debug->torus_init_point.getZ();
		dimension[3] = debug->Angular.getX();
		dimension[4] = debug->Angular.getY();
		dimension[5] = debug->Angular.getZ();
	}

	const double user_define_height = dimension[0]; //y(0)
	const double user_define_x = dimension[1]; // x(0)
	const double user_define_z = dimension[2];
	const double user_define_angle = dimension[5]; // Rz(0)
	const double user_fefine_angle1 = dimension[3];
	const double user_fefine_angle2 = dimension[4];


	//for (int ii = 0; ii < 6; ii++)
	//std::cout << "dimension is" << dimension[ii] << ",";
	std::cout << std::endl;
	/* Control vector
	v0 ,w0 -- varialbe
	*/
	btVector3 user_define_LinearInitialVelocity(0, 1, 0); // not user define
	btVector3 control_define_AngularInitialVelocity(0, 0, 1.0); // not user define
																//==============================================================================

	const int throwingObjectID = 6; // torus
	const int fixObjectID = 5; // rod

							   //==============================================================================
							   /* Collision Fix */

	btVector3 torus_init_point(user_define_x, user_define_height, user_define_z);
	btVector3 torus_init_angular(user_fefine_angle1, user_fefine_angle2, user_define_angle);
	btQuaternion torus_init_quaternion(torus_init_angular.getX(), torus_init_angular.getY(), torus_init_angular.getZ());

	User_fix_information after_collisionWorld(torus_init_point, torus_init_quaternion, user_define_LinearInitialVelocity, control_define_AngularInitialVelocity, 0);
	std::cout << "after collision\n";
	btVector3Output(torus_init_point, "init point");
	btVector3Output(user_define_LinearInitialVelocity, "init lin");
	btVector3Output(control_define_AngularInitialVelocity, "init ang");
	bool collision_checking = 0; //default : good shot
	int converge_step = 0;

	do {

		GimpactConcaveDemo* c_world = new GimpactConcaveDemo();
		if (converge_step == 0)
			collision_checking = c_world->CollisionWorld(after_collisionWorld, throwingObjectID, fixObjectID, 1);
		else
			collision_checking = c_world->CollisionWorld(after_collisionWorld, throwingObjectID, fixObjectID, 0);
		delete c_world;

		converge_step++;
		//std::cout << "checking :" << converge_step << std::endl;

		if (converge_step >= 20) { debug_test = false; break; }
		else { debug_test = true; }
	} while (collision_checking);
	this->convergence_step = converge_step;
	//==============================================================================


	//==============================================================================
	/* After Fix , Apply !!! */
	startTransform.setOrigin(after_collisionWorld.torus_init_point);
	startTransform.setRotation(after_collisionWorld.torus_init_quaternion);
	kinematicTorus = localCreateRigidBody(1.0, startTransform, m_trimeshShape);
	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[throwingObjectID];
	btRigidBody* body = btRigidBody::upcast(obj);
	btTransform trans = obj->getWorldTransform();
	btVector3 origin = trans.getOrigin();
	btVector3Output(user_define_LinearInitialVelocity, std::string("original_user_define_LinearInitialVelocity"));
	btVector3Output(after_collisionWorld.LinearVelocity, std::string("after_collisionWorld.LinearVelocity"));
	btVector3Output(control_define_AngularInitialVelocity, std::string("original_user_define_AngularInitialVelocity"));
	btVector3Output(after_collisionWorld.AngularVelocity, std::string("after_collisionWorld.AngularVelocity"));
	body->setLinearVelocity(after_collisionWorld.LinearVelocity);
	body->setAngularVelocity(after_collisionWorld.AngularVelocity);
	//==============================================================================




}


void GimpactConcaveDemo::btVector3Output(const btVector3 &v, const std::string s) {
	std::cout << s << v.getX() << "," << v.getY() << "," << v.getZ() << std::endl;
}


bool GimpactConcaveDemo::CollisionWorld(User_fix_information& user_fix, int object_id1, int object_id2, int opt_flag) {

	std::cout << "collision\n";
	User_fix_information passing_user_information(user_fix.torus_init_point, user_fix.torus_init_quaternion, user_fix.LinearVelocity, user_fix.AngularVelocity, 0);
	this->CollisioninitPhysics(passing_user_information, object_id1, object_id2, opt_flag);
	this->CollisionFix(passing_user_information);


	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[object_id1];
	btRigidBody* body = btRigidBody::upcast(obj);
	btTransform trans = obj->getWorldTransform();
	user_fix.torus_init_point = passing_user_information.torus_init_point;
	user_fix.torus_init_quaternion = passing_user_information.torus_init_quaternion;
	user_fix.LinearVelocity = passing_user_information.LinearVelocity;
	user_fix.AngularVelocity = passing_user_information.AngularVelocity;

	if (this->Collision_info.collision_flag) return true;
	else return false;
}

void GimpactConcaveDemo::CollisioninitPhysics(User_fix_information& user_fix, int object_id1, int object_id2, int opt_flag) {



	/// Init Bullet
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	//btOverlappingPairCache* broadphase = new btSimpleBroadphase();
	//m_broadphase = new btSimpleBroadphase();

	int  maxProxies = 1024;
	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	m_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

	//create trimesh model and shape
	initGImpactCollision();



	/// Create Scene
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();


	btCollisionShape* staticboxShape1 = new btBoxShape(btVector3(200, 1, 200));//floor
	m_collisionShapes.push_back(staticboxShape1);
	startTransform.setOrigin(btVector3(0, -1, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape1);

	btCollisionShape* staticboxShape2 = new btBoxShape(btVector3(1, 50, 200));//left wall
	m_collisionShapes.push_back(staticboxShape2);
	startTransform.setOrigin(btVector3(-200, 15, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape2);

	btCollisionShape* staticboxShape3 = new btBoxShape(btVector3(1, 50, 200));//right wall
	m_collisionShapes.push_back(staticboxShape3);
	startTransform.setOrigin(btVector3(200, 15, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape3);

	btCollisionShape* staticboxShape4 = new btBoxShape(btVector3(200, 50, 1));//front wall
	m_collisionShapes.push_back(staticboxShape4);
	startTransform.setOrigin(btVector3(0, 15, 200));
	localCreateRigidBody(mass, startTransform, staticboxShape4);

	btCollisionShape* staticboxShape5 = new btBoxShape(btVector3(200, 50, 1));//back wall
	m_collisionShapes.push_back(staticboxShape5);
	startTransform.setOrigin(btVector3(0, 15, -200));
	localCreateRigidBody(mass, startTransform, staticboxShape5);

	CollisionTorusInit(user_fix, object_id1, object_id2, opt_flag);

}


void GimpactConcaveDemo::CollisionTorusInit(User_fix_information& user_fix, int object_id1, int object_id2, int opt_flag) {
	//==============================================================================
	/* Create Cylinder*/
	btTransform startTransform;
	startTransform.setRotation(btQuaternion(0.0, 0, 0));
	// half extent
	btCollisionShape* staticboxShape6 = new btBoxShape(btVector3(0.6, 20, 0.6));//back wall
	m_collisionShapes.push_back(staticboxShape6);
	startTransform.setOrigin(btVector3(0, 0, 0));
	double theta = 0;
	startTransform.setRotation(btQuaternion(0.0, 0, theta));
	double rod_x = 20 * sin(theta);
	double rod_y = 20 * cos(theta);
	std::cout << "destination are:" << -rod_x << "," << rod_y << std::endl;
	btVector3 destination(-rod_x, rod_y, 0);
	localCreateRigidBody(0, startTransform, staticboxShape6);
	//==============================================================================



	//==============================================================================
	/*Torus Property */
	float step = 2.5;
	float massT = 1.0;
	// Trail and error (Because torus is created by the code author) 
	const double torus_width_radius = 2.5;
	const double torus_height_radius = 0.476;
	//==============================================================================
	/*First stage BVP
	v0 random -- varialbe
	w0 user setting -- user value, fixed for program tester
	x, y fixed -- fixed
	* /
	/* x[1] equation
	destination_y = -1/2g*t^2 + v0*t + y(0)
	t = (v0 +- sqrt(v0^2 + 2*g*( y(0) - destination_y ) ) )/g
	*/
	/* User setting */
	const double user_define_height = 22; //y(0)
	const double user_define_x = -10; // x(0)
	const double user_define_angle = 3.22; // R(0)
	btVector3 user_define_LinearInitialVelocity = user_fix.LinearVelocity;
	/* Control vector */
	btVector3 control_define_AngularInitialVelocity = user_fix.AngularVelocity;

	/**/
	//==============================================================================


	if (opt_flag) {
		//==============================================================================
		/*Initial BVP with little Optimizations
		for Parameter setting
		*/
		btVector3 torus_init_point = user_fix.torus_init_point;
		btVector3 torus_init_angular(0, 0, user_define_angle);
		btQuaternion torus_init_quaternion = user_fix.torus_init_quaternion;

		{ // First stage
			double t_solution_BVP1 = InitBVP(torus_init_point, destination, user_define_LinearInitialVelocity);
			std::cout << "t_solution is" << t_solution_BVP1 << std::endl;

		}
		{ // Second Stage : Optimization Stage

			double t_solution_BVP1 = (user_define_LinearInitialVelocity.getY() + sqrt(user_define_LinearInitialVelocity.getY() * user_define_LinearInitialVelocity.getY() + 2 * 9.8*(torus_init_point.getY() - destination.getY()))) / 9.8;
			user_define_LinearInitialVelocity.setX((destination.getX() - torus_init_point.getX()) / t_solution_BVP1);
			user_define_LinearInitialVelocity.setZ((destination.getZ() - torus_init_point.getZ()) / t_solution_BVP1);
			double target = user_define_LinearInitialVelocity.getX();

			//==============================================================================
			/* LM Optimization
			double low_bound = user_define_LinearInitialVelocity.getY();
			std::cout << "low bound is" << low_bound << std::endl;

			real_1d_array x = "[0]";
			real_1d_array bndl = std::string("[" + std::to_string(low_bound) + "]").c_str();
			real_1d_array bndu = "[+100]";

			double epsx = 0.0000000001;
			ae_int_t maxits = 0;
			minlmstate state;
			minlmreport rep;


			double* parameter = new double[4];
			parameter[0] = torus_init_point.getY() - destination.getY();
			parameter[1] = destination.getX() - torus_init_point.getX();
			minlmcreatev(1, x, 0.0001, state);
			minlmsetbc(state, bndl, bndu);
			minlmsetcond(state, epsx, maxits);
			minlmoptimize(state, function1_fvec,NULL, parameter);
			minlmresults(state, x, rep);

			printf("%s\n", x.tostring(1).c_str());
			std::string str = x.tostring(1);
			str.pop_back();
			str.erase(0,1);
			double vy_opt = std::stod(str);
			double t_opt = (vy_opt + sqrt(pow(vy_opt, 2) + 2 * 9.8*parameter[0])) / 9.8;
			double vx_opt = parameter[1] / t_opt;
			std::cout << "LM opt time,velocity is(" << t_opt << "," << vx_opt << "," << vy_opt << ")"<< std::endl;
			*/
			//==============================================================================4
			/*NLP OPT*/

			{

				double y_ = user_define_LinearInitialVelocity.getY();
				real_1d_array x0 = std::string("[" + std::to_string(y_) + "]").c_str();
				real_1d_array s = "[1]";
				double epsx = 0.000001;
				ae_int_t maxits = 0;
				minnlcstate state;
				minnlcreport rep;
				real_1d_array x1;

				//
				// Create optimizer object, choose AUL algorithm and tune its settings:
				// * rho=1000       penalty coefficient
				// * outerits=5     number of outer iterations to tune Lagrange coefficients
				// * epsx=0.000001  stopping condition for inner iterations
				// * s=[1,1]        all variables have unit scale
				// * exact low-rank preconditioner is used, updated after each 10 iterations
				// * upper limit on step length is specified (to avoid probing locations where exp() is large)
				//
				minnlccreate(1, x0, state);

				minnlcsetalgoslp(state);
				minnlcsetcond(state, epsx, maxits);
				minnlcsetscale(state, s);
				minnlcsetstpmax(state, 10.0);

				//
				// Set constraints:
				//
				// Nonlinear constraints are tricky - you can not "pack" general
				// nonlinear function into double precision array. That's why
				// minnlcsetnlc() does not accept constraints itself - only constraint
				// counts are passed: first parameter is number of equality constraints,
				// second one is number of inequality constraints.
				//
				// As for constraining functions - these functions are passed as part
				// of problem Jacobian (see below).
				//
				// NOTE: MinNLC optimizer supports arbitrary combination of boundary, general
				//       linear and general nonlinear constraints. This example does not
				//       show how to work with boundary or general linear constraints, but you
				//       can easily find it in documentation on minnlcsetbc() and
				//       minnlcsetlc() functions.
				//
				minnlcsetnlc(state, 0, 2);

				// with f0 being target function, f1 being equality constraint "f1=0",
				// f2 being inequality constraint "f2<=0". Number of equality/inequality
				// constraints is specified by minnlcsetnlc(), with equality ones always
				// being first, inequality ones being last.
				//
				double* parameter = new double[4];
				parameter[0] = torus_init_point.getY() - destination.getY();
				parameter[1] = destination.getX() - torus_init_point.getX();
				parameter[2] = destination.getZ() - torus_init_point.getZ();
				alglib::minnlcoptimize(state, nlcfunc2_jac, NULL, parameter);

				minnlcresults(state, x1, rep);


				printf("%s\n", x1.tostring(2).c_str());
				std::string str = x1.tostring(2);
				str.pop_back();
				str.erase(0, 1);
				double vy_opt = std::stod(str);
				double t_opt = (vy_opt + sqrt(pow(vy_opt, 2) + 2 * 9.8*parameter[0])) / 9.8;
				double vx_opt = parameter[1] / t_opt;
				double vz_opt = parameter[2] / t_opt;
				std::cout << "LM opt time,velocity is(" << t_opt << "," << vx_opt << "," << vy_opt << "," << vz_opt << ")" << std::endl;
				user_define_LinearInitialVelocity.setX(vx_opt);
				user_define_LinearInitialVelocity.setY(vy_opt);
				user_define_LinearInitialVelocity.setZ(vz_opt);
			}

		}


	}
	//==============================================================================



	//==============================================================================
	/* Shoot 1 -- get the collision information
	if (NO COLLISION) RIGHT SHOOT
	else
	1a.Point -- P_collide
	1b.Collision time -- T_C
	1c.\Delta angular position to set parallel to the destinations
	1d.Run Shoot 2 Algorithm
	*/
	startTransform.setOrigin(user_fix.torus_init_point);
	startTransform.setRotation(user_fix.torus_init_quaternion);

	kinematicTorus = localCreateRigidBody(1.0, startTransform, m_trimeshShape);

	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[object_id1];
	btRigidBody* body = btRigidBody::upcast(obj);
	btTransform trans = obj->getWorldTransform();
	btVector3 origin = trans.getOrigin();
	//btVector3Output(user_define_LinearInitialVelocity, std::string("user_define_LinearInitialVelocity"));
	body->setLinearVelocity(user_define_LinearInitialVelocity);
	body->setAngularVelocity(control_define_AngularInitialVelocity);
	//==============================================================================


	//==============================================================================
	/* PreSimulation */
	PreSimulation(simulation_precision, object_id1, object_id2);
	std::cout << "collision time is:" << Collision_info.collision_time << std::endl;
	btVector3Output(Collision_info.collision_point, "collision point is:");
	btVector3Output(Collision_info.collision_angle, "collision angle is:");
	//btVector3Output(Collision_point, "Collision_point is:");
	//std::cout << "collision time is" << Collision_time << std::endl;
	//==============================================================================




	/* Fix or not*/
	btVector3 fix_or_ont_torus_init_point = user_fix.torus_init_point;
	btQuaternion  fix_or_ont_torus_quaternion = user_fix.torus_init_quaternion;
	btVector3  fix_or_ont_Linear = user_define_LinearInitialVelocity;
	btVector3  fix_or_ont_Angular = control_define_AngularInitialVelocity;

	//==============================================================================
	/* Reset*/
	std::cout << "INTO FIX\n";


	user_fix.torus_init_point = fix_or_ont_torus_init_point;
	user_fix.torus_init_quaternion = fix_or_ont_torus_quaternion;
	user_fix.LinearVelocity = fix_or_ont_Linear;
	user_fix.AngularVelocity = fix_or_ont_Angular;
}


void GimpactConcaveDemo::CollisionReset(User_fix_information user_fix, int object_id1) {
	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[object_id1];
	btRigidBody* body = btRigidBody::upcast(obj);
	btTransform trans = obj->getWorldTransform();
	trans.setOrigin(user_fix.torus_init_point);
	trans.setRotation(user_fix.torus_init_quaternion);
	obj->setWorldTransform(trans);
	body->clearForces();
	body->setAngularVelocity(user_fix.LinearVelocity);
	body->setLinearVelocity(user_fix.AngularVelocity);
}


bool GimpactConcaveDemo::shooting_condition(double step, double precision, double& Collision_Time, btCollisionObject& object1, btCollisionObject& object2, btVector3& Collision_point) {
	//Assume world->stepSimulation or world->performDiscreteCollisionDetection has been called
	static double t_c = step * precision;
	int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;
				//std::cout << "collision occurs\n";
				btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[6];
				btRigidBody* body = btRigidBody::upcast(obj);
				btTransform trans = obj->getWorldTransform();
				btVector3 origin = trans.getOrigin();
				Collision_Time = step * precision * pow(10, -3);
				Collision_point = trans.getOrigin();
				if (&object1 == obA || &object1 == obB) {
					if (&object2 == obA || &object2 == obB)
						return true;
				}
			}
		}
	}
	return false;
}


void GimpactConcaveDemo::TestingOutputState(double time_now, double T_C) {
	if (time_now - T_C > 0.001 && time_now - T_C < 0.2) {

		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[6];
		btTransform trans = obj->getWorldTransform();
		btMatrix3x3 mm;
		mm.setRotation(trans.getRotation());
		btQuaternion qq = trans.getRotation();

		double tmp = double(qq.getX());
		double tmp1 = double(qq.getY());
		double tmp2 = double(qq.getZ());
		double tmp3 = double(qq.getW());
		Eigen::Quaternion<double> q_eigen(tmp3, tmp, tmp1, tmp2);
		auto euler = q_eigen.toRotationMatrix().eulerAngles(0, 1, 2);
		std::cout << "Euler from quaternion in roll, pitch, yaw" << std::endl << euler << std::endl;
		//trans.setRotation(trans.getRotation())
	}
}


double GimpactConcaveDemo::InitBVP(btVector3 torus_init_point, btVector3 destination, btVector3& user_define_LinearInitialVelocity) {
	std::cout << "torus y is " << torus_init_point.getY() << std::endl;
	std::cout << "destination is" << destination.getY() << std::endl;


	double Discriminant = user_define_LinearInitialVelocity.getY() * user_define_LinearInitialVelocity.getY() + 2 * 9.8*(torus_init_point.getY() - destination.getY());
	double target = user_define_LinearInitialVelocity.getY();
	if (Discriminant < 0) {
		const double c1 = torus_init_point.getY() - destination.getY();
		Optimization_Init_BVP(target, c1);
	}

	user_define_LinearInitialVelocity.setY(target);
	static double t_solution_BVP1 = (user_define_LinearInitialVelocity.getY() + sqrt(user_define_LinearInitialVelocity.getY() * user_define_LinearInitialVelocity.getY() + 2 * 9.8*(torus_init_point.getY() - destination.getY()))) / 9.8;
	std::cout << "BVP1 solution :: " << t_solution_BVP1 << std::endl;
	if (t_solution_BVP1 < 0) {
		std::cout << "BVP1 t error\n";
		exit(1);
	}
	return t_solution_BVP1;
}


void  d1_General_OPT(btVector3& target, std::vector<double> parameter) {

}

void GimpactConcaveDemo::Optimization_Init_BVP(double& target, double c1) {
	//Exhaustive method
	double Discriminant = pow(target, 2) + 2 * 9.8*(c1);
	for (; Discriminant < 0.1;) {
		target += 0.1;
		Discriminant = pow(target, 2) + 2 * 9.8*(c1);
	}
	//std::cout << "target is" << target << std::endl;
}


void GimpactConcaveDemo::Optimization_velocity(double& target, double c1) {

}

void GimpactConcaveDemo::PreSimulation(double dt, int object_id1, int object_id2) {

	{
		// default object_id = 6;
		double precision = dt;
		double Collision_Time = 0;
		btCollisionObject* obj2 = m_dynamicsWorld->getCollisionObjectArray()[object_id2];
		btCollisionObject* obj1 = m_dynamicsWorld->getCollisionObjectArray()[object_id1];
		btRigidBody* body = btRigidBody::upcast(obj1);
		btTransform trans = obj1->getWorldTransform();

		btVector3 collision_point(0, 0, 0);

		bool collsion_f = false;
		int max_step = 1.5 / dt;
		for (int pre_simulation_step = 1; pre_simulation_step < max_step; pre_simulation_step++) {
			trans = obj1->getWorldTransform();
			if (!collsion_f  && shooting_condition(pre_simulation_step, precision, Collision_Time, *obj1, *obj2, collision_point)) {
				//static btVector3 collision = collision_point;
				//std::cout << "precision is" << precision << std::endl;
				//std::cout << "pre_simulation_step is" << pre_simulation_step << std::endl;
				//std::cout << "collision time is " << Collision_info.collision_time << std::endl;
				Collision_info.collision_point = collision_point;
				Collision_info.collision_time = precision * pre_simulation_step;
				btVector3 p1 = trans.getOrigin();
				btVector3 p2 = collision_point;
				btQuaternion bt_q = trans.getRotation();
				Eigen::Quaternion<double> eigen_q(bt_q.getW(), bt_q.getX(), bt_q.getY(), bt_q.getZ());
				Eigen::Vector3d euler = eigen_q.toRotationMatrix().eulerAngles(0, 1, 2);
				Collision_info.collision_angle.setX(euler[0]);
				Collision_info.collision_angle.setY(euler[1]);
				Collision_info.collision_angle.setZ(euler[2]);
				Collision_info.collision_flag = true;
				collsion_f = true;

			}
			//btVector3Output(body->getLinearVelocity(), "linear velocity is");
			//btVector3Output(trans.getOrigin(), "origin is");
			m_dynamicsWorld->stepSimulation(dt);
		}


		/* In or Not In Checking */
		body = btRigidBody::upcast(obj1);
		trans = obj1->getWorldTransform();
		btVector3 v1 = trans.getOrigin();
		btVector3Output(trans.getOrigin(), "origin is");
		for (int InorNotIn = 0; InorNotIn < 10; InorNotIn++) {
			body->setActivationState(ACTIVE_TAG);
			body->applyCentralForce(btVector3(-100, 0, 0));
			m_dynamicsWorld->stepSimulation(dt);
		}
		trans = obj1->getWorldTransform();
		btVector3 v2 = trans.getOrigin();
		if (v1.distance(v2) > 0.5) {
			std::cout << "Need 2 shot\n";
		}
		else {
			std::cout << "NO 2 shot\n";
			Collision_info.collision_flag = false;
		}
	}
}

void function1_fvec(const real_1d_array &x, real_1d_array &fi, void *ptr)
{
	//
	// this callback calculates
	// f0(x0,x1) = x1^2
	// f1(x0,x1) = x2^2
	//

	//
	// this callback calculates
	// f0(x0,x1) = 100*(x0+3)^4,
	// f1(x0,x1) = (x1-3)^4
	//

	double *parameter = (double*)(ptr);
	double t_solution_BVP1 = (x[0] + sqrt(pow(x[0], 2) + 2 * 9.8*parameter[0])) / 9.8;
	double c1 = parameter[1] / t_solution_BVP1;
	fi[0] = pow(x[0], 2) + 2 * pow(c1, 2) + pow(t_solution_BVP1, 2);

}

void  nlcfunc2_jac(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac, void *ptr)
{
	//
	// this callback calculates
	//     Let x = v_y
	//     f0(x) = x^2 + v_x^2
	//     f1(x) = t_bvp <= 1.5
	//     f2(x) = -x <= 0.0
	//
	// and Jacobian matrix J = [dfi/dxj]
	//

	double *parameter = (double*)(ptr);
	double c1 = parameter[1];
	double c2 = parameter[0] * 2 * 9.8;
	double c3 = 9.8;
	double c4 = parameter[2];
	double C = (c1*c3) / (x[0] + std::sqrt(pow(x[0], 2) + c2));
	double C1 = (c4*c3) / (x[0] + std::sqrt(pow(x[0], 2) + c2));
	fi[0] = pow(x[0], 2) + pow(C, 2) + pow(C1, 2);
	fi[1] = ((1.0 / c3) * (x[0] + std::sqrt(pow(x[0], 2) + c2))) - 1.5;
	fi[2] = -x[0];
	double denom = x[0] + std::sqrt(pow(x[0], 2) + c2);
	jac[0][0] = 2 * x[0] + (pow(c1, 2) + pow(c4, 2))*(2 * c3 / denom)*(-1)*(c3 / pow(denom, 2))*(1 + (x[0] / std::sqrt(pow(x[0], 2) + c2)));
	jac[1][0] = (1.0 / c3) * (1 + (x[0] / std::sqrt(pow(x[0], 2) + c2)));
	jac[2][0] = -1;
}


void GimpactConcaveDemo::CollisionFix(User_fix_information& user_fix) {

	if (Collision_info.collision_flag) {
		//==============================================================================
		/* Shoot 2 (1 variable) -- Angular Velocity fix
		------------------------------------------------------------------------
		|PURPOSE :
		|use 1c.
		|fix old angular velocity to new angular velocity w0 (which greater than random(which define the +/- of \Delta ) )
		|s.t x0old + w0old * T_C + \Delta = x0old + w0new * T_C = parallel state
		|
		|SO w0_new = (w0old * T_C + \Delta) / T_C
		------------------------------------------------------------------------
		// Test 1 : t = 4.544 sec (1/200.)
		// Test 2 : t = 1.6 sec (1/60.)
		*/

		const double time_ = Collision_info.collision_time;
		btVector3 angular_new = user_fix.AngularVelocity;
		std::cout << "collision_info is:" << Collision_info.collision_angle.getZ() << std::endl;

		angular_new.setZ((user_fix.AngularVelocity.getZ() * time_ - Collision_info.collision_angle.getZ()) / time_);
		//angular_new.setZ(angular_new.getZ() + 0.01);
		
		if (fabs(angular_new.getZ()) >  2*3.1415926) {
			if (angular_new > 0)
			angular_new.setZ(angular_new.getZ() - 2*3.1415926);
			else
			angular_new.setZ(angular_new.getZ() + 2*3.1415926);
		}
		//angular_new.setY((user_fix.AngularVelocity.getY() * time_ + Collision_info.collision_angle.getY()) / time_);
		//angular_new.setX((user_fix.AngularVelocity.getX() * time_ + Collision_info.collision_angle.getX()) / time_);
		btVector3Output(user_fix.AngularVelocity, std::string("user_angular"));
		btVector3Output(angular_new, std::string("angular_new"));
		
		user_fix.AngularVelocity = angular_new;
		//==============================================================================
	}
}


bool GimpactConcaveDemo::DebugDataTest(btVector3 origin, btVector3 angular) {

	btQuaternion qq;
	btVector3 lin;
	btVector3 ang;
	User_fix_information user(origin, qq, lin, ang, &angular);
	//================================
	debug_test = false;
	//================================


	setTexturing(true);
	setShadows(false);

	setCameraDistance(45.f);


	/// Init Bullet
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	//btOverlappingPairCache* broadphase = new btSimpleBroadphase();
	//m_broadphase = new btSimpleBroadphase();

	int  maxProxies = 1024;
	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	m_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

	//create trimesh model and shape
	initGImpactCollision();



	/// Create Scene
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();


	btCollisionShape* staticboxShape1 = new btBoxShape(btVector3(200, 1, 200));//floor
	m_collisionShapes.push_back(staticboxShape1);
	startTransform.setOrigin(btVector3(0, -1, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape1);

	btCollisionShape* staticboxShape2 = new btBoxShape(btVector3(1, 50, 200));//left wall
	m_collisionShapes.push_back(staticboxShape2);
	startTransform.setOrigin(btVector3(-200, 15, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape2);

	btCollisionShape* staticboxShape3 = new btBoxShape(btVector3(1, 50, 200));//right wall
	m_collisionShapes.push_back(staticboxShape3);
	startTransform.setOrigin(btVector3(200, 15, 0));
	localCreateRigidBody(mass, startTransform, staticboxShape3);

	btCollisionShape* staticboxShape4 = new btBoxShape(btVector3(200, 50, 1));//front wall
	m_collisionShapes.push_back(staticboxShape4);
	startTransform.setOrigin(btVector3(0, 15, 200));
	localCreateRigidBody(mass, startTransform, staticboxShape4);

	btCollisionShape* staticboxShape5 = new btBoxShape(btVector3(200, 50, 1));//back wall
	m_collisionShapes.push_back(staticboxShape5);
	startTransform.setOrigin(btVector3(0, 15, -200));
	localCreateRigidBody(mass, startTransform, staticboxShape5);


	TorusInit(&user);
	if (debug_test) { return true; }
	else return false;
}