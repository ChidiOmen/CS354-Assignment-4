#pragma once
#ifndef __Block_h_
#define __Block_h_
#include <OgreEntity.h>
#include <OgreSceneNode.h>
#include <Ogre.h>
#include <OgreSceneManager.h>
#include <btBulletDynamicsCommon.h>
#define PI 3.1415926535898
class Block
{

public:
	enum Type {step,ceiling,rail,gap, sideRun};
	int id;
	Type type;
	int position;
	bool multi;
	bool server;

	Block();
	Block(Ogre::SceneManager *newManager, int num, int z, bool m, bool s);

	//void setPos(int x, int y, int z);
	//Ogre::Vector3 getPos();
	int destroy();
	//void collision();
	int getID();
	Ogre::SceneManager *blockManager;
	Ogre::SceneNode* blockNode1;
	Ogre::Entity* blockEntity1;
	Ogre::SceneNode* blockNode2;
	Ogre::Entity* blockEntity2;
	Ogre::SceneNode* blockNode3;
	Ogre::Entity* blockEntity3;
	Ogre::SceneNode* blockNode4;
	Ogre::Entity* blockEntity4;
	void setPosition(int x, int y, int z);
	void setSize(double x, double y, double z);
	btVector3 getPosition();
	int getZ();
	int getType();

	//btDefaultMotionState* blockMotionState;
	//btCollisionShape* blockShape;
	//btRigidBody blockRigidBody;
	//btDiscreteDynamicsWorld* dynamicsWorld;
private:
	void buildBlock();
};

#endif // #ifndef __Block_h_
