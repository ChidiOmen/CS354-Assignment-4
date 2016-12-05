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
	enum Type {step,ceiling,pillar};
	int id;
	Type type;
	int position;

	Block();
	Block(Ogre::SceneManager *newManager, int num, int z);

	//void setPos(int x, int y, int z);
	//Ogre::Vector3 getPos();
	int destroy();
	//void collision();
	int getID();
	Ogre::SceneManager *blockManager;
	Ogre::SceneNode* blockNode;
	Ogre::Entity* blockEntity;
	void setPosition(int x, int y, int z);
	void setSize(double x, double y, double z);
	btVector3 getPosition();

	//btDefaultMotionState* blockMotionState;
	//btCollisionShape* blockShape;
	//btRigidBody blockRigidBody;
	//btDiscreteDynamicsWorld* dynamicsWorld;
private:
	void buildBlock();
};

#endif // #ifndef __Block_h_
