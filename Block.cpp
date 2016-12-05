
#include "Block.h"

Block::Block() {
	id=1;
	type = step;
	position = 0;
	//buildBlock();
}

Block::Block(Ogre::SceneManager *newManager, int num, int z) {
	id = num;
	type = ceiling;
	position = z;
	blockManager = newManager;
	buildBlock();
}

/*void Block::setPos(int x, int y, int z) {
	blockNode->setPosition(x, y, z);
}

Ogre::Vector3 Block::getPos() {
	return Ogre::Vector3(blockNode->getPosition().x, blockNode->getPosition().y, blockNode->getPosition().z);
}*/

int Block::destroy()
{
	return 0;
}


int Block::getID()
{
	return id;
}


void Block::buildBlock() {
	blockNode = blockManager->getRootSceneNode()->createChildSceneNode();
	blockEntity = blockManager->createEntity("block"+id, "Wood.mesh");
	blockEntity->setCastShadows(true);
	blockNode->attachObject(blockEntity);
	switch(type) {
		case step:
			blockNode->setPosition(0,25,position);
			//blockNode->setOrientation(Ogre::Quaternion((Ogre::Radian)PI/2, Ogre::Vector3(0.0, 1.0, 0.0)));
			blockNode->setScale(75,25,50);
		break;
		case ceiling:
			blockNode->setPosition(0,40,position);
			//blockNode->setOrientation(Ogre::Quaternion((Ogre::Radian)PI/2, Ogre::Vector3(0.0, 1.0, 0.0)));
			blockNode->setScale(75,25,50);
		break;
		case pillar:
			blockNode->setPosition(0,25,position);
			//blockNode->setOrientation(Ogre::Quaternion((Ogre::Radian)PI/2, Ogre::Vector3(0.0, 1.0, 0.0)));
			blockNode->setScale(75,25,50);
		break;
	}

	/*blockShape = new btBoxShape(btVector3(75,25,50));
	blockMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,25,1000+(5000*id))));
	btRigidBody::btRigidBodyConstructionInfo blockRigidBodyCI(0, blockMotionState, blockShape, btVector3(0,0,0));
	blockRigidBody = new btRigidBody(blockRigidBodyCI);
	blockRigidBody->setRestitution(1);
	blockRigidBody->setFriction(0);
	blockRigidBody->setDamping(0,0);
	dynamicsWorld->addRigidBody(blockRigidBody);*/
}

void Block::setPosition(int x, int y, int z) {
	blockNode->setPosition(Ogre::Vector3(x, y, z));
}
btVector3 Block::getPosition() {
	int x = blockNode->getPosition().x;
	int y = blockNode->getPosition().y;
	int z = blockNode->getPosition().z;
	return btVector3(x,y,z);
}
