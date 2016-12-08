
#include "Block.h"

Block::Block() {
	id=1;
	type = step;
	position = 0;
}

Block::Block(Ogre::SceneManager *newManager, int num, int z) {
	id = num;
	int temp = rand()%2;
	switch(temp) {
		case (0): 
		type = step;
		break;
		case (1): 
		type = ceiling;
		break;
		case (2): 
		type = rail;
		break;
		case (3):
		type = gap;
		break;
	}
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
	switch(type) {
		case step:
			blockNode1 = blockManager->getRootSceneNode()->createChildSceneNode();
			blockNode1->setPosition(0,25,position);
			blockNode1->setScale(75,25,50);
			blockEntity1 = blockManager->createEntity("block"+id, "Wood.mesh");
			blockEntity1->setCastShadows(true);
			blockNode1->attachObject(blockEntity1);
		break;
		case ceiling:
			blockNode1 = blockManager->getRootSceneNode()->createChildSceneNode();
			blockNode1->setPosition(0,115,position+100);
			blockNode1->setScale(75,100,200);
			blockEntity1 = blockManager->createEntity("block"+id, "Brick.mesh");
			blockEntity1->setCastShadows(true);
			blockNode1->attachObject(blockEntity1);

			blockNode2 = blockManager->getRootSceneNode()->createChildSceneNode();
			blockNode2->setPosition(-175,105,position+100);
			blockNode2->setScale(40,25,200);
			blockEntity2 = blockManager->createEntity("blockleft"+id, "Brick.mesh");
			blockEntity2->setCastShadows(true);
			blockNode2->setOrientation(Ogre::Quaternion((Ogre::Radian)PI/2, Ogre::Vector3(0.0, 0.0, 1.0)));
			blockNode2->attachObject(blockEntity2);
			
			blockNode3 = blockManager->getRootSceneNode()->createChildSceneNode();
			blockNode3->setPosition(175,105,position+100);
			blockNode3->setScale(40,25,200);
			blockEntity3 = blockManager->createEntity("blockright"+id, "Brick.mesh");
			blockEntity3->setCastShadows(true);
			blockNode3->setOrientation(Ogre::Quaternion((Ogre::Radian)PI/2, Ogre::Vector3(0.0, 0.0, 1.0)));
			blockNode3->attachObject(blockEntity3);
		break;
		case rail:
			blockNode1 = blockManager->getRootSceneNode()->createChildSceneNode();
			blockNode1->setPosition(0,100,position);
			blockNode1->setScale(15,100,200);
			blockEntity1 = blockManager->createEntity("block"+id, "Metal.mesh");
			blockEntity1->setCastShadows(true);
			blockNode1->attachObject(blockEntity1);
		break;
		case gap:
			blockNode1 = blockManager->getRootSceneNode()->createChildSceneNode();
			blockNode1->setPosition(0,100,position);
			blockNode1->setScale(15,100,200);
			blockEntity1 = blockManager->createEntity("block"+id, "Brick.mesh");
			blockEntity1->setCastShadows(true);
			blockNode1->attachObject(blockEntity1);

			blockNode2 = blockManager->getRootSceneNode()->createChildSceneNode();
			blockNode2->setPosition(0,100,position);
			blockNode2->setScale(15,100,200);
			blockEntity2 = blockManager->createEntity("blockright"+id, "Brick.mesh");
			blockEntity2->setCastShadows(true);
			blockNode2->attachObject(blockEntity2);
		break;
	}


}

void Block::setPosition(int x, int y, int z) {
	blockNode1->setPosition(Ogre::Vector3(x, y, z));
}
btVector3 Block::getPosition() {
	int x = blockNode1->getPosition().x;
	int y = blockNode1->getPosition().y;
	int z = blockNode1->getPosition().z;
	return btVector3(x,y,z);
}
int Block::getZ() {
	return position;
}
int Block::getType() {
	if(type==step){
		return 0;
	}
	else if(type==ceiling) {
		return 1;
	}
	else if(type==rail) {
		return 2;
	}
	else if(type==gap) {
		return 3;
	}
	else return 0;
}
