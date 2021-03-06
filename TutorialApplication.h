/*
   -----------------------------------------------------------------------------
Filename:    TutorialApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
|___/
Tutorial Framework (for Ogre 1.9)
http://www.ogre3d.org/wiki/
-----------------------------------------------------------------------------
*/

#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include "BaseApplication.h"
#include "Wordlist.h"
#include <string>
#include <sstream>
#include <btBulletDynamicsCommon.h>
#include <SDL_net.h>
#include <SDL.h>
#include<time.h>


#include "Block.h"

//---------------------------------------------------------------------------

class TutorialApplication : public BaseApplication
{
  public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);
    virtual void startBullet();
    virtual void endBullet();
    bool isServer;
    bool connectionOpened;
    bool sceneCreated;
	TCPsocket server;
	TCPsocket client;
    clock_t lastUpdate;
    

  protected:
    
    
    CEGUI::Window *myImageWindow;
    CEGUI::Window *wpmWindow;
    CEGUI::Window *speedWindow;
    CEGUI::Window *delayWindow;
    CEGUI::Window *lifeWindow1;
    CEGUI::Window *lifeWindow2;
    CEGUI::Window *lifeWindow3;
    CEGUI::Window *lifeWindow4;
    CEGUI::Window *lifeWindow5;
    CEGUI::Window *goWindow;
    bool gameEnd;
    int dodgeTimer;
    clock_t startTime;
    //Ogre::Real mSpd;
    //Ogre::Real pointTimer;
    int scoreCount;
    int wordCount;
    btScalar rvx;
    btScalar rvy;
    btScalar rvz;
    bool multiplayer;
    int obstNum;
    int cameraHeight;
    bool flipping;
    bool sliding;
    bool grinding;
    bool sideRunning;
    bool finished;
    bool delay;
    double flipSpeed;

	std::vector<Block*> blocks;
	Ogre::SceneNode* blockNode;
	Ogre::Entity* blockEntity;
	btCollisionShape* blockShape;
	btDefaultMotionState* blockMotionState;

    Ogre::AnimationState* mAnimationState1;
    Ogre::AnimationState* mAnimationState2;

    int playerSpeed;
    int gameTimer;
    int speedTimer;

    CEGUI::Window *typingWord1;
    CEGUI::Window *typingWord2;
    CEGUI::Window *typedWord1; 
    CEGUI::Window *typedWord2; 
    //CEGUI::Window *dodgeInstr;
    //CEGUI::Window *fasterInstr; 


    virtual void createScene(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent&);
    virtual void createViewports();
    virtual void gameStep(const Ogre::FrameEvent& fe);
    virtual void CEGUI_setup();
    virtual void resetGame();
    virtual void updateClient();
    //virtual bool mouseMoved(const OIS::MouseEvent &me);
    virtual bool mousePressed(const OIS::MouseEvent &me,
        OIS::MouseButtonID id);
};

//---------------------------------------------------------------------------

#endif // #ifndef __TutorialApplication_h_

//---------------------------------------------------------------------------
