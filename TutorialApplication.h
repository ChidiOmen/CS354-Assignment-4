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
#include <string>
#include <sstream>

//---------------------------------------------------------------------------

class TutorialApplication : public BaseApplication
{
  public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);
    virtual void startBullet();
    virtual void endBullet();

  protected:
    CEGUI::Window *myImageWindow;
    CEGUI::Window *lifeWindow1;
    CEGUI::Window *lifeWindow2;
    CEGUI::Window *lifeWindow3;
    CEGUI::Window *goWindow;
    bool gameEnd;
    int iscore;
    int lifecounter;
    Ogre::Real mSpd;
    Ogre::Real pointTimer;
    int pointMultiplier;
    int scoreCount;
    Ogre::AnimationState* mAnimationState;

    virtual void createScene(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent&);
    virtual void createViewports();
    virtual void gameStep(const Ogre::FrameEvent& fe);
    virtual void CEGUI_setup();
    virtual void updateScore();
    virtual bool updateLives();
    virtual void resetGame();
    //virtual bool mouseMoved(const OIS::MouseEvent &me);
    virtual bool mousePressed(const OIS::MouseEvent &me,
        OIS::MouseButtonID id);
};

//---------------------------------------------------------------------------

#endif // #ifndef __TutorialApplication_h_

//---------------------------------------------------------------------------
