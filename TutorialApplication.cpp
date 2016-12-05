/*
   -----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
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
Sample code found at
http://paginas.fe.up.pt/~ruirodrig/wiki/doku.php?id=teaching:djco:ogre3d:ogretutorial01sampleapp
-----------------------------------------------------------------------------
*/

/*
g++ -pthread -I/lusr/opt/ogre-1.9/include -I/lusr/opt/ogre-1.9/include/OGRE
 -I/usr/include/ois -I/lusr/opt/cegui-0.8.4/include/cegui-0 -I/usr/include/bullet
 -D_GNU_SOURCE=1 -D_REENTRANT -I/usr/include/SDL -g -O2
 -o OgreApp OgreApp-BaseApplication.o OgreApp-TutorialApplication.o  -lOgreOverlay
 -lboost_system -L/lusr/opt/ogre-1.9/lib -lOIS -L/lusr/opt/cegui-0.8.4/lib
 -lCEGUIOgreRenderer-0 -lOgreMain -lpthread -lCEGUIBase-0 -lBulletSoftBody
 -lBulletDynamics -lBulletCollision -lLinearMath -lSDL -pthread -Wl,-rpath
 -Wl,/lusr/lib/cegui-0.8
*/

#include "TutorialApplication.h"
#include <OgreManualObject.h>
#include <unistd.h>
#include "/usr/include/SDL/SDL.h"
#include "/usr/include/SDL/SDL_mixer.h"
#include <stdio.h>
#include <btBulletDynamicsCommon.h>
#include <time.h>
#include "BaseApplication.h"
#include <ctime>
#include <string>
using namespace std;

const int numBlocks = 2;

btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btBroadphaseInterface* broadphase;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld* dynamicsWorld;

btCollisionShape* runShape;
btCollisionShape* floorShape;
btCollisionShape* blockShape;

btRigidBody* runRigidBody;
btRigidBody* floorRigidBody;
btRigidBody* blockRigidBodies[numBlocks];

btDefaultMotionState* runMotionState;
btDefaultMotionState* floorMotionState;

int maxSpeed = 40;
int numTokens = 0;

// Sound effect stuff
bool success = true;
bool play = true;
static Uint8 *audio_pos; // global pointer to the audio buffer to be played
static Uint32 audio_len; // remaining length of the sample we have to play
void my_audio_callback(void *userdata, Uint8 *stream, int len);
void play_sound(int i);
static Uint32 wav_length; // length of our sample
static Uint8 *wav_buffer0; // buffer containing our audio file
static Uint8 *wav_buffer1; // buffer containing our audio file
static Uint8 *wav_buffer2; // buffer containing our audio file
static Uint8 *wav_buffer3; // buffer containing our audio file
static SDL_AudioSpec wav_spec; // the specs of our piece of music
clock_t lastClock;
//---------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}

TutorialApplication::~TutorialApplication(void)
{
}

void TutorialApplication::createViewports()
{
  Ogre::Viewport* vp = mWindow->addViewport(mCamera);
  vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));
  mCamera->setAspectRatio(
      Ogre::Real(vp->getActualWidth()) /
      Ogre::Real(vp->getActualHeight()));

}

void TutorialApplication::startBullet()
{
  lastClock = clock();
  // start Physics
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  broadphase = new btDbvtBroadphase();
  solver = new btSequentialImpulseConstraintSolver();
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, -10, 0));




  runShape = new btBoxShape(btVector3(20, 0, 20));
  floorShape = new btBoxShape(btVector3(400, 1, 30000));
  blockShape = new btBoxShape(btVector3(400, 100, 1));


  btTransform startTransform;
  startTransform.setIdentity();
  btScalar massive(1.f);
  startTransform.setOrigin(btVector3(0, 20, 0));

  floorMotionState = new btDefaultMotionState(
      btTransform(btQuaternion(0,0,0,1), btVector3(-200,0,14000)));
  btRigidBody::btRigidBodyConstructionInfo floorRigidBodyCI(
      0, floorMotionState, floorShape, btVector3(0, 0, 0));
  floorRigidBody = new btRigidBody(floorRigidBodyCI);
  floorRigidBody->setRestitution(1.0);
  floorRigidBody->setFriction(0);
  floorRigidBody->setDamping(0, 0);
  dynamicsWorld->addRigidBody(floorRigidBody);

  runMotionState = 
      new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)));
  btScalar runMass = 3;
  btVector3 runInertia(0, 0, 0);
  runShape->calculateLocalInertia(runMass, runInertia);
  btRigidBody::btRigidBodyConstructionInfo runRigidBodyCI(runMass, runMotionState, 
      runShape, runInertia);
  runRigidBody = new btRigidBody(runRigidBodyCI);
  runRigidBody->setRestitution(0);
  runRigidBody->setFriction(0);
  runRigidBody->setDamping(0, 0);
  runRigidBody->setLinearFactor(btVector3(0, 1, 1));
  dynamicsWorld->addRigidBody(runRigidBody);


}

void TutorialApplication::endBullet()
{
    // exit Physics

    dynamicsWorld->removeRigidBody(runRigidBody);
    delete runRigidBody->getMotionState();
    delete runRigidBody;
    delete runShape;

    dynamicsWorld->removeRigidBody(floorRigidBody);
    delete floorRigidBody->getMotionState();
    delete floorRigidBody;
    delete floorShape;

    for (int i = 0; i < numBlocks; i++) {
      btRigidBody* brb = blockRigidBodies[i];
      if(brb) {
        dynamicsWorld -> removeRigidBody(brb);
        delete brb->getMotionState();
        delete brb;
      }
    }
    delete blockShape;

    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;



}

void TutorialApplication::createScene(void)
{

  //CEGUI setup

  startBullet();
  CEGUI_setup();
  // Start channel
  int channel;
  int audio_rate = 22050;
  Uint16 audio_format = MIX_DEFAULT_FORMAT;
  int audio_channels = 2;
  int audio_buffers = 2048;
  //Initialize SDL
  if( SDL_Init( SDL_INIT_AUDIO ) < 0 )
  {
    printf( "SDL could not initialize!" );
    success = false;
  }
  // Play a single sound
  if( SDL_LoadWAV("res/scratch.wav", &wav_spec,
        &wav_buffer0, &wav_length) == NULL ){
    success = false;
  }
  if( SDL_LoadWAV("res/low.wav", &wav_spec,
        &wav_buffer1, &wav_length) == NULL ){
    success = false;
  }
  if( SDL_LoadWAV("res/medium.wav", &wav_spec,
        &wav_buffer2, &wav_length) == NULL ){
    success = false;
  }
  if( SDL_LoadWAV("res/high.wav", &wav_spec,
        &wav_buffer3, &wav_length) == NULL ){
    success = false;
  }
  // set the callback function
  wav_spec.callback = my_audio_callback;
  wav_spec.userdata = NULL;
  // set our global static variables
  audio_pos = wav_buffer0; // copy sound buffer
  audio_len = wav_length; // copy file length
  /* Open the audio device */
  if ( SDL_OpenAudio(&wav_spec, NULL) < 0 ){
    fprintf(stderr, "Couldn't open audio: %s\n", SDL_GetError());
  }
  /* Start playing */
  SDL_PauseAudio(0);

  // Initialize ball velicity to 0
  int initX = 0;
  int initY = 0;
  int initZ = 0;
  pointMultiplier = 1;

  while (initX <= 1 ) {
    initX = abs(std::rand() % 9);
  }
  while (initY <= 1) {
    initY = abs(std::rand() % 9);
  }
  while (initZ <= 1) {
    initZ = abs(std::rand() % 9);
  }

  mDir = Ogre::Vector3(initX, initY, initZ);
  btVector3 ballVel = btVector3(initX, initY, initZ);
  if (ballVel.length() != 0) {
    ballVel *= maxSpeed/ballVel.length();
  }

  clock_t startTime = clock();
  wordCount=0;
  playerSpeed = 35;
  runRigidBody->setLinearVelocity(btVector3(0, 0, playerSpeed));

  // Initialize the position of the ball
  mPos = Ogre::Vector3::ZERO;
  // Ambient light set in RGB
  mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

  //Set overall timer
  gameTimer = 1;

  // Create an entity that is the mesh to be displayed
  //runEnt = mSceneMgr->createEntity("mySphere", Ogre::SceneManager::PT_CUBE);
  mSceneMgr->setSkyDome(true, "Examples/SpaceSkyPlane", 5, 8);
  runEnt = mSceneMgr->createEntity("robot.mesh");
  //runEnt->setMaterialName("Examples/BumpyMetal");
  runEnt->setCastShadows(true);
  runNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(
    Ogre::Vector3(0,0,0));
  runNode->attachObject(runEnt);
  Ogre::Vector3 src = runNode->getOrientation() * Ogre::Vector3::UNIT_X;
  Ogre::Quaternion quat = src.getRotationTo(Ogre::Vector3(0,0,1));
  runNode->rotate(quat);
  runNode->setScale(0.6, 0.4, 1);
  mAnimationState = runEnt->getAnimationState("Walk");
  mAnimationState->setLoop(true);
  mAnimationState->setEnabled(true);


  // Create the floor
  Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
  Ogre::MeshManager::getSingleton().createPlane(
      "floor",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      plane,
      400, 30000, 20, 20,
      true,
      1, 5, 5,
      Ogre::Vector3::UNIT_Z);
  Ogre::Entity* floorEntity = mSceneMgr->createEntity("floor");
  Ogre::SceneNode* floorNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  floorNode->attachObject(floorEntity);
  floorNode->setPosition(0,0,14000);
  floorEntity->setCastShadows(false);

  floorEntity->setMaterialName("Examples/CloudySky");


  mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

  for(int i = 0; i < numBlocks; i++) {
    /*int distance = rand() % 30000;
    Ogre::Plane randomPlane(Ogre::Vector3::NEGATIVE_UNIT_Z, -1*distance);

    Ogre::MeshManager::getSingleton().createPlane(
    "floor" + i,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    randomPlane,
    100, 400, 20, 20,
    true,
    1, 5, 5,
    Ogre::Vector3::UNIT_X);
    Ogre::Entity* entity = mSceneMgr->createEntity("floor" + i);
    entity->setMaterialName("Examples/BumpyMetal");
    entity->setCastShadows(false);
    mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, 50, 0))
    ->attachObject(entity);

    btDefaultMotionState* blockMotionState = new btDefaultMotionState(
    btTransform(btQuaternion(0,0,0,1), btVector3(-200,0,distance)));
    btRigidBody::btRigidBodyConstructionInfo floorRigidBodyCI(
        0, blockMotionState, blockShape, btVector3(0, 0, 0));
    blockRigidBodies[i] = new btRigidBody(floorRigidBodyCI);
    blockRigidBodies[i]->setRestitution(0.8);
    blockRigidBodies[i]->setFriction(0);
    blockRigidBodies[i]->setDamping(0, 0);
    dynamicsWorld->addRigidBody(blockRigidBodies[i]);*/

  	blockEntity = mSceneMgr->createEntity("block"+i, "Wood.mesh");
  	blockEntity->setCastShadows(true);
  	blockNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  	blockNode->setPosition(Ogre::Vector3(0,25,2000+(5000*i)));
  	blockNode->attachObject(blockEntity);
  	blockNode->setScale(75,25,50);

  	blockShape = new btBoxShape(btVector3(75,25,50));
  	blockMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,25,2000+(5000*i))));
  	btRigidBody::btRigidBodyConstructionInfo blockRigidBodyCI(0, blockMotionState, blockShape, btVector3(0,0,0));
  	blockRigidBodies[i] = new btRigidBody(blockRigidBodyCI);
  	blockRigidBodies[i]->setRestitution(1);
  	blockRigidBodies[i]->setFriction(0);
  	blockRigidBodies[i]->setDamping(0,0);
  	dynamicsWorld->addRigidBody(blockRigidBodies[i]);
  }

}

// Game loop code
void TutorialApplication::gameStep(const Ogre::FrameEvent& fe) {

  mPos = runNode->getPosition();
  
  mCamera->setPosition(mPos + Ogre::Vector3(0, 100, -500));
  //mCamera->lookAt(mPos);

  //so you dont accidentally get a trillion points at once
  if(pointTimer > 0) pointTimer -= fe.timeSinceLastFrame;

  // Render
  // Step Simulation
  dynamicsWorld->stepSimulation(1 / 60.f, 10);

  if(mPos.z>=27000) {
     //playerSpeed*=0.995;
       playerSpeed=0;
     runRigidBody->setLinearVelocity(btVector3(0, 0, playerSpeed));
  }
  else if(CLOCKS_PER_SEC != 0){
    gameTimer = clock()/CLOCKS_PER_SEC;
  }

  myImageWindow->setText("Time: " + Ogre::StringConverter::toString(gameTimer) + " seconds");
  if (gameTimer != 0){
    wpmWindow->setText("WPM: " + Ogre::StringConverter::toString(wordCount*60/gameTimer) + " Words Per Minute");
  }
  speedWindow->setText("Speed: " + Ogre::StringConverter::toString(rvz) + "m/s");

  if(rvz==0) {
      mAnimationState->setEnabled(false);
  }
  else mAnimationState->setEnabled(true);

  btTransform trans;
  runRigidBody->getMotionState()->getWorldTransform(trans);

  // Do inputs: mouseMoved() is below
  int initX = 0;
  int initY = 0;
  int initZ = 0;

  // Initialize some stufff for random dir
  while (initX <= 1 ) {
    initX = abs(std::rand() % 9);
  }
  while (initY <= 1) {
    initY = abs(std::rand() % 9);
  }
  while (initZ <= 1) {
    initZ = abs(std::rand() % 9);
  }

  //Resets GUI if player wants to restart
  if(gameEnd && mKeyboard->isKeyDown(OIS::KC_RETURN)){
    resetGame();
    runRigidBody->translate(btVector3(-1*trans.getOrigin().getX(), -1*trans.getOrigin().getY(), -1*trans.getOrigin().getZ()));
    runRigidBody->setLinearVelocity(btVector3(10*initX, 10*initY, 10*initZ));
    play_sound(0);
    mPos = Ogre::Vector3::ZERO;
    runNode->setPosition(mPos);
    play_sound(0);
    gameEnd = false;
  }
 
  mPos.x = trans.getOrigin().getX();
  mPos.y = trans.getOrigin().getY();
  mPos.z = trans.getOrigin().getZ();

  btVector3 runVel = runRigidBody->getLinearVelocity();
  rvx = runVel.getX();
  rvy = runVel.getY();
  rvz = runVel.getZ();

  //Change User's Input
  typedWord1->setText(userInput);
  typedWord2->setText(userInput);
  if(dodgeWord.compare(userInput)==0) {
      userInput = "";
      do {
           dodgeWord = wordList_test[rand()%7];
      }while(speedWord.compare(dodgeWord)==0);
      typingWord1->setText(dodgeWord);
      if(runRigidBody && mPos.y <= 1) {
        if (numTokens < 5) {
          numTokens++;
          if (numTokens == 1) {
            lifeWindow1->setVisible(true);
          }
          else if (numTokens == 2) {
            lifeWindow2->setVisible(true);
          }
          else if (numTokens == 3) {
            lifeWindow3->setVisible(true);
          }
          else if (numTokens == 4) {
            lifeWindow4->setVisible(true);
          }
          else if (numTokens == 5) {
            lifeWindow5->setVisible(true);
          }
        }
      }
      wordCount++;
  }
  else if(speedWord.compare(userInput)==0) {
      userInput = "";
      do {
          speedWord = wordList_test[rand()%7];
      }while(speedWord.compare(dodgeWord)==0);
      typingWord2->setText(speedWord);
      playerSpeed *= 1.25;
      runRigidBody->setLinearVelocity(btVector3(0, 0, playerSpeed));
      wordCount++;
  }

  /*if(rvz < 10){
    runRigidBody->translate(btVector3(0, 0, -1000));
    playerSpeed /= 2;
    if(playerSpeed<20)
	playerSpeed=20;
    runRigidBody->setLinearVelocity(btVector3(0, 0, playerSpeed));
  }*/

  // Displaying runner position
  std::cout << mPos.x << " " << mPos.y << " " << mPos.z << std::endl;

  // Displaying runner velocity
  std::cout << rvx << " " << rvy << " " << rvz << std::endl;

  runNode->setPosition(
        Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
}

void TutorialApplication::CEGUI_setup(){
  using namespace std;
  using namespace CEGUI;
  gameEnd = false;
  iscore = 0;
  scoreCount = 0;
  lifecounter = 3;
  pointTimer = -1;
  mRenderer = &CEGUI::OgreRenderer::bootstrapSystem();
  CEGUI::ImageManager::setImagesetDefaultResourceGroup("Imagesets");
  CEGUI::Font::setDefaultResourceGroup("Fonts");
  CEGUI::Scheme::setDefaultResourceGroup("Schemes");
  CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
  CEGUI::WindowManager::setDefaultResourceGroup("Layouts");
  CEGUI::SchemeManager::getSingleton().createFromFile("TaharezLook.scheme");
  CEGUI::Window *sheet = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow", "AwwwwSheet");
  CEGUI::System::getSingleton().getDefaultGUIContext().setRootWindow(sheet);

  //Score keeping
  myImageWindow = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","Time");
  myImageWindow->setPosition(CEGUI::UVector2(CEGUI::UDim(0,0),CEGUI::UDim(0,0)));
  myImageWindow->setSize(USize(UDim(0.2,0),UDim(0.04,0)));
  myImageWindow->setText("Time: " + Ogre::StringConverter::toString("0 seconds"));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(myImageWindow);
  
  wpmWindow = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","WordsPerMinute");
  wpmWindow->setPosition(CEGUI::UVector2(CEGUI::UDim(0,0),CEGUI::UDim(0.04,0)));
  wpmWindow->setSize(USize(UDim(0.2,0),UDim(0.04,0)));
  wpmWindow->setText("Words: " + Ogre::StringConverter::toString(wordCount));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(wpmWindow);
  
  speedWindow = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","Speed");
  speedWindow->setPosition(CEGUI::UVector2(CEGUI::UDim(0,0),CEGUI::UDim(0.08,0)));
  speedWindow->setSize(USize(UDim(0.2,0),UDim(0.04,0)));
  speedWindow->setText("Speed: " + Ogre::StringConverter::toString(rvz) + "m/s");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(speedWindow);

  //CEGUI Life indicator
  lifeWindow1 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/RadioButton","rLife1");
  lifeWindow1->setPosition(CEGUI::UVector2(CEGUI::UDim(0.465, 0),CEGUI::UDim(0.95,0)));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(lifeWindow1);
  lifeWindow2 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/RadioButton","rLife2");
  lifeWindow2->setPosition(CEGUI::UVector2(CEGUI::UDim(0.495, 0),CEGUI::UDim(0.95,0)));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(lifeWindow2);
  lifeWindow3 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/RadioButton","rLife3");
  lifeWindow3->setPosition(CEGUI::UVector2(CEGUI::UDim(0.525, 0),CEGUI::UDim(0.95,0)));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(lifeWindow3);
  lifeWindow4 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/RadioButton","rLife4");
  lifeWindow4->setPosition(CEGUI::UVector2(CEGUI::UDim(0.555, 0),CEGUI::UDim(0.95,0)));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(lifeWindow4);
  lifeWindow5 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/RadioButton","rLife5");
  lifeWindow5->setPosition(CEGUI::UVector2(CEGUI::UDim(0.585, 0),CEGUI::UDim(0.95,0)));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(lifeWindow5);

  lifeWindow1->setVisible(false);
  lifeWindow2->setVisible(false);
  lifeWindow3->setVisible(false);
  lifeWindow4->setVisible(false);
  lifeWindow5->setVisible(false);


  //Create Text box that says game over 
  goWindow = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","GAMEOVER");
  goWindow->setPosition(CEGUI::UVector2(CEGUI::UDim(0.4,0),CEGUI::UDim(0.4,0)));
  goWindow->setSize(USize(UDim(0.2,0),UDim(0.1,0)));
  goWindow->setText("        GAME OVER\n Press Enter to reset\n         Esc to Quit");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(goWindow);
  goWindow->setVisible(false);

  //Create text box : SOUND ON/OFF Right Click 
  CEGUI::Window *soundToggle = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","soundo");
  soundToggle->setPosition(CEGUI::UVector2(CEGUI::UDim(0.8,0),CEGUI::UDim(0,0)));
  soundToggle->setSize(USize(UDim(0.25,0),UDim(0.043,0)));
  soundToggle->setText("Toggle Sound: Right Mouse");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(soundToggle);

  //Create text box to indicate lives 
  CEGUI::Window *lifeIndicator = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","lifeIndicatorW");
  lifeIndicator->setPosition(CEGUI::UVector2(CEGUI::UDim(0.39,0),CEGUI::UDim(0.945,0)));
  lifeIndicator->setSize(USize(UDim(0.07,0),UDim(0.04,0)));
  lifeIndicator->setText("TOKENS:");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(lifeIndicator);

  //Create text box for words to be typed
  typingWord1 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","typingWord1");
  typingWord1->setPosition(CEGUI::UVector2(CEGUI::UDim(0.15,0),CEGUI::UDim(0.25,0)));
  typingWord1->setSize(USize(UDim(0.1,0),UDim(0.04,0)));
  dodgeWord = "DODGE";
  typingWord1->setText(dodgeWord);
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(typingWord1);

  typingWord2 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","typingWord2");
  typingWord2->setPosition(CEGUI::UVector2(CEGUI::UDim(0.75,0),CEGUI::UDim(0.25,0)));
  typingWord2->setSize(USize(UDim(0.1,0),UDim(0.04,0)));
  speedWord = "FASTER";
  typingWord2->setText(speedWord);
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(typingWord2);

  //Create text box for user's typing
  typedWord1 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","typedWord1");
  typedWord1->setPosition(CEGUI::UVector2(CEGUI::UDim(0.15,0),CEGUI::UDim(0.35,0)));
  typedWord1->setSize(USize(UDim(0.1,0),UDim(0.04,0)));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(typedWord1);

  typedWord2 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","typedWord2");
  typedWord2->setPosition(CEGUI::UVector2(CEGUI::UDim(0.75,0),CEGUI::UDim(0.35,0)));
  typedWord2->setSize(USize(UDim(0.1,0),UDim(0.04,0)));
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(typedWord2);
}

void TutorialApplication::updateScore(){
  using namespace CEGUI;
  using namespace std;
  //Update GUI
  pointTimer = 0.1;   
  stringstream ss;
  iscore+=pointMultiplier;
  ++scoreCount;
  ss << iscore;
  string score = ss.str();
  myImageWindow->setText("Score: "+ score);
  //update score multiplier
  if(scoreCount == 5){
    pointMultiplier = 2;
  } 
  else if(scoreCount == 10){
    pointMultiplier = 5;
  } 
  else if(scoreCount == 20){
    pointMultiplier = 10;
  }
  else if(scoreCount == 39){
    pointMultiplier = 20;
  }
}

bool TutorialApplication::updateLives(){
  using namespace CEGUI;
  using namespace std;
  mSpd = 50;
  pointMultiplier = 1;
  scoreCount = 0;
  if(lifecounter == 3){
    --lifecounter;
    lifeWindow3->setVisible(false);
    return false;
  }
  else if(lifecounter == 2){
    --lifecounter;
    lifeWindow2->setVisible(false);
    return false;
  }
  else { //(lifecounter == 1)
    --lifecounter;
    lifeWindow1->setVisible(false);
    goWindow->setVisible(true);
    return true;
  }
}

void TutorialApplication::resetGame(){
  using namespace CEGUI;
  using namespace std;
  lifecounter = 3;
  lifeWindow1->setVisible(false);
  lifeWindow2->setVisible(false);
  lifeWindow3->setVisible(false);
  lifeWindow4->setVisible(false);
  lifeWindow5->setVisible(false);
  goWindow->setVisible(false);
  iscore = 0;
  scoreCount = 0;
  myImageWindow->setText("Score: 0");
  mSpd = 50;
  pointMultiplier = 1;
}


bool TutorialApplication::mousePressed(const OIS::MouseEvent &arg,
    OIS::MouseButtonID id) {
}

bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent& evt){
  if(mWindow->isClosed())
    return false;
  if(mShutDown)
    return false;
  mAnimationState->addTime(evt.timeSinceLastFrame);

  // Run game step
  TutorialApplication::gameStep(evt);
    // Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();
    mTrayMgr->frameRenderingQueued(evt);

    if (!mTrayMgr->isDialogVisible())
    {
        mCameraMan->frameRenderingQueued(evt);   // If dialog isn't up, then update the camera
        if (mDetailsPanel->isVisible())          // If details panel is visible, then update its contents
        {
            mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(mCamera->getDerivedPosition().x));
            mDetailsPanel->setParamValue(1, Ogre::StringConverter::toString(mCamera->getDerivedPosition().y));
            mDetailsPanel->setParamValue(2, Ogre::StringConverter::toString(mCamera->getDerivedPosition().z));
            mDetailsPanel->setParamValue(4, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().w));
            mDetailsPanel->setParamValue(5, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().x));
            mDetailsPanel->setParamValue(6, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().y));
            mDetailsPanel->setParamValue(7, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().z));
        }
    }

  return true;
}


void play_sound(int i) {
  if (play) {
    // set the callback function
    wav_spec.callback = my_audio_callback;
    wav_spec.userdata = NULL;
    // set our global static variables
  
    switch (i) {
      case 0: audio_pos = wav_buffer0; break;
      case 1: audio_pos = wav_buffer1; break;
      case 2: audio_pos = wav_buffer2; break;
      case 3: audio_pos = wav_buffer3; break;
    }
    audio_len = wav_length; // copy file length
    /* Start playing */
    SDL_PauseAudio(0);
  }
}

void my_audio_callback(void *userdata, Uint8 *stream, int len) {
  if (audio_len < 0.1)
    return;

  len = ( len > audio_len ? audio_len : len );
  SDL_MixAudio(stream, audio_pos, len, SDL_MIX_MAXVOLUME);// mix from one buffer into another

  audio_len = 0;

}

//---------------------------------------------------------------------------

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
  INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
#else
    int main(int argc, char *argv[])
#endif
    {
      // Create application object
      TutorialApplication app;

      try {
        app.go();
      } catch(Ogre::Exception& e)  {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBox(NULL, e.getFullDescription().c_str(),
            "An exception has occurred!",
            MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        std::cerr << "An exception has occurred: " <<

          e.getFullDescription().c_str() << std::endl;
#endif
      }

      app.endBullet();

      return 0;
    }

#ifdef __cplusplus
}
#endif

//---------------------------------------------------------------------------
