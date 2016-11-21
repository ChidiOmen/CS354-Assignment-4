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

#include "TutorialApplication.h"
#include <OgreManualObject.h>
#include <unistd.h>
#include "/usr/include/SDL/SDL.h"
#include "/usr/include/SDL/SDL_mixer.h"
#include <stdio.h>
#include <btBulletDynamicsCommon.h>
#include <time.h>

btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btBroadphaseInterface* broadphase;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld* dynamicsWorld;

btCollisionShape* runShape;
btCollisionShape* floorShape;

btRigidBody* runRigidBody;
btRigidBody* floorRigidBody;

btDefaultMotionState* runMotionState;
btDefaultMotionState* floorMotionState;

int maxSpeed = 40;

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

void TutorialApplication::createCamera()
{
  mCamera = mSceneMgr->createCamera("PlayerCam");
  mCamera->setPosition(Ogre::Vector3(500, -20, 900));
  mCamera->setNearClipDistance(5);
  mCameraMan = new OgreBites::SdkCameraMan(mCamera);
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
  dynamicsWorld->setGravity(btVector3(0, 0, 0));




  runShape = new btBoxShape(btVector3(20, 80, 20));
  floorShape = new btStaticPlaneShape(btVector3(0, 1, 0), -40);

  btTransform startTransform;
  startTransform.setIdentity();
  btScalar massive(1.f);
  startTransform.setOrigin(btVector3(0, 20, 0));
  //createRigidBody(massive, startTransform, runShape, btVector4(0,0,1,1));

  floorMotionState = new btDefaultMotionState(
      btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)));
  btRigidBody::btRigidBodyConstructionInfo floorRigidBodyCI(
      0, floorMotionState, floorShape, btVector3(0, 0, 0));
  floorRigidBody = new btRigidBody(floorRigidBodyCI);
  floorRigidBody->setRestitution(1.0);
  floorRigidBody->setFriction(0);
  floorRigidBody->setDamping(0, 0);
  dynamicsWorld->addRigidBody(floorRigidBody);

  runMotionState = 
      new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)));
  btScalar ballMass = 3;
  btVector3 ballInertia(0, 0, 0);
  runShape->calculateLocalInertia(ballMass, ballInertia);
  btRigidBody::btRigidBodyConstructionInfo runRigidBodyCI(ballMass, runMotionState, 
      runShape, ballInertia);
  runRigidBody = new btRigidBody(runRigidBodyCI);
  runRigidBody->setRestitution(0);
  runRigidBody->setFriction(0);
  runRigidBody->setDamping(0, 0);
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
  //mSpd = 50;
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
  ballVel *= maxSpeed/ballVel.length();
  runRigidBody->setLinearVelocity(btVector3(0, 0, -50));
  //runRigidBody->applyCentralImpulse(btVector3(20*initX, 20*initY, 20*initZ));
  // Initialize the position of the ball
  mPos = Ogre::Vector3::ZERO;
  // Ambient light set in RGB
  mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

  dynamicsWorld->setGravity(btVector3(0, -1, 0));

  // Create an entity that is the mesh to be displayed
  runEnt = mSceneMgr->createEntity("mySphere", Ogre::SceneManager::PT_CUBE);
  runEnt->setMaterialName("Examples/BumpyMetal");
  runEnt->setCastShadows(true);
  runNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(
    Ogre::Vector3(0,0,0));
  runNode->attachObject(runEnt);
  runNode->setScale(0.2, 0.8, 0.2);
  // Create the ground
  Ogre::Plane plane(Ogre::Vector3::UNIT_Y, -41);
  Ogre::MeshManager::getSingleton().createPlane(
      "ground",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      plane,
      1500, 1500, 20, 20,
      true,
      1, 5, 5,
      Ogre::Vector3::UNIT_Z);
  Ogre::Entity* groundEntity = mSceneMgr->createEntity("ground");
  mSceneMgr->getRootSceneNode()->createChildSceneNode()
    ->attachObject(groundEntity);
  groundEntity->setCastShadows(false);
  groundEntity->setMaterialName("Examples/GrassFloor");
  mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

  // Create round paddle
  Ogre::ManualObject* paddle = mSceneMgr->createManualObject("paddle");
  mRadius = 300;
  float const accuracy = 35;
  paddle->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
  // Go around the circle joining points at intervals = to Pi/accuracy.
  unsigned point_index = 0;
  for (float theta = 0; theta <= 2 * Ogre::Math::PI;
      theta += Ogre::Math::PI / accuracy) {
    paddle->position(mRadius * cos(theta), mRadius * sin(theta), 0);
    paddle->index(point_index++);
  }
  for (float theta = 0; theta <= 2 * Ogre::Math::PI;
      theta += Ogre::Math::PI / accuracy) {
    paddle->position(mRadius / 2 * cos(theta), mRadius / 2 * sin(theta), 0);
    paddle->index(point_index++);
  }
  paddle->index(0); // Rejoins last point with first
  paddle->end();
  mPNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  mPNode->attachObject(paddle);
  // Set the initial state of the paddle
  mPPos = Ogre::Vector3(0, 0, 750);

}

// Game loop code
void TutorialApplication::gameStep(const Ogre::FrameEvent& fe) {

  mPos = runNode->getPosition();

  //so you dont accidentally get a trillion points at once
  if(pointTimer > 0) pointTimer -= fe.timeSinceLastFrame;

  // Render
  // Step Simulation
  dynamicsWorld->stepSimulation(1 / 60.f, 10);

  btTransform trans;
  runRigidBody->getMotionState()->getWorldTransform(trans);

  //std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;

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


  btVector3 ballVel = runRigidBody->getLinearVelocity();
  btScalar speed = ballVel.length();
  if (speed > maxSpeed) {
    ballVel *= maxSpeed/speed;
    runRigidBody->setLinearVelocity(ballVel);
  }

  ballVel = runRigidBody->getLinearVelocity();
  btScalar bvx = ballVel.getX();
  btScalar bvy = ballVel.getY();
  btScalar bvz = ballVel.getZ();

  // if (bvx < 20 && bvx > 0) {
  //   bvx = 20;
  // }
  // else if (bvx > -20 && bvx < 0) {
  //   bvx = -20;
  // }
  // if (bvy < 20 && bvy > 0) {
  //   bvy = 20;
  // }
  // else if (bvy > -20 && bvy < 0) {
  //   bvy = -20;
  // }
  // if (bvz < 20 && bvz > 0) {
  //   bvz = 20;
  // }
  // else if (bvz > -20 && bvz < 0) {
  //   bvz = -20;
  // }

  if (trans.getOrigin().getX() > 720) {
    //bvx = -20;
    runRigidBody->applyCentralImpulse(btVector3(-40, 0, 0));
    play_sound(1);
  }
  else if (trans.getOrigin().getX() < -720) {
    //bvx = 20;
    runRigidBody->applyCentralImpulse(btVector3(40, 0, 0));
    play_sound(1);
  }
  if (trans.getOrigin().getY() > 720) {
    //bvy = -20;
    runRigidBody->applyCentralImpulse(btVector3(0, -40, 0));
    play_sound(1);
  }
  else if (trans.getOrigin().getY() < -720) {
    // bvy = 20;
    runRigidBody->applyCentralImpulse(btVector3(0, 40, 0));
    play_sound(1);
  }
  if (trans.getOrigin().getZ() < -720) {
    //bvz = 20;
    runRigidBody->applyCentralImpulse(btVector3(0, 0, 40));
    play_sound(1);
  }

  //runRigidBody->setLinearVelocity(btVector3(bvx, bvy, bvz));

  std::cout << bvx << " " << bvy << " " << bvz << std::endl;

  // Update position with new direction
  // mPos = mPos + mDir * 50 * fe.timeSinceLastFrame;
  // Ogre::Real move = 50 * fe.timeSinceLastFrame;
  // runNode->translate(mDir * move);
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
  myImageWindow = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","ScoreKeeping");
  myImageWindow->setSize(USize(UDim(0.2,0),UDim(0.04,0)));
  myImageWindow->setText("Score: 0");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(myImageWindow);

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

  //Create Text box that says game over 
  goWindow = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","GAMEOBER");
  goWindow->setPosition(CEGUI::UVector2(CEGUI::UDim(0.4,0),CEGUI::UDim(0.4,0)));
  goWindow->setSize(USize(UDim(0.2,0),UDim(0.1,0)));
  goWindow->setText("        GAME OVER\n Press Enter to reset\n         Esc to Quit");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(goWindow);
  goWindow->setVisible(false);

  //Create text box : SOUND ON/OFF Right Click 
  CEGUI::Window *soundToggle = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","soundo");
  soundToggle->setPosition(CEGUI::UVector2(CEGUI::UDim(0.4,0),CEGUI::UDim(0,0)));
  soundToggle->setSize(USize(UDim(0.25,0),UDim(0.043,0)));
  soundToggle->setText("Toggle Sound: Right Mouse");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(soundToggle);

  //Create text box to indicate lives 
  CEGUI::Window *lifeIndicator = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText","lifeIndicatorW");
  lifeIndicator->setPosition(CEGUI::UVector2(CEGUI::UDim(0.39,0),CEGUI::UDim(0.945,0)));
  lifeIndicator->setSize(USize(UDim(0.07,0),UDim(0.04,0)));
  lifeIndicator->setText("LIVES:");
  CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(lifeIndicator);
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
  lifeWindow1->setVisible(true);
  lifeWindow2->setVisible(true);
  lifeWindow3->setVisible(true);
  goWindow->setVisible(false);
  iscore = 0;
  scoreCount = 0;
  myImageWindow->setText("Score: 0");
  mSpd = 50;
  pointMultiplier = 1;
}

// Method called when mouse moves
// bool TutorialApplication::mouseMoved(const OIS::MouseEvent& me) {
//   // blank
// }

bool TutorialApplication::mousePressed(const OIS::MouseEvent &arg,
    OIS::MouseButtonID id) {
  // blank
}

bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent& evt){
  if(mWindow->isClosed())
    return false;
  if(mShutDown)
    return false;

  // Run game step
  TutorialApplication::gameStep(evt);
  // Need to capture/update each device
  mKeyboard->capture();
  mMouse->capture();

  mTrayMgr->frameRenderingQueued(evt);

  if (!mTrayMgr->isDialogVisible())
  {
    // If dialog isn't up, then update the camera
    mCameraMan->frameRenderingQueued(evt);
    // If details panel is visible, then update its contents
    if (mDetailsPanel->isVisible())
    {
      mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(
            mCamera->getDerivedPosition().x));
      mDetailsPanel->setParamValue(1, Ogre::StringConverter::toString(
            mCamera->getDerivedPosition().y));
      mDetailsPanel->setParamValue(2, Ogre::StringConverter::toString(
            mCamera->getDerivedPosition().z));
      mDetailsPanel->setParamValue(4, Ogre::StringConverter::toString(
            mCamera->getDerivedOrientation().w));
      mDetailsPanel->setParamValue(5, Ogre::StringConverter::toString(
            mCamera->getDerivedOrientation().x));
      mDetailsPanel->setParamValue(6, Ogre::StringConverter::toString(
            mCamera->getDerivedOrientation().y));
      mDetailsPanel->setParamValue(7, Ogre::StringConverter::toString(
            mCamera->getDerivedOrientation().z));
    }
  }
  return true;
}

  //Need to inject timestamps to CEGUI System.
  //CEGUI::System::getSingleton().injectTimePulse(evt.timeSinceLastFrame);

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
  //SDL_memcpy (stream, audio_pos, len);          // simply copy from one buffer into the other
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
      //app.startBullet();

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
