#include <Ogre.h>
#include <OIS.h>

#include "btBulletDynamicsCommon.h"

#include <vector>
#include <string>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace Ogre;

// from Collin for ship movement
Ogre::Entity * TEST;
SceneNode * TESTSCENE;
static Ogre::Real mRotate = .83; 
static Ogre::Real turnShip = 0;
Real mMove = 750;
Vector3 mDirection;
btVector3 bDirection;
btTransform ShipTransform;
Vector3 NewPosition;
Vector3 bulletToOgreOrientationForBullets;
Vector3 OrientationWithout_Z;
Quaternion RigidBodyOrientationForBullets;
Quaternion bulletToOgreOrientation;
Quaternion RigidBodyOrientation;//changed from btQuaternion to Quaternion
Ogre::Real rad;
btVector3 Direction;
btVector3 bulletDirectionalVector;
btVector3 RigidBodyPosition;


const Ogre::Real sqrt_one_half = 0.7071067811865475244; // Square root of 1/2
const int horzSize= 580;
const int vertSize = 450;
static int count = 0;


// this pattern updates the scenenode position when it changes within the bullet simulation
// taken from BulletMotionState docs page24
class MyMotionState : public btMotionState {
public:
	MyMotionState(const btTransform &initialpos, Ogre::SceneNode *node) {
		mVisibleobj = node;
		mPos1 = initialpos;
	}
	virtual ~MyMotionState() {    }
	void setNode(Ogre::SceneNode *node) {
		mVisibleobj = node;
	}

	// this was added to try and get the scenenode to be pointing in the same direction as the ship is in bullet
	void  updateTransform(btTransform& newpos) {
        mPos1 = newpos;
    }

	virtual void getWorldTransform(btTransform &worldTrans) const {
		worldTrans = mPos1;
	}
	virtual void setWorldTransform(const btTransform &worldTrans) {
		if(NULL == mVisibleobj) return; // silently return before we set a node
		btQuaternion rot = worldTrans.getRotation();
		mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
		btVector3 pos = worldTrans.getOrigin();
		// TODO **** XXX need to fix this up such that it renders properly since this doesnt know the scale of the node
		// also the getCube function returns a cube that isnt centered on Z
		mVisibleobj->setPosition(pos.x(), pos.y()+5, pos.z()-5);
	}
protected:
	Ogre::SceneNode *mVisibleobj;
	btTransform mPos1;
};

//------------------------------------------------------------------------------------------ 

class Application : public FrameListener, public OIS::KeyListener
{
public:

	// slight difference here bool go vs void go so this has a return in it, messagePump causes a segFault and is not in Assignment 4
	bool go()
	{
		mContinue = true;
		createRoot();
		defineResources();
		setupRenderSystem();
		createRenderWindow();
		initializeResourceGroups();
		setupScene();
		createBulletSim();
		setupInputSystem();
		createFrameListener();

		mRoot->startRendering();

		/*
		// this definitely caused the exit seg fault
		while(true)
		{
		Ogre::WindowEventUtilities::messagePump();
		if(mWindow->isClosed())
		return false;
		if(!mRoot->renderOneFrame())
		return false;
		}
		*/

		return true;
	}

	//------------------------------------------------------------------------------------------ 

	~Application()
	{
		mInputManager->destroyInputObject(mKeyboard);
		OIS::InputManager::destroyInputSystem(mInputManager);

		delete mRoot;

		// cleanup bulletdyanmics

		//cleanup in the reverse order of creation/initialization
		//remove the rigidbodies from the dynamics world and delete them
		for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			dynamicsWorld->removeCollisionObject( obj );
			delete obj;
		}

		//delete collision shapes
		for (int j=0;j<collisionShapes.size();j++)
		{
			btCollisionShape* shape = collisionShapes[j];
			collisionShapes[j] = 0;
			delete shape;
		}

		delete dynamicsWorld;
		delete solver;
		delete overlappingPairCache;
		delete dispatcher;
		delete collisionConfiguration;


		// delete our dynamic ogreObjects

		// an iterator returned from an erase is a valid iterator to the next element in the vector
		for(std::vector<ogreObject *>::iterator itr = ptrToOgreObjects.begin(); itr != ptrToOgreObjects.end(); ) {

			// delete the ogreObject
			delete (*itr);

			// and then remove that pointer out of the vector
			itr = ptrToOgreObjects.erase(itr);
		}
	}

	//------------------------------------------------------------------------------------------ 

private:
	Root *mRoot;
	SceneManager *mSceneMgr;
	OIS::Keyboard *mKeyboard;
	OIS::InputManager *mInputManager;
	bool mContinue;
	Ogre::RenderWindow* mWindow;
	
	btRigidBody* leftSideBody;
	btRigidBody* rightSideBody;
	btRigidBody* topSideBody;
	btRigidBody* bottomSideBody;

	SceneNode *parent;
	Entity * moveShip;

	// save the scenenode and btrigidbody pointers to make it easier to move the ship
	SceneNode * shipSceneNode;
	btRigidBody * shipRigidBody;
	btVector3 Velocity;

	int magnitudeAAA;
	Vector3 locationAAA, directionAAA;

	// to create delays based off of evt.timeSinceLastFrame
	Real mDelay;
	Real mDelay2;
	Real mDelay3;
	Real mDelay4;

	// need globals for asteroid randomness


	
	// bullet dynamics
	int i;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	btCollisionShape* groundShape;
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	// struct to hold all objects we create in Ogre/Bullet
	struct ogreObject {
		Entity * entityObject;
		SceneNode * sceneNodeObject;
		MyMotionState * myMotionStateObject;
		btCollisionShape * btCollisionShapeObject;
		btRigidBody * btRigidBodyObject;
		btCollisionObject * btCollisionObjectObject;
		std::string objectType;
		Vector3 objectPosition;
		btVector3 objectVelocity;
 		int objectSize;
		int objectSpeedMultiplier;
		bool objectDelete;  // no longer needed, delete done by if pointer is NULL
	};

	ogreObject * ptrToOgreObject;				 // pointer to an ogreObject struct
	std::vector<ogreObject *> ptrToOgreObjects;  // vector of pointers to actual ogreObject structs

	// theses are very important!!!, they will be used in creating a pointer to a pointer of an ogreObject in the contactPairs method
	// they hold an ogreObject pointer as an l-value because you can only take the address of an l-value object
	ogreObject * needAddressOfObject1;
	ogreObject * needAddressOfObject2;

	// struct to hold contact pairs from contact manifold
	struct contactPair {
		const	btCollisionObject * collisionObject1;
		const	btCollisionObject * collisionObject2;
		const	btRigidBody * rigidBodyObject1;
		const	btRigidBody * rigidBodyObject2;
		ogreObject * ptrToOgreObject1;
		ogreObject * ptrToOgreObject2;
		ogreObject ** ptrToOgreObject1Ptr;
		ogreObject ** ptrToOgreObject2Ptr;
 		Vector3 positionObject1;
		Vector3 positionObject2;
	};

	std::vector<contactPair> contactPairs;  // vector of actual contactPair structs

	enum collisionTypes {
	border_asteroid = 1, border_bullet = 2, border_ship = 3, border_spaceship = 4, asteroid_bullet = 5, asteroid_ship = 6,
	asteroid_spaceship = 7, bullet_ship = 8, bullet_spaceship = 9, ship_spaceship = 10, asteroid_asteroid = 11, bullet_bullet = 12};

	//------------------------------------------------------------------------------------------ 

	// frame listener
	bool frameStarted(const FrameEvent &evt)
	{
		// setting up the delay
		mDelay -= evt.timeSinceLastFrame;
		mDelay2 += evt.timeSinceLastFrame;
		mDelay3 += evt.timeSinceLastFrame;
		mDelay4 += evt.timeSinceLastFrame;

		mKeyboard->capture();

		// update physics simulation
		dynamicsWorld->stepSimulation(evt.timeSinceLastFrame,50);

		// this should hold collision information, is this in the right place?
		getContactPairs(contactPairs);

		// only keep the first time contacts in the vector to deal with
		firstContactPairsOnly(contactPairs);

		// this will determine the type of collision the pair had
		//		determineCollisionTypes(whatCollided);

/*
		//create some bullets with a delay between them
		if (mDelay < 0.0f && count < 10) {
			createBullet(btVector3(-50, -150, 0),1.0f,btVector3(0.1,0.1,0.1),btVector3(50.0, 25.0, 0));
			++count;
			mDelay  = 0.5;
		}
*/
		/*

		// this will handle the pair type collision
		if (mDelay2 > 25.0f) {
		handleCollisions(contactPairs);
		mDelay2 = 0.0f;
		}

		if (mDelay3 > 35.0f) {
		handleCollisions(contactPairs);
		mDelay3 = 0.0f;

		}
		*/

		if (mDelay4 > 4.0f) {

			asteroidRandomness(magnitudeAAA, locationAAA, directionAAA);
			createAsteroid(3, magnitudeAAA, btVector3(locationAAA.x, locationAAA.y, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(directionAAA.x, directionAAA.y, 0));
            asteroidRandomness(magnitudeAAA, locationAAA, directionAAA);
            createAsteroid(3, magnitudeAAA, btVector3(locationAAA.x, locationAAA.y, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(directionAAA.x, directionAAA.y, 0));
			
			// create an asteroid
//			createAsteroid(3, 4, btVector3(50, 0, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(50.0, 25.0, 0));

			// create an asteroid
//			createAsteroid(3, 4, btVector3(-200, 0, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(-50.0, -25.0, 0));

			// create an asteroid
//			createAsteroid(3, 4, btVector3(100, 100, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(50.0, 25.0, 0));

			// create an asteroid
//			createAsteroid(3, 4, btVector3(300, 250, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(-50.0, -25.0, 0));
/*
			//create some bullets with a delay between them
		if (mDelay < 0.0f && count < 10) {
			createBullet(btVector3(-325, -150, 0),1.0f,btVector3(0.1,0.1,0.1),btVector3(50.0, 25.0, 0));
			++count;
			mDelay  = 0.5;
		}
*/
			// create a ship
			//		createShip(btVector3(0, 0, 0),1.0f,btVector3(0.1,0.1,0.1));
			//		createShip(btVector3(50, 50, 50),1.0f,btVector3(0.1,0.1,0.1));

			mDelay4 = 0.0f;

		}
        if (mDelay3 > 15.0f) {

        createSpaceShip(btVector3(-450, -300, 0),1.0f,btVector3(0.6,0.1,0.1),btVector3(25.0, 0, 0));
        mDelay3 = 0.0f;
        
        }
		// this is the method that handles all of the collisions that bullet detected
		handleCollisions(contactPairs);


	if(!(shipRigidBody == NULL))
		shipRigidBody->setAngularVelocity(btVector3(0, 0, turnShip * 10));

	if(!(shipRigidBody == NULL))
		shipRigidBody->setLinearVelocity(btVector3( NewPosition.x, NewPosition.y, NewPosition.z));


		return mContinue;
	}

	//------------------------------------------------------------------------------------------ 

	//------------------------------------------------------------------------------------------ 
	// KeyListener
	bool keyPressed(const OIS::KeyEvent &e) {
		switch (e.key) {
		case OIS::KC_ESCAPE:
			mContinue = false;
			break;

        case OIS::KC_UP:
			if(!(shipRigidBody == NULL)) {
                RigidBodyOrientation = shipSceneNode->getOrientation();
                //bulletToOgreOrientation = Quaternion(RigidBodyOrientation);
                OrientationWithout_Z = RigidBodyOrientation * Vector3::NEGATIVE_UNIT_Z;    //changed bulletToOgreOrientation to RigidBodyOrientation        
                bulletDirectionalVector = btVector3(OrientationWithout_Z.x, OrientationWithout_Z.y, OrientationWithout_Z.z);
                bulletDirectionalVector.normalize();
                bulletDirectionalVector *= -100;
                NewPosition = Vector3(bulletDirectionalVector.x(), bulletDirectionalVector.y(), bulletDirectionalVector.z());
                //NewPosition.y = mMove;
		        //NewPosition.z = 0;
            }
		break;

        case OIS::KC_DOWN:
				
		    if(!(shipRigidBody == NULL)) {
                RigidBodyOrientation = shipSceneNode->getOrientation();
                //bulletToOgreOrientation = Quaternion(RigidBodyOrientation);
                OrientationWithout_Z = RigidBodyOrientation * Vector3::NEGATIVE_UNIT_Z;    //changed bulletToOgreOrientation to RigidBodyOrientation        
                bulletDirectionalVector = btVector3(OrientationWithout_Z.x, OrientationWithout_Z.y, OrientationWithout_Z.z);
                bulletDirectionalVector.normalize();
                bulletDirectionalVector *= -100;
                NewPosition = Vector3(-bulletDirectionalVector.x(), -bulletDirectionalVector.y(), -bulletDirectionalVector.z());
                //NewPosition.y = -mMove;
		        //NewPosition.z = 0;
            
            }
		    
		break;

        
        case OIS::KC_LEFT:
				
            turnShip = mRotate;
			//shipRigidBody->setAngularVelocity(btVector3(0, 0, turnShip * 10));
        break;
 
		case OIS::KC_RIGHT:
				
		    turnShip = -mRotate;
//			
//			ogreObject *  temp;
//			temp = (ogreObject *)shipRigidBody->getUserPointer();
//			turnShip = -mRotate;
		break;
        
        /*case OIS::KC_H:
            Asteroid2(Size, Scale);
        break;
        */            
		case OIS::KC_SPACE:
			if(!(shipRigidBody == NULL)) {
				RigidBodyPosition = shipRigidBody->getCenterOfMassPosition();
            
            //NewPosition = NewPosition.normalise();
            
            RigidBodyOrientationForBullets = shipSceneNode->getOrientation();
            bulletToOgreOrientationForBullets = RigidBodyOrientationForBullets * Vector3::NEGATIVE_UNIT_Z;
            Direction = btVector3(bulletToOgreOrientationForBullets.x, bulletToOgreOrientationForBullets.y, bulletToOgreOrientationForBullets.z);
            Direction.normalize();
            Direction *= -60;
            
            Direction += RigidBodyPosition; 

            //rad = Quat.w;

//			if (mDelay < 0.0f && count < 10) {
            if (mDelay < 0.0f) {

 			createBullet(btVector3(Direction.x(), Direction.y(), 0),1.0f,btVector3(0.1,0.1,0.1),btVector3(-bulletToOgreOrientationForBullets.x, -bulletToOgreOrientationForBullets.y, -bulletToOgreOrientationForBullets.z));
			//++count;
			mDelay  = 0.2;
				
		    }
			} 
		break;
		default:
			break;
		}
		return true;
	}
//------------------------------------------------------------------------------------------

//	bool keyReleased(const OIS::KeyEvent &e) { return true; }
	//------------------------------------------------------------------------------------------
/*
	bool keyReleased(const OIS::KeyEvent &e) { 
        switch(e.key){
				case OIS::KC_UP:
				    mDirection.z = 0;
					//mDirection = Vector3(.001, 0.001, 0.001);
					break;
 
				case OIS::KC_DOWN:
				    mDirection.z = 0;
					//mDirection = Vector3(.001, 0.001, 0.001);
					break;
 
				case OIS::KC_LEFT:
				
					turnShip = 0;
					break;
 
				case OIS::KC_RIGHT:
				
					turnShip = 0;
					break;
		   
		   }    
        
        return true; 
    }
	*/
	//------------------------------------------------------------------------------------------ 
bool keyReleased(const OIS::KeyEvent &e) { 
        switch(e.key){
				case OIS::KC_UP:
				    mDirection.z = -2;
					//mDirection = Vector3(.001, 0.001, 0.001);
					break;
 
				case OIS::KC_DOWN:
				    mDirection.z = 2;
					//mDirection = Vector3(.001, 0.001, 0.001);
					break;
 
				case OIS::KC_LEFT:
				
					turnShip = 0;
					break;
 
				case OIS::KC_RIGHT:
				
					turnShip = 0;
					break;
		   
		   }    
        
        return true; 
    }
	//------------------------------------------------------------------------------------------ 
/*

	bool processUnbufferedInput(const Ogre::FrameEvent& evt)
{
	static Ogre::Real mRotate = 0.25;   // The rotate constant
	static Ogre::Real mMove = 250;      // The movement constant
	// giving our ship a little linear velocity
		static float Speed = 10.f;

	// this was a test to prove that this function is hooked into the framestarted function and it is!!! K key does shutdown program!!!
	if (mKeyboard->isKeyDown(OIS::KC_K))
		return false;

	// rotate the ship to the left
	if(mKeyboard->isKeyDown(OIS::KC_LEFT))
    {
		// blows up if there is no Ship!!!
//		if(mSceneMgr->getSceneNode("Ship1")){

		mSceneMgr->getSceneNode("Ship")->roll(Ogre::Degree(mRotate * 10));
//		mSceneMgr->getSceneNode("CamNinjaNode")->rotate(cameraRotationPositive, Ogre::Node::TS_LOCAL);
//		}
    }

	if(mKeyboard->isKeyDown(OIS::KC_RIGHT))
    {
//		entity will get you the parent scenenode, the one attached to which you can use with scenemanger to yaw
//		if(mSceneMgr->getSceneNode("Ship1")){

		mSceneMgr->getSceneNode("Ship")->roll(Ogre::Degree(-mRotate * 10));
//		mSceneMgr->getSceneNode("CamNinjaNode")->rotate(cameraRotationPositive, Ogre::Node::TS_LOCAL);
//		}

    }

	if(mKeyboard->isKeyDown(OIS::KC_DOWN))
    {
	

		

		//Vector conversion
//		btVector3 Velocity = btVector3(-50.0, -25.0, 0);
		// Velocity += btVector3(0, 10, 0);
		Velocity += btVector3(0,1,0);

		//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
//		Velocity.normalize();
//		Velocity *= Speed;

		//Now we finally propel our sphere
		//BoxBody->setLinearVelocity(FireVelocity * UPM); //Remember that accelerations were game units? So are velocities

		// this gives the sphere velocity in the direction that the camera is currently pointing in
		// no velocity for ship to start

//		ptrToOgreObject->btRigidBodyObject->applyImpulse(
		shipRigidBody->setLinearVelocity(Velocity);
//		shipRigidBody->applyForce(


	}

	return true;
}

*/
	//------------------------------------------------------------------------------------------ 

	// another void vs bool
	bool createRoot()
	{
		// creation du Root : 1ère chose à faire
		// changed this to plugins_d.cfg, fixed blank rendering subsystem
		mRoot = new Ogre::Root("plugins_d.cfg", "ogre.cfg", "Ogre.log");

		// Configuration : appel de la fenêtre de config si ogre.cfg n'existe pas
		if(!(mRoot->restoreConfig() || mRoot->showConfigDialog()))
		{
			return false;
		}
	}

	//------------------------------------------------------------------------------------------ 

	void defineResources()
	{
		String secName, typeName, archName;
		ConfigFile cf;
		// changed this to resources_d.cfg, fixed blank rendering subsystem
		cf.load("resources_d.cfg");

		ConfigFile::SectionIterator seci = cf.getSectionIterator();
		while (seci.hasMoreElements())
		{
			secName = seci.peekNextKey();
			ConfigFile::SettingsMultiMap *settings = seci.getNext();
			ConfigFile::SettingsMultiMap::iterator i;
			for (i = settings->begin(); i != settings->end(); ++i)
			{
				typeName = i->first;
				archName = i->second;
				ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
			}
		}
	}

	//------------------------------------------------------------------------------------------ 

	void setupRenderSystem()
	{
		if (!mRoot->restoreConfig() && !mRoot->showConfigDialog())
			throw Exception(52, "User canceled the config dialog!", "Application::setupRenderSystem()");

		//// Do not add this to the application
		//RenderSystem *rs = mRoot->getRenderSystemByName("Direct3D9 Rendering Subsystem");
		//                                      // or use "OpenGL Rendering Subsystem"
		//mRoot->setRenderSystem(rs);
		//rs->setConfigOption("Full Screen", "No");
		//rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit colour");
	}

	//------------------------------------------------------------------------------------------ 

	void createRenderWindow()
	{
		mRoot->initialise(true, "Tutorial Render Window");

		//// Do not add this to the application
		//mRoot->initialise(false);
		//HWND hWnd = 0;  // Get the hWnd of the application!
		//NameValuePairList misc;
		//misc["externalWindowHandle"] = StringConverter::toString((int)hWnd);
		//RenderWindow *win = mRoot->createRenderWindow("Main RenderWindow", 800, 600, false, &misc);
	}

	//------------------------------------------------------------------------------------------ 

	void initializeResourceGroups()
	{
		TextureManager::getSingleton().setDefaultNumMipmaps(5);
		ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	}

	//------------------------------------------------------------------------------------------ 

	void setupScene()
	{
		mSceneMgr = mRoot->createSceneManager(ST_GENERIC, "Default SceneManager");
		//mSceneMgr = mRoot->createSceneManager("DefaultSceneManager", "DefaultSceneManager");
		Camera *cam = mSceneMgr->createCamera("Camera");
		Viewport *vp = mRoot->getAutoCreatedWindow()->addViewport(cam);

		// added this from assignment 4 so camera looking at my new cube
		cam->setPosition(Ogre::Vector3(0, 0, 1000));
		cam->lookAt(Ogre::Vector3(0, 0, 0));
		cam->setNearClipDistance(0.1);
		cam->setFarClipDistance(50000);

		if (mRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_INFINITE_FAR_PLANE))
		{
			cam->setFarClipDistance(0);   // enable infinite far clip distance if we can
		}


		// mSceneMgr->setAmbientLight(ColourValue(0.25, 0.25, 0.25));
		// a little more ambient, didn't seem to brighten the scene
		mSceneMgr->setAmbientLight(ColourValue(0.4, 0.4, 0.4));
//		mSceneMgr->setShadowTechnique( SHADOWTYPE_STENCIL_ADDITIVE );

		

		/*  no floor, only space
		// make a rock wall on the floor
		Plane plane(Vector3::UNIT_Y, 0);
		MeshManager::getSingleton().createPlane("ground",ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,1500,1500,20,20,true,1,5,5,Vector3::UNIT_Z);
		ent = mSceneMgr->createEntity("GroundEntity", "ground");
		mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ent);
		//      ent->setMaterialName("Examples/Rockwall");
		ent->setMaterialName("Asteroids/Black");
		//      ent->setCastShadows(false);
		*/

		// make a light to see stuff with
		Light *light = mSceneMgr->createLight("Light1");
		light->setType(Light::LT_POINT);
		light->setPosition(Vector3(250, 150, 250));
		light->setDiffuseColour(ColourValue::White);
		light->setSpecularColour(ColourValue::White);

		// Create the scene node
		// I think this is why the camera is so far away from my cube, as in overriding my cam settings above, yes true statement
		/*
		SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode("CamNode1", Vector3(-200, 100, 200));
		node->yaw(Degree(-45));
		node->attachObject(cam);
		*/

		// initialize  delay for deletes in framerenderstart
		mDelay2 = 0.0f;
		mDelay3 = 0.0f;
		mDelay4 = 0.0f;

		// working on manual object
		ManualObject* manual = mSceneMgr->createManualObject("manual");
		manual->begin("BaseWhiteNoLighting", RenderOperation::OT_LINE_STRIP);

		// define vertex position of index 0..3
		manual->position(-100.0, -100.0, 0.0);
		manual->position( 100.0, -100.0, 0.0);
		manual->position( 100.0,  100.0, 0.0);
		manual->position(-100.0,  100.0, 0.0);

		// define usage of vertices by refering to the indexes
		manual->index(0);
		manual->index(1);
		manual->index(2);
		manual->index(3);
		manual->index(0);

		manual->end();
		mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manual);

		shipRigidBody = NULL;
		shipSceneNode = NULL;

		/* initialize random seed: */
		srand (time(NULL));


	}

	//------------------------------------------------------------------------------------------ 

	void setupInputSystem()
	{
		size_t windowHnd = 0;
		std::ostringstream windowHndStr;
		OIS::ParamList pl;
		RenderWindow *win = mRoot->getAutoCreatedWindow();

		win->getCustomAttribute("WINDOW", &windowHnd);
		windowHndStr << windowHnd;
		pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
		mInputManager = OIS::InputManager::createInputSystem(pl);

		try
		{
			mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
			mKeyboard->setEventCallback(this);
			//mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, false));
			//mJoy = static_cast<OIS::JoyStick*>(mInputManager->createInputObject(OIS::OISJoyStick, false));
		}
		catch (const OIS::Exception &e)
		{
			throw new Exception(42, e.eText, "Application::setupInputSystem");
		}
	}

	//------------------------------------------------------------------------------------------ 


	void createFrameListener()
	{
		mRoot->addFrameListener(this);
	}

	//------------------------------------------------------------------------------------------  

	void createBulletSim(void) {

		//collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
		collisionConfiguration = new btDefaultCollisionConfiguration();

		//use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new   btCollisionDispatcher(collisionConfiguration);

		//btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
		overlappingPairCache = new btDbvtBroadphase();

		//the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		solver = new btSequentialImpulseConstraintSolver;

		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

		//  we do not want gravity, we are in space
		dynamicsWorld->setGravity(btVector3(0, 0, 0));

		//  create all of the inital objects as the game starts

		// create the screen boundaries using bullet bounding boxes
		createBoundaries();

		// create a enemy space ship, unique names needed!!!
		//createSpaceShip(btVector3(-450, -300, 0),1.0f,btVector3(0.6,0.1,0.1),btVector3(25.0, 0, 0));  
		//createSpaceShip(btVector3(-450, 300, 0),1.0f,btVector3(0.6,0.1,0.1),btVector3(25.0, 0, 0));

		// create an asteroid
//		createAsteroid(3, 4, btVector3(150, 150, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(50.0, 0, 0));

		// create an asteroid
//		createAsteroid(3, 4, btVector3(-300, 0, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(-50.0, -50.0, 0));

		// create an asteroid
//		createAsteroid(3, 4, btVector3(100, -200, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(-25.0,-25.0,0));

		// create an asteroid
//		createAsteroid(3, 4, btVector3(-200, -250, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(10.0, 90.0,0));

		// create an asteroid
		createAsteroid(3, 4, btVector3(380.0, -250.0, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(horzSize, -vertSize,0));
		createAsteroid(3, 4, btVector3(530, 0, 0),1.0f,btVector3(0.3,0.3,0.3),btVector3(horzSize, -vertSize,0));


		// create some bullets
//		for(int i=0; i<10; ++i)
//			createBullet(btVector3(-100, -100, 0),1.0f,btVector3(0.1,0.1,0.1),"Bullet");

		// create a ship
//		createShip(btVector3(-250, 100, 0),0.25f,btVector3(0.5,0.5,0.5));
//		createShip(btVector3(-250, 150, 0),1.0f,btVector3(0.6,0.1,0.1));
		createShip(btVector3(0, 0, 0),1.0f,btVector3(0.4,0.4,0.4));
//		createShip(btVector3(525, -390, 0),0.25f,btVector3(0.5,0.5,0.5));
	}

	//------------------------------------------------------------------------------------------ 

	void createSpaceShip(const btVector3 &Position, btScalar Mass, const btVector3 &scale, const btVector3 &Velocity)
	{
		// empty ogre vectors for the cubes size and position
		Ogre::Vector3 size = Ogre::Vector3::ZERO;
		Ogre::Vector3 pos = Ogre::Vector3::ZERO;

		// Convert the bullet physics vector to the ogre vector
		pos.x = Position.getX();
		pos.y = Position.getY();
		pos.z = Position.getZ();
		
		// need to put the 4 pointers into the vector of objects
		ptrToOgreObject = new ogreObject;

		ptrToOgreObject->entityObject = mSceneMgr->createEntity("cube.mesh");
		//boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
//		boxentity->setCastShadows(true);
		ptrToOgreObject->sceneNodeObject = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		ptrToOgreObject->sceneNodeObject->showBoundingBox(true);
		ptrToOgreObject->sceneNodeObject->attachObject(ptrToOgreObject->entityObject);

		ptrToOgreObject->entityObject->setMaterialName("SpaceShip/Red");
		
		ptrToOgreObject->sceneNodeObject->scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		//boxNode->setScale(Vector3(0.1,0.1,0.1));
		Ogre::AxisAlignedBox boundingB = ptrToOgreObject->entityObject->getBoundingBox();
		//Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
		boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		size = boundingB.getSize()*0.95f;
		btTransform Transform;
		Transform.setIdentity();
		Transform.setOrigin(Position);
		ptrToOgreObject->myMotionStateObject = new MyMotionState(Transform,ptrToOgreObject->sceneNodeObject);
		//Give the rigid body half the size
		// of our cube and tell it to create a BoxShape (cube)
		btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);
		ptrToOgreObject->btCollisionShapeObject = new btBoxShape(HalfExtents);
		btVector3 LocalInertia;
		ptrToOgreObject->btCollisionShapeObject->calculateLocalInertia(Mass, LocalInertia);

		ptrToOgreObject->btRigidBodyObject = new btRigidBody(Mass, ptrToOgreObject->myMotionStateObject, ptrToOgreObject->btCollisionShapeObject, LocalInertia);

		// btRigidBody is derived from btCollisionObject so the pointer ends up being the same
		// proven by doing the upcast in the contactsmanifold
		ptrToOgreObject->btCollisionObjectObject = ptrToOgreObject->btRigidBodyObject;

		// Store a pointer to the Ogre Node so we can update it later
		ptrToOgreObject->btRigidBodyObject->setUserPointer(ptrToOgreObject);

		// giving our space ship a little linear velocity
		static float Speed = 75.f;

		btVector3 nonConstVelocity;
		nonConstVelocity = Velocity;
		//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
		nonConstVelocity.normalize();
		nonConstVelocity *= Speed;

		ptrToOgreObject->objectVelocity = nonConstVelocity;

		// this gives the sphere velocity in the direction that the camera is currently pointing in
		ptrToOgreObject->btRigidBodyObject->setLinearVelocity(nonConstVelocity);

		// Add it to the physics world
		dynamicsWorld->addRigidBody(ptrToOgreObject->btRigidBodyObject);
//		collisionShapes.push_back(pushToVec.btCollisionShapeObject);

		ptrToOgreObject->objectType = "SpaceShip";
 		ptrToOgreObject->objectPosition = Vector3(0.0, 0.0, 0.0);
		ptrToOgreObject->objectDelete = false;

		ptrToOgreObjects.push_back(ptrToOgreObject);

	}

	//------------------------------------------------------------------------------------------ 

	// pulled this function from assignment 4, going to add one cube to see if it works
	void createAsteroid(int asteroidSize, int asteroidSpeed, const btVector3 &Position, btScalar Mass, const btVector3 &scale, const btVector3 &Velocity){
		// empty ogre vectors for the cubes size and position
		Ogre::Vector3 size = Ogre::Vector3::ZERO;
		Ogre::Vector3 pos = Ogre::Vector3::ZERO;
		
		// Convert the bullet physics vector to the ogre vector
		pos.x = Position.getX();
		pos.y = Position.getY();
		pos.z = Position.getZ();

		// need to put the 4 pointers into the vector of objects
		ptrToOgreObject = new ogreObject;

		// remember getting rid of name here means no entity with same name
		//       asteroidEntity = mSceneMgr->createEntity(name, "sphere.mesh");
		ptrToOgreObject->entityObject = mSceneMgr->createEntity("sphere.mesh");
		//boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
		//      asteroidEntity->setCastShadows(true);
		ptrToOgreObject->sceneNodeObject = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		ptrToOgreObject->sceneNodeObject->showBoundingBox(true);
		ptrToOgreObject->sceneNodeObject->attachObject(ptrToOgreObject->entityObject);

		ptrToOgreObject->entityObject->setMaterialName("Asteroid/Gray");

		//add every asteroidNode to the allBulletNodes vector
//		allAsteroidNodes.push_back(asteroidNode);

		ptrToOgreObject->sceneNodeObject->scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		//boxNode->setScale(Vector3(0.1,0.1,0.1));
		Ogre::AxisAlignedBox boundingB = ptrToOgreObject->entityObject->getBoundingBox();
		//Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
		boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		size = boundingB.getSize()*0.95f;
		btTransform Transform;
		Transform.setIdentity();
		Transform.setOrigin(Position);
		ptrToOgreObject->myMotionStateObject = new MyMotionState(Transform,ptrToOgreObject->sceneNodeObject);
		//Give the rigid body half the size
		// of our cube and tell it to create a BoxShape (cube)
		btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);

		// add from bullet physiscs.org favorites, sphere collision shape
//		ptrToOgreObject->btCollisionShapeObject = new btSphereShape(1);
		ptrToOgreObject->btCollisionShapeObject = new btBoxShape(HalfExtents);
		btVector3 LocalInertia;
		ptrToOgreObject->btCollisionShapeObject->calculateLocalInertia(Mass, LocalInertia);
		ptrToOgreObject->btRigidBodyObject = new btRigidBody(Mass, ptrToOgreObject->myMotionStateObject, ptrToOgreObject->btCollisionShapeObject, LocalInertia);

		ptrToOgreObject->btCollisionObjectObject = ptrToOgreObject->btRigidBodyObject;

		// Store a pointer to the Ogre Node so we can update it later
		//       RigidBody->setUserPointer((void *) (boxNode));

		ptrToOgreObject->btRigidBodyObject->setUserPointer(ptrToOgreObject);


		//	   RigidBody->setUserPointer(bodies[bodies.size()-1]);

		//  set the asteroid size so we know what to do with it hits something
		ptrToOgreObject->objectSize = asteroidSize;

		// giving our cube a little linear velocity
		// this is our speed constant
		static float Speed = 20.f;

		btVector3 nonConstVelocity;
		nonConstVelocity = Velocity;
		//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
		nonConstVelocity.normalize();


		nonConstVelocity *= (Speed * asteroidSpeed);

		ptrToOgreObject->objectSpeedMultiplier = asteroidSpeed;

//		ptrToOgreObject->objectVelocity = nonConstVelocity;

		//Now we finally propel our sphere
		//BoxBody->setLinearVelocity(FireVelocity * UPM); //Remember that accelerations were game units? So are velocities

		// this gives the sphere velocity in the direction that the camera is currently pointing in
		ptrToOgreObject->btRigidBodyObject->setLinearVelocity(nonConstVelocity);
		ptrToOgreObject->objectVelocity = nonConstVelocity;

		// Add it to the physics world
		//		dynamicsWorld->addRigidBody(bob);
		dynamicsWorld->addRigidBody(ptrToOgreObject->btRigidBodyObject);
//		collisionShapes.push_back(asteroidShape);

		ptrToOgreObject->objectType = "Asteroid";
 		ptrToOgreObject->objectPosition = Vector3(0.0, 0.0, 0.0);
		ptrToOgreObject->objectDelete = false;

		ptrToOgreObjects.push_back(ptrToOgreObject);
	}

	//------------------------------------------------------------------------------------------ 

	// pulled this function from assignment 4, going to add one cube to see if it works
	void createBullet(const btVector3 &Position, btScalar Mass, const btVector3 &scale, const btVector3 &Velocity){
		// empty ogre vectors for the cubes size and position
		Ogre::Vector3 size = Ogre::Vector3::ZERO;
		Ogre::Vector3 pos = Ogre::Vector3::ZERO;

		// Convert the bullet physics vector to the ogre vector
		pos.x = Position.getX();
		pos.y = Position.getY();
		pos.z = Position.getZ();

		// need to put the 4 pointers into the vector of objects
		ptrToOgreObject = new ogreObject;

		// more than one at a time so no entity name
		ptrToOgreObject->entityObject = mSceneMgr->createEntity("sphere.mesh");
		//boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
		//      asteroidEntity->setCastShadows(true);
		ptrToOgreObject->sceneNodeObject = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		ptrToOgreObject->sceneNodeObject->showBoundingBox(true);
		ptrToOgreObject->sceneNodeObject->attachObject(ptrToOgreObject->entityObject);

//		bulletEntity->setMaterialName("Examples/RustySteel");
		ptrToOgreObject->entityObject->setMaterialName("Bullet/Blue");

		//add every bulletNode to the allBulletNodes vector
//		allBulletNodes.push_back(bulletNode);

		ptrToOgreObject->sceneNodeObject->scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		//boxNode->setScale(Vector3(0.1,0.1,0.1));
		Ogre::AxisAlignedBox boundingB = ptrToOgreObject->entityObject->getBoundingBox();
		//Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
		boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		size = boundingB.getSize()*0.95f;
		btTransform Transform;
		Transform.setIdentity();
		Transform.setOrigin(Position);
		ptrToOgreObject->myMotionStateObject = new MyMotionState(Transform,ptrToOgreObject->sceneNodeObject);
		//Give the rigid body half the size
		// of our cube and tell it to create a BoxShape (cube)
		btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);

		// add from bullet physiscs.org favorites, sphere collision shape
//		ptrToOgreObject->btCollisionShapeObject = new btSphereShape(1);
		ptrToOgreObject->btCollisionShapeObject = new btBoxShape(HalfExtents);
		btVector3 LocalInertia;
		ptrToOgreObject->btCollisionShapeObject->calculateLocalInertia(Mass, LocalInertia);
		ptrToOgreObject->btRigidBodyObject = new btRigidBody(Mass, ptrToOgreObject->myMotionStateObject, ptrToOgreObject->btCollisionShapeObject, LocalInertia);

		ptrToOgreObject->btCollisionObjectObject = ptrToOgreObject->btRigidBodyObject;

		// Store a pointer to the Ogre Node so we can update it later
		//       RigidBody->setUserPointer((void *) (boxNode));

		ptrToOgreObject->btRigidBodyObject->setUserPointer(ptrToOgreObject);

//		this needs to be btCollisionObject not btRigidBody
//		bulletBody->m_friction = 0.0;

		
		//	   RigidBody->setUserPointer(bodies[bodies.size()-1]);

		// giving our bullets a little linear velocity
		// Define our bullet speed
		static float Speed = 350.f;


		//can only create bullets if ship or spaceship exisits
        //if(shipSceneNode != NULL){
           //NewPosition = shipSceneNode->_getDerivedOrientation() * Vector3::NEGATIVE_UNIT_Z;
           // NewPosition = NewPosition.normalise();
        //}

		btVector3 nonConstVelocity;
		nonConstVelocity = Velocity;
		//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
		nonConstVelocity.normalize();
		nonConstVelocity *= Speed;

		ptrToOgreObject->objectVelocity = nonConstVelocity;

		//Now we finally propel our sphere
		//BoxBody->setLinearVelocity(FireVelocity * UPM); //Remember that accelerations were game units? So are velocities

		// this gives the sphere velocity in the direction that the camera is currently pointing in
		ptrToOgreObject->btRigidBodyObject->setLinearVelocity(nonConstVelocity);

		// Add it to the physics world
		//		dynamicsWorld->addRigidBody(bob);
		dynamicsWorld->addRigidBody(ptrToOgreObject->btRigidBodyObject);
//		collisionShapes.push_back(bulletShape);

		ptrToOgreObject->objectType = "Bullet";
 		ptrToOgreObject->objectPosition = Vector3(0.0, 0.0, 0.0);
		ptrToOgreObject->objectDelete = false;

		ptrToOgreObjects.push_back(ptrToOgreObject);
	}

//------------------------------------------------------------------------------------------ 
	
	void createShip(const btVector3 &Position, btScalar Mass, const btVector3 &scale){
		// empty ogre vectors for the cubes size and position
		Ogre::Vector3 size = Ogre::Vector3::ZERO;
		Ogre::Vector3 pos = Ogre::Vector3::ZERO;

		// Convert the bullet physics vector to the ogre vector
		pos.x = Position.getX();
		pos.y = Position.getY();
		pos.z = Position.getZ();

		// need to put the 4 pointers into the vector of objects
		ptrToOgreObject = new ogreObject;

		// remember getting rid of name here means no entity with same name
//		pushToVec.entityObject = mSceneMgr->createEntity(name, "razor.mesh");
		ptrToOgreObject->entityObject = mSceneMgr->createEntity("razor.mesh");
//		shipEntity = mSceneMgr->createEntity(name, "ogrehead.mesh");
		//boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
		//      asteroidEntity->setCastShadows(true);
		ptrToOgreObject->sceneNodeObject = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		ptrToOgreObject->sceneNodeObject->showBoundingBox(true);
		ptrToOgreObject->sceneNodeObject->attachObject(ptrToOgreObject->entityObject);

		
//		ptrToOgreObject->entityObject->setMaterialName("Ship/Yellow");

//		ptrToOgreObject->sceneNodeObject->getOrientation();

		//	   sqrt(0.5)	0	0	sqrt(0.5)	90° rotation around Z axis	   

//	shipNode->setOrientation(shipNode->getOrientation() * Ogre::Quaternion(sqrt_one_half,-sqrt_one_half,0.0,0.0));

		//	   shipNode->roll(Ogre::Degree(90),Ogre::Node::TS_LOCAL);
//		mSceneMgr->getSceneNode("tom")->setOrientation(shipNode->getOrientation() * Ogre::Quaternion(sqrt_one_half,-sqrt_one_half,0.0,0.0));

//		ptrToOgreObject->sceneNodeObject->getOrientation();

		//shipEntity->setMaterialName("RZR-002");

		//	   shipEntity->setMaterialName("Numbers/Number_6");

		ptrToOgreObject->sceneNodeObject->scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		//boxNode->setScale(Vector3(0.1,0.1,0.1));
		Ogre::AxisAlignedBox boundingB = ptrToOgreObject->entityObject->getBoundingBox();
		//Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
		boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		size = boundingB.getSize()*0.95f;


// this went away for Collin and Bullet
//		btTransform Transform;
//		Transform.setIdentity();
//		Transform.setOrigin(Position);
//      ptrToOgreObject->myMotionStateObject = new MyMotionState(Transform,ptrToOgreObject->sceneNodeObject);


		ShipTransform.setIdentity();
		ShipTransform.setOrigin(Position);
		btQuaternion Rotate = btQuaternion(sqrt_one_half, 0, 0, sqrt_one_half);
		ShipTransform.setRotation(Rotate);
//		btQuaternion Rotate = btQuaternion(sqrt_one_half, 0, 0, sqrt_one_half);
//		ShipTransform.setRotation(Rotate);

		
		ptrToOgreObject->myMotionStateObject = new MyMotionState(ShipTransform,ptrToOgreObject->sceneNodeObject);
		//Give the rigid body half the size
		// of our cube and tell it to create a BoxShape (cube)
		btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);

		// add from bullet physiscs.org favorites, sphere collision shape
//		ptrToOgreObject->btCollisionShapeObject = new btSphereShape(1);
		ptrToOgreObject->btCollisionShapeObject = new btBoxShape(HalfExtents);
		btVector3 LocalInertia;
		ptrToOgreObject->btCollisionShapeObject->calculateLocalInertia(Mass, LocalInertia);
		ptrToOgreObject->btRigidBodyObject = new btRigidBody(Mass, ptrToOgreObject->myMotionStateObject, ptrToOgreObject->btCollisionShapeObject, LocalInertia);

		// save the btRigidBody pointer of the ship so easier to move the ship later
		shipRigidBody = ptrToOgreObject->btRigidBodyObject;
		// save the SceneNode pointer of the ship so easier to move the ship later
		shipSceneNode = ptrToOgreObject->sceneNodeObject;

		ptrToOgreObject->btCollisionObjectObject = ptrToOgreObject->btRigidBodyObject;

		// Store a pointer to the Ogre Node so we can update it later
		ptrToOgreObject->btRigidBodyObject->setUserPointer(ptrToOgreObject);

		// this is to try and stop crazy spinning
		ptrToOgreObject->btRigidBodyObject->setAngularFactor(btVector3(0, 0, 0));

/* pulling velocity from here and moving into key press method
		// giving our cube a little linear velocity
		// Define our bullet speed
		static float Speed = 10.f;

		//Vector conversion
//		btVector3 Velocity = btVector3(-50.0, -25.0, 0);
		btVector3 Velocity = btVector3(0, 1, 0);

		//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
		Velocity.normalize();
		Velocity *= Speed;

		//Now we finally propel our sphere
		//BoxBody->setLinearVelocity(FireVelocity * UPM); //Remember that accelerations were game units? So are velocities

		// this gives the sphere velocity in the direction that the camera is currently pointing in
		// no velocity for ship to start

//		ptrToOgreObject->btRigidBodyObject->applyImpulse(
		ptrToOgreObject->btRigidBodyObject->setLinearVelocity(Velocity);
*/
		// Add it to the physics world
		dynamicsWorld->addRigidBody(ptrToOgreObject->btRigidBodyObject);
//		collisionShapes.push_back(shipShape);

		ptrToOgreObject->objectType = "Ship";
 		ptrToOgreObject->objectPosition = Vector3(0.0, 0.0, 0.0);
		ptrToOgreObject->objectDelete = false;

		ptrToOgreObjects.push_back(ptrToOgreObject);



//		btCollisionShape* tom = shipShape->getName();
//		btCollisionShape* bob = shipShape->getName();
	}

	//------------------------------------------------------------------------------------------ 

	// make the screen boundary bounding boxes
	void createBoundaries(void)
	{
		// create rigidbody, plane for left side (1, 1500, 1500)
		btCollisionShape* leftSideShape = new btBoxShape(btVector3(btScalar(1.),btScalar(1500.),btScalar(1500.)));
		collisionShapes.push_back(leftSideShape);

		btTransform leftSideTransform;
		leftSideTransform.setIdentity();
		// left edge is -580 on x-axis 
		leftSideTransform.setOrigin(btVector3(-horzSize,0,0));
//		leftSideTransform.setOrigin(btVector3(-580,0,0));
		{
			// local scope, names reused for generic static rigidbodies
			btScalar mass(0.);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				leftSideShape->calculateLocalInertia(mass,localInertia);

			// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(leftSideTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,leftSideShape,localInertia);
			leftSideBody = new btRigidBody(rbInfo);

//			leftSideBody->setUserPointer(leftSideBody);
			leftSideBody->setUserPointer(NULL);

			//add the body to the dynamics world
			dynamicsWorld->addRigidBody(leftSideBody);
		}

		// create rigidbody, plane for right side (1, 1500, 1500)
		btCollisionShape* rightSideShape = new btBoxShape(btVector3(btScalar(1.),btScalar(1500.),btScalar(1500.)));
		collisionShapes.push_back(rightSideShape);

		btTransform rightSideTransform;
		rightSideTransform.setIdentity();
		// right edge is 580 on x-axis 
		rightSideTransform.setOrigin(btVector3(horzSize,0,0));
//		rightSideTransform.setOrigin(btVector3(580,0,0));

		{
			// local scope, names reused for generic static rigidbodies
			btScalar mass(0.);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				rightSideShape->calculateLocalInertia(mass,localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(rightSideTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,rightSideShape,localInertia);
			rightSideBody = new btRigidBody(rbInfo);

//			rightSideBody->setUserPointer(rightSideBody);
			rightSideBody->setUserPointer(NULL);

			//add the body to the dynamics world
			dynamicsWorld->addRigidBody(rightSideBody);
		}

		// create rigidbody, plane for top side (1500, 1, 1500)
		btCollisionShape* topSideShape = new btBoxShape(btVector3(btScalar(1500.),btScalar(1.),btScalar(1500.)));
		collisionShapes.push_back(topSideShape);

		btTransform topSideTransform;
		topSideTransform.setIdentity();
		// top side is 450 on y-axis
		topSideTransform.setOrigin(btVector3(0,vertSize,0));
//		topSideTransform.setOrigin(btVector3(0,450,0));

		{
			// local scope, names reused for generic static rigidbodies
			btScalar mass(0.);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				topSideShape->calculateLocalInertia(mass,localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(topSideTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,topSideShape,localInertia);
			topSideBody = new btRigidBody(rbInfo);

//			topSideBody->setUserPointer(topSideBody);
			topSideBody->setUserPointer(NULL);

			//add the body to the dynamics world
			dynamicsWorld->addRigidBody(topSideBody);
		}

		// create rigidbody, plane for bottom side (1500, 1, 1500)
		btCollisionShape* bottomSideShape = new btBoxShape(btVector3(btScalar(1500.),btScalar(1.),btScalar(1500.)));
		collisionShapes.push_back(bottomSideShape);

		btTransform bottomSideTransform;
		bottomSideTransform.setIdentity();
		// bottom side is -450 on y-axis
		bottomSideTransform.setOrigin(btVector3(0,-vertSize,0));
//		bottomSideTransform.setOrigin(btVector3(0,-450,0));

		{
			// local scope, names reused for generic static rigidbodies
			btScalar mass(0.);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				bottomSideShape->calculateLocalInertia(mass,localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(bottomSideTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,bottomSideShape,localInertia);
			bottomSideBody = new btRigidBody(rbInfo);

//			bottomSideBody->setUserPointer(bottomSideBody);
			bottomSideBody->setUserPointer(NULL);


			//add the body to the dynamics world
			dynamicsWorld->addRigidBody(bottomSideBody);
		}
	}


	//------------------------------------------------------------------------------------------ 
	//  Assume world->stepSimulation or world->performDiscreteCollisionDetection has been called
	//  This method was found on the internet regarding bullet and then modified
	//  It will give us a vector containing contactPair structures and those structures hold 
	//  the relevant information neccessary to deal with what contacted what later
	void getContactPairs(std::vector<contactPair> &contactPairs)
	{

		//  dynamicsworld->stepsimulation is called in frameStarted method
		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
		for (int i=0;i<numManifolds;i++)
		{

			btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

			//  these were incompatible with this verison of bullet so replaced with the following:
			//  btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			//  btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
			const btCollisionObject* obA = contactManifold->getBody0();
			const btCollisionObject* obB = contactManifold->getBody1();

			int numContacts = contactManifold->getNumContacts();
			for (int j=0;j<numContacts;j++)
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (pt.getDistance()<0.f)
				{

					// temporary struct to hold extracted information which we push into the vector each iteration
					contactPair pushToVec;  

					// get the pointers to the pair of contact objects
					pushToVec.collisionObject1 = contactManifold->getBody0();
					pushToVec.collisionObject2 = contactManifold->getBody1();

					// up cast those pointers to get the pair of rigidbodypointers
					pushToVec.rigidBodyObject1 = ((btRigidBody*)contactManifold->getBody0());
					pushToVec.rigidBodyObject2 = ((btRigidBody*)contactManifold->getBody1());

					// this is very important!!! retrieve the pointer to a complete ogreObject structure
					// which are created in the create methods, that is where the setUserPointer is being done
					pushToVec.ptrToOgreObject1 = (ogreObject *)obA->getUserPointer();
					pushToVec.ptrToOgreObject2 = (ogreObject *)obB->getUserPointer();

					// this is very important!!! also need a pointer to these object pointers so that later
					// while iterating through the vector you can erase subsequent contacts for all objects
					// an address can only be assigned to a l-value, that is why these globals are needed
					needAddressOfObject1 = (ogreObject *)obA->getUserPointer();
					needAddressOfObject2 = (ogreObject *)obB->getUserPointer();

					// these then are the pointers to pointers, which will be used in the firstContactPairsOnly method
					pushToVec.ptrToOgreObject1Ptr = &needAddressOfObject1;
					pushToVec.ptrToOgreObject2Ptr = &needAddressOfObject2;

					// get bullet contact position and normal
					const btVector3& ptA = pt.getPositionWorldOnA();
					const btVector3& ptB = pt.getPositionWorldOnB();
					const btVector3& normalOnB = pt.m_normalWorldOnB;

//					pushToVec.btPositionObject1 = ptA; 
//					pushToVec.btPositionObject2 = ptB;

					// take that contact position and put it in an ogre vector
					pushToVec.positionObject1.x = ptA.getX();
					pushToVec.positionObject1.y = ptA.getY();
					pushToVec.positionObject1.z = ptA.getZ();
					pushToVec.positionObject2.x = ptB.getX();
					pushToVec.positionObject2.y = ptB.getY();
					pushToVec.positionObject2.z = ptB.getZ();

					// push the temporary struct into the vector
					contactPairs.push_back(pushToVec);
				}
			}
		}
	}

	//------------------------------------------------------------------------------------------ 
	// This method iterates through the contactPairs vector and erases all elements that have both pointers set to NULL
	// you are settting the pointer to the pointer to NULL so that in real time you can you will see the NULLS in the vector
	// assumption is that for the first pair can never have both pointers as NULL, and remember the boundaries are already set to NULL
	// you will end up with a vector
	// remove all contact pairs that occured after the 1st contact
	void firstContactPairsOnly(std::vector<contactPair> &contactPairs) {

		// only iterate through the vector if it is not empty
		if(!contactPairs.empty())

			// an iterator returned from an erase is a valid iterator to the next element in the vector
			for(std::vector<contactPair>::iterator itr = contactPairs.begin(); itr != contactPairs.end(); ) {
				// code to delete duplicates right here
				// if both both are null erase element
				// else set both to Null and move forward one

				// this works because assuming that boundary sceneNodepointers never null though we never set them!!!
				// should be okay to set them to null because they are never used
				// the 1st pair will never both be NULL, worst case is 1 NULL for boundary
				// remember boundaries are always NULL so if both are NULL object already hit something
				// so we can erase that pair out of the vector

				// very key !!! dereference the ptr to ptr to get the null and delete those that are
				// this still doesn't work because it is saved pointers and not real time pointers
				if(*(itr->ptrToOgreObject1Ptr) == NULL && *(itr->ptrToOgreObject2Ptr) == NULL)
					// delete the no longer needed object
					itr = contactPairs.erase(itr);
				else {
					// set the ptr to ptr of contact objects to NULL but leave in vector because this is a first contact
					*(itr->ptrToOgreObject1Ptr) = NULL;
					*(itr->ptrToOgreObject2Ptr) = NULL;
					++itr;
				}

			
			}  // after this vector iteration we are left with strictly first contacts
	}

	//------------------------------------------------------------------------------------------ 

	//  this method wraps ogreObjects back around to the opposite side of the screen when they hit the edge of the screen
	//  we are paasing this method type, position, velocity, and size
	void screenWrap(ogreObject * ptrToOgreObject, Vector3 objectPosition) {

		
		// first figure out how much to offset for new creation so creates with out boundary collision
		Real objectOffset = 50.0;  // see if 50 is enough for everything
		// second determine the new position to create the object at

		//  reverse object position for screen wrap

		// if the collision happens in the cornors then translate to opposite corner
		if((objectPosition.x >= horzSize - 25.0 || -objectPosition.x >= horzSize - 25.0)  &&
			(objectPosition.y >= vertSize - 25.0 || -objectPosition.y >= vertSize - 25.0)) {
				objectPosition.x = -objectPosition.x;
				objectPosition.y = -objectPosition.y;
				objectPosition.z = 0;
		//  if the collison happens only in the x-axis then only translate in the x
		} else if(objectPosition.x >= horzSize - 5.0 || -objectPosition.x >= horzSize - 5.0) {
					objectPosition.x = -objectPosition.x;
					objectPosition.y = objectPosition.y;
					objectPosition.z = 0;
				//  must be that collison happened only in the y-axis then only translate in the y
				} else if(objectPosition.y >= vertSize - 5.0 || -objectPosition.y >= vertSize - 5.0) {
							objectPosition.x = objectPosition.x;
							objectPosition.y = -objectPosition.y;
							objectPosition.z = 0;
						}


		// we need to scale down the contact point some so that the whole newly created object is inside of the boundaries
		objectPosition.x *= 0.9;
		objectPosition.y *= 0.9;
		
		if(ptrToOgreObject->objectType == "Asteroid") {
			if(ptrToOgreObject->objectSize == 3)
				createAsteroid(ptrToOgreObject->objectSize, ptrToOgreObject->objectSpeedMultiplier, btVector3(objectPosition.x,objectPosition.y,objectPosition.z),
					1.0f,btVector3(0.3,0.3,0.3),ptrToOgreObject->objectVelocity);
			else if(ptrToOgreObject->objectSize == 2)
					createAsteroid(ptrToOgreObject->objectSize, ptrToOgreObject->objectSpeedMultiplier, btVector3(objectPosition.x,objectPosition.y,objectPosition.z),
						1.0f,btVector3(0.2,0.2,0.2),ptrToOgreObject->objectVelocity);
			else if(ptrToOgreObject->objectSize == 1)
				createAsteroid(ptrToOgreObject->objectSize, ptrToOgreObject->objectSpeedMultiplier, btVector3(objectPosition.x,objectPosition.y,objectPosition.z),
					1.0f,btVector3(0.1,0.1,0.1),ptrToOgreObject->objectVelocity);
	}
		
		if(ptrToOgreObject->objectType == "Ship") {
			// need to offset a little bit so new ship is created just inside the boundary
			createShip(btVector3(objectPosition.x,objectPosition.y,objectPosition.z),0.25f,btVector3(0.4,0.4,0.4)/*,ptrToOgreObject->objectVelocity*/);
	}

	}

//------------------------------------------------------------------------------------------ 
void asteroidRandomness(int &magnitude, Vector3 &newLocation, Vector3 &newDirection)
{
  int position_x, position_y, velocity_x, velocity_y, posOrNeg;

	// randomness for asteroids

	// how fast will it be moving
	magnitude = rand() % 10 + 1;

	//  what position will it appear at
	// want it outside of centered ship and inside of boundaries
	position_x = 50 + (rand() % (horzSize - 100));
	position_y = 50 + (rand() % (vertSize - 100));

	// random positive or negative as well
	posOrNeg = rand() % 2;
	if(posOrNeg == 1)
		position_x = -position_x;
	posOrNeg = rand() % 2;
	if(posOrNeg == 1)
		position_y = -position_y;

	newLocation = Vector3(position_x, position_y, 0);

	// what direction will it be moving
	velocity_x = rand() % 10;
	velocity_y = rand() % 10;

	// random positive or negative as well
	posOrNeg = rand() % 2;
	if(posOrNeg == 1)
		velocity_x = -velocity_x;
	posOrNeg = rand() % 2;
	if(posOrNeg == 1)
		velocity_y = -velocity_y;

	newDirection = Vector3(velocity_x, velocity_y, 0);
}
	//------------------------------------------------------------------------------------------ 
	//  This method actually handles the 1st contact pairs that are in the vector
	void handleCollisions(std::vector<contactPair> &contactPairs)
	{

		int objectSize1, objectSize2;
		Vector3 collisionPoint1, collisionPoint2;
		

		// only iterate through the vector if it is not empty
		if(!contactPairs.empty())

			// we iterate through the contactPairs vector and handle every
			for(std::vector<contactPair>::iterator itr = contactPairs.begin(); itr < contactPairs.end(); ++itr) {


				//this is all going to change now, how are we going to do this????
				// no bondaries, not even in vector,  if find pointer in vector then deal with it
				// 1) no boudaries un vector because not an object
				// 2) if find pointer in vector then deal with it and then set it to null so do not deal with it again
				// 3) if not in vector then it is a boundary
				// all of these need to be nested for speed as well
				if(itr->ptrToOgreObject1 == NULL && itr->ptrToOgreObject2->objectType == "Asteroid") {

//					btVector3 translateObject = btVector3(250, -250, 0);
//					itr->rigidBodyObject2->translate(translateObject);
					// need to translate asyeroid to other side of screen
					// so delete the old one and then make a new one 
					// before you delete need the current position and velocity
					// ogreObject has the velocitiy and contactPairs has the position
					screenWrap(itr->ptrToOgreObject2, itr->positionObject2); 
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;
				
				}

				if(itr->ptrToOgreObject2 == NULL && itr->ptrToOgreObject1->objectType == "Asteroid") {
					screenWrap(itr->ptrToOgreObject1, itr->positionObject1); 
					deleteOgreObjects(itr->ptrToOgreObject1);
					continue;
				}

				if(itr->ptrToOgreObject1 == NULL && itr->ptrToOgreObject2->objectType == "Bullet") {
					screenWrap(itr->ptrToOgreObject2, itr->positionObject2); 
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;
				}

				if(itr->ptrToOgreObject2 == NULL && itr->ptrToOgreObject1->objectType == "Bullet") {
					screenWrap(itr->ptrToOgreObject1, itr->positionObject1); 
					deleteOgreObjects(itr->ptrToOgreObject1);
					continue;
				}

				if(itr->ptrToOgreObject1 == NULL && itr->ptrToOgreObject2->objectType == "Ship") {
					screenWrap(itr->ptrToOgreObject2, itr->positionObject2); 
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;
				}

				if(itr->ptrToOgreObject2 == NULL && itr->ptrToOgreObject1->objectType == "Ship") {
					screenWrap(itr->ptrToOgreObject1, itr->positionObject1); 
					deleteOgreObjects(itr->ptrToOgreObject1);
					continue;
				}

				if(itr->ptrToOgreObject1 == NULL && itr->ptrToOgreObject2->objectType == "SpaceShip") {
					screenWrap(itr->ptrToOgreObject2, itr->positionObject2); 
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;
				}

				if(itr->ptrToOgreObject2 == NULL && itr->ptrToOgreObject1->objectType == "SpaceShip") {
					screenWrap(itr->ptrToOgreObject1, itr->positionObject1); 
					deleteOgreObjects(itr->ptrToOgreObject1);
					continue;
				}

				if(itr->ptrToOgreObject1->objectType == "Asteroid" && itr->ptrToOgreObject2->objectType == "Bullet") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;
				}

				if(itr->ptrToOgreObject2->objectType == "Asteroid" && itr->ptrToOgreObject1->objectType == "Bullet") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;

				}

				if(itr->ptrToOgreObject1->objectType == "Asteroid" && itr->ptrToOgreObject2->objectType == "Ship") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
                    createShip(btVector3(0, 0, 0),1.0f,btVector3(0.4,0.4,0.4));
					continue;

				}

				if(itr->ptrToOgreObject2->objectType == "Asteroid" && itr->ptrToOgreObject1->objectType == "Ship") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
                    createShip(btVector3(0, 0, 0),1.0f,btVector3(0.4,0.4,0.4));
					continue;

				}

				if(itr->ptrToOgreObject1->objectType == "Asteroid" && itr->ptrToOgreObject2->objectType == "SpaceShip") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;

				}

				if(itr->ptrToOgreObject2->objectType == "Asteroid" && itr->ptrToOgreObject1->objectType == "SpaceShip") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;

				}

				if(itr->ptrToOgreObject1->objectType == "Bullet" && itr->ptrToOgreObject2->objectType == "Ship") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
                    createShip(btVector3(0, 0, 0),1.0f,btVector3(0.4,0.4,0.4));
					continue;

				}

				if(itr->ptrToOgreObject2->objectType == "Bullet" && itr->ptrToOgreObject1->objectType == "Ship") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
                    createShip(btVector3(0, 0, 0),1.0f,btVector3(0.4,0.4,0.4));
					continue;

				}

				if(itr->ptrToOgreObject1->objectType == "Bullet" && itr->ptrToOgreObject2->objectType == "SpaceShip") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;


				}
				if(itr->ptrToOgreObject2->objectType == "Bullet" && itr->ptrToOgreObject1->objectType == "SpaceShip") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;

				}

				if(itr->ptrToOgreObject1->objectType == "Ship" && itr->ptrToOgreObject2->objectType == "SpaceShip") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
                    createShip(btVector3(0, 0, 0),1.0f,btVector3(0.4,0.4,0.4));
					continue;

				}

				if(itr->ptrToOgreObject2->objectType == "Ship" && itr->ptrToOgreObject1->objectType == "SpaceShip") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
                    createShip(btVector3(0, 0, 0),1.0f,btVector3(0.4,0.4,0.4));
					continue;

				}

				if(itr->ptrToOgreObject1->objectType == "Asteroid" && itr->ptrToOgreObject2->objectType == "Asteroid") {
					
					// get the sizes from ogreObject
					// get the positions from bullet
					// delete both objects
					// create two new ones at collision point
					// if size is 1 then just delete it

					// get all of the necessary information first because need to delete before create
					objectSize1 = itr->ptrToOgreObject1->objectSize;
					objectSize2 = itr->ptrToOgreObject1->objectSize;
					collisionPoint1 = itr->positionObject1;
					collisionPoint2 = itr->positionObject2;

					// delete the objects that collided before creating new ones at the collision point
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);

					// create the new objects	
					// order matter here so go small to large or else large just keeps getting smaller (no size 2)
					if(objectSize1 == 2) {
						asteroidRandomness(magnitudeAAA, locationAAA, directionAAA);
						// in this case location is not random, comes from collision point so overwrite with collision point
						locationAAA.x = collisionPoint1.x;
						locationAAA.y = collisionPoint1.y;
						createAsteroid(1, magnitudeAAA, btVector3(locationAAA.x, locationAAA.y, 0),1.0f,btVector3(0.1,0.1,0.1),btVector3(directionAAA.x, directionAAA.y, 0));
					} else if(objectSize1 == 3) {
								asteroidRandomness(magnitudeAAA, locationAAA, directionAAA);
								locationAAA.x = collisionPoint1.x + 25;
								locationAAA.y = collisionPoint1.y;
								createAsteroid(2, magnitudeAAA, btVector3(locationAAA.x, locationAAA.y, 0),1.0f,btVector3(0.2,0.2,0.2),btVector3(directionAAA.x, directionAAA.y, 0));
							}
					if(objectSize2 == 2) {
						asteroidRandomness(magnitudeAAA, locationAAA, directionAAA);
						locationAAA.x = collisionPoint2.x;
						locationAAA.y = collisionPoint2.y;
						createAsteroid(1, magnitudeAAA, btVector3(locationAAA.x, locationAAA.y, 0),1.0f,btVector3(0.1,0.1,0.1),btVector3(directionAAA.x, directionAAA.y, 0));
					} else if(objectSize2 == 3) {
								asteroidRandomness(magnitudeAAA, locationAAA, directionAAA);
								locationAAA.x = collisionPoint2.x - 25;
								locationAAA.y = collisionPoint2.y;
								createAsteroid(2, magnitudeAAA, btVector3(locationAAA.x, locationAAA.y, 0),1.0f,btVector3(0.2,0.2,0.2),btVector3(directionAAA.x, directionAAA.y, 0));
							} 


					continue;

				}

				if(itr->ptrToOgreObject1->objectType == "Bullet" && itr->ptrToOgreObject2->objectType == "Bullet") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;

				}
				// this is for testing only
				if(itr->ptrToOgreObject2->objectType == "SpaceShip" && itr->ptrToOgreObject1->objectType == "SpaceShip") {
					deleteOgreObjects(itr->ptrToOgreObject1);
					deleteOgreObjects(itr->ptrToOgreObject2);
					continue;

				}

//				if(itr->ptrToOgreObject1 == NULL && itr->ptrToOgreObject2 == NULL) {
//				continue;
//				}

				
				
			}//	deleteOgreObjects(ogreObjects); this goes away because this was the old remove hard objects from vector
				// empty the contactPairs vector before iterating through the manifold again

				contactPairs.clear();
	}

	//------------------------------------------------------------------------------------------ 
	//  This method deletes all aspects (Ogre and Bullet) of the objects we create, it is passed a pointer to the object to delete
	void deleteOgreObjects(ogreObject * &ptrToOgreObject)  // Very important, this has to be by reference!!!
	{

		// if it is a Ship then set the extra pointers used to move the Ship to NULL before the delete
		if(ptrToOgreObject->objectType == "Ship") {
			shipSceneNode = NULL;
			shipRigidBody = NULL;
			}

		//  delete the ogre aspects of the object
		//  detach the entity from the parent sceneNode, destroy the entity, destroy the sceneNode, and set the entity and sceneNode pointers to NULL
		ptrToOgreObject->entityObject->detachFromParent();
		mSceneMgr->destroyEntity(ptrToOgreObject->entityObject);
		ptrToOgreObject->entityObject = NULL;
		mSceneMgr->destroySceneNode(ptrToOgreObject->sceneNodeObject);
		ptrToOgreObject->sceneNodeObject = NULL;
		ptrToOgreObject->objectDelete = true;

		// delete the bullet aspect of the objects, our objects should always have motion state
		// delete the motionState, remove and delete the btCollisionObject, delete the btCollisonShape, and set all pointers to NULL
		if(ptrToOgreObject->btRigidBodyObject && ptrToOgreObject->btRigidBodyObject->getMotionState())
			delete ptrToOgreObject->btRigidBodyObject->getMotionState();
		ptrToOgreObject->myMotionStateObject = NULL;
		dynamicsWorld->removeCollisionObject(ptrToOgreObject->btCollisionObjectObject);
		delete ptrToOgreObject->btCollisionObjectObject;
		ptrToOgreObject->btCollisionObjectObject = NULL;
		ptrToOgreObject->btRigidBodyObject = NULL;
		delete ptrToOgreObject->btCollisionShapeObject;
		ptrToOgreObject->btCollisionShapeObject = NULL;

		

		// now need to clean up the memory and set the pointer to NULL for that memory
//			delete ptrToOgreObject;
//			ptrToOgreObject = NULL;  // this is back here because this is where the delete works cleanly -- need hook to delete the stupid pointer out of the ogreObjects vector this does not get back to the actual pointer in the ptrToOgreObjects vector


			// not handling something correctly in this function
			removeDynamicOgreObject(ptrToOgreObject, ptrToOgreObjects);

				
	}

	//------------------------------------------------------------------------------------------ 
	//  This method removes all of the NULL pointers out of the vector of pointers to ogreObjects
	void removeDynamicOgreObject(ogreObject * ptrToOgreObject, std::vector<ogreObject *> &ptrToOgreObjects)
	{

		// only iterate through the vector if it is not empty
		if(!ptrToOgreObjects.empty())

			// an iterator returned from an erase is a valid iterator to the next element in the vector
			for(std::vector<ogreObject *>::iterator itr = ptrToOgreObjects.begin(); itr != ptrToOgreObjects.end(); ++itr) {

				//  itr is a pointer to a pointer to an ogreObject so we must dereference it 
				//  twice before we can get to the actual struct members
				if(*itr == ptrToOgreObject) {

					// now need to clean up the dynamic ogreObjects we have created


					delete *itr;
					// set the pointer to the ogreObject to NUll
					*itr = NULL;

					// and then delete that pointer out of the vector
					ptrToOgreObjects.erase(itr);

					// no need to continue iterating and the iterator just became invalid
					break;  
				}

			}
	}

};



#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"

INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
#else
int main(int argc, char **argv)
#endif
{
	try
	{
		Application app;
		app.go();
	}
	catch(Exception& e)
	{
#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
		MessageBoxA(NULL, e.getFullDescription().c_str(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
		fprintf(stderr, "An exception has occurred: %s\n",
			e.getFullDescription().c_str());
#endif
	}

	return 0;
}

/*
// make a simple ship, no ship primatives in ogre
ManualObject* createShipMesh(Ogre::String name, Ogre::String matName)
{
ManualObject* ship = new ManualObject(name);
cube->begin(matName);



ship->triangle(0,1,2);      ship->triangle(3,1,0);
ship->triangle(0,3,2);      ship->triangle(1,3,2);

cube->end();
}

*/
