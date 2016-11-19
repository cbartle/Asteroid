#include "CEGUIManager.h"

CEGUIManager* CEGUIManager::_instance = NULL;
CEGUIManager::CEGUIManager(void)
{
}


CEGUIManager::~CEGUIManager(void)
{
}

	

CEGUIManager* CEGUIManager::getSingleton()
{
 if (_instance != nullptr)
   {
     return _instance;
   }
 else	        
   {
      _instance = new CEGUIManager();
	  _instance->mRenderer =  &CEGUI::OgreRenderer::bootstrapSystem();
   }
  return _instance;
}

bool CEGUIManager::initialize(CEGUI::SubscriberSlot evnt, Ogre::TexturePtr tex)
{
  //mRenderer = &CEGUI::OgreRenderer::bootstrapSystem();
  CEGUI::Imageset::setDefaultResourceGroup("Imagesets");
  CEGUI::Font::setDefaultResourceGroup("Fonts");
  CEGUI::Scheme::setDefaultResourceGroup("Schemes");
  CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
  CEGUI::WindowManager::setDefaultResourceGroup("Layouts");
  CEGUI::SchemeManager::getSingleton().create("TaharezLook.scheme");
  CEGUI::System::getSingleton().setDefaultMouseCursor("TaharezLook", "MouseArrow");
//  CEGUI::Window *guiRoot = CEGUI::WindowManager::getSingleton().loadWindowLayout("TextDemo.layout"); 
//  CEGUI::System::getSingleton().setGUISheet(guiRoot);
  CEGUI::WindowManager &wmgr = CEGUI::WindowManager::getSingleton();
  CEGUI::Window *sheet = wmgr.createWindow("DefaultWindow", "CEGUIDemo/Sheet");
  CEGUI::Window *quit = wmgr.createWindow("TaharezLook/Button", "CEGUIDemo/QuitButton");
  quit->setText("Quit");
  quit->setSize(CEGUI::UVector2(CEGUI::UDim(0.15, 0), CEGUI::UDim(0.05, 0)));
  sheet->addChildWindow(quit);
  CEGUI::System::getSingleton().setGUISheet(sheet);
  quit->subscribeEvent(CEGUI::PushButton::EventClicked,evnt);
   CEGUI::Texture &guiTex = mRenderer->createTexture(tex);
/*  
  CEGUI::Imageset &imageSet =
     CEGUI::ImagesetManager::getSingleton().create("RTTImageset", guiTex);
  imageSet.defineImage("RTTImage",
                     CEGUI::Point(0.0f, 0.0f),
                     CEGUI::Size(guiTex.getSize().d_width,
                                 guiTex.getSize().d_height),
                     CEGUI::Point(0.0f, 0.0f));
  CEGUI::Window *si = CEGUI::WindowManager::getSingleton().createWindow(
	  "TaharezLook/StaticImage", "RTTWindow");
  si->setSize(CEGUI::UVector2(CEGUI::UDim(0.5f, 0),
                            CEGUI::UDim(0.4f, 0)));
  si->setPosition(CEGUI::UVector2(CEGUI::UDim(0.5f, 0),
                                CEGUI::UDim(0.0f, 0)));
  si->setProperty("Image", CEGUI::PropertyHelper::imageToString(&imageSet.getImage("RTTImage")));
  sheet->addChildWindow(si);
  */
  return true;
}

