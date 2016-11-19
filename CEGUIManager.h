#pragma once
#include "CEGUI.h"
#include "RendererModules\Ogre\CEGUIOgreRenderer.h"
#include <string>
#include "Ogre.h"

class CEGUIManager
{
public:
	CEGUIManager(void);
	~CEGUIManager(void);
	CEGUIManager* getSingleton(void);
	bool initialize( CEGUI::SubscriberSlot evnt, Ogre::TexturePtr tex);
private:
	static CEGUIManager* _instance;
    CEGUI::OgreRenderer* mRenderer;

};

