ACLOCAL_AMFLAGS= -I m4
noinst_HEADERS= BaseApplication.h TutorialApplication.h Wordlist.h

bin_PROGRAMS= OgreApp
OgreApp_CPPFLAGS= -I$(top_srcdir)
OgreApp_SOURCES= BaseApplication.cpp TutorialApplication.cpp
OgreApp_CXXFLAGS= $(OGRE_CFLAGS) $(OIS_CFLAGS) $(CEGUI_CFLAGS) $(bullet_CFLAGS) $(sdl_CFLAGS)
OgreApp_LDADD= $(OGRE_LIBS) $(OIS_LIBS) $(CEGUI_LIBS) $(CEGUI_OGRE_LIBS) $(bullet_LIBS) $(sdl_LIBS)
OgreApp_LDFLAGS= -lOgreOverlay -lboost_system -R/lusr/lib/cegui-0.8

EXTRA_DIST= buildit makeit
AUTOMAKE_OPTIONS= foreign
