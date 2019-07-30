/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <viewer/Viewer.h>
#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/marxbot/Marxbot.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <QApplication>
#include <QtGui>
#include <iostream>

#ifdef USE_SDL
#include <SDL.h>
#endif

/*!	\file Studio.cpp
	\brief Test of the Qt-based viewer widget
*/

using namespace Enki;
using namespace std;

double world_width = 50.0;
double world_height = 50.0;

class EnkiPlayground : public ViewerWidget
{	
public:
	EnkiPlayground(World *world, QWidget *parent = 0) :
		ViewerWidget(world, parent)
	{
	        EPuck *epuck = new EPuck;
        	epuck->pos = Enki::Point(25,25);
        	epuck->leftSpeed =  2;
        	epuck->rightSpeed = 2;
        	epuck->angle = 0.0*M_PI;
        	world->addObject(epuck);

 		for (int i = 0; i < 9; i++)
		{
			PhysicalObject* o = new PhysicalObject;

			if (i == 0)
    			{o->pos = Enki::Point(world_width/4, world_height/4);}
  			if (i == 1)
   			{o->pos = Enki::Point(world_width/4, world_height/2);}
  			if (i == 2)
    			{o->pos = Enki::Point(world_width/4, 3*world_height/4);}
 			if (i == 3)
    			{o->pos = Enki::Point(world_width/2, world_height/4);}
  			if (i == 4)
    			{o->pos = Enki::Point(world_width/2, world_height/2);}
  			if (i == 5)
    			{o->pos = Enki::Point(world_width/2, 3*world_height/4);}
  			if (i == 6)
    			{o->pos = Enki::Point(3*world_width/4, world_height/4);}
  			if (i == 7)
    			{o->pos = Enki::Point(3*world_width/4, world_height/2);}
  			if (i == 8)
    			{o->pos = Enki::Point(3*world_width/4, 3*world_height/4);}
			
			o->setCylindric(3.7/2.0, 10.0, 152);
			o->setColor(Color::red);
			world->addObject(o);
		}
	}
	
	
	~EnkiPlayground()
	{
	}
	
	virtual void timerEvent(QTimerEvent * event)
	{
		ViewerWidget::timerEvent(event);
	}
	
	virtual void sceneCompletedHook()
	{
		
	}
};

// http://qtnode.net/wiki?title=Qt_with_cmake
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	
	// Create the world and the viewer
	bool igt(app.arguments().size() > 1);
	QImage gt;
	if (igt)
		gt = QGLWidget::convertToGLFormat(QImage(app.arguments().last()));
	igt = !gt.isNull();
	#if QT_VERSION >= QT_VERSION_CHECK(4,7,0)
	const uint32_t *bits = (const uint32_t*)gt.constBits();
	#else
	uint32_t *bits = (uint32_t*)gt.bits();
	#endif
	World world(50,50, Color(0.9, 0.9, 0.9), igt ? World::GroundTexture(gt.width(), gt.height(), bits) : World::GroundTexture());
	EnkiPlayground viewer(&world);
	
	viewer.show();
	
	return app.exec();
}

