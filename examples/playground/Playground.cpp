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
//time
double ctrl_stepsize = 0.1;
unsigned maxSteps = 500;
unsigned c_step;
int timerId;
EPuck *epuck = new EPuck;

class EnkiPlayground : public ViewerWidget
{	
public:
	EnkiPlayground(World *world,QWidget *parent = 0) :
		ViewerWidget(world, parent)
	{
	        
        	epuck->pos = Enki::Point(25,25);
        	epuck->leftSpeed =  4.4;
        	epuck->rightSpeed = 4.4;
        	epuck->angle = 0.5*M_PI;
        	world->addObject(epuck);

			c_step = 0;
  			timerId = startTimer(100);

        
	}
	
	~EnkiPlayground()
	{
	}
	
	virtual void timerEvent(QTimerEvent * event)
	{
		if (event->timerId() == timerId)
 		{
    		
      		world->step(ctrl_stepsize, 10);
			
            int Closest_sensor = 0; // ID of the sensor with the highest proximity reading.
		    int Sensor_num = 8;
			int maxDistance = 200;
			int rightDistance;
			int leftDistance;
			int totalDistance;
			int miniCornerDistance = 2200;
			int is_Cornered = 0;
			int ps_values[8]; // Array that stores the proximity readings.
			int ps_nearWall[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			int newLeftmotor_speed;
			int newRightmotor_speed;
			double motor_speed = 10.24;
			double corner_speed = 5.12;
            
			
		    ps_values[0] = epuck->infraredSensor0.getValue();
			ps_values[1] = epuck->infraredSensor1.getValue();
			ps_values[2] = epuck->infraredSensor2.getValue();
			ps_values[3] = epuck->infraredSensor3.getValue();
			ps_values[4] = epuck->infraredSensor4.getValue();
			ps_values[5] = epuck->infraredSensor5.getValue();
			ps_values[6] = epuck->infraredSensor6.getValue();
			ps_values[7] = epuck->infraredSensor7.getValue();

			for (int i = 0; i < Sensor_num; i++) {
				if (ps_values[i] > maxDistance) {
					ps_nearWall[i] = 1;
				}
				else {
					ps_nearWall[i] = 0;
				}

				if (ps_values[i] > ps_values[Closest_sensor]) {
					Closest_sensor = i;
				}
			}

			rightDistance = ps_values[0] + 0.9*ps_values[1] + 0.6*ps_values[2];
			leftDistance = ps_values[5] + 0.9*ps_values[6] + 0.6*ps_values[7];
			totalDistance = rightDistance + leftDistance;

			is_Cornered = totalDistance > miniCornerDistance;
            
			if (is_Cornered == 1){
				
				if (leftDistance > rightDistance) {
					epuck->leftSpeed =  corner_speed;
        			epuck->rightSpeed = -corner_speed;
				}
				else {
					epuck->leftSpeed =  -corner_speed;
        			epuck->rightSpeed = corner_speed;
				}
				
		    }
			else if (ps_nearWall[Closest_sensor] == 1) { 
				// e-puck meet obstacle
				switch (Closest_sensor) {
				case 0: 
					// obstacle is close in front, turn on.
					epuck->leftSpeed =  -UniformRand(6.4, 12.8)();
        		    epuck->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 1: 
					//obstacle is near the front right, turn away.
					epuck->leftSpeed =  UniformRand(0.0, 2.0)();
        		    epuck->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 2: 
					// obstacle to the right side, slightly turn away.
					epuck->leftSpeed =  UniformRand(0.0, 2.0)();
        		    epuck->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 3:
					epuck->leftSpeed =  UniformRand(6.4, 12.8)();
        		    epuck->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 4:
				    epuck->leftSpeed =  UniformRand(6.4, 12.8)();
        		    epuck->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 5: // obstacle to the left side, slightly turn away.
					epuck->leftSpeed =  UniformRand(6.4, 12.8)();
        		    epuck->rightSpeed = UniformRand(0.0, 2.0)();
					break;
				case 6: // obstacle is somewhat near the front left, turn away.
					epuck->leftSpeed =  UniformRand(6.4, 12.8)();
        		    epuck->rightSpeed = UniformRand(0.0, 2.0)();
					break;
				case 7: // obstacle is close in front, turn on spot.
					epuck->leftSpeed =  UniformRand(6.4, 10.4)();
        		    epuck->rightSpeed = -UniformRand(6.4, 10.4)();
					break;
				default:
					epuck->leftSpeed =  UniformRand(6.4, 12.8)();
        		    epuck->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				}

			}
			else if (ps_nearWall[Closest_sensor] == 0) { 
				// Go forwards
				epuck->leftSpeed =  UniformRand(-12.8, 12.8)();
        		epuck->rightSpeed = UniformRand(-12.8, 12.8)();
			}
			
			c_step++;
    		        updateGL();
			if(c_step==maxSteps)
				close();
      		//close();
  		}
		
		//ViewerWidget::timerEvent(event);
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

