#ifndef TESTENKI_H
#define TESTENKI_H

#include <../enki/PhysicalEngine.h>
#include <../enki/robots/e-puck/EPuck.h>
#include <sys/time.h>

#include <iostream>
#include <fstream>

extern double ctrl_stepsize;
extern double world_width;
extern double world_height;

using namespace Enki;

//Agent
class Agent: public EPuck
{
public:
       Agent();
       ~Agent();
       virtual void controlStep(double dt);
};

//Replica
class Agent1: public EPuck
{
public:
       Agent1();
       ~Agent1();
       virtual void controlStep(double dt);
};

//World
class TestWorld: public Enki::World
{
public:
       TestWorld(double width, double height);
       ~TestWorld();
       void creatAgent(Agent* a);
       void run();
};

//world1
class TestWorld1: public Enki::World
{
public:
       TestWorld1(double width, double height);
       ~TestWorld1();
       void creatReplica(Agent1* r);
       void run();
};

#ifdef ViewerMode
//Viewer
#include <../viewer/Viewer.h>
#include <QtGui/QApplication>
#include <QtGui/QtGui>

class TestViewer: public ViewerWidget
{
protected:
          unsigned maxSteps;
          unsigned timeMultiplier;
          unsigned c_step;
          int timerId;
public:
       TestViewer(World *world, unsigned maxSteps, unsigned timeMultiplier, QWidget *parent);
       virtual void timerEvent(QTimerEvent* event);
       virtual void wheelEvent(QWheelEvent* event);
};
#endif
#endif
