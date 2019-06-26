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
class userSensor;

class Replica: public DifferentialWheeled
{
private:
       friend class userSensor;
public:
       std::vector<userSensor *>userSensors;
       double theta[8];
       double radius[8];
       Replica(double modelValue[16]);
       ~Replica();
       virtual void controlStep(double dt);
};

class userSensor: public IRSensor
{
public:
        userSensor(Robot *owner, Vector pos, double height, double orientation, double range, double m, double x0, double c, double noiseSd);
        ~userSensor();
};

//Object
class Object: public PhysicalObject
{
public:
       Object();
       ~Object();
};

//World
class TestWorld: public Enki::World
{
public:
       TestWorld(double width, double height);
       ~TestWorld();
       void creatObject(unsigned position);
       void creatAgent(Agent* a);
       void creatReplica(Replica* r);
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
