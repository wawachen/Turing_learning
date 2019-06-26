#include "TestEnki.h"

using namespace std;
//counters
unsigned c_agent = 1;
unsigned c_object = 9;
unsigned c_replica = 1;
//global features
double robot_radius = 3.7;
double world_width = 0;
double world_height = 0;
//time
double ctrl_stepsize = 0.1;
//Agent
Agent::Agent(): EPuck(CAPABILITY_BASIC_SENSORS){}

Agent::~Agent() 
{
  //std::cout<< "agent deleted"<<std::endl;
}

void Agent::controlStep(double dt)
{
/********
  #define deg2rad(x) ((x)*M_PI/180.)
  double S[8];
  double S_x = 0.0, S_y = 0.0;
  double alpha;
  int d;

  double maxvalue = 3000.0;
  double defaultSpeed = 0.8*maxSpeed;
  double agent_theta[8] = {deg2rad(-18), deg2rad(-45), deg2rad(-90), deg2rad(-142), deg2rad(142), deg2rad(90), deg2rad(45), deg2rad(18)};

  S[0] = min(1.0, infraredSensor0.getValue() / maxvalue);
  S[1] = min(1.0, infraredSensor1.getValue() / maxvalue);
  S[2] = min(1.0, infraredSensor2.getValue() / maxvalue);
  S[3] = min(1.0, infraredSensor3.getValue() / maxvalue);
  S[4] = min(1.0, infraredSensor4.getValue() / maxvalue);
  S[5] = min(1.0, infraredSensor5.getValue() / maxvalue);
  S[6] = min(1.0, infraredSensor6.getValue() / maxvalue);
  S[7] = min(1.0, infraredSensor7.getValue() / maxvalue);
  std::cout<<S[0]<<" "<<S[1]<<" "<<S[2]<<" "<<S[3]<<" "<<S[4]<<" "<<S[5]<<" "<<S[6]<<" "<<S[7]<<std::endl;
  for (unsigned i = 0; i < 8; i++)
  {
    S_x += S[i] * cos(agent_theta[i]);
    S_y += S[i] * sin(agent_theta[i]);
  }
  if (S_x > 0)
    alpha = min(1.0, sqrt(pow(S_x,2.0) + pow(S_y,2.0)));
  if (S_y > 0) {d = 1;}
  else if (S_y < 0) {d = -1;}
  else {d = 0;}

  leftSpeed = (1 - alpha) *  defaultSpeed + d * alpha * maxSpeed;
  rightSpeed = (1 - alpha) *  defaultSpeed - d * alpha * maxSpeed;
************/
  EPuck::controlStep(dt);
}

//Replica
Agent1::Agent1(): EPuck(CAPABILITY_BASIC_SENSORS){}

Agent1::~Agent1() 
{
  //std::cout<< "agent deleted"<<std::endl;
}

void Agent1::controlStep(double dt)
{
/********
  #define deg2rad(x) ((x)*M_PI/180.)
  double S[8];
  double S_x = 0.0, S_y = 0.0;
  double alpha;
  int d;

  double maxvalue = 3000.0;
  double defaultSpeed = 0.8*maxSpeed;
  double agent_theta[8] = {deg2rad(-18), deg2rad(-45), deg2rad(-90), deg2rad(-142), deg2rad(142), deg2rad(90), deg2rad(45), deg2rad(18)};

  S[0] = min(1.0, infraredSensor0.getValue() / maxvalue);
  S[1] = min(1.0, infraredSensor1.getValue() / maxvalue);
  S[2] = min(1.0, infraredSensor2.getValue() / maxvalue);
  S[3] = min(1.0, infraredSensor3.getValue() / maxvalue);
  S[4] = min(1.0, infraredSensor4.getValue() / maxvalue);
  S[5] = min(1.0, infraredSensor5.getValue() / maxvalue);
  S[6] = min(1.0, infraredSensor6.getValue() / maxvalue);
  S[7] = min(1.0, infraredSensor7.getValue() / maxvalue);
  std::cout<<S[0]<<" "<<S[1]<<" "<<S[2]<<" "<<S[3]<<" "<<S[4]<<" "<<S[5]<<" "<<S[6]<<" "<<S[7]<<std::endl;
  for (unsigned i = 0; i < 8; i++)
  {
    S_x += S[i] * cos(agent_theta[i]);
    S_y += S[i] * sin(agent_theta[i]);
  }
  if (S_x > 0)
    alpha = min(1.0, sqrt(pow(S_x,2.0) + pow(S_y,2.0)));
  if (S_y > 0) {d = 1;}
  else if (S_y < 0) {d = -1;}
  else {d = 0;}

  leftSpeed = (1 - alpha) *  defaultSpeed + d * alpha * maxSpeed;
  rightSpeed = (1 - alpha) *  defaultSpeed - d * alpha * maxSpeed;
************/
  EPuck::controlStep(dt);
}


//World
TestWorld::TestWorld(double width, double height): Enki::World(width, height)
{
  
}

TestWorld::~TestWorld() 
{
   //std::cout<< "world deleted"<<std::endl;
}

void TestWorld::creatAgent(Agent* a)
{
  //Agent* a = new Agent();
  Enki::Point new_pos;
  bool conflict = true;
  unsigned c_conflict = 0;
  while (conflict)
  {
    conflict = false;
    new_pos = Enki::Point(
    UniformRand(robot_radius, world_width - robot_radius)(),
    UniformRand(robot_radius, world_height - robot_radius)()
    );
    for (Enki::World::ObjectsIterator i=objects.begin();i != objects.end();++i)
    {
      Agent* a_other = dynamic_cast<Agent*>(*i);
      if (a_other)
      {
        if ((a_other->pos - new_pos).norm2() < 4. * robot_radius ^ 2)
        {
          conflict = true;
          c_conflict++;
          if (c_conflict > c_agent * c_agent + 1000)
          {
            std::cerr<<"Aborting because of too many attempts to put agent in the arena."<< std::endl;
            exit(1);
          }
          break;
        }
      }
     }
     if (conflict == false)
        c_conflict = 0;
  }
  a->pos = new_pos;
  a->angle = UniformRand(0, 2.*M_PI)();
  addObject(a);
}

void TestWorld::creatReplica(Agent1* r)
{
  //Replica* r = new Replica(modelValue);
  Enki::Point new_pos;
  bool conflict = true;
  unsigned c_conflict = 0;
  while (conflict)
  {
    conflict = false;
    new_pos = Enki::Point(
    UniformRand(robot_radius, world_width - robot_radius)(),
    UniformRand(robot_radius, world_height - robot_radius)()
    );
    for (Enki::World::ObjectsIterator i=objects.begin();i != objects.end();++i)
    {
      Agent1* r_other = dynamic_cast<Agent1*>(*i);
      if (r_other)
      {
        if ((r_other->pos - new_pos).norm2() < 4. * robot_radius ^ 2)
        {
          conflict = true;
          c_conflict++;
          if (c_conflict > c_replica * c_replica + 1000)
          {
            std::cerr<<"Aborting because of too many attempts to put agent in the arena."<< std::endl;
            exit(1);
          }
          break;
        }
      }
     }
     if (conflict == false)
        c_conflict = 0;
  }
  r->pos = new_pos;
  r->angle = UniformRand(0, 2.*M_PI)();
  addObject(r);
}

void TestWorld::run()
{
    step(ctrl_stepsize, 10);
}
#ifdef ViewerMode
//Viewer
TestViewer::TestViewer(World *world, unsigned maxSteps, unsigned timeMultiplier, QWidget *parent):
  ViewerWidget(world, parent),
  maxSteps(maxSteps),
  timeMultiplier(timeMultiplier)
{
  c_step = 0;
  timerId = startTimer(100);
}

void TestViewer::timerEvent(QTimerEvent* event)
{
  if (event->timerId() == timerId)
  {
    for (unsigned i= 0; i < timeMultiplier; i++)
    {
      world->step(ctrl_stepsize, 10);
      c_step++;  
      if ((int)c_step == maxSteps)
        break;
    }
    updateGL();
    if ((int)c_step == maxSteps)
      close();
  }
}

void TestViewer::wheelEvent(QWheelEvent * event)
{
  if (event->delta() > 0)
    timeMultiplier *= 2;
  else if (event->delta() < 0 && timeMultiplier > 1)
    timeMultiplier /= 2;
  setWindowTitle(QString(tr("Simulation runnig at %0 times real-time").arg(timeMultiplier)));

}
#endif
