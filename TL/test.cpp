#include "test.h"
#include <vector>

int maxSteps = 100;
double maxSpeed = 12.8;
double maxvalue = 3000.0;
double y2 = 50;
double x2 = 50;
double robot_radius = 3.7;


double f(double x,double x1,double y1,double k)
{
  return k*(x-x1)+y1;
}

double g(double y,double x1,double y1,double k)
{
  return x1+(y-y1)/k;
} 

//calculate the real distance from replica to the wall
double replica_cal_distance(Agent1* r)
{
  double x1 = r->pos.x;
  double y1 = r->pos.y;
  double k = tan(r->angle);
  Enki::point new_pos;

 //if parallel
  if((r->angle==0)||(r->angle==2.*M_PI)){
    new_pos = Enki::point(x2,f(x2,x1,y1,k));
    return (r->pos-new_pos).norm()-robot_radius;
  } else if(r->angle==M_PI){
    new_pos = Enki::point(0,f(0,x1,y1,k));
    return (r->pos-new_pos).norm()-robot_radius;
  } else if(r->angle==0.5*M_PI){
    new_pos = Enki::point(g(y2,x1,y1,k),y2);
    return (r->pos-new_pos).norm()-robot_radius;
  } else if(r->angle==1.5*M_PI){
    new_pos = Enki::point(g(0,x1,y1,k),0);
    return (r->pos-new_pos).norm()-robot_radius;
  } else{
  //if not parallel
  //find the intersecting points
    Enki::point points[4]={
      Enki::point(g(y2,x1,y1,k),y2),
      Enki::point(g(0,x1,y1,k),0),
      Enki::point(0,f(0,x1,y1,k)),
      Enki::point(x2,f(x2,x1,y1,k))
    };
    std::vector<int>check_inner;

    if(g(y2,x1,y1,k)>=0 && g(y2,x1,y1,k)<=x2){inner_check.push_back(0);}
    if(g(0,x1,y1,k)>=0 && g(0,x1,y1,k)<=x2){inner_check.push_back(1);}
    if(f(0,x1,y1,k)>=0 && f(0,x1,y1,k)<=y2){inner_check.push_back(2);}
    if(f(x2,x1,y1,k)>=0 && f(x2,x1,y1,k)<=y2){inner_check.push_back(3);}

    if((r->angle>0)&&(r->angle<M_PI)){
      if(points[inner_check[1]].y>points[inner_check[2]].y){
        return (r->pos-points[inner_check[1]]).norm()-robot_radius;
      }else{
        return (r->pos-points[inner_check[2]]).norm()-robot_radius;
      }
    }else if((r->angle>M_PI)&&(r->angle<2.*M_PI)){
      if(points[inner_check[1]].y>points[inner_check[2]].y){
        return (r->pos-points[inner_check[2]]).norm()-robot_radius;
      }else{
        return (r->pos-points[inner_check[1]]).norm()-robot_radius;
      }
    } 

  } 
   
}

//simlate the sensor readings of tof sensor on the agent 
double agent_get_reading(Agent* a)
{
  double x1 = a->pos.x;
  double y1 = a->pos.y;
  double k = tan(a->angle);
  Enki::point new_pos;

 //if parallel
  if((a->angle==0)||(a->angle==2.*M_PI)){
    new_pos = Enki::point(x2,f(x2,x1,y1,k));
    return (a->pos-new_pos).norm()-robot_radius;
  } else if(a->angle==M_PI){
    new_pos = Enki::point(0,f(0,x1,y1,k));
    return (a->pos-new_pos).norm()-robot_radius;
  } else if(a->angle==0.5*M_PI){
    new_pos = Enki::point(g(y2,x1,y1,k),y2);
    return (a->pos-new_pos).norm()-robot_radius;
  } else if(a->angle==1.5*M_PI){
    new_pos = Enki::point(g(0,x1,y1,k),0);
    return (a->pos-new_pos).norm()-robot_radius;
  } else{
  //if not parallel
  //find the intersecting points
    Enki::point points[4]={
      Enki::point(g(y2,x1,y1,k),y2),
      Enki::point(g(0,x1,y1,k),0),
      Enki::point(0,f(0,x1,y1,k)),
      Enki::point(x2,f(x2,x1,y1,k))
    };
    std::vector<int>check_inner;

    if(g(y2,x1,y1,k)>=0 && g(y2,x1,y1,k)<=x2){inner_check.push_back(0);}
    if(g(0,x1,y1,k)>=0 && g(0,x1,y1,k)<=x2){inner_check.push_back(1);}
    if(f(0,x1,y1,k)>=0 && f(0,x1,y1,k)<=y2){inner_check.push_back(2);}
    if(f(x2,x1,y1,k)>=0 && f(x2,x1,y1,k)<=y2){inner_check.push_back(3);}

    if((a->angle>0)&&(a->angle<M_PI)){
      if(points[inner_check[1]].y>points[inner_check[2]].y){
        return (a->pos-points[inner_check[1]]).norm()-robot_radius;
      }else{
        return (a->pos-points[inner_check[2]]).norm()-robot_radius;
      }
    }else if((a->angle>M_PI)&&(a->angle<2.*M_PI)){
      if(points[inner_check[1]].y>points[inner_check[2]].y){
        return (a->pos-points[inner_check[2]]).norm()-robot_radius;
      }else{
        return (a->pos-points[inner_check[1]]).norm()-robot_radius;
      }
    } 

  } 
}

void AgentBehaviour(const int &C_num)
{
  Agent* a = new Agent();
  TestWorld world(50, 50);
  world.creatAgent(a);

  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = agent_get_reading(a);
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    a->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    a->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
  }
}

void ReplicaBehaviour(const int &C_num, double modelValue[2])
{
  Agent1* r = new Agent1();
  TestWorld world(50, 50);
  world.creatReplica(r);
  
  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = modelValue[0]*replica_cal_distance(r)+modelValue[1];
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    r->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    r->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
  }
}
