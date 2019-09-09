#include "test.h"
#include <vector>

int maxSteps = 100;
double maxSpeed = 12.8;
double maxvalue = 3000.0;
double y2 = 50;
double x2 = 50;
extern double robot_radius;

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
  Enki::Point new_pos;

 //if parallel
  if(r->angle==0.0){
    new_pos = Enki::Point(x2,f(x2,x1,y1,k));
    return (r->pos-new_pos).norm()-robot_radius;
  } else if((r->angle==1.0*M_PI)||(r->angle==(-1.0*M_PI))){
    new_pos = Enki::Point(0,f(0,x1,y1,k));
    return (r->pos-new_pos).norm()-robot_radius;
  } else if(r->angle==0.5*M_PI){
    new_pos = Enki::Point(g(y2,x1,y1,k),y2);
    return (r->pos-new_pos).norm()-robot_radius;
  } else if(r->angle==(-0.5*M_PI)){
    new_pos = Enki::Point(g(0,x1,y1,k),0);
    return (r->pos-new_pos).norm()-robot_radius;
  } else{
  //if not parallel
  //find the intersecting points
    Enki::Point points[4]={
      Enki::Point(g(y2,x1,y1,k),y2),
      Enki::Point(g(0,x1,y1,k),0),
      Enki::Point(0,f(0,x1,y1,k)),
      Enki::Point(x2,f(x2,x1,y1,k))
    };
    std::vector<int> inner_check;

    if(g(y2,x1,y1,k)>=0 && g(y2,x1,y1,k)<=x2){inner_check.push_back(0);}
    if(g(0,x1,y1,k)>=0 && g(0,x1,y1,k)<=x2){inner_check.push_back(1);}
    if(f(0,x1,y1,k)>=0 && f(0,x1,y1,k)<=y2){inner_check.push_back(2);}
    if(f(x2,x1,y1,k)>=0 && f(x2,x1,y1,k)<=y2){inner_check.push_back(3);}

    if((r->angle>0.0)&&(r->angle<1.0*M_PI)){
      if(points[inner_check[0]].y>points[inner_check[1]].y){
        return (r->pos-points[inner_check[0]]).norm()-robot_radius;
      }else{
        return (r->pos-points[inner_check[1]]).norm()-robot_radius;
      }
    }else if((r->angle<0.0)&&(r->angle>(-1.0*M_PI))){
      if(points[inner_check[0]].y>points[inner_check[1]].y){
        return (r->pos-points[inner_check[1]]).norm()-robot_radius;
      }else{
        return (r->pos-points[inner_check[0]]).norm()-robot_radius;
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
  Enki::Point new_pos;

 //if parallel
  if(a->angle==0.0){
    new_pos = Enki::Point(x2,f(x2,x1,y1,k));
    return (a->pos-new_pos).norm()-robot_radius;
  } else if((a->angle==1.0*M_PI)||(a->angle==(-1.0*M_PI))){
    new_pos = Enki::Point(0,f(0,x1,y1,k));
    return (a->pos-new_pos).norm()-robot_radius;
  } else if(a->angle==0.5*M_PI){
    new_pos = Enki::Point(g(y2,x1,y1,k),y2);
    return (a->pos-new_pos).norm()-robot_radius;
  } else if(a->angle==(-0.5*M_PI)){
    new_pos = Enki::Point(g(0,x1,y1,k),0);
    return (a->pos-new_pos).norm()-robot_radius;
  } else{
  //if not parallel
  //find the intersecting points
    Enki::Point points[4]={
      Enki::Point(g(y2,x1,y1,k),y2),
      Enki::Point(g(0,x1,y1,k),0),
      Enki::Point(0,f(0,x1,y1,k)),
      Enki::Point(x2,f(x2,x1,y1,k))
    };
    std::vector<int> inner_check;

    if(g(y2,x1,y1,k)>=0 && g(y2,x1,y1,k)<=x2){inner_check.push_back(0);}
    if(g(0,x1,y1,k)>=0 && g(0,x1,y1,k)<=x2){inner_check.push_back(1);}
    if(f(0,x1,y1,k)>=0 && f(0,x1,y1,k)<=y2){inner_check.push_back(2);}
    if(f(x2,x1,y1,k)>=0 && f(x2,x1,y1,k)<=y2){inner_check.push_back(3);}

    if((a->angle>0.0)&&(a->angle<1.0*M_PI)){
      if(points[inner_check[0]].y>points[inner_check[1]].y){
        return (a->pos-points[inner_check[0]]).norm()-robot_radius;
      }else{
        return (a->pos-points[inner_check[1]]).norm()-robot_radius;
      }
    }else if((a->angle<0.0)&&(a->angle>(-1.0*M_PI))){
      if(points[inner_check[0]].y>points[inner_check[1]].y){
        return (a->pos-points[inner_check[1]]).norm()-robot_radius;
      }else{
        return (a->pos-points[inner_check[0]]).norm()-robot_radius;
      }
    } 

  } 
}

void AgentBehaviour(const int &C_num)
{
  double d;
  Agent* a = new Agent();
  TestWorld world(50, 50);
  world.creatAgent(a);

  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    d = 2*agent_get_reading(a)+1;
    CF_input[0] = 2*std::min(agent_get_reading(a)/200.0,1.0)+1; //modified
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    if(d<=11.0){
		  a->leftSpeed =  -8.4;
      a->rightSpeed = 8.4;
		}else{
            a->leftSpeed =  6.4;
        	  a->rightSpeed = 6.4;
    }
   
    //std::cout<<"agent:"<<a->leftSpeed<<" "<<a->rightSpeed<<" "<<CF_output[0]<<std::endl;
  }
  
}

void ReplicaBehaviour(const int &C_num, double modelValue[2])
{
  double d;
  Agent1* r = new Agent1();
  TestWorld world(50, 50);
  world.creatReplica(r);
  
  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    d = modelValue[0]*replica_cal_distance(r)+modelValue[1];
    CF_input[0] = modelValue[0]*std::min(replica_cal_distance(r)/200.0,1.0)+modelValue[1];
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    if(d<=11.0){
		  r->leftSpeed =  -8.4;
      r->rightSpeed = 8.4;
		}else{
            r->leftSpeed =  6.4;
        	  r->rightSpeed = 6.4;
    }
    //std::cout<<"model:"<<r->leftSpeed<<" "<<r->rightSpeed<<" "<<CF_output[0]<<std::endl;
  }
}
