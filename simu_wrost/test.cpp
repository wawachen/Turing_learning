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

void Agent_output_information(Agent* a,char *argv[]){
  stringstream position_track,speed_track;
  position_track<< "./data/position/" << argv[1] << "Agent_positionTrack" << ".txt";
  outData.open(position_track.str().c_str(), std::ios::out|std::ios::app);  
  if (!outData)
  {
    cout << "Error: Can't open the file.\n";
    exit(1);
  }  
  
  outData << a->pos.x <<" "<< a->pos.y << std::endl;
  outData.close();

  speed_track<< "./data/speed/" << argv[1] << "Agent_speedTrack" << ".txt";
  outData.open(speed_track.str().c_str(), std::ios::out|std::ios::app);  
  if (!outData)
  {
    cout << "Error: Can't open the file.\n";
    exit(1);
  }  
  
  outData <<a->leftSpeed<<" "<<a->rightSpeed<< std::endl;
  outData.close();
}

void replica_output_information(Agent1* r,char *argv[]){
  stringstream position_track,speed_track;
  position_track<< "./data/position/" << argv[1] << "Replica_positionTrack" << ".txt";
  outData.open(position_track.str().c_str(), std::ios::out|std::ios::app);  
  if (!outData)
  {
    cout << "Error: Can't open the file.\n";
    exit(1);
  }  
  
  outData << r->pos.x <<" "<< r->pos.y << std::endl;
  outData.close();

  speed_track<< "./data/speed/" << argv[1] << "Replica_speedTrack" << ".txt";
  outData.open(speed_track.str().c_str(), std::ios::out|std::ios::app);  
  if (!outData)
  {
    cout << "Error: Can't open the file.\n";
    exit(1);
  }  
  
  outData <<r->leftSpeed<<" "<<r->rightSpeed<< std::endl;
  outData.close();
}

std::tuple<double, double,double> AgentBehaviour(double old_x,double old_y,double old_o,char *argv[])
{
  Agent* a = new Agent();
  TestWorld world(50, 50);
  world.creatAgent(a,old_x,old_y,old_o);

  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = 2*std::min(agent_get_reading(a)/200.0,1.0)+1; //modified
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input);
    a->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    a->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
    Agent_output_information(a,argv);
    //std::cout<<"agent:"<<a->leftSpeed<<" "<<a->rightSpeed<<" "<<CF_output[0]<<std::endl;
  }
  
  return std::make_tuple(a->pos.x,a->pos.y,a->angle);
}

std::tuple<double, double,double> ReplicaBehaviour(double modelValue[2],double old_x,double old_y,double old_o,char *argv[])
{
  Agent1* r = new Agent1();
  TestWorld world(50, 50);
  world.creatReplica(r,old_x,old_y,old_o);
  
  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = modelValue[0]*std::min(replica_cal_distance(r)/200.0,1.0)+modelValue[1];
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input);
    r->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    r->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
    //std::cout<<"model:"<<r->leftSpeed<<" "<<r->rightSpeed<<" "<<CF_output[0]<<std::endl;
    replica_output_information(r,argv);
  }
  return std::make_tuple(r->pos.x,r->pos.y,r->angle);
}
