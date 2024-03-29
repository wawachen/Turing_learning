#include "test.h"
#include <vector>

int maxSteps_phy = 10;
int maxSteps_sim = 10;

double maxSpeed_phy = 1100.0;
double maxSpeed_sim = 12.8;
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
unsigned int agent_get_reading(SerialComm *comm)
{
  char RxBuffer[45];
  char command[20];
  uint8_t bytesToSend;
  int bytes;
  char msg[50];
  unsigned int distance;
  int reconnectFlag = 1;

  bytesToSend = 2;
  command[0]=-0x0D;       //ToF request
  command[1]=0;             //binary command ending
   
  while(reconnectFlag){
    reconnectFlag = 0;
    comm->flush();
    bytes=comm->writeData(command, bytesToSend, 1000000);
 
    memset(RxBuffer, 0x0, 45);
    bytes=comm->readData((char*)RxBuffer,2,1000000);
            
    if(bytes == 0) {
        std::cerr << "Nothing found"<< std::endl;
        reconnectFlag = 1;
        
    } else if(bytes<2) {
        sprintf(msg, "ToF: only %d bytes red", bytes);
        std::cerr << msg << std::endl;
        reconnectFlag = 1;
    } else {
        distance = (uint16_t)(((uint8_t)RxBuffer[1]<<8)|((uint8_t)RxBuffer[0]))/10;
        distance = (distance>200)?200:distance;
    }
  }
  return distance;
   
}

void AgentBehaviour(const int &C_num,SerialComm *comm)
{
  char command[20];
  
  double leftSpeed,rightSpeed;
  char high_left,low_left,high_right,low_right;
  int speed_left,speed_right;

  for (int Step = 0; Step < maxSteps_phy; Step++)
  { 
    CF_input[0] = std::min((double)agent_get_reading(comm)/200.0,1.0);  
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    leftSpeed = 2 * maxSpeed_phy * CF_output[1] - maxSpeed_phy;
    rightSpeed = 2 * maxSpeed_phy * CF_output[2] - maxSpeed_phy;
    //std::cout<<"agent:"<<a->leftSpeed<<" "<<a->rightSpeed<<" "<<CF_output[0]<<std::endl;
    std::cout<<"The sensor reading: "<< (double)agent_get_reading(comm)<<std::endl;
    	// send moving comand
    speed_left = (int)leftSpeed;
    high_left = (speed_left>>8) & 0xFF;
    low_left = speed_left & 0xFF;
    speed_right = (int)rightSpeed;
    high_right = (speed_right>>8) & 0xFF;
    low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    comm->writeData(command, 6, 500000);
    //std::cout<<"Start moving"<<std::endl;
    //std::cout<<"agent:"<<leftSpeed<<" "<<rightSpeed<<std::endl;
    usleep(1000000);
  }
  //send stop comand 
  speed_left = 0;
  high_left = (speed_left>>8) & 0xFF;
  low_left = speed_left & 0xFF;
  speed_right = 0;
  high_right = (speed_right>>8) & 0xFF;
  low_right = speed_right & 0xFF;

  memset(command, 0x0, 20);
  sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
  comm->writeData(command, 6, 20000);
  //std::cout<<"Stop moving"<<std::endl;
  usleep(100000);
  
}


std::tuple<double, double,double> ReplicaBehaviour(const int &C_num, double modelValue[2],double old_x,double old_y,double old_o)
{
  Agent1* r = new Agent1();
  TestWorld world(50, 50);
  world.creatReplica(r,old_x,old_y,old_o);
  
  for (int Step = 0; Step < maxSteps_sim; Step++)
  {
    world.run();
    CF_input[0] = modelValue[0]*std::min(replica_cal_distance(r)/200.0,1.0)+modelValue[1];
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    r->leftSpeed = 2 * maxSpeed_sim * CF_output[1] - maxSpeed_sim;
    r->rightSpeed = 2 * maxSpeed_sim * CF_output[2] - maxSpeed_sim;
    //std::cout<<"model:"<<r->leftSpeed<<" "<<r->rightSpeed<<std::endl;
  }
  return std::make_tuple(r->pos.x,r->pos.y,r->angle);
}