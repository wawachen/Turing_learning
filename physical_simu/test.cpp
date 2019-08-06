#include "test.h"
#include <vector>

int maxSteps = 100;
double maxSpeed = 12.8;
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

void random_move_replica(Agent1* r){
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
	double corner_speed = 5.12;
            
			
	ps_values[0] = r->infraredSensor0.getValue();
	ps_values[1] = r->infraredSensor1.getValue();
	ps_values[2] = r->infraredSensor2.getValue();
	ps_values[3] = r->infraredSensor3.getValue();
	ps_values[4] = r->infraredSensor4.getValue();
	ps_values[5] = r->infraredSensor5.getValue();
	ps_values[6] = r->infraredSensor6.getValue();
	ps_values[7] = r->infraredSensor7.getValue();

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
			r->leftSpeed =  corner_speed;
            r->rightSpeed = -corner_speed;
		}
		else {
			r->leftSpeed =  -corner_speed;
            r->rightSpeed = corner_speed;
		}
				
	}
	else if (ps_nearWall[Closest_sensor] == 1) { 
		// e-puck meet obstacle
		switch (Closest_sensor) { 
				case 0: 
					// obstacle is close in front, turn on.
					r->leftSpeed =  -UniformRand(6.4, 12.8)();
        	        r->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 1: 
					//obstacle is near the front right, turn away.
					r->leftSpeed =  UniformRand(0.0, 2.0)();
        	        r->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 2: 
					// obstacle to the right side, slightly turn away.
					r->leftSpeed =  UniformRand(0.0, 2.0)();
        	        r->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 3:
					r->leftSpeed =  UniformRand(6.4, 12.8)();
        	        r->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 4:
				  r->leftSpeed =  UniformRand(6.4, 12.8)();
        	      r->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 5: // obstacle to the left side, slightly turn away.
					r->leftSpeed =  UniformRand(6.4, 12.8)();
        	        r->rightSpeed = UniformRand(0.0, 2.0)();
					break;
				case 6: // obstacle is somewhat near the front left, turn away.
					r->leftSpeed =  UniformRand(6.4, 12.8)();
        	        r->rightSpeed = UniformRand(0.0, 2.0)();
					break;
				case 7: // obstacle is close in front, turn on spot.
					r->leftSpeed =  UniformRand(6.4, 10.4)();
        	        r->rightSpeed = -UniformRand(6.4, 10.4)();
					break;
				default:
					r->leftSpeed =  UniformRand(6.4, 12.8)();
        	        r->rightSpeed = UniformRand(6.4, 12.8)();
					break;
		   }

    }
	else if (ps_nearWall[Closest_sensor] == 0) { 
				// Go forwards
		r->leftSpeed =  UniformRand(0.0, 12.8)();
        r->rightSpeed = UniformRand(0.0, 12.8)();
	}
}

void random_move_agent(Agent* a){
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
	 double corner_speed = 5.12;
            
			
	 ps_values[0] = a->infraredSensor0.getValue();
	 ps_values[1] = a->infraredSensor1.getValue();
	 ps_values[2] = a->infraredSensor2.getValue();
	 ps_values[3] = a->infraredSensor3.getValue();
	 ps_values[4] = a->infraredSensor4.getValue();
	 ps_values[5] = a->infraredSensor5.getValue();
	 ps_values[6] = a->infraredSensor6.getValue();
	 ps_values[7] = a->infraredSensor7.getValue();

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
			a->leftSpeed =  corner_speed;
        	a->rightSpeed = -corner_speed;
		}
		else {
			a->leftSpeed =  -corner_speed;
        	a->rightSpeed = corner_speed;
		}
				
	 }
	 else if (ps_nearWall[Closest_sensor] == 1) { 
				// e-puck meet obstacle
				switch (Closest_sensor) {
				case 0: 
					// obstacle is close in front, turn on.
					a->leftSpeed =  -UniformRand(6.4, 12.8)();
        	        a->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 1: 
					//obstacle is near the front right, turn away.
					a->leftSpeed =  UniformRand(0.0, 2.0)();
        			a->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 2: 
					// obstacle to the right side, slightly turn away.
					a->leftSpeed =  UniformRand(0.0, 2.0)();
        			a->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 3:
					a->leftSpeed =  UniformRand(6.4, 12.8)();
        			a->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 4:
				    a->leftSpeed =  UniformRand(6.4, 12.8)();
        			a->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				case 5: // obstacle to the left side, slightly turn away.
					a->leftSpeed =  UniformRand(6.4, 12.8)();
        			a->rightSpeed = UniformRand(0.0, 2.0)();
					break;
				case 6: // obstacle is somewhat near the front left, turn away.
					a->leftSpeed =  UniformRand(6.4, 12.8)();
        			a->rightSpeed = UniformRand(0.0, 2.0)();
					break;
				case 7: // obstacle is close in front, turn on spot.
					a->leftSpeed =  UniformRand(6.4, 10.4)();
        			a->rightSpeed = -UniformRand(6.4, 10.4)();
					break;
				default:
					a->leftSpeed =  UniformRand(6.4, 12.8)();
        			a->rightSpeed = UniformRand(6.4, 12.8)();
					break;
				}

	}
	else if (ps_nearWall[Closest_sensor] == 0) { 
				// Go forwards
		a->leftSpeed =  UniformRand(0.0, 12.8)();
        a->rightSpeed = UniformRand(0.0, 12.8)();
	}
}

void AgentBehaviour(const int &C_num,TestWorld world,Agent* a)
{
   //,TestWorld world,Agent* a

  //for randomness
  for (int Step_rd = 0; Step_rd < 500; Step_rd++){
    world.run();
    random_move_agent(a);
  }

  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = 2*agent_get_reading(a)+1; //modified
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    a->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    a->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
    //std::cout<<"agent:"<<a->leftSpeed<<" "<<a->rightSpeed<<" "<<CF_output[0]<<std::endl;
  }
  
}

void ReplicaBehaviour(const int &C_num, double modelValue[2],TestWorld1 world,Agent1* r)
{
  //,TestWorld world,Agent1* r

  //for randomness
  for (int Step_rd = 0; Step_rd < 500; Step_rd++){
    world.run();
    random_move_replica(r);
  }
  

  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = modelValue[0]*replica_cal_distance(r)+modelValue[1];
    CF_input[1] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    r->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    r->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
    //std::cout<<"model:"<<r->leftSpeed<<" "<<r->rightSpeed<<" "<<CF_output[0]<<std::endl;
  }
}
