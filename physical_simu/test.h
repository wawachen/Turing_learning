#ifndef TEST_H
#define TEST_H

#include "TestEnki.h"
#include "parameters.h"
#include "classifier.h"
#include "model.h"

void AgentBehaviour(const int &C_num,TestWorld world,Agent* a);
void ReplicaBehaviour(const int &C_num,double modelValue[2],TestWorld world,Agent1* r);
//calculate the real distance from replica to the wall
double replica_cal_distance(Agent1* r);
//simlate the sensor readings of tof sensor on the agent 
double agent_get_reading(Agent* a);
void random_move_agent(Agent* a);
void random_move_replica(Agent1* r);

double f(double x,double x1,double y1,double k);
double g(double y,double x1,double y1,double k); 

#endif
