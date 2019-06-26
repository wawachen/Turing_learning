#ifndef TEST_H
#define TEST_H

#include "TestEnki.h"
#include "parameters.h"
#include "classifier.h"
#include "model.h"

void AgentBehaviour(const int &C_num);
void ReplicaBehaviour(const int &C_num);
//calculate the real distance from replica to the wall
unsigned int replica_cal_distance(Agent1* r);
//simlate the sensor readings of tof sensor on the agent 
unsigned int agent_get_reading(Agent* a);

int f(int x);
int g(int y); 

#endif
