#ifndef TEST_H
#define TEST_H

#include "TestEnki.h"
#include "parameters.h"
#include "classifier.h"
#include "model.h"
#include <tuple>

std::tuple<double,double,double> AgentBehaviour(const int &C_num,double old_x,double old_y,double old_o,char *argv[]);
std::tuple<double,double,double> ReplicaBehaviour(const int &C_num,double modelValue[2],double old_x,double old_y,double old_o,char *argv[]);
//calculate the real distance from replica to the wall
double replica_cal_distance(Agent1* r);
//simlate the sensor readings of tof sensor on the agent 
double agent_get_reading(Agent* a);

void Agent_output_information(Agent* a,char *argv[]);
void replica_output_information(Agent1* r,char *argv[]);

double f(double x,double x1,double y1,double k);
double g(double y,double x1,double y1,double k); 

#endif
