#include "test.h"

int maxSteps = 100;
double maxSpeed = 12.8;
double maxvalue = 3000.0;

void AgentBehaviour(const int &C_num)
{
  Agent* a = new Agent();
  TestWorld world(50, 50);
  world.creatAgent(a);

  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = min(1.0, a->infraredSensor0.getValue() / maxvalue);
    CF_input[1] = min(1.0, a->infraredSensor1.getValue() / maxvalue);
    CF_input[2] = min(1.0, a->infraredSensor2.getValue() / maxvalue);
    CF_input[3] = min(1.0, a->infraredSensor3.getValue() / maxvalue);
    CF_input[4] = min(1.0, a->infraredSensor4.getValue() / maxvalue);
    CF_input[5] = min(1.0, a->infraredSensor5.getValue() / maxvalue);
    CF_input[6] = min(1.0, a->infraredSensor6.getValue() / maxvalue);
    CF_input[7] = min(1.0, a->infraredSensor7.getValue() / maxvalue);
    CF_input[8] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    a->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    a->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
  }
}

void ReplicaBehaviour(const int &C_num, double modelValue[16])
{
  Replica* r = new Replica(modelValue);
  TestWorld world(50, 50);
  world.creatReplica(r);
  
  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[0] = min(1.0, r->userSensors[0]->getValue() / maxvalue);
    CF_input[1] = min(1.0, r->userSensors[1]->getValue() / maxvalue);
    CF_input[2] = min(1.0, r->userSensors[2]->getValue() / maxvalue);
    CF_input[3] = min(1.0, r->userSensors[3]->getValue() / maxvalue);
    CF_input[4] = min(1.0, r->userSensors[4]->getValue() / maxvalue);
    CF_input[5] = min(1.0, r->userSensors[5]->getValue() / maxvalue);
    CF_input[6] = min(1.0, r->userSensors[6]->getValue() / maxvalue);
    CF_input[7] = min(1.0, r->userSensors[7]->getValue() / maxvalue);
    CF_input[8] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    r->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    r->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
  }
}
