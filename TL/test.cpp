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
    CF_input[8] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    a->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    a->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
  }
}

void ReplicaBehaviour(const int &C_num)
{
  Agent1* r = new Agent1();
  TestWorld world(50, 50);
  world.creatReplica(r);
  
  for (int Step = 0; Step < maxSteps; Step++)
  {
    world.run();
    CF_input[8] = 1.0;
    CF_elmanNetwork(CF_input, C_num);
    r->leftSpeed = 2 * maxSpeed * CF_output[1] - maxSpeed;
    r->rightSpeed = 2 * maxSpeed * CF_output[2] - maxSpeed;
  }
}
