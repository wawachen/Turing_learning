#include "parameters.h"
#include "classifier.h"
#include "model.h"
#include "test.h"
#include "TestEnki.h"

#include <math.h>
#include <cstdlib>
#include <sys/time.h>
#include <sys/stat.h>
#include <ctime>
#include <cstdio>

gsl_rng *r;
ofstream outData;
void gsl_init();

double CF_input[CF_input_n];
double CF_output[CF_output_n];
  

int main(int argc, char *argv[])
{
  int C_num, M_num, R_num;
  int generation = 0;
  bool flag[2] = {true, false};

 Agent* a = new Agent();
 TestWorld world(50, 50);
 world.creatAgent(a);

 Agent1* rp = new Agent1();
 TestWorld1 world1(50, 50);
 world1.creatReplica(rp);

  gsl_init();
  generate_initial_classifiers();
  generate_initial_models();

  //std::clock_t start;
  //double duration;
  

  while (generation < MAX_GENERATION)
  {

    //start = std::clock();
    for (M_num=0; M_num < MODEL_POPSIZE; M_num++)
    {
      double modelValue[2]; // k and b for y = k*x+b
      for (unsigned i = 0; i < 2; i++)
      {modelValue[i] = model[M_num].chrom[i];}
      //std::cout <<"The model parameter is :"<<"k= " << modelValue[0] <<" b="<< modelValue[1]<<std::endl;

      for(C_num = 0; C_num < CLASSIFIER_POPSIZE; C_num++)
      {
        for (R_num = 0; R_num < 2; R_num++)
        {
          
          if (flag[R_num] == true) 
          {
            AgentBehaviour(C_num,world,a);//,world,a
            calculate_classifier_fitness(C_num, M_num, R_num);
            classifier_initial();
          }
          else
          {
            ReplicaBehaviour(C_num, modelValue,world1,rp);//,world1,rp
            calculate_classifier_fitness(C_num, M_num, R_num);
            calculate_model_fitness(C_num, M_num, R_num);
            classifier_initial();
          }     
        }
      }
    //std::cout<<M_num<<std::endl;
    }
    
    sort_model_population();
    sort_classifier_population();

    output_best_model(argv);
    output_best_classifier(argv);

    generate_next_model();
    generate_next_classifier();

    //duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    std::cout <<"The generation is :"<<generation << std::endl;

    generation++;    
  }
  
  a->~Agent();
  rp->~Agent1();
  world.~TestWorld();
  world1.~TestWorld1();

  

  gsl_rng_free(r);
  return 0;    
}

void gsl_init()
{
  unsigned long int Seed;
  struct timeval tv;
  struct timezone tz;

  gettimeofday(&tv,&tz);
  Seed = tv.tv_sec * 1000000 + tv.tv_usec;
  r = gsl_rng_alloc(gsl_rng_default);
  gsl_rng_set(r,Seed);
}
