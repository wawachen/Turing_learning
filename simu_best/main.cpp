#include "parameters.h"
#include "classifier.h"
#include "model.h"
#include "test.h"

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
  int  R_num;
  //int generation = 0;
  bool flag[2] = {true, false};
  double new_x,new_y,new_o;
  double new_xr,new_yr,new_or;

  double old_x = 25.0;
  double old_y = 25.0;
  double old_o = 0.5*M_PI;

  double old_xr = 25.0;
  double old_yr = 25.0;
  double old_or = 0.5*M_PI;

  gsl_init();
  //generate_initial_classifiers();
  //generate_initial_models();

  //std::clock_t start;
  //double duration;
  //while (generation < MAX_GENERATION)
  //{

    //start = std::clock();

    //for (M_num=0; M_num < MODEL_POPSIZE; M_num++)
    //{
      double modelValue[2]; // k and b for y = k*x+b
       
      modelValue[0] = 2.12702;
      modelValue[1] = 0.999896;
      //std::cout <<"The model parameter is :"<<"k= " << modelValue[0] <<" b="<< modelValue[1]<<std::endl;
      
      //for(C_num = 0; C_num < CLASSIFIER_POPSIZE; C_num++)
     // {
        for (R_num = 0; R_num < 2; R_num++)
        {
          if (flag[R_num] == true) 
          {
            std::tie(new_x,new_y,new_o) = AgentBehaviour(old_x,old_y,old_o,argv);
            old_x = new_x;
            old_y = new_y;
            old_o = new_o;
            
          }
          else
          {
            std::tie(new_xr,new_yr,new_or) = ReplicaBehaviour(modelValue,old_xr,old_yr,old_or,argv);
            old_xr = new_xr;
            old_yr = new_yr;
            old_or = new_or;
           
          }     
        }
     // }
    //std::cout<<M_num<<std::endl;
    //}
    //5.38356 -5.54655 -5.41125 -1.16921 -1.03672 -3.43513 -0.679598 -1.11472 2.99325 1.50104 5.52292 1.93677 -5.28667 5.88771 2.14715 0.547022 2.69901 4.65387 2.60607 2.97834 0.78169 -0.576072 3.92517 2.15016 -0.493324 -3.79169 -1.78965 -1.56175 0.934739 -5.3999 -3.60894 -6.01132 1.47312 3.38767 -0.639048 3.50232 0.561929 -7.43317 -4.67197 0.190478 -6.23126 4.49873 3.14062 1.77513 4.75511 1.33851 1.73456 3.82333 3.77152 -3.20921 -1.93728 -0.709833 7.18901
    //sort_model_population();
    //sort_classifier_population();

    //output_best_model(argv);
    //output_best_classifier(argv);

    //generate_next_model();
    //generate_next_classifier();

    //duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    //std::cout <<"The generation is :"<<generation << std::endl;

    //generation++;    
  //}
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
