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

#include "SerialComm.h"
#include <iostream>
#include <stdio.h>
#include <chrono>

gsl_rng *r;
ofstream outData;
void gsl_init();

double CF_input[CF_input_n];
double CF_output[CF_output_n];
int battery_detection(SerialComm *comm);

int main(int argc, char *argv[])
{
  int C_num, M_num, R_num;
  int generation = 0;
  bool flag[2] = {true, false};
  SerialComm *comm;
  char *portName = "/dev/rfcomm0";
  int err = 0;
  double new_xr,new_yr,new_or;
  int pause_key;

  double old_xr = 25.0;
  double old_yr = 25.0;
  double old_or = 0.5*M_PI;

  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds ms;
  typedef std::chrono::duration<float> fsec;

  gsl_init();
  generate_initial_classifiers();
  generate_initial_models();

  //std::clock_t start;
  //double duration;

  comm = new SerialComm();
  err = comm->connect(portName);
  if(err==-1) {
    std::cerr << "Unable to open serial port " << portName << std::endl;
    return 1;
  }

  while (generation < MAX_GENERATION)
  {
    
    for (M_num=0; M_num < MODEL_POPSIZE; M_num++)
    {
      double modelValue[2]; // k and b for y = k*x+b
      for (unsigned i = 0; i < 2; i++)
      {modelValue[i] = model[M_num].chrom[i];}
      //std::cout <<"The model parameter is :"<<"k= " << modelValue[0] <<" b="<< modelValue[1]<<std::endl;

      for(C_num = 0; C_num < CLASSIFIER_POPSIZE; C_num++)
      {
        //start = std::clock();
        //auto t0 = Time::now();
        for (R_num = 0; R_num < 2; R_num++)
        {
          if (flag[R_num] == true) 
          {
            auto t0 = Time::now();
            AgentBehaviour(C_num,comm);
            calculate_classifier_fitness(C_num, M_num, R_num);
            classifier_initial();
            
            while(battery_detection(comm)<=2900){
              std::cout << "Please change battery to continue:  "<< std::endl;
              std::cin.get();
              err = comm->connect(portName);
              if(err==-1) {
                std::cerr << "Unable to open serial port " << portName << std::endl;
                return 1;
              }
            }
            auto t1 = Time::now();
            fsec fs = t1 - t0;
            ms d = std::chrono::duration_cast<ms>(fs);
            std::cout << "time: "<< fs.count() << "s\n";
          }
          else
          {
            std::tie(new_xr,new_yr,new_or) = ReplicaBehaviour(C_num, modelValue,old_xr,old_yr,old_or);
            old_xr = new_xr;
            old_yr = new_yr;
            old_or = new_or;
            calculate_classifier_fitness(C_num, M_num, R_num);
            calculate_model_fitness(C_num, M_num, R_num);
            classifier_initial();
            
          }     
        }
        //duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
        //auto t1 = Time::now();
        //fsec fs = t1 - t0;
        //ms d = std::chrono::duration_cast<ms>(fs);
        //std::cout << "time: "<< fs.count() << "s\n";
        //std::cout<<"time: "<<duration<<std::endl;
      }
      
    //std::cout<<M_num<<std::endl;
    }
    
    std::cout <<"The generation is :"<<generation<< std::endl;

    sort_model_population();
    sort_classifier_population();

    output_best_model(argv);
    output_best_classifier(argv);

    generate_next_model();
    generate_next_classifier();

    //duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

    generation++;    
  }

  //close communication
  if(comm!=NULL) {
        comm->disconnect();
        comm=NULL;
  }

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

int battery_detection(SerialComm *comm){
  char RxBuffer[45];
  char msg[50];
  int bytes;
  int reconnectFlag = 1;
  int batteryRaw;
  uint8_t bytesToSend;
  char command[20];

  bytesToSend = 2;
  command[0]=-'b';          //ToF request
  command[1]= 0;             //binary command ending

  while(reconnectFlag){
  reconnectFlag = 0;
  comm->flush();
  bytes=comm->writeData(command, bytesToSend, 1000000);

  memset(RxBuffer, 0x0, 45);   
  bytes=comm->readData((char*)RxBuffer,2,1000000);

  if(bytes == 0){
    reconnectFlag = 1;
  } else if(bytes<2){
    sprintf(msg, "Battery: only %d bytes red", bytes);
    std::cerr << msg << std::endl;
    reconnectFlag = 1;
  } else {
    batteryRaw = (uint8_t)RxBuffer[0]+(uint8_t)RxBuffer[1]*256;
  }
  }
  return batteryRaw;
}
