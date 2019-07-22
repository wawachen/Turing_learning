#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
using namespace std;
//********************COEA constants*************
#define CLASSIFIER_GENESIZE          106         //CF_input_n * CF_hidden_n + CF_hidden_n * CF_hidden_n + (CF_hidden_n + 1) * CF_output_n
#define CLASSIFIER_SELECTIONSIZE     50            //(mu+lmbuda):50+50
#define MODEL_GENESIZE               4             //the same structure as the animal
#define MODEL_SELECTIONSIZE          50            //(mu+lmbuda):50+50
#define MAX_GENERATION               200        //maximum generations
#define CLASSIFIER_POPSIZE           100            //number of classifier population size
#define MODEL_POPSIZE                100            //number of model population size
#define LOWERBOUND                   0.01          //lower bound for the mutations
//********************classifier constants*******
const unsigned CF_input_n = 2;                     //including bias;
const unsigned CF_hidden_n = 5;
const unsigned CF_output_n = 3;

//****************global variables***************
extern gsl_rng *r;
extern ofstream outData;
extern double CF_input[CF_input_n];
extern double CF_output[CF_output_n];

#endif
