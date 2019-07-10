#ifndef CLASSIFIER_H
#define CLASSIFIER_H

//**************classifier*****************
#include <math.h>
#include "parameters.h"

struct Classifier 
{
    double chrom[CLASSIFIER_GENESIZE];
    double fitness;
};  

void generate_initial_classifiers();
//void generate_initial_classifiers(const int &generation);
void generate_next_classifier();
void calculate_classifier_fitness(const unsigned&, const unsigned&, const unsigned&);
void classifier_selectoperator();
void classifier_mutationoperator();
void swap_classifier_population(int a, int b);
void sort_classifier_population();
void CF_elmanNetwork(const double CF_input[], const unsigned &C_num); //for three dimensions: CF_input[][POPSIZE][CF_input_n]
void calculate_classifier_fitness();     
void classifier_parameters_set();     
void classifier_initial();              
void output_best_classifier(const int &generation, char *argv[]);
//void output_classifier_genes(const int &generation, char *argv[]);

#endif
