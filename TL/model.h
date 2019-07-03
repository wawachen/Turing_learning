#ifndef MODEL_H
#define MODEL_H

#include <math.h>
#include "parameters.h"

struct Model                                              //define chronmosome structs
{
    double chrom[MODEL_GENESIZE];
    double fitness;
};

extern struct Model model[MODEL_POPSIZE];

void generate_initial_models();
void generate_next_model();
void calculate_model_fitness(const unsigned&, const unsigned&, const unsigned&);
void model_selectoperator();
void model_mutationoperator();
void swap_model_population(int a, int b);
void sort_model_population();
void output_best_model(const int &generation, char *argv[]);

#endif
