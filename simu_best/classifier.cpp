//*************classifier*************************
#include "classifier.h"

static double average_fitness;
static double CF_hidden[CF_hidden_n];//TODO: set to 0 initially

struct Classifier classifier[CLASSIFIER_POPSIZE];
double superman[53]= {5.38356,-5.54655,-5.41125,-1.16921,-1.03672,-3.43513,-0.679598, -1.11472, 2.99325, 1.50104, 5.52292, 1.93677, -5.28667, 5.88771, 2.14715, 0.547022, 2.69901, 4.65387, 2.60607, 2.97834, 0.78169, -0.576072, 3.92517, 2.15016, -0.493324, -3.79169, -1.78965, -1.56175, 0.934739, -5.3999, -3.60894, -6.01132, 1.47312, 3.38767, -0.639048, 3.50232, 0.561929, -7.43317, -4.67197, 0.190478, -6.23126, 4.49873, 3.14062, 1.77513, 4.75511, 1.33851, 1.73456, 3.82333, 3.77152, -3.20921, -1.93728, -0.709833, 7.18901};

void calculate_classifier_fitness(const unsigned& C_num, const unsigned& M_num, const unsigned& R_num) //C_num [0,49] M_num [0,49] R_num [0,3]
{
    static double sum_CF[CLASSIFIER_POPSIZE];
    if (R_num < 1){ //R_num is pointing at animals
        sum_CF[C_num] += CF_output[0]/(MODEL_POPSIZE*2); //outputting 1 for an animal is a correct judgement
    }
    else{ //R_num is pointing at models
        sum_CF[C_num] += (1 - CF_output[0])/(MODEL_POPSIZE*2); //outputting 0 for a model is a correct judgement
    }

    if((C_num == (CLASSIFIER_POPSIZE - 1)) && (M_num == (MODEL_POPSIZE - 1)) && (R_num == 1)){ //after testing each classifier for each model
        for(unsigned i = 0; i < CLASSIFIER_POPSIZE; i++){
            classifier[i].fitness = sum_CF[i]; //save
            sum_CF[i] = 0; //reset
        }
    }
}

void CF_elmanNetwork(const double CF_input[])
{
    // backup & clean CF_hidden nodes
    double pastCF_hidden[CF_hidden_n];
    for (unsigned i = 0; i < CF_hidden_n; i++)
    {
        pastCF_hidden[i] = CF_hidden[i];
        CF_hidden[i] = 0.0;
    }

    // clean output nodes
    for (unsigned j = 0; j < CF_output_n; j++)
    {
        CF_output[j] = 0.0;
    }

    // calculate activations
    unsigned offset = 0;

    // 1) input -> CF_hidden (include bias, CF_input[2] = 1.0)
    for (unsigned i = 0; i < CF_hidden_n; i++)
    {
        for (unsigned j = 0; j < CF_input_n; j++)
        {
            CF_hidden[i] += superman[i * CF_input_n + j] * CF_input[j]; //weight * input
        }
    }
    offset = CF_input_n * CF_hidden_n;

    // 2) CF_hidden <-> CF_hidden
    for (unsigned i = 0; i < CF_hidden_n; i++)
    {
        for (unsigned j = 0; j < CF_hidden_n; j++)
        {
            CF_hidden[i] += superman[offset + i * CF_hidden_n + j] * pastCF_hidden[j]; //weight * input
        }
    }
    offset += CF_hidden_n * CF_hidden_n;

    // 2b) apply saturation on CF_hidden nodes
    for (unsigned j = 0; j < CF_hidden_n; j++)
    {
        CF_hidden[j] = 1.0/( 1 + exp( -CF_hidden[j] ) ); //sigmoidal activation
    }

    // 3) CF_hidden -> outputs
    for (unsigned i = 0; i < CF_output_n; i++)
    {
        CF_output[i] = superman[offset + i * (CF_hidden_n + 1)] * (1.0);   //add a bias
        for (unsigned j = 0; j < CF_hidden_n; j++)
        {
            CF_output[i] += superman[offset + i * (CF_hidden_n + 1) + (j + 1)] * CF_hidden[j]; //weight * input
        }
    }

    // 3b) apply saturation on output nodes
    for (unsigned j = 0; j < CF_output_n; j++)
    {
        CF_output[j] = 1.0/( 1 + exp( -CF_output[j] ) ); //sigmoidal activation
    }
    if (CF_output[0] >= 0.5)
    {
        CF_output[0] = 1.0;
    }
    else
    {
        CF_output[0] = 0.0;
    }
}

void classifier_initial()
{
    for(unsigned i = 0; i < CF_hidden_n; i++)
    {
        CF_hidden[i] = 0.0;
    }
}

void generate_initial_classifiers()
{
    for(unsigned i = 0; i < CLASSIFIER_POPSIZE; i++)
    {
        for(unsigned j = 0; j < CLASSIFIER_GENESIZE/2; j++)
        {
            classifier[i].chrom[j] = 0.0;
            classifier[i].chrom[j + CLASSIFIER_GENESIZE/2] = 1.0;
        }
    }
}

void generate_next_classifier()
{
    classifier_selectoperator();
    classifier_mutationoperator();
}

void classifier_selectoperator()
{
    unsigned i, j, k, index;
    struct Classifier selectparent[2], offspring;
    for(i = CLASSIFIER_SELECTIONSIZE; i < CLASSIFIER_POPSIZE; i++)
    {
        for (j = 0; j < 2; j++)
        {
            index = gsl_rng_uniform_int(r,CLASSIFIER_SELECTIONSIZE);
            selectparent[j] = classifier[index];
        }

        for (k = 0; k < CLASSIFIER_GENESIZE/2; k++)
        {
            index = gsl_rng_uniform_int(r, 2);
            offspring.chrom[k] = selectparent[index].chrom[k];
            offspring.chrom[k + CLASSIFIER_GENESIZE/2] = (selectparent[0].chrom[k + CLASSIFIER_GENESIZE/2] + selectparent[1].chrom[k + CLASSIFIER_GENESIZE/2])/2;
        }
        classifier[i] = offspring;
    }
}

void classifier_mutationoperator()
{
    unsigned i, j;
    double p_m;
	for (i = CLASSIFIER_SELECTIONSIZE;i < CLASSIFIER_POPSIZE; i++)
	{
            p_m = exp(gsl_ran_gaussian(r, 1/(2 * sqrt(CLASSIFIER_GENESIZE))));
            for (j = 0; j < CLASSIFIER_GENESIZE/2; j++)
            {
               classifier[i].chrom[j + CLASSIFIER_GENESIZE/2] = classifier[i].chrom[j + CLASSIFIER_GENESIZE/2] * p_m * exp(gsl_ran_gaussian(r, 1/(2 * sqrt(2 * sqrt(CLASSIFIER_GENESIZE/2)))));
               if(classifier[i].chrom[j + CLASSIFIER_GENESIZE/2] < LOWERBOUND)
               {
                  classifier[i].chrom[j + CLASSIFIER_GENESIZE/2] = LOWERBOUND;
               }// mutate lambda

               classifier[i].chrom[j] += gsl_ran_gaussian(r, classifier[i].chrom[j + CLASSIFIER_GENESIZE/2]);// mutate mu with the corresponding lambda
            }
	}
}

void sort_classifier_population()
{
    unsigned i, j, k = 0;
    double sum = 0.0;
    struct Classifier bestclassifier;
    for (i = 0; i < CLASSIFIER_POPSIZE; i++)
    {
        bestclassifier = classifier[i];
        for (j = (i+1); j < CLASSIFIER_POPSIZE; j++ )
        {
           if (classifier[j].fitness > bestclassifier.fitness)
	       {
              bestclassifier = classifier[j];
              k = j;
           }
        }
	    if (bestclassifier.fitness > classifier[i].fitness)
        {
           swap_classifier_population(i, k);
        }
	   sum += classifier[i].fitness;
    }
    average_fitness = sum/(CLASSIFIER_POPSIZE);
}

void swap_classifier_population(int a, int b)
{
    struct Classifier temp;
    /* Swap chrom */
    for (unsigned i = 0; i < CLASSIFIER_GENESIZE; i++)
    {
        temp.chrom[i] = classifier[a].chrom[i];
        classifier[a].chrom[i] = classifier[b].chrom[i];
        classifier[b].chrom[i] = temp.chrom[i];
    }
    /* Swap fitness */
    temp.fitness = classifier[a].fitness;
    classifier[a].fitness = classifier[b].fitness;
    classifier[b].fitness = temp.fitness;
}


void output_best_classifier(char *argv[])
{
  stringstream classifierGenes, classifierFitness;
  
  classifierGenes<< "./data/classifier/" << argv[1] << "_genes_" << ".txt";
  outData.open(classifierGenes.str().c_str(), std::ios::out|std::ios::app);  
  if (!outData)
  {
    cout << "Error: Can't open the classifier gene file.\n";
    exit(1);
  }  
  
  for(unsigned i = 0; i < CLASSIFIER_GENESIZE/2; i++)
  {
    //outData << classifier[0].chrom[i] << " " << classifier[0].chrom[i + CLASSIFIER_GENESIZE/2] << " ";
    if(i== (CLASSIFIER_GENESIZE/2-1)){
      outData << classifier[0].chrom[i] << std::endl;
    }else{
      outData << classifier[0].chrom[i] << " ";
    }
  }
  outData.close();

  classifierFitness<< "./data/classifier/" << argv[1] << "_fitness_" << ".txt";
  outData.open(classifierFitness.str().c_str(), std::ios::out|std::ios::app);  
  if (!outData)
  {
    cout << "Error: Can't open the classifier fitness file.\n";
    exit(1);
  }  
  
  outData << average_fitness << " " << classifier[0].fitness<< std::endl;
  outData.close();
}

