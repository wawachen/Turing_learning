//*************model*************************
#include "model.h"

static double average_fitness;
struct Model model[MODEL_POPSIZE];

void calculate_model_fitness(const unsigned& C_num, const unsigned& M_num, const unsigned& R_num) //C_num [0,99] M_num [0,99] R_num [0,3]
{                                                                                                 //consider NumberOfModels is an increment variable
    static double sum_MD[MODEL_POPSIZE];
    sum_MD[M_num] += CF_output[0]/CLASSIFIER_POPSIZE; //call this function only when R_num >= 2 (R_num is pointing at the model)

    if((C_num == (CLASSIFIER_POPSIZE - 1)) && (M_num == (MODEL_POPSIZE - 1)) && (R_num == 1)){ //after testing each model for each classifier
        for(unsigned i = 0; i < MODEL_POPSIZE; i++){
            model[i].fitness = sum_MD[i]; //save
            sum_MD[i] = 0; //reset
        }
    }
}

void generate_initial_models() //build a 100x4 (MODEL_POPSIZE x MODEL_GENESIZE), and set parameters for each model to mu={0.0, 0,0} & lambda={1.0, 1.0}
{
    for(unsigned i = 0; i < MODEL_POPSIZE; i++) //MODEL_POPSIZE = 50
    {
        for(unsigned j = 0; j < MODEL_GENESIZE/2; j++) //MODEL_GENESIZE = 4
        {
            model[i].chrom[j] = 0.0;
            model[i].chrom[j + MODEL_GENESIZE/2] = 1.0;
        }
    }
}

void generate_next_model()
{
    model_selectoperator();
    model_mutationoperator();
}

void model_selectoperator() //rank based selection
{
    unsigned i, j, k, index;
    struct Model selectparent[2], offspring;
    for(i = MODEL_SELECTIONSIZE; i < MODEL_POPSIZE; i++) //MODEL_SELECTIONSIZE = 25
    {
        for (j = 0; j < 2; j++)
        {
            index = gsl_rng_uniform_int(r, MODEL_SELECTIONSIZE);
            selectparent[j] = model[index]; //randomly select 2 of the top 25 models as parents
        }

        for (k = 0; k < MODEL_GENESIZE/2; k++)
        {
            index = gsl_rng_uniform_int(r, 2);
            offspring.chrom[k] = selectparent[index].chrom[k]; //set mu of the offspring
            offspring.chrom[k + MODEL_GENESIZE/2] = (selectparent[0].chrom[k + MODEL_GENESIZE/2] + selectparent[1].chrom[k + MODEL_GENESIZE/2])/2; //set lambda of the offspring
        }
        model[i] = offspring; //replace the last 25 models with the offsprings generated above
    }
}

void model_mutationoperator()
{
    unsigned i, j;
    double p_m;
    for (i = MODEL_SELECTIONSIZE; i < MODEL_POPSIZE; i++)
    {
		p_m = exp(gsl_ran_gaussian(r, 1/(2*sqrt(MODEL_GENESIZE))));
		for (j = 0; j < MODEL_GENESIZE/4; j ++)
		{
		   double sigma, x, sigma_, y;
		   do
		   {
			   sigma = model[i].chrom[j + MODEL_GENESIZE/2] * p_m * exp(gsl_ran_gaussian(r, 1/(2 * sqrt(2 * sqrt(MODEL_GENESIZE/2)))));
			   if(sigma < LOWERBOUND)
			   {
				  sigma = LOWERBOUND;
			   }
			   x = model[i].chrom[j] + gsl_ran_gaussian(r, sigma);
		   }while(fabs(x)>M_PI);
		   do
		   {
			   sigma_ = model[i].chrom[j + 8 + MODEL_GENESIZE/2] * p_m * exp(gsl_ran_gaussian(r, 1/(2 * sqrt(2 * sqrt(MODEL_GENESIZE/2)))));
			   if(sigma_ < LOWERBOUND)
			   {
				  sigma_ = LOWERBOUND;
			   }
			   y = model[i].chrom[j+8] + gsl_ran_gaussian(r, sigma_);
		   }while((y<=0.0) || (fabs(y)>3.7));
		   model[i].chrom[j + MODEL_GENESIZE/2] = sigma;
		   model[i].chrom[j] = x;
		   model[i].chrom[j + 8 + MODEL_GENESIZE/2] = sigma_;
		   model[i].chrom[j+8] = y;
		}
    }
}

void sort_model_population() //fitness biased survivor selection: arrange from high to low
{
    unsigned i, j, k = 0;
    double sum = 0.0; //sum of overall fitness of 100 models
    struct Model bestmodel; //the intermediate variable used in the arrangement, not the model with highest fitness
    for (i = 0; i < MODEL_POPSIZE; i++)
    {
        bestmodel = model[i];
        for (j = (i+1); j < MODEL_POPSIZE; j++)
        {
            if (model[j].fitness > bestmodel.fitness)
            {
                bestmodel = model[j];
                k = j;
            }
        }
        if (bestmodel.fitness > model[i].fitness)
        {
		swap_model_population(i, k);
        }
        sum += model[i].fitness;
    } //the model with highest fitness is model[0], not the "bestmodel"
    average_fitness = sum/(MODEL_POPSIZE); //calcualte the average fitness
}

void swap_model_population(int a, int b)
{
    struct Model temp;
    /* Swap chrom */
    for (unsigned i = 0; i < MODEL_GENESIZE; i++)
    {
        temp.chrom[i] = model[a].chrom[i];
        model[a].chrom[i] = model[b].chrom[i];
        model[b].chrom[i] = temp.chrom[i];
    }
    /* Swap fitness */
    temp.fitness = model[a].fitness;
    model[a].fitness = model[b].fitness;
    model[b].fitness = temp.fitness;
}

void output_best_model(const int &generation, char *argv[]) //model[0]
{
  stringstream modelGenes, modelFitness;

  modelGenes<< "./data/model/" << argv[1] << "_genes_" << generation << ".txt";
  outData.open(modelGenes.str().c_str(), std::ios::trunc);  
  if (!outData)
  {
    cout << "Error: Can't open the model gene file.\n";
    exit(1);
  }  
  for(unsigned i = 0; i < MODEL_GENESIZE/2; i++)
  {
    //outData << model[0].chrom[i] << " " << model[0].chrom[i + MODEL_GENESIZE/2] << " ";
    outData << model[0].chrom[i] << " ";
  }
  outData.close();

  modelFitness<< "./data/model/" << argv[1] << "_fitness_" << generation << ".txt";
  outData.open(modelFitness.str().c_str(), std::ios::trunc);  
  if (!outData)
  {
    cout << "Error: Can't open the model fitness file.\n";
    exit(1);
  }  
  outData << average_fitness << " " << model[0].fitness;
  outData.close();
}
