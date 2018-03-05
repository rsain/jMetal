package org.uma.jmetal.algorithm.multiobjective.moead;

import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.distance.impl.EuclideanDistanceBetweenSolutionsInObjectiveSpace;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

/**
 *
 * Implementation of the MOEA/D-AWA algorithm.
 *
 * @author Rubén Saborido (rsain@uma.es)
 * @version 1.0
 *
 * Based on the original C++ code (made available by their authors on ResearchGate)
 *
 * Yutao Qi, Xiaoliang Ma, Fang Liu, Licheng Jiao, Jianyong Sun, and Jianshe Wu.
 * 2014.
 * Moea/d with adaptive weight adjustment.
 * Evol. Comput. 22, 2 (June 2014), 231-264.
 * DOI=http://dx.doi.org/10.1162/EVCO_a_00109
 *
 */

public class MOEADAWA extends AbstractMOEAD<DoubleSolution> {
    //Parameters
    private double rateUpdateWeight; //default value = 0.05
    private int nus; //The maximal number of subproblem adjusted in one adaptive weight adjustment. Default value = rateUpdateWeight x populationSize
    private int wag; //The iteration intervals of utilizing the adaptive weight vector adjustment strategy
    private double rateEvol; //The iteration interval of utilizing the adaptive weight vector adjustment strategy. Default value = 0.8 (80% of computing resources are devoted to MOEA/D while the rest 20% are assigned to the adaptive weight vector adjustment)
    private int archiveSize; //number of best solutions saved. Default value = (1.5*populationSize)

    //Variables
    private List<DoubleSolution> archive;
    private List<DoubleSolution> newPopulation;
    private List<DoubleSolution> saved;
    private double[] utility;
    private DifferentialEvolutionCrossover differentialEvolutionCrossover;

    public MOEADAWA(Problem<DoubleSolution> problem,
                    int populationSize,
                    int resultPopulationSize,
                    int maxEvaluations,
                    CrossoverOperator<DoubleSolution> crossover,
                    MutationOperator<DoubleSolution> mutation,
                    FunctionType functionType,
                    String dataDirectory,
                    double neighborhoodSelectionProbability,
                    int maximumNumberOfReplacedSolutions,
                    int neighborSize,
                    double rateUpdateWeight,
                    int nus,
                    int wag,
                    double rateEvol,
                    int archiveSize) {
        super(problem, populationSize, resultPopulationSize, maxEvaluations, crossover, mutation, functionType,
                dataDirectory, neighborhoodSelectionProbability, maximumNumberOfReplacedSolutions, neighborSize);

        this.differentialEvolutionCrossover = (DifferentialEvolutionCrossover)crossoverOperator ;

        if (rateUpdateWeight == 0) //if not initilized we use the default value (0.05)
            this.rateUpdateWeight = 0.05;
        else
            this.rateUpdateWeight = rateUpdateWeight;

        if (rateEvol == 0) //if not initilized we use the default value (0.8)
            this.rateEvol = 0.8;
        else
            this.rateEvol = rateEvol;

        if (nus == 0) //if not initilized we use the default value (populationSize * rateUpdateWeight)
            this.nus = (int) (populationSize * this.rateUpdateWeight); //the maximal number of subproblem adjusted in one adaptive weight adjustment.
        else
            this.nus = nus;

        if (wag == 0) //if not initilized we use the default value
            this.wag = (int)((1.0 - this.rateEvol) * (5.0*maxEvaluations/populationSize) * this.rateUpdateWeight);
        else
            this.wag = wag;

        if (archiveSize == 0) //if not initilized we use the default value (1.5*populationSize)
            this.archiveSize = ((int) (1.5 * populationSize));
        else
            this.archiveSize = archiveSize;
    }

    @Override
    public void run() {
        int generation, subproblem, subProblemId;
        double evaluationsToUpdateArchivePopulation;
        int[] subproblems;
        NeighborType neighborType;
        DoubleSolution child;
        List<DoubleSolution> parents, children;

        //Step 1: initialization
        initializeUniformWeight();
        WS_transformation(lambda);
        initializeNeighborhood();
        initializePopulation() ;
        evaluations = populationSize ;
        initializeIdealPoint() ;
        improveIdealPoint(1e-7) ;
        initializeUtilityFunctions() ;
        updateNeighbourTable();
        generation = 0 ;
        evaluationsToUpdateArchivePopulation = populationSize; //maxEvaluations * rateEvol - 2000;

        do {
            //Step 3: evolution
            subproblems = tourSelection(10);
            for (subproblem = 0; subproblem < subproblems.length; subproblem++) {
                subProblemId = subproblems[subproblem];

                neighborType = chooseNeighborType() ;
                parents = parentSelection(subProblemId, neighborType) ;

                differentialEvolutionCrossover.setCurrentSolution(population.get(subProblemId));
                children = differentialEvolutionCrossover.execute(parents);

                child = children.get(0) ;
                mutationOperator.execute(child);
                problem.evaluate(child);

                updateIdealPoint(child);
                updateNeighborhood(child, subProblemId, neighborType);

                evaluations++;

                if (evaluations >= evaluationsToUpdateArchivePopulation)
                    newPopulation.add((DoubleSolution) child.copy());
            }

            generation++;

            ///use newly offspring solutions to update external population
            if ( generation == (int) ( evaluationsToUpdateArchivePopulation * 5.0 / population.size() ) )
            {
                for (DoubleSolution s : population)
                    newPopulation.add((DoubleSolution) s.copy());

                archivePopulationUpdate();
            }

            //use newly offspring solutions to update external population
            if (evaluations > evaluationsToUpdateArchivePopulation && generation % 5 == 0) {
                archivePopulationUpdate();
            }

            //Step 2.1 and step 2.2 Update the utility function
            if (generation % 50 == 0) {
                utilityFunction();
            }

            //Step 4: Adaptive Weight Adjustment
            //The next condition is gen >= evol_rate * Gmax and gen mod wag == 0 but that one in the original code
            if (evaluations >=  maxEvaluations * rateEvol && generation % wag == 0)	{
                deleteOvercrowdedSubproblems();       //Step 4.2 Delete the overcrowded subproblems
                addNewSubproblemIntoSparseRegions(); //Step 4.3 Add new subproblems into the sparse regions
                updateNeighbourTable();               //Step 4.4 Update the neighbor subproblems
            }
        } while (evaluations < maxEvaluations);

        deleteOvercrowdedSubproblems();
        addNewSubproblemIntoSparseRegions();
    }

    private void archivePopulationUpdate() {
        int i,j, index;
        double minCrowdDistance;
        double[] sparsityLevel;
        DominanceComparator<DoubleSolution> dominanceComparator = new DominanceComparator<>();

        //1. get non-donimated solutions from the union external_population and new_population
        for (i = 0;  i < newPopulation.size(); i++)	{
            int dominance = 0;
            for (j = 0; j < archive.size(); j++)
            {
                dominance = dominanceComparator.compare(archive.get(j), newPopulation.get(i));
                if (dominance == -1) //archive[j] donimates newPopulation[i]
                    break;
                else if (dominance == 1) //newPopulation[i] donimates archive[j]
                {
                    archive.remove(j);
                    j--;
                }
            }
            if (dominance != -1) 
                archive.add((DoubleSolution) newPopulation.get(i).copy());
        }

        //2. desiverty preserve to external_population
        while (archive.size() > archiveSize)	{
            //calculate crowding degree
            sparsityLevel = new double[archive.size()];
            Arrays.fill(sparsityLevel, 1.0);
            sparsityLevel = calculateSparsityLevel(archive);
            //2.delete the indivivual
            index = 0;
            minCrowdDistance = 1e20;

            for (i = 0; i < archive.size(); i++)		{
                if (sparsityLevel[i] < minCrowdDistance)			{
                    index = i;
                    minCrowdDistance = sparsityLevel[i];
                }
            }
            archive.remove(index);
        }

        newPopulation.clear();
    }

    private double[] calculateSparsityLevel(List<DoubleSolution> nondonimatedSolutions) {
        int i,j,k;
        double[] sparsityLevel;
        double[] crowdingDistance;
        EuclideanDistanceBetweenSolutionsInObjectiveSpace euclideanDistance = new EuclideanDistanceBetweenSolutionsInObjectiveSpace();

        sparsityLevel = new double[nondonimatedSolutions.size()];
        Arrays.fill(sparsityLevel, 1.0);

        //1.calculate the crowding degree
        for (i = 0; i < nondonimatedSolutions.size(); i++)	{
            crowdingDistance = new double[this.problem.getNumberOfObjectives()+1];
            Arrays.fill(crowdingDistance, 1.0e20);

            for (j = 0; j < nondonimatedSolutions.size(); j++)		{
                //calculate distance to nobj-nearest neighbour
                double distance_i_j = euclideanDistance.getDistance(nondonimatedSolutions.get(i), nondonimatedSolutions.get(j));
                if (distance_i_j < crowdingDistance[this.problem.getNumberOfObjectives()])			{
                    for (k=this.problem.getNumberOfObjectives(); k >=1 && distance_i_j < crowdingDistance[k-1]; k--)		{
                        crowdingDistance[k] = crowdingDistance[k-1];
                    }
                    crowdingDistance[k] = distance_i_j;
                }
            }

            for (k = 1; k <= this.problem.getNumberOfObjectives(); k++)
                sparsityLevel[i] = sparsityLevel[i]*crowdingDistance[k];
        }

        return sparsityLevel;
    }

    private void updateNeighbourTable() {
        int i,j,k;
        int[] indexes;
        double[] dist;

        //1. update the neighbour individual
        dist = new double[population.size()];
        Arrays.fill(dist,0.0);

        indexes = new int[population.size()];
        Arrays.fill(indexes, 0);

        this.neighborhood = new int[population.size()][neighborSize];
        for (i = 0; i < population.size(); i++)	{
            // calculate the distances based on weight vectors
            for(j=0; j<population.size(); j++)		{
                dist[j]    = euclideanDistance(lambda[i],lambda[j]);
                indexes[j]  = j;
            }
            // find 'niche' nearest neighboring subproblems
            minFastSort(dist,indexes,population.size(),neighborSize);
            // save the indexes of the nearest 'niche' neighboring weight vectors

            for(k=0; k<neighborSize; k++)
                this.neighborhood[i][k] = indexes[k];
        }
    }

    private void minFastSort(double[] x, int[] idx, int n, int m)
    {
        int i, j, id;
        double temp;

        for(i=0; i<m; i++)
        {
            for(j=i+1; j<n; j++)
                if(x[i]>x[j])
                {
                    temp        = x[i];
                    x[i]        = x[j];
                    x[j]        = temp;
                    id          = idx[i];
                    idx[i]      = idx[j];
                    idx[j]      = id;
                }
        }
    }

    private void addNewSubproblemIntoSparseRegions() {
        //Algorithm 3 of MOEA/D-AWA: Adding new subproblems into the sparse regions
        int i,j,m, dominance, indexMinSparsityLevel;
        double sum;
        Random rnd;
        double[] lambdaSp;
        DoubleSolution subproblem;
        DominanceComparator<DoubleSolution> dc = new DominanceComparator<>();

        //Step 1:  delete the individuals, which is donimated by population in archive
        for (j = 0; j < archive.size(); j++)	{
            dominance = 0;
            for (i = 0;  i < population.size(); i++)
            {
                dominance = dc.compare(population.get(i), archive.get(j));
                if (dominance == -1) //population[i] donimates archive[j]
                    break;
                else if (dominance == 1)//archive[j] donimates population[i]
                {
                    population.set(i, (DoubleSolution) archive.get(j).copy());
                }
            }
            if (dominance == -1)	{
                archive.remove(j);
                j--;
            }
        }

        lambdaSp=new double[problem.getNumberOfObjectives()];
        rnd = new Random();
        Arrays.fill(lambdaSp,0.0);

        for (i = 0; i < this.nus; i++)	{
            //Step 2: calculate the min sparsity level of individual in archive to population
            indexMinSparsityLevel = findIndivualWithMinSparsityLevel();

            //Step 3: Add a new subproblem to the sparse region

            //calculate the best lambda to individual archive
            sum = 0.0;
            for (m = 0; m < problem.getNumberOfObjectives(); m++)
                sum += 1.0 / ( archive.get(indexMinSparsityLevel).getObjective(m) - idealPoint[m] + 1e-5);
            for (m = 0; m < problem.getNumberOfObjectives(); m++)
                lambdaSp[m] = 1.0 / ( archive.get(indexMinSparsityLevel).getObjective(m) - idealPoint[m] + 1e-5) / sum;

            saved.set(population.size()-nus+i, (DoubleSolution) population.get(rnd.nextInt(population.size())).copy());
            subproblem = archive.get(indexMinSparsityLevel);
            utility[population.size()-nus+i] = 1.0;
            for (m = 0; m < problem.getNumberOfObjectives(); m++)
                this.lambda[population.size()-nus+i][m] = lambdaSp[m];

            population.add((DoubleSolution) subproblem.copy());
        }
    }

    private int findIndivualWithMinSparsityLevel() {
        int i,j,k, index;
        double minSparsityLevel;
        //-----  not consider the boundy solution in archive  ----//
        boolean[] flag=new boolean[archive.size()];
        Arrays.fill(flag, true);

        for (i = 0 ; i < archive.size(); i++)	{
            for (j = 0; j < problem.getNumberOfObjectives(); j++) 		{
                if ( Math.abs(archive.get(i).getObjective(j) - idealPoint[j]) < 1e-4)
                    flag[i] = false;
            }
        }
        //2. calculate the sparsity level of each individual in archive if it is added to population
        double[] sparsityLevel=new double[archive.size()];
        Arrays.fill(sparsityLevel, 1.0);

        //2.1 calculate the sparsity level
        for (j = 0; j < archive.size(); j++)	{
            if (flag[j])		{
                double[] crowdingDistance = new double[problem.getNumberOfObjectives()+1];
                Arrays.fill(crowdingDistance, 1.0e20);
                for (i = 0; i < population.size(); i++)			{
                    //calculate distance to nobj-nearest neighbour
                    double distance_i_j = euclideanDistance(getObjectiveValues(archive.get(j)), getObjectiveValues(population.get(i)));
                    if (distance_i_j < crowdingDistance[problem.getNumberOfObjectives()])				{
                        for (k=problem.getNumberOfObjectives(); k >=1 && distance_i_j < crowdingDistance[k-1]; k--)
                        {
                            crowdingDistance[k] = crowdingDistance[k-1];
                        }
                        crowdingDistance[k] = distance_i_j;
                    }
                }
                //adapt multiple as crowd_degree
                for (k = 1; k <= problem.getNumberOfObjectives(); k++)
                    sparsityLevel[j] = sparsityLevel[j]*crowdingDistance[k];
            }
        }

        index = -1;
        minSparsityLevel = -1.0;
        for (i = 0; i < archive.size(); i++)	{
            if (flag[i])		{
                if (sparsityLevel[i] > minSparsityLevel)			{
                    index = i;
                    minSparsityLevel = sparsityLevel[i];
                }
            }
        }

        return index;
    }

    private double euclideanDistance (double[] v1, double[] v2) {
        double diff;
        double distance = 0.0;

        if (v1.length != v2.length)
            throw new JMetalException("Vectors have different length!") ;

        for (int nObj = 0; nObj < v1.length;nObj++){
            diff = v1[nObj] - v2[nObj];
            distance += Math.pow(diff,2.0);
        }

        return Math.sqrt(distance);
    }

    private double[] getObjectiveValues(Solution s) {
        int i;
        double[] result = new double[s.getNumberOfObjectives()];

        for (i=0; i<s.getNumberOfObjectives(); i++)
            result[i] = s.getObjective(i);

        return result;
    }

    private void deleteOvercrowdedSubproblems() {
        //Algorithm 2 of MOEA/D-AWA: Deleting the overcrowded subproblems
        int i,j, index;
        double fitnessValue, distance, f1, tmpDist, minSparsityLevel;
        double[] sparsityLevel;

        //Step 1: Update the current evolutional population
        for (i = 0; i < population.size(); i++)	{
            fitnessValue = fitnessFunction(population.get(i), lambda[i]);
            distance = euclideanDistance(getObjectiveValues(population.get(i)), idealPoint);
            for (j = 0; j < archive.size(); j++)		{
                f1 = fitnessFunction(archive.get(j), lambda[i]);
                tmpDist = euclideanDistance(getObjectiveValues(archive.get(j)), idealPoint);
                if (f1 < fitnessValue)		{
                    if ( (i<this.problem.getNumberOfObjectives() && tmpDist<distance) || i>=this.problem.getNumberOfObjectives()) {
                        population.set(i, (DoubleSolution) archive.get(j).copy());
                        fitnessValue = f1;
                        distance = tmpDist;
                    }
                }
            }
        }

        for (j = 0; j < nus; j++)
        {
            //Step 2: Calculate sparsity level for each individual in the evolutionary population
            sparsityLevel = new double[population.size()];
            Arrays.fill(sparsityLevel, 1.0);
            sparsityLevel = calculateSparsityLevel(population);

            index = 0;
            minSparsityLevel = 1e20;
            for (i = this.problem.getNumberOfObjectives(); i < population.size(); i++)		{
                if (sparsityLevel[i] < minSparsityLevel)			{
                    index = i;
                    minSparsityLevel = sparsityLevel[i];
                }
            }
            //Step 3: delete the subproblem with minmum sparsity level
            population.remove(index);
        }
    }

    private int[] tourSelection(int depth) {
        int i, bestIdd, i2, bestSub, s2;
        int [] result;
        List<Integer> selected = new ArrayList<Integer>();
        List<Integer> candidate = new ArrayList<Integer>();

        for (int k = 0; k < problem.getNumberOfObjectives(); k++) {
            // WARNING! HERE YOU HAVE TO USE THE WEIGHT PROVIDED BY QINGFU Et AL (NOT SORTED!!!!)
            selected.add(k);
        }

        for (int n = problem.getNumberOfObjectives(); n < populationSize; n++) {
            // set of unselected weights
            candidate.add(n);
        }

        while (selected.size() < (int) (populationSize / 5.0)) {
            bestIdd = (int) (randomGenerator.nextDouble() * candidate.size());
            bestSub = candidate.get(bestIdd);

            for (i = 1; i < depth; i++) {
                i2 = (int) (randomGenerator.nextDouble() * candidate.size());
                s2 = candidate.get(i2);
                if (utility[s2] > utility[bestSub]) {
                    bestIdd = i2;
                    bestSub = s2;
                }
            }
            selected.add(bestSub);
            candidate.remove(bestIdd);
        }

        result = new int[selected.size()];
        for (i=0; i<selected.size(); i++)
        {
            result[i] = selected.get(i);
        }
        return result;
    }

    private void utilityFunction() throws JMetalException {
        int n;
        double newValue, oldValue, uti, delta;

        utility = new double[populationSize];

        for (n = 0; n < populationSize; n++) {
            newValue = fitnessFunction(population.get(n), lambda[n]);
            oldValue = fitnessFunction(saved.get(n), lambda[n]);
            delta = (oldValue - newValue)/oldValue;
            if (delta > 0.001) {
                utility[n] = 1.0;
            } else {
                uti = (0.95 + (0.05 * delta / 0.001)) * utility[n];
                utility[n] = uti < 1.0 ? uti : 1.0;
            }
            saved.set(n, (DoubleSolution) population.get(n).copy());
        }
    }

    private void improveIdealPoint(double value) {
        int objective;

        for (objective=0; objective < idealPoint.length; objective++) {
            idealPoint[objective] = idealPoint[objective] - value;
        }
    }

    private void initializePopulation() {
        int i;
        DoubleSolution newSolution;

        saved = new ArrayList<>();
        archive = new ArrayList<>();
        newPopulation = new ArrayList<>();

        population = new ArrayList<>(populationSize);

        for (i = 0; i < populationSize; i++) {
            newSolution = (DoubleSolution)problem.createSolution();

            problem.evaluate(newSolution);
            population.add(newSolution);
            saved.add((DoubleSolution) newSolution.copy());
        }
    }

    private void initializeUtilityFunctions() {
        int i;

        utility = new double[populationSize];
        for (i = 0; i < populationSize; i++) {
            utility[i] = 1;
        }
    }

    //Algorithm 1 in paper: New weight vector initialization
    private void WS_transformation(double[][] lambda) {
        int subproblem, objective;
        double prod, sum;
        double[] WS;

        for (subproblem = 0; subproblem < lambda.length; subproblem ++) {
            sum = prod = 0.0;
            WS = new double[lambda[subproblem].length];

            for (objective = 0; objective < lambda[subproblem].length; objective++)
            {
                prod = prod * lambda[subproblem][objective];
                sum = sum + (1/lambda[subproblem][objective]);
            }

            if (prod != 0)
            {
                for (objective = 0; objective < lambda[subproblem].length; objective++)
                {
                    WS[objective] = (1/(lambda[subproblem][objective]))/sum;
                }
            }
            else
            {
                sum = 0.0;
                for (objective = 0; objective < lambda[subproblem].length; objective++)
                {
                    sum = sum + (1/(lambda[subproblem][objective]+(1e-17)));
                }

                for (objective = 0; objective < lambda[subproblem].length; objective++)
                {
                    WS[objective] = (1/(lambda[subproblem][objective]+1e-17))/sum;
                }
            }

            lambda[subproblem] = WS;
        }
    }

    @Override
    public String getName() {
        return "MOEAD-AWA";
    }

    @Override
    public String getDescription() {
        return "Multi-Objective Evolutionary Algorithm based on Decomposition. Version with adaptive weight adjustment (AWA)";
    }
}
