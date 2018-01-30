package org.uma.jmetal.runner.multiobjective;

import java.util.ArrayList;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIBuilder;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.operator.impl.crossover.SBXCrossover;
import org.uma.jmetal.operator.impl.mutation.PolynomialMutation;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ1;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ2;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ3;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ4;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ5;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ6;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ7;
import org.uma.jmetal.runner.AbstractAlgorithmRunner;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.AlgorithmRunner;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.ProblemUtils;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;

import java.util.List;

/**
 * Class to configure and run the NSGA-III algorithm
 */
public class NSGAIIIRunner extends AbstractAlgorithmRunner {
  /**
   * @param args Command line arguments.
   * @throws java.io.IOException
   * @throws SecurityException
   * @throws ClassNotFoundException
   * Usage: three options
   *        - org.uma.jmetal.runner.multiobjective.NSGAIIIRunner
   *        - org.uma.jmetal.runner.multiobjective.NSGAIIIRunner problemName
   *        - org.uma.jmetal.runner.multiobjective.NSGAIIIRunner problemName paretoFrontFile
   */
  public static void main(String[] args) throws JMetalException {
	    Problem<DoubleSolution> problem;
	    Algorithm<List<DoubleSolution>> algorithm;
	    CrossoverOperator<DoubleSolution> crossover;
	    MutationOperator<DoubleSolution> mutation;
	    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection;

	    List<String> problems = new ArrayList<>();
	    problems.add("DTLZ1");
	    problems.add("DTLZ2");
    problems.add("DTLZ3");
    problems.add("DTLZ4");
    problems.add("DTLZ5");
    problems.add("DTLZ6");
    problems.add("DTLZ7");
    for (String problemName:problems) {


    //String problemName = "DTLZ1";//args[0];//"org.uma.jmetal.problem.multiobjective.dtlz.DTLZ1" ;

    //problem = ProblemUtils.loadProblem(problemName);

    for (int i = 4; i <7 ; i=i+2) {
      problem = new DTLZ1(7,i);
      switch (problemName){
        case "DTLZ1":
          problem = new DTLZ1(7,i);
          break;
        case "DTLZ2":
          problem = new DTLZ2(12,i);
          break;
        case "DTLZ3":
          problem = new DTLZ3(12,i);
          break;
        case "DTLZ4":
          problem = new DTLZ4(12,i);
          break;
        case "DTLZ5":
          problem = new DTLZ5(12,i);
          break;
        case "DTLZ6":
          problem = new DTLZ6(12,i);
          break;
        case "DTLZ7":
          problem = new DTLZ7(22,i);
          break;
      }


      double crossoverProbability = 0.9;
      double crossoverDistributionIndex = 30.0;
      crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex);

      double mutationProbability = 1.0 / problem.getNumberOfVariables();
      double mutationDistributionIndex = 20.0;
      mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

      selection = new BinaryTournamentSelection<DoubleSolution>();

      algorithm = new NSGAIIIBuilder<>(problem)
          .setCrossoverOperator(crossover)
          .setMutationOperator(mutation)
          .setSelectionOperator(selection)
          .setMaxIterations(250)
          .build();

      AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm)
          .execute();

      List<DoubleSolution> population = algorithm.getResult();
      long computingTime = algorithmRunner.getComputingTime();

      new SolutionListOutput(population)
          .setSeparator("\t")
          .setVarFileOutputContext(new DefaultFileOutputContext("VAR.tsv"))
          .setFunFileOutputContext(new DefaultFileOutputContext("FUN_"+problemName+"_"+i+"_obj.tsv"))
          .print();

      JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");
      JMetalLogger.logger.info("Objectives values have been written to file FUN.tsv");
      JMetalLogger.logger.info("Variables values have been written to file VAR.tsv");
    }
    }
  }
}
