//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
// 
//  You should have received a copy of the GNU Lesser General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

package org.uma.jmetal.runner.singleobjective;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.InteractiveAlgorithm;
import org.uma.jmetal.algorithm.singleobjective.arp.ARPSingle;
import org.uma.jmetal.algorithm.singleobjective.arp.ARPSingleBuilder;
import org.uma.jmetal.algorithm.singleobjective.differentialevolution.DifferentialEvolutionARPBuilder;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.operator.impl.selection.DifferentialEvolutionSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ1;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ3;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ4;
import org.uma.jmetal.problem.multiobjective.zdt.ZDT1;
import org.uma.jmetal.problem.singleobjective.ACH;
import org.uma.jmetal.runner.AbstractAlgorithmRunner;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.AlgorithmRunner;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.MultithreadedSolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.referencePoint.ReferencePoint;
import org.uma.jmetal.util.referencePoint.impl.IdealPoint;
import org.uma.jmetal.util.referencePoint.impl.NadirPoint;

/**
 * Class to configure and run the R-NSGA-II algorithm
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 * @author Cristobal Barba <cbarba@lcc.uma.es>
 */
public class ARPRDifferentialEvolutionaryRunner extends AbstractAlgorithmRunner {
  private static final int DEFAULT_NUMBER_OF_CORES = 1 ;

  /**
   * @param args Command line arguments.
   * @throws JMetalException
   * @throws FileNotFoundException
   * Invoking command:
    java org.uma.jmetal.runner.multiobjective.RNSGAIIRunner_Ant problemName [referenceFront]
   */
  public static void main(String[] args) throws JMetalException, FileNotFoundException {
    DoubleProblem problem;
    ACH problemRun;
    Algorithm<DoubleSolution> algorithm;
    InteractiveAlgorithm<DoubleSolution,DoubleSolution> algorithmRun;

    DifferentialEvolutionSelection selection;
    DifferentialEvolutionCrossover crossover;
    SolutionListEvaluator<DoubleSolution> evaluator ;




    problem =new DTLZ3(7,4);//ZDT1();//  ProblemUtils.<DoubleSolution> loadProblem(problemName);//Tanaka();//

    int numberOfCores ;
    if (args.length == 1) {
      numberOfCores = Integer.valueOf(args[0]) ;
    } else {
      numberOfCores = DEFAULT_NUMBER_OF_CORES ;
    }

    if (numberOfCores == 1) {
      evaluator = new SequentialSolutionListEvaluator<DoubleSolution>() ;
    } else {
      evaluator = new MultithreadedSolutionListEvaluator<DoubleSolution>(numberOfCores, problem) ;
    }

    crossover = new DifferentialEvolutionCrossover(0.5, 0.5, "rand/1/bin") ;
    selection = new DifferentialEvolutionSelection();


    IdealPoint idealPoint = new IdealPoint(problem.getNumberOfObjectives());
    idealPoint.update(problem.createSolution());
    NadirPoint nadirPoint = new NadirPoint(problem.getNumberOfObjectives());
    nadirPoint.update(problem.createSolution());
    double considerationProbability = 0.9;
    List<Double> rankingCoeficient = new ArrayList<>();
    for (int i = 0; i < problem.getNumberOfObjectives() ; i++) {
      rankingCoeficient.add(1.0/problem.getNumberOfObjectives());
    }
    double tolerance = 0.5;

    List<Double> referencePoint = new ArrayList<>() ;
    referencePoint.add(0.0);
    referencePoint.add(0.0);
    referencePoint.add(0.0);
    referencePoint.add(0.0);
    double rho =0.05;
    problemRun = new ACH(referencePoint,(AbstractDoubleProblem)problem,rho);
    algorithmRun = new DifferentialEvolutionARPBuilder(problemRun)
        .setCrossover(crossover)
        .setSelection(selection)
        .setSolutionListEvaluator(evaluator)
        .setMaxEvaluations(250000)
        .setPopulationSize(100)
        .build() ;
   /* algorithmRun = new RNSGAIIBuilder<DoubleSolution>(problem, crossover, mutation, referencePoint,epsilon)
        .setSelectionOperator(selection)
        .setMaxEvaluations(25000)
        .setPopulationSize(100)
        .build() ;*/

    algorithm = new ARPSingleBuilder<DoubleSolution>(problemRun, algorithmRun)
        .setConsiderationProbability(0.7)
        .setMaxEvaluations(20)
        .setTolerance(0.001)
        .setNumberOfObjectives(problem.getNumberOfObjectives())
        .build();

    AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm)
        .execute() ;

    DoubleSolution solution = algorithm.getResult() ;
    long computingTime = algorithmRunner.getComputingTime() ;

    List<DoubleSolution> population = new ArrayList<>(1) ;
    population.add(solution) ;
    new SolutionListOutput(population)
        .setSeparator("\t")
        .setVarFileOutputContext(new DefaultFileOutputContext("VAR.tsv"))
        .setFunFileOutputContext(new DefaultFileOutputContext("FUN.tsv"))
        .print();

    JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");
    JMetalLogger.logger.info("Objectives values have been written to file FUN.tsv");
    JMetalLogger.logger.info("Variables values have been written to file VAR.tsv");
    System.out.println("Reference Points-----"+((ARPSingle)algorithm).getReferencePoints().size());
    writeLargerTextFile("ReferencePointsDE_DTLZ3_4.txt",((ARPSingle)algorithm).getReferencePoints());
    writeLargerDoubleFile("DistancesDE_DTLZ3_4.txt",((ARPSingle)algorithm).getDistances());
    evaluator.shutdown();
  }
  private static List<Double> getReferencePoint(ReferencePoint referencePoint){
    List<Double> result = new ArrayList<>();
    for (int i = 0; i < referencePoint.getNumberOfObjectives(); i++) {
      result.add(referencePoint.getObjective(i));
    }
    return result;
  }
  private static void writeLargerTextFile(String aFileName, List<ReferencePoint> list)  {
    Path path = Paths.get(aFileName);

    try (BufferedWriter writer = Files.newBufferedWriter(path)){
      int i =0;
      while(i<list.size()){
        String line="";
        for (int j = 0; j <list.get(i).getNumberOfObjectives() ; j++) {
          line += list.get(i).getObjective(j) + " ";
        }
        line = line.substring(0,line.lastIndexOf(" "));
        i+=2;
        writer.write(line);
        writer.newLine();
      }
      writer.close();
    }catch (Exception e){

    }
  }
  private static void writeLargerDoubleFile(String aFileName, List<Double> list)  {
    Path path = Paths.get(aFileName);
    try (BufferedWriter writer = Files.newBufferedWriter(path)){
      int i =0;
      for (Double value:list) {

        writer.write(value+"");
        writer.newLine();
      }
      writer.close();
    }catch (Exception e){

    }
  }
}
