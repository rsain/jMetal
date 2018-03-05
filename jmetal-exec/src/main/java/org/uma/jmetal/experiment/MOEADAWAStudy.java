package org.uma.jmetal.experiment;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.moead.AbstractMOEAD;
import org.uma.jmetal.algorithm.multiobjective.moead.MOEADAWA;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.operator.impl.mutation.PolynomialMutation;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ7;
import org.uma.jmetal.qualityindicator.impl.*;
import org.uma.jmetal.qualityindicator.impl.hypervolume.PISAHypervolume;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.experiment.Experiment;
import org.uma.jmetal.util.experiment.ExperimentBuilder;
import org.uma.jmetal.util.experiment.component.*;
import org.uma.jmetal.util.experiment.util.ExperimentAlgorithm;
import org.uma.jmetal.util.experiment.util.ExperimentProblem;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Example of experimental study based on solving the ZDT problems with four versions of NSGA-II,
 * each of them applying a different crossover probability (from 0.7 to 1.0).
 *
 * This experiment assumes that the reference Pareto front are known, so the names of files
 * containing them and the directory where they are located must be specified.
 *
 * Six quality indicators are used for performance assessment.
 *
 * The steps to carry out the experiment are: 1. Configure the experiment 2. Execute the algorithms
 * 3. Compute the quality indicators 4. Generate Latex tables reporting means and medians 5.
 * Generate Latex tables with the result of applying the Wilcoxon Rank Sum Test 6. Generate Latex
 * tables with the ranking obtained by applying the Friedman test 7. Generate R scripts to obtain
 * boxplots
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class MOEADAWAStudy {
  private static final int INDEPENDENT_RUNS = 1;

  public static void main(String[] args) throws IOException {
    String experimentBaseDirectory = "/home/ruben/borrar/";

    List<ExperimentProblem<DoubleSolution>> problemList = new ArrayList<>();
    problemList.add(new ExperimentProblem<>(new DTLZ7(7,3)));

    List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithmList =
            configureAlgorithmList(problemList);

    List<String> referenceFrontFileNames = Arrays.asList("DTLZ7.3D.pf");

    Experiment<DoubleSolution, List<DoubleSolution>> experiment =
            new ExperimentBuilder<DoubleSolution, List<DoubleSolution>>("NSGAIIStudyMaO")
                    .setAlgorithmList(algorithmList)
                    .setProblemList(problemList)
                    .setExperimentBaseDirectory(experimentBaseDirectory)
                    .setOutputParetoFrontFileName("FUN")
                    .setOutputParetoSetFileName("VAR")
                    .setReferenceFrontDirectory("/home/ruben/Documents/Research/GWASFGA-MaO/MOEAFramework-2.12/pf")
                    .setReferenceFrontFileNames(referenceFrontFileNames)
                    .setIndicatorList(Arrays.asList(
                            new Epsilon<>(),
                            new Spread<>(),
                            new GenerationalDistance<>(),
                            new PISAHypervolume<>(),
                            new InvertedGenerationalDistance<>(),
                            new InvertedGenerationalDistancePlus<>()))
                    .setIndependentRuns(INDEPENDENT_RUNS)
                    .setNumberOfCores(8)
                    .build();

    new ExecuteAlgorithms<>(experiment).run();
    new ComputeQualityIndicators<>(experiment).run();
    new GenerateLatexTablesWithStatistics(experiment).run();
    new GenerateWilcoxonTestTablesWithR<>(experiment).run();
    new GenerateFriedmanTestTables<>(experiment).run();
    new GenerateBoxplotsWithR<>(experiment).setRows(3).setColumns(3).run();
  }

  /**
   * The algorithm list is composed of pairs {@link Algorithm} + {@link Problem} which form part of
   * a {@link ExperimentAlgorithm}, which is a decorator for class {@link Algorithm}. The {@link
   * ExperimentAlgorithm} has an optional tag component, that can be set as it is shown in this example,
   * where four variants of a same algorithm are defined.
   */
  private static List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> configureAlgorithmList(
          List<ExperimentProblem<DoubleSolution>> problemList) {
    List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms = new ArrayList<>();

      for (ExperimentProblem<DoubleSolution> aProblemList : problemList) {
          Algorithm<List<DoubleSolution>> algorithm = new MOEADAWA(
                  aProblemList.getProblem(),
                  300,
                  300,
                  100000,
                  new DifferentialEvolutionCrossover(),
                  new PolynomialMutation(),
                  AbstractMOEAD.FunctionType.TCHE,
                  "MOEAD_Weights",
                  0.9,
                  10,
                  30, //0.1 * populationSize
                  0,
                  0,
                  0,
                  0,
                  0
          );

          algorithms.add(new ExperimentAlgorithm<>(algorithm, "MOEADAWA", aProblemList.getTag()));
      }

    return algorithms;
  }
}