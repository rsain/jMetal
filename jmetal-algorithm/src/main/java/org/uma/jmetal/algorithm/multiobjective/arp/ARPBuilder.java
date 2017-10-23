package org.uma.jmetal.algorithm.multiobjective.arp;

import java.util.List;
import org.uma.jmetal.algorithm.InteractiveAlgorithm;
import org.uma.jmetal.algorithm.multiobjective.rnsgaii.RNSGAIIARP;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.AlgorithmBuilder;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.comparator.RankingAndCrowdingDistanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;
import org.uma.jmetal.util.referencePoint.AutomaticReferencePoint;

/**
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class ARPBuilder<S extends Solution<?>> implements AlgorithmBuilder<ARP<S>> {

  /**
   * NSGAIIBuilder class
   */
  private final Problem<S> problem;
  private int maxEvaluations;
  private InteractiveAlgorithm<S,List<S>> algorithm;
  private double considerationProbability;
  private double tolerance;
  private List<Double> rankingCoeficient;
  private int numberReferencePoints;
  /**
   * ARPBuilder constructor
   */
  public ARPBuilder(Problem<S> problem, InteractiveAlgorithm<S,List<S>> algorithm) {
    this.problem = problem;
    this.maxEvaluations = 25000;
    this.algorithm = algorithm;
    this.numberReferencePoints =1;
  }

  public ARPBuilder<S> setMaxEvaluations(int maxEvaluations) {
    if (maxEvaluations < 0) {
      throw new JMetalException("maxEvaluations is negative: " + maxEvaluations);
    }
    this.maxEvaluations = maxEvaluations;

    return this;
  }


  public ARPBuilder<S> setAlgorithm(InteractiveAlgorithm<S,List<S>> algorithm) {
    if (algorithm==null) {
      throw new JMetalException("algorithm is null");
    }
    this.algorithm = algorithm;
    return this;
  }

  public ARPBuilder<S> setConsiderationProbability(double considerationProbability) {
    if (considerationProbability < 0.0) {
      throw new JMetalException("considerationProbability is negative: " + considerationProbability);
    }
    this.considerationProbability = considerationProbability;
    return this;
  }

  public ARPBuilder<S> setTolerance(double tolerance) {
    if (tolerance < 0.0) {
      throw new JMetalException("tolerance is negative: " + tolerance);
    }
    this.tolerance = tolerance;
    return this;
  }

  public ARPBuilder<S> setRankingCoeficient(List<Double> rankingCoeficient) {
    this.rankingCoeficient = rankingCoeficient;
    return this;
  }

  public ARP<S> build() {
    ARP<S> algorithmRun = null ;
    algorithmRun = new ARP<S>(problem,algorithm,considerationProbability,tolerance, maxEvaluations,
          rankingCoeficient,numberReferencePoints);

    return algorithmRun ;
  }


  /* Getters */
  public Problem<S> getProblem() {
    return problem;
  }

  public int getMaxIterations() {
    return maxEvaluations;
  }

}
