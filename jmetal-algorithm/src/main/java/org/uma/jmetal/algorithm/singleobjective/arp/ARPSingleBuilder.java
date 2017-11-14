package org.uma.jmetal.algorithm.singleobjective.arp;

import java.util.List;
import org.uma.jmetal.algorithm.InteractiveAlgorithm;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.AlgorithmBuilder;
import org.uma.jmetal.util.JMetalException;

/**
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class ARPSingleBuilder<S extends Solution<?>> implements AlgorithmBuilder<ARPSingle<S>> {

  /**
   * NSGAIIBuilder class
   */
  private final Problem<S> problem;
  private int maxEvaluations;
  private InteractiveAlgorithm<S,S> algorithm;
  private double considerationProbability;
  private double tolerance;
  private List<Double> rankingCoeficient;
  private int numberOfObjectives;
  private List<Double> asp;
  /**
   * ARPBuilder constructor
   */
  public ARPSingleBuilder(Problem<S> problem, InteractiveAlgorithm<S,S> algorithm) {
    this.problem = problem;
    this.maxEvaluations = 25000;
    this.algorithm = algorithm;
  }

  public ARPSingleBuilder<S> setMaxEvaluations(int maxEvaluations) {
    if (maxEvaluations < 0) {
      throw new JMetalException("maxEvaluations is negative: " + maxEvaluations);
    }
    this.maxEvaluations = maxEvaluations;

    return this;
  }

  public ARPSingleBuilder<S>  setAsp(List<Double> asp) {
    this.asp = asp;
    return this;
  }

  public ARPSingleBuilder<S> setAlgorithm(InteractiveAlgorithm<S,S> algorithm) {
    if (algorithm==null) {
      throw new JMetalException("algorithm is null");
    }
    this.algorithm = algorithm;
    return this;
  }

  public ARPSingleBuilder<S> setConsiderationProbability(double considerationProbability) {
    if (considerationProbability < 0.0) {
      throw new JMetalException("considerationProbability is negative: " + considerationProbability);
    }
    this.considerationProbability = considerationProbability;
    return this;
  }

  public ARPSingleBuilder<S> setTolerance(double tolerance) {
    if (tolerance < 0.0) {
      throw new JMetalException("tolerance is negative: " + tolerance);
    }
    this.tolerance = tolerance;
    return this;
  }

  public ARPSingleBuilder<S> setRankingCoeficient(List<Double> rankingCoeficient) {
    this.rankingCoeficient = rankingCoeficient;
    return this;
  }

  public ARPSingle<S> build() {
    ARPSingle<S> algorithmRun = null ;
    algorithmRun = new ARPSingle<S>(problem,algorithm,considerationProbability,tolerance, maxEvaluations,
          rankingCoeficient, numberOfObjectives,asp);

    return algorithmRun ;
  }

  public ARPSingleBuilder<S> setNumberOfObjectives(int numberOfObjectives) {
    this.numberOfObjectives = numberOfObjectives;
    return this;
  }

  /* Getters */
  public Problem<S> getProblem() {
    return problem;
  }

  public int getMaxIterations() {
    return maxEvaluations;
  }

}
