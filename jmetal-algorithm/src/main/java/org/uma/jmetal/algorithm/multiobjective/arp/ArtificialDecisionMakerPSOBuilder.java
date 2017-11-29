package org.uma.jmetal.algorithm.multiobjective.arp;

import java.util.List;
import org.uma.jmetal.algorithm.InteractiveAlgorithm;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.AlgorithmBuilder;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;

/**
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class ArtificialDecisionMakerPSOBuilder<S extends Solution<?>> implements AlgorithmBuilder<ArtificialDecisionMakerPSO<S>> {

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
  private List<Double> asp;
  private SolutionListEvaluator<DoubleSolution> evaluator;

  /**
   * ARPBuilder constructor
   */
  public ArtificialDecisionMakerPSOBuilder(Problem<S> problem, InteractiveAlgorithm<S,List<S>> algorithm
      ) {
    this.problem = problem;
    this.maxEvaluations = 25000;
    this.algorithm = algorithm;
    this.numberReferencePoints =1;
    this.evaluator  = new SequentialSolutionListEvaluator<>();
    ;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setMaxEvaluations(int maxEvaluations) {
    if (maxEvaluations < 0) {
      throw new JMetalException("maxEvaluations is negative: " + maxEvaluations);
    }
    this.maxEvaluations = maxEvaluations;

    return this;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setAsp(List<Double> asp) {
    this.asp = asp;
    return this;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setAlgorithm(InteractiveAlgorithm<S,List<S>> algorithm) {
    if (algorithm==null) {
      throw new JMetalException("algorithm is null");
    }
    this.algorithm = algorithm;
    return this;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setConsiderationProbability(double considerationProbability) {
    if (considerationProbability < 0.0) {
      throw new JMetalException("considerationProbability is negative: " + considerationProbability);
    }
    this.considerationProbability = considerationProbability;
    return this;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setTolerance(double tolerance) {
    if (tolerance < 0.0) {
      throw new JMetalException("tolerance is negative: " + tolerance);
    }
    this.tolerance = tolerance;
    return this;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setNumberReferencePoints(int numberReferencePoints) {
    this.numberReferencePoints = numberReferencePoints;
    return this;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setEvaluator(
      SolutionListEvaluator<DoubleSolution> evaluator) {
    this.evaluator = evaluator;
    return this;
  }

  public ArtificialDecisionMakerPSOBuilder<S> setRankingCoeficient(List<Double> rankingCoeficient) {
    this.rankingCoeficient = rankingCoeficient;
    return this;
  }

  public ArtificialDecisionMakerPSO<S> build() {
    ArtificialDecisionMakerPSO<S> algorithmRun = null ;
    algorithmRun = new ArtificialDecisionMakerPSO<S>(problem,algorithm,considerationProbability,tolerance, maxEvaluations,
          rankingCoeficient,numberReferencePoints,asp,evaluator);

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
