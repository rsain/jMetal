package org.uma.jmetal.algorithm.singleobjective.differentialevolution;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import org.uma.jmetal.algorithm.InteractiveAlgorithm;
import org.uma.jmetal.algorithm.impl.AbstractDifferentialEvolution;
import org.uma.jmetal.algorithm.multiobjective.arp.AutomaticReferencePoint;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.operator.impl.selection.DifferentialEvolutionSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.problem.singleobjective.ACH;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.comparator.ObjectiveComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.referencePoint.ReferencePoint;

/**
 * This class implements a differential evolution algorithm.
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
@SuppressWarnings("serial")
public class DifferentialEvolutionARP extends DifferentialEvolution implements
    InteractiveAlgorithm<DoubleSolution,DoubleSolution> {


  /**
   * Constructor
   *
   * @param problem Problem to solve
   * @param maxEvaluations Maximum number of evaluations to perform
   */
  public DifferentialEvolutionARP(DoubleProblem problem, int maxEvaluations, int populationSize,
      DifferentialEvolutionCrossover crossoverOperator,
      DifferentialEvolutionSelection selectionOperator,
      SolutionListEvaluator<DoubleSolution> evaluator) {
    super(problem, maxEvaluations, populationSize, crossoverOperator, selectionOperator, evaluator);
  }


  @Override
  public void updateInterestPoint(List<ReferencePoint> newReferencePoints) {
    List<Double> referencePoint= new ArrayList<>();
    ReferencePoint rf=newReferencePoints.get(0);
    //for (ReferencePoint rf:newReferencePoints) {
      for (int i = 0; i < rf.getNumberOfObjectives() ; i++) {
        referencePoint.add(rf.getObjective(i));
      }
   // }
    ((ACH)getProblem()).updatePointOfInterest(referencePoint);
  }
}
