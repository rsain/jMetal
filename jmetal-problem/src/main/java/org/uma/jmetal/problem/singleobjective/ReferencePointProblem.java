package org.uma.jmetal.problem.singleobjective;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.distance.impl.EuclideanDistanceBetweenSolutionsInObjectiveSpace;
import org.uma.jmetal.util.distance.impl.EuclideanDistanceBetweenSolutionsInSolutionSpace;

@SuppressWarnings("serial")
public class ReferencePointProblem extends AbstractDoubleProblem {
  /**
   * Constructor
   * Creates a default instance of the Reference Point problem
   *
   * @param front
   */
  private List<Double> asp;
  private DoubleProblem problem;
  private DoubleSolution solutionAsp;
  EuclideanDistanceBetweenSolutionsInSolutionSpace euclidean;
  public ReferencePointProblem(List<Double> asp,DoubleProblem problem) {
    this.asp = new ArrayList<>(asp) ;
    this.problem = problem;
    this.euclidean = new EuclideanDistanceBetweenSolutionsInSolutionSpace();
    setNumberOfVariables(problem.getNumberOfObjectives());
    setNumberOfObjectives(1);
    setNumberOfConstraints(0) ;
    setName("Finding Reference Point ");

    List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables()) ;
    List<Double> upperLimit = new ArrayList<>(getNumberOfVariables()) ;

    //Collections.sort(asp);
    solutionAsp = problem.createSolution();//problem.createSolution();
    for (int i = 0; i < getNumberOfVariables(); i++) {
      solutionAsp.setObjective(i,asp.get(i));
      solutionAsp.setVariableValue(i,asp.get(i));
      lowerLimit.add(Collections.min(asp));
      upperLimit.add(Collections.max(asp));
    }

    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);
  }

  /** Evaluate() method */
  @Override
  public void evaluate(DoubleSolution solution) {
    double result = Math.abs(euclidean.getDistance(solution,solutionAsp));
    solution.setObjective(0, result);
  }
}

