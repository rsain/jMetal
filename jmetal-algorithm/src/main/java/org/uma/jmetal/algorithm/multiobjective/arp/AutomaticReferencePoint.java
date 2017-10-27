package org.uma.jmetal.algorithm.multiobjective.arp;

import java.util.ArrayList;
import java.util.List;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.InteractiveAlgorithm;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.util.referencePoint.ReferencePoint;

public abstract class AutomaticReferencePoint<S, R> implements Algorithm<R> {

  protected InteractiveAlgorithm<S,R> algorithm;
  protected Problem<S> problem ;
  protected List<Integer> indexOfRelevantObjectiveFunctions;
  protected List<S> paretoOptimalSolutions;

  public AutomaticReferencePoint(Problem<S> problem,InteractiveAlgorithm<S,R> algorithm){
    this.problem = problem;
    this.algorithm =algorithm;
    this.indexOfRelevantObjectiveFunctions = new ArrayList<>();
    this.paretoOptimalSolutions = new ArrayList<>();
  }

  protected abstract List<ReferencePoint> generatePreferenceInformation();
  protected abstract boolean isStoppingConditionReached();
  protected abstract void initProgress();
  protected abstract void updateProgress();
  protected abstract List<Integer> relevantObjectiveFunctions(R front);
  protected abstract List<ReferencePoint>  calculateReferencePoints(
      List<Integer> indexOfRelevantObjectiveFunctions,R front,List<S> paretoOptimalSolutions);
  protected abstract void updateParetoOptimal(R front,List<S> paretoOptimalSolutions);
  public abstract List<ReferencePoint> getReferencePoints();

  @Override
  public void run() {
    List<ReferencePoint> initialReferencePoints=generatePreferenceInformation();
    R front;
    List<ReferencePoint> interestingPoint=initialReferencePoints;
    initProgress();
    while (!isStoppingConditionReached()) {
      this.algorithm.updateInterestPoint(interestingPoint);
      this.algorithm.run();
      front=this.algorithm.getResult();
      updateParetoOptimal(front,paretoOptimalSolutions);
      indexOfRelevantObjectiveFunctions=relevantObjectiveFunctions(front);
      interestingPoint = calculateReferencePoints(indexOfRelevantObjectiveFunctions,front,paretoOptimalSolutions);
      updateProgress();
    }
  }

  @Override
  public R getResult() {
    return this.algorithm.getResult();
  }

  @Override
  public String getName() {
    return "AutomaticReferencePoint";
  }

  @Override
  public String getDescription() {
    return "AutomaticReferencePoint";
  }
}
