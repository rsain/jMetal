package org.uma.jmetal.algorithm.multiobjective.arp;

import org.uma.jmetal.algorithm.InteractiveAlgorithm;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.impl.AbstractBinaryProblem;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.problem.impl.AbstractGenericProblem;
import org.uma.jmetal.problem.impl.AbstractIntegerDoubleProblem;
import org.uma.jmetal.problem.impl.AbstractIntegerPermutationProblem;
import org.uma.jmetal.problem.impl.AbstractIntegerProblem;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.comparator.ObjectiveComparator;
import org.uma.jmetal.util.point.Point;
import org.uma.jmetal.util.point.impl.ArrayPoint;
import org.uma.jmetal.util.point.util.distance.EuclideanDistance;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;
import org.uma.jmetal.util.referencePoint.ReferencePoint;
import org.uma.jmetal.util.referencePoint.impl.IdealPoint;
import org.uma.jmetal.util.referencePoint.impl.NadirPoint;

import java.util.*;

public class ARP<S extends Solution<?>> extends  AutomaticReferencePoint<S,List<S>> {
                                       //    calculateDistance(front.get(0),currentReferencePoint);

  protected IdealPoint idealOjectiveVector = null;
  protected NadirPoint nadirObjectiveVector = null;
  protected List<Double> rankingCoeficient = null;
  protected ReferencePoint asp = null;
  protected double tolerance;
  protected int numberReferencePoints;
  protected JMetalRandom random = null;
  protected double considerationProbability;
  protected  int numberOfObjectives;
  protected double varyingProbability;
  protected int evaluations;
  protected int maxEvaluations;
  protected List<ReferencePoint> allReferencePoints;
  protected ReferencePoint currentReferencePoint;
  protected List<Double> distances;
  protected List<Double> distancesRP;
  private S solutionRun=null;
  public ARP(Problem<S> problem,
      InteractiveAlgorithm<S,List<S>> algorithm,double considerationProbability,double tolerance,int maxEvaluations
  ,List<Double> rankingCoeficient,int numberReferencePoints) {
    super(problem, algorithm);
    this.considerationProbability = considerationProbability;
    this.tolerance = tolerance;
    numberOfObjectives=problem.getNumberOfObjectives();
    this.random = JMetalRandom.getInstance();
    this.maxEvaluations = maxEvaluations;
    this.rankingCoeficient = rankingCoeficient;
    if(rankingCoeficient==null || rankingCoeficient.isEmpty()){
      initialiceRankingCoeficient();
    }
    this.numberReferencePoints = numberReferencePoints;
    this.allReferencePoints = new ArrayList<>();
    this.distances = new ArrayList<>();
    this.distancesRP = new ArrayList<>();
  }


  private  void  initialiceRankingCoeficient(){
    rankingCoeficient = new ArrayList<>();
    for (int i = 0; i < problem.getNumberOfObjectives() ; i++) {
      rankingCoeficient.add(1.0/problem.getNumberOfObjectives());
    }
  }

  private void updateObjectiveVector(List<S> solutionList){
   for (int j = 0; j < numberOfObjectives; j++) {
      Collections.sort(solutionList, new ObjectiveComparator<>(j));
      double objetiveMinn = solutionList.get(0).getObjective(j);
      double objetiveMaxn = solutionList.get(solutionList.size() - 1).getObjective(j);
      idealOjectiveVector.setObjective(j,objetiveMinn);
      nadirObjectiveVector.setObjective(j,objetiveMaxn);
    }
    if(problem instanceof AbstractDoubleProblem){
      AbstractDoubleProblem aux =(AbstractDoubleProblem)problem;
      for (int i = 0; i < numberOfObjectives ; i++) {
        idealOjectiveVector.setObjective(i,aux.getLowerBound(i));
        nadirObjectiveVector.setObjective(i,aux.getUpperBound(i));
      }
    }else if(problem instanceof AbstractIntegerProblem){
      AbstractIntegerProblem aux =(AbstractIntegerProblem)problem;
      for (int i = 0; i < numberOfObjectives ; i++) {
        idealOjectiveVector.setObjective(i,aux.getLowerBound(i));
        nadirObjectiveVector.setObjective(i,aux.getUpperBound(i));
      }
    }else if(problem instanceof AbstractIntegerDoubleProblem){
      AbstractIntegerDoubleProblem aux =(AbstractIntegerDoubleProblem)problem;
      for (int i = 0; i < numberOfObjectives ; i++) {
        idealOjectiveVector.setObjective(i,aux.getLowerBound(i).doubleValue());
        nadirObjectiveVector.setObjective(i,aux.getUpperBound(i).doubleValue());
      }
    }
    asp = idealOjectiveVector;
  }

  @Override
  protected List<ReferencePoint> generatePreferenceInformation() {

    idealOjectiveVector = new IdealPoint(numberOfObjectives);
    nadirObjectiveVector = new NadirPoint(numberOfObjectives);

    List<S> solutions = new ArrayList<>();
    solutions.add(problem.createSolution());
    updateObjectiveVector(solutions);
   // solutionRun = solutions.get(0);
    asp = idealOjectiveVector;
    List<ReferencePoint> referencePoints  = new ArrayList<>();
    for (int i=0;i < numberReferencePoints;i++){
      ReferencePoint referencePoint = new IdealPoint(numberOfObjectives);
      for (int j = 0; j < numberOfObjectives; j++) {
        double rand = random.nextDouble(0.0, 1.0);
        if (rand < considerationProbability * rankingCoeficient.get(i)) {
          referencePoint.setObjective(j, asp.getObjective(j));
        } else {
          referencePoint.setObjective(j, nadirObjectiveVector.getObjective(j));
        }
      }
      referencePoints.add(referencePoint);
    }
    currentReferencePoint = referencePoints.get(0);
    allReferencePoints.addAll(referencePoints);
    return referencePoints;
  }

  @Override
  protected boolean isStoppingConditionReached() {
    boolean stop = evaluations >= maxEvaluations;
   // if(distancesRP!=null){
    //  stop = stop || distancesRP.contains(0.0);
   // }
    if(indexOfRelevantObjectiveFunctions!=null   ){
      stop = stop || indexOfRelevantObjectiveFunctions.size()==numberOfObjectives;
    }
    return stop;
  }

  @Override
  protected void initProgress() {
    evaluations =0;
    varyingProbability = considerationProbability;
  }

  @Override
  protected void updateProgress() {
    evaluations++;
  }

  @Override
  protected List<Integer> relevantObjectiveFunctions(List<S> front) {
    List<Integer> order = new ArrayList<>();
    List<Integer> indexRelevantObjectivesFunctions = new ArrayList<>();
    //updateObjectiveVector(front);
    SortedMap<Double, List<Integer>> map = new TreeMap<>(Collections.reverseOrder());
    for (int i = 0; i < rankingCoeficient.size(); i++) {
      List<Integer> aux = map.getOrDefault(rankingCoeficient.get(i), new ArrayList<>());
      aux.add(i);
      map.putIfAbsent(rankingCoeficient.get(i),aux);
    }
    Set<Double> keys =map.keySet();
    for (Double key:keys) {
      order.addAll(map.get(key));
    }
    S solution = getSolution(front,currentReferencePoint);
    for (Integer i : order) {
      double rand = random.nextDouble(0.0, 1.0);
      if ((asp.getObjective(i) - solution.getObjective(i)) < tolerance
          && rand < considerationProbability) {
        indexRelevantObjectivesFunctions.add(i);
      } else if (rand < varyingProbability) {
        indexRelevantObjectivesFunctions.add(i);
      }
      varyingProbability -= (varyingProbability / i) * indexRelevantObjectivesFunctions.size();
    }
    return indexRelevantObjectivesFunctions;
  }

  @Override
  protected List<ReferencePoint> calculateReferencePoints(
      List<Integer> indexOfRelevantObjectiveFunctions, List<S> front,List<S> paretoOptimalSolutions) {
    List<ReferencePoint> result = new ArrayList<>();
    List<S> temporal = new ArrayList<>(front);

    for(int numRefPoint=0;numRefPoint<numberReferencePoints;numRefPoint++){
      if(solutionRun!=null) {
        calculateDistance(solutionRun, asp);
       // calculateDistanceRP(solutionRun, currentReferencePoint);
      }
      S solution = getSolution(temporal,currentReferencePoint);
      solutionRun = solution;
      temporal.remove(solution);
     // if (indexOfRelevantObjectiveFunctions.size() == numberOfObjectives) {
       // result.add(getReferencePointFromSolution(solution));
      //} else {
        ReferencePoint referencePoint = new IdealPoint(numberOfObjectives);
        for (int i = 0; i < referencePoint.getNumberOfObjectives(); i++) {
          if (indexOfRelevantObjectiveFunctions.contains(i)) {
            referencePoint.setObjective(i,
                asp.getObjective(i) - (asp.getObjective(i) - solution.getObjective(i)) / 2);
          } else {
            //predict the i position of reference point
            referencePoint.setObjective(i, prediction(i,paretoOptimalSolutions,solution));
          }
        }
        result.add(referencePoint);
      //}
    }

    currentReferencePoint = result.get(0);
    allReferencePoints.addAll(result);

    return result;
  }

  private void calculateDistance(S solution, ReferencePoint referencePoint){
    EuclideanDistance euclideanDistance = new EuclideanDistance();

    double distance = euclideanDistance.compute(getPointFromSolution(solution),
        getPointFromReferencePoint(referencePoint));
    distances.add(distance);
  }

  private void calculateDistanceRP(S solution, ReferencePoint referencePoint){
    EuclideanDistance euclideanDistance = new EuclideanDistance();

    double distance = euclideanDistance.compute(getPointFromSolution(solution),
        getPointFromReferencePoint(referencePoint));
    distancesRP.add(distance);
  }
  private ReferencePoint getReferencePointFromSolution(S solution) {
    ReferencePoint result = new IdealPoint(solution.getNumberOfObjectives());
    result.update(solution);
    return result;
  }
 /* @Override
  protected double calculateComponentReferencePoint(int index,List<S> front) {
    S solution = getSolution(front,currentReferencePoint);
    double result = asp.getObjective(index) - (asp.getObjective(index) -
          solution.getObjective(index)) / 2;
    return result;
  }

  @Override
  protected double prediction(int index,List<List<S>> paretoOptimalSolutions) {
    //FALTA PREDICTION
    return currentReferencePoint.getObjective(index);
  }*/
 private double prediction(int index,List<S> paretoOptimalSolutions,S solution) {
   //FALTA PREDICTION
   DecisionTreeEstimator<S> dte = new DecisionTreeEstimator<S>(paretoOptimalSolutions);

   double data=dte.doPrediction(index,solution);
   return data;//currentReferencePoint.getObjective(index);
 }
  @Override
  protected void updateParetoOptimal(List<S> front,List<S> paretoOptimalSolutions) {
    paretoOptimalSolutions.addAll(front);
  }

  @Override
  public List<ReferencePoint> getReferencePoints() {
    return allReferencePoints;
  }

  @Override
  public List<Double> getDistances() {
    //for (ReferencePoint referencePoint:allReferencePoints) {
    //   calculateDistance(getSolution(paretoOptimalSolutions,referencePoint),referencePoint);
    //}
    return distances;
  }

  private S getSolution(List<S> front, ReferencePoint referencePoint) {
    S result = front.get(0);
    EuclideanDistance euclideanDistance = new EuclideanDistance();
    SortedMap<Double, S> map = new TreeMap<>();
    for (S solution : front) {
      double distance = euclideanDistance.compute(getPointFromSolution(solution),
          getPointFromReferencePoint(referencePoint));
      map.put(distance, solution);
    }
    result = map.get(map.firstKey());
    return result;
  }
  private Point getPointFromSolution(S solution) {
    Point result = new ArrayPoint(solution.getNumberOfObjectives());
    for (int i = 0; i < solution.getNumberOfObjectives(); i++) {
      result.setDimensionValue(i, solution.getObjective(i));
    }
    return result;
  }

  private Point getPointFromReferencePoint(ReferencePoint referencePoint) {
    Point result = new ArrayPoint(referencePoint.getNumberOfObjectives());
    for (int i = 0; i < referencePoint.getNumberOfObjectives(); i++) {
      result.setDimensionValue(i, referencePoint.getObjective(i));
    }
    return result;
  }
}
