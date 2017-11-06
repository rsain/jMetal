package org.uma.jmetal.operator.impl.selection;

import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.comparator.CrowdingDistanceComparator;
import org.uma.jmetal.util.comparator.PreferenceDistanceComparator;
import org.uma.jmetal.util.solutionattribute.Ranking;
import org.uma.jmetal.util.solutionattribute.impl.DominanceRanking;
import org.uma.jmetal.util.solutionattribute.impl.PreferenceDistance;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RankingAndPreferenceSelection<S extends Solution<?>>
        implements SelectionOperator<List<S>,List<S>> {
    private final int solutionsToSelect ;
    private List<Double> interestPoint;
    private double epsilon;
    int rankingIndex = 0;
    /** Constructor */
    public RankingAndPreferenceSelection(int solutionsToSelect,List<Double> interestPoint,double epsilon) {
        this.solutionsToSelect = solutionsToSelect ;
        this.interestPoint = interestPoint;
        this.epsilon =epsilon;
    }

    /* Getter */
    public int getNumberOfSolutionsToSelect() {
        return solutionsToSelect;
    }

    @Override
    public List<S> execute(List<S> solutionList) {
        if (null == solutionList) {
            throw new JMetalException("The solution list is null");
        } else if (solutionList.isEmpty()) {
            throw new JMetalException("The solution list is empty") ;
        }  else if (solutionList.size() < solutionsToSelect) {
            throw new JMetalException("The population size ("+solutionList.size()+") is smaller than" +
                    "the solutions to selected ("+solutionsToSelect+")")  ;
        }
        int index=0;
        int numberOfObjectives = solutionList.get(0).getNumberOfObjectives();
        int nInteresPoint=this.interestPoint.size()/numberOfObjectives;
        List<S> solution = new ArrayList<>(solutionsToSelect);
        List<S> aux = new ArrayList<>(solutionList);
        Ranking<S> ranking = new DominanceRanking<S>();
        ranking.computeRanking(aux);
        rankingIndex=0;
        for(int n = 0;n<nInteresPoint;n++) {



            List<Double> auxInterestPoint = nextInterestPoint(index, numberOfObjectives);
            index += numberOfObjectives;
            PreferenceDistance<S> preferenceDistance = new PreferenceDistance<>(auxInterestPoint, epsilon);
            /*for (int i = 0; i <  ranking.getNumberOfSubfronts(); i++) {
                preferenceDistance.computeDensityEstimator(ranking.getSubfront(i));
                preferenceDistance.epsilonClean(ranking.getSubfront(i));
            }*/
           solution.addAll(preferenceDistanceSelection(ranking,solutionsToSelect/nInteresPoint,preferenceDistance));
        }

        return solution;
    }
    protected List<S> preferenceDistanceSelection(Ranking<S> ranking,int sizeList,PreferenceDistance<S> preferenceDistance) {


        List<S> population = new ArrayList<>(sizeList);



        while (population.size() < sizeList) {
            preferenceDistance.computeDensityEstimator(ranking.getSubfront(rankingIndex));
            preferenceDistance.epsilonClean(ranking.getSubfront(rankingIndex));

            if (subfrontFillsIntoThePopulation(ranking, rankingIndex, population,sizeList)) {

                addRankedSolutionsToPopulation(ranking, rankingIndex, population);
                rankingIndex++;
            } else {
               //addLastRankedSolutionsToPopulationPreference(ranking, rankingIndex, population,preferenceDistance,sizeList);
                addLastRankedSolutionsToPopulation(ranking, rankingIndex, population,sizeList);
                rankingIndex++;
            }
        }

        return population ;
    }

    protected boolean subfrontFillsIntoThePopulation(Ranking<S> ranking, int rank, List<S> population,int sizeList) {
        return ranking.getSubfront(rank).size() < (sizeList - population.size()) ;
    }

    protected void addRankedSolutionsToPopulation(Ranking<S> ranking, int rank, List<S> population) {
        List<S> front ;

        front = ranking.getSubfront(rank);

        for (int i = 0 ; i < front.size(); i++) {
            population.add(front.get(i));
        }
    }

    protected void addLastRankedSolutionsToPopulation(Ranking<S> ranking, int rank, List<S>population,int size) {
        List<S> currentRankedFront = ranking.getSubfront(rank) ;

        Collections.sort(currentRankedFront, new CrowdingDistanceComparator<S>()) ;
        int i = 0 ;
        while (population.size() < size) {
            population.add(currentRankedFront.get(i)) ;
            i++ ;
        }
    }
    protected void addLastRankedSolutionsToPopulationPreference(Ranking<S> ranking, int rank, List<S>population,PreferenceDistance preferenceDistance,int sizeList) {
        List<S> currentRankedFront = ranking.getSubfront(rank) ;


         Collections.sort(currentRankedFront, new PreferenceDistanceComparator<>(preferenceDistance)) ;
        int i = 0 ;
        while (population.size() < sizeList) {
            population.add(currentRankedFront.get(i)) ;
            i++ ;
        }
    }
    private List<Double> nextInterestPoint(int index, int size){
        List<Double> result= null;
        if(index<this.interestPoint.size()){
            result = new ArrayList<>(size);
            for(int i=0;i<size;i++){
                result.add(this.interestPoint.get(index));
                index++;
            }
        }
        return  result;
    }
}
