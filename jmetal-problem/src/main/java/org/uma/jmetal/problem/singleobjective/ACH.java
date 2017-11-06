package org.uma.jmetal.problem.singleobjective;


import java.util.ArrayList;
import java.util.List;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.util.JMetalException;

/**
 * @author Juan J. Durillo
 * Modified by Antonio J. Nebro
 *
 */
@SuppressWarnings("serial")
public class ACH extends AbstractDoubleProblem {

	private final List<Double> interestPoint;
	private List<Double> utopia = null;
	private List<Double> nadir  = null;
  private double rho;
	private AbstractDoubleProblem problem;
  public ACH(List<Double> interestPoint,AbstractDoubleProblem problem,List<Double> utopia,
			List<Double> nadir,double rho){
		this.interestPoint = interestPoint;
		this.problem = problem;
		this.rho = rho;
		setNumberOfVariables(problem.getNumberOfVariables());
		setNumberOfObjectives(1);
		setNumberOfConstraints(problem.getNumberOfConstraints()) ;
		this.utopia = utopia;
		this.nadir = nadir;
	}
	public ACH(List<Double> interestPoint,AbstractDoubleProblem problem,double rho){
		this.interestPoint = interestPoint;
		this.problem = problem;
		this.rho = rho;
		setNumberOfVariables(problem.getNumberOfVariables());
		setNumberOfObjectives(1);
		setNumberOfConstraints(problem.getNumberOfConstraints()) ;
		createNadir(interestPoint.size());
		createUtopia(interestPoint.size());
	}

	@Override
	public Double getUpperBound(int index) {
		return problem.getUpperBound(index);
	}

	@Override
	public Double getLowerBound(int index) {
		return problem.getLowerBound(index);
	}


	private void createUtopia(int numberObjectives){
  	utopia = new ArrayList<>();
		for (int i = 0; i < numberObjectives ; i++) {
			utopia.add(problem.getLowerBound(i));
		}
	}
	private void createNadir(int numberObjectives){
		nadir = new ArrayList<>();
		for (int i = 0; i < numberObjectives ; i++) {
			nadir.add(problem.getUpperBound(i));
		}
	}

	public void updatePointOfInterest(List<Double> newInterestPoint ) {
		if (this.interestPoint.size()!=newInterestPoint.size())
			throw new JMetalException("Wrong dimension of the interest point vector");

		for (int i = 0; i < newInterestPoint.size(); i++) {
            this.interestPoint.set(i,newInterestPoint.get(i));
		}
	}


	public AbstractDoubleProblem getProblem() {
		return problem;
	}

	public void setNadir(List<Double> nadir) {
		this.nadir = nadir;
		
	}
	public void setUtopia(List<Double> utopia) {
		this.utopia = utopia;
	}

	@Override
	public void evaluate(DoubleSolution solution) {
  	double result = 0.0;
		problem.evaluate(solution);
		int numberOfObjectives = solution.getNumberOfObjectives();
		double firstPart = Double.MIN_VALUE;
		double secondPart = 0.0;
		for (int i = 0; i < numberOfObjectives ; i++) {
			double max = (solution.getObjective(i) - interestPoint.get(i))/(nadir.get(i)-utopia.get(i));
			if(max>firstPart){
				firstPart = max;
			}

			secondPart += solution.getObjective(i)/(nadir.get(i)-utopia.get(i));
		}
		secondPart *=rho;
		result = firstPart + secondPart;
		solution.setObjective(0,result);
	}
	@Override
	public DoubleSolution createSolution() {
		return problem.createSolution();
	}


}
