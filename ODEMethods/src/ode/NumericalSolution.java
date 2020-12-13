package ode;

import java.util.ArrayList;

public class NumericalSolution {
	private ArrayList<NumericalSolutionPoint> pointList;
	
	public NumericalSolution() {
		pointList = new ArrayList<>();
	}
	
	public NumericalSolution(InitialValueProblem problem) {
		pointList = new ArrayList<>();
		pointList.add(new NumericalSolutionPoint(problem.getInitialTime(), problem.getInitialState()));
	}
	
	public NumericalSolutionPoint add(double time, double state[]) {
		NumericalSolutionPointn
	}

}
