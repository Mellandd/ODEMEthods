package ode;

import java.util.List;

public abstract class Event {
	
    private final double MAXIMUM_TIME_RESOLUTION = 1e-12;
	private double tolerance;
	protected SolutionObject function;
	
	
	public Event(double tolerance, SolutionObject function) {
		this.tolerance = tolerance;
		this.function = function;
	}
	
	abstract public void action(double time);
	
	abstract protected double crossingFunction(double time);
	
	public void calculateCrossing() {
		List<NumericalSolutionPoint> points = function.getList();
		int numberCrossings = 0;
		double time = points.get(0).getTime();
		double value = crossingFunction(time);
		double previousValue = crossingFunction(time);
		int n = points.size();
		int i = 1;
		while(i < n) {
			time = points.get(i).getTime();
			value = crossingFunction(time);
			if (value*previousValue < 0) {
				double previousTime = points.get(i-1).getTime();
				findZero(previousTime, time);
				numberCrossings++;
			}
			previousValue = value;
			i++;
		}
		if (numberCrossings == 0) {
			System.out.println("There is no crossing in this interval");
		}
		return;
	}
	
    private void findZero (double initialTime, double finalTime) {
        tolerance = Math.abs(tolerance);
        if (tolerance==0) tolerance = 1.0e-8;

        // Trivial checks
        double initialState = crossingFunction(initialTime);
        if (Math.abs(initialState)<=tolerance) {
        	action(initialTime);
        	return;
        }

        double finalState = crossingFunction(finalTime);
        if (Math.abs(finalState)  <=tolerance) {
        	action(finalTime);
        	return;
        }
        
        if (initialState*finalState>0) {
        	System.out.println("Can't calculate the zero.");
        	return;
        }
        
        do {
            double middleTime  = (initialTime+finalTime)/2;
            double middleState = crossingFunction(middleTime);
            if (Math.abs(middleState)<=tolerance) {
            	action(middleTime);
            	return;
            }
            if (initialState*middleState<0) {
                finalTime = middleTime;
                finalState = middleState;
            }
            else {
                initialTime  = middleTime;
                initialState = middleState;
            }
        } while (Math.abs(finalTime-initialTime) > MAXIMUM_TIME_RESOLUTION);
        System.out.println("Maximum time reached before getting the zero.");
        return;
    }
}
