package ode;

import java.util.Iterator;
import java.util.List;

import methods.FixedStepMethod;

public class SolutionObject{
	private List<NumericalSolutionPoint> points;
	private FixedStepMethod interpolator;
	double initialTime, lastTime;
	
	public SolutionObject(List<NumericalSolutionPoint> points, FixedStepMethod interpolator) {
		this.points = points;
		this.interpolator = interpolator;
		this.initialTime = points.get(0).getTime();
		this.lastTime = points.get(points.size() - 1).getTime();
	}
	
    public double[] getState(double time) {
    	if (time >= initialTime && time <= lastTime) {
    		double actualTime = initialTime;
    		NumericalSolutionPoint actualPoint = null;
    		int n = points.size();
    		for (int i = 0; i < n; i++) {
				actualPoint = points.get(i);
				actualTime = actualPoint.getTime();
			}
    		Iterator<NumericalSolutionPoint> iterator = points.iterator();
    		while(iterator.hasNext() && actualTime <= time) {
    			actualPoint = iterator.next();
    			actualTime = actualPoint.getTime();
    		}
    		if (actualTime == time) {
    			return actualPoint.getState();
    		} else {
    			double[] pointToCalculate = actualPoint.getState().clone();
    			interpolator.solveSingleStep(actualTime, time- actualTime, pointToCalculate);
    			return pointToCalculate;
    		}
    	} else {
    		System.out.println("Time given not in the calculated intervale.");
    		return null;
    	}
    }

    public double getState(double time, int index) {
    	if (time >= initialTime && time <= lastTime) {
    		double actualTime = initialTime;
    		NumericalSolutionPoint actualPoint = null;
    		Iterator<NumericalSolutionPoint> iterator = points.iterator();
    		while(iterator.hasNext() && actualTime <= time) {
    			actualPoint = iterator.next();
    			actualTime = actualPoint.getTime();
    		}
    		if (actualPoint == null) {
    			
    		}
    		if (actualTime == time) {
    			return actualPoint.getState()[index];
    		} else {
    			double[] pointToCalculate = actualPoint.getState().clone();
    			interpolator.solveSingleStep(actualTime, time- actualTime, pointToCalculate);
    			return pointToCalculate[index];
    		}
    	} else {
    		System.out.println("Time given not in the calculated intervale.");
    		return Double.NaN;
    	}
    }
    
    public List<NumericalSolutionPoint> getList(){
    	return points;
    }
}
