package methods;

import java.util.Iterator;

import ode.InitialValueProblem;
import ode.NumericalSolution;
import ode.NumericalSolutionPoint;

public abstract class FixedStepMethod {
	
	protected InitialValueProblem problem;
	protected double step;
	protected NumericalSolution solution;
	protected long evaluationCounter = 0;
	
	protected FixedStepMethod(InitialValueProblem problem, double step) {
		this.problem = problem;
		this.step = step;
		this.solution = new NumericalSolution(this.problem);
	}
	
	abstract public double doStep(double deltaTime, double time, double[] state);
	
	public double getStep() {
		return step;
	}
	
	public NumericalSolutionPoint step() {
		NumericalSolutionPoint lastPoint = solution.getLastPoint();
		double time = lastPoint.getTime();
		double[] state = lastPoint.getState();
		time = doStep(step, time, state);
		if (Double.isNaN(time)) return null;
		return solution.add(time, state);
	}
	
	public double solve(double finalTime) {
		NumericalSolutionPoint lastPoint = solution.getLastPoint();
		double time = lastPoint.getTime();
		double[] state = lastPoint.getState();
		if (step > 0) {
			while (time < finalTime) {
				time = doStep(step, time, state);
				if (Double.isNaN(time)) return Double.NaN;
				solution.add(time, state);
			}
		}
		else if (step<0) {
			while (time>finalTime) {
				time = doStep(step,time,state);
	            if (Double.isNaN(time)) return Double.NaN;
	            solution.add(time, state);
	        }
	    } // does nothing if mStep = 0
	    return time;
	}
	
    public NumericalSolution getSolution() { return solution; }
    
    public void resetEvaluationCounter() { 
        evaluationCounter = 0;
    }
    
    public long getEvaluationCounter() {
        return evaluationCounter;
    }

    protected void addToEvaluationCounter(int add) {
        evaluationCounter += add;
    }
    
    static public double maxHalfStepError (NumericalSolution fullStep, NumericalSolution halfStep) {
        Iterator<NumericalSolutionPoint> iteratorFull = fullStep.iterator();
        Iterator<NumericalSolutionPoint> iteratorHalf = halfStep.iterator();
        double maxError = 0;
        while (iteratorFull.hasNext() && iteratorHalf.hasNext()) {
            double[] stateFull = iteratorFull.next().getState();
            double[] stateHalf = iteratorHalf.next().getState();
            double estimatedError = 0;
            for (int i=0; i<stateFull.length; i++) {
                double estimatedErrorInI = Math.abs(stateHalf[i]-stateFull[i]); 
                estimatedError = Math.max(estimatedError,estimatedErrorInI);
            }
            maxError = Math.max(maxError, estimatedError);
            if (!iteratorHalf.hasNext()) return maxError;
            iteratorHalf.next();
        }
        return maxError;
    }
	

}
