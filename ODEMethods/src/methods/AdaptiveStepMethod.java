package methods;

import java.util.ArrayList;
import java.util.List;

import interpolation.HermiteInterpolator;
import interpolation.StateFunction;
import ode.InitialValueProblem;
import ode.NumericalSolution;
import ode.NumericalSolutionPoint;

public abstract class AdaptiveStepMethod extends FixedStepMethod{
	protected double tolerance = 1.0e-4;
	protected NumericalSolution realSolution;
	protected List<Double> stepList = new ArrayList<>();
	protected double step;
    protected double currentStep;
    protected double minimumStepAllowed; // Non-convergence minimum
	
    public AdaptiveStepMethod(InitialValueProblem problem, double step, double tolerance) {
        super(problem,step);
        this.step = step;
        currentStep = step;
        minimumStepAllowed = Math.abs(step / 1.0e6);
        realSolution = new NumericalSolution(this.problem);
    }
    
    public double getTolerance() {
    	return tolerance;
    }
    
    public List<Double> getStepList() {
    	return stepList;
    }
    
    @Override
	public NumericalSolutionPoint step() {
		NumericalSolutionPoint lastPoint = realSolution.getLastPoint();
		double time = lastPoint.getTime();
		double[] state = lastPoint.getState().clone();
		double actualTime = solution.getLastPoint().getTime();
		if (step > 0) {
			while(time < actualTime + step) {
				time = doStep(currentStep, time, state);
				if (Double.isNaN(time)) return null;
				realSolution.add(time, state);
			}
			if (time > actualTime + step) {
				List<NumericalSolutionPoint> pointsToInterpolate = realSolution.getList();
				NumericalSolutionPoint fromPoint = pointsToInterpolate.get(pointsToInterpolate.size() - 2);
				NumericalSolutionPoint toPoint = pointsToInterpolate.get(pointsToInterpolate.size() - 1);
				StateFunction interpolator = new HermiteInterpolator(problem, fromPoint, toPoint);
				return solution.add(actualTime + step, interpolator.getState(actualTime + step));
			}
		} else if (step < 0) {
			while(time > actualTime + step) {
				time = doStep(currentStep, time, state);
				if (Double.isNaN(time)) return null;
				realSolution.add(time, state);
			}
			if (time > actualTime + step) {
				List<NumericalSolutionPoint> pointsToInterpolate = realSolution.getList();
				NumericalSolutionPoint fromPoint = pointsToInterpolate.get(pointsToInterpolate.size() - 2);
				NumericalSolutionPoint toPoint = pointsToInterpolate.get(pointsToInterpolate.size() - 1);
				StateFunction interpolator = new HermiteInterpolator(problem, fromPoint, toPoint);
				return solution.add(actualTime + step, interpolator.getState(actualTime + step));
			}
		}
		if (Double.isNaN(time)) return null;
		return solution.add(time, state);
	}
}
