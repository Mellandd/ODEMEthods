package methods;

import java.util.ArrayList;
import java.util.List;

import ode.InitialValueProblem;

public abstract class AdaptiveStepMethod extends FixedStepMethod{
	protected double tolerance = 1.0e-4;
	protected List<Double> stepList = new ArrayList<>();
	
    public AdaptiveStepMethod(InitialValueProblem problem, double step) {
        super(problem,step);
    }
    
    public void setTolerance(double tolerance) {
    	this.tolerance = tolerance;
    }
    
    public double getTolerance() {
    	return tolerance;
    }
    
    public List<Double> getStepList() {
    	return stepList;
    }
}
