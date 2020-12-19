package ode;

public interface InitialValueProblem {
	
	public double getInitialTime();
	
	public double[] getInitialState();
	
	public double[] getDerivative(double time, double[] state);

}
