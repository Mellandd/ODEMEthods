package methods;

import java.util.Arrays;

import ode.InitialValueProblem;

public class FixedStepBackwardEulerMethod  extends FixedStepMethod{

	private double tolerance;
	private int maxIter;
	public FixedStepBackwardEulerMethod(InitialValueProblem problem, double step, double tolerance, int maxIter) {
		super(problem, step);
		this.tolerance = tolerance;
		this.maxIter = maxIter;
	}

	@Override
	public double doStep(double deltaTime, double time, double[] state) {
		if (problem.getPartialDerivative(time,state) == null) {
			System.out.println("Partial derivative not found.");
			return Double.NaN;
		}
		double[] w0 = Arrays.copyOf(state, state.length);
		Boolean FLAG = true;
		double[] w = Arrays.copyOf(state, state.length);
		double[] fy;
		double[] f;
		int j = 1;
		double coc, error;
		while(FLAG) {
	        super.addToEvaluationCounter(2);
			f= problem.getDerivative(time+ deltaTime, w0);
			fy = problem.getPartialDerivative(time + deltaTime, w0);
			for (int i = 0; i < state.length; i++) {
				coc = (w0[i]-deltaTime*f[i]-state[i])/(1.-deltaTime*fy[i]);
				w[i]=w0[i]-coc;
			}
			error = 0;
			for (int i = 0; i < state.length; i++) {
				error += (w[i]-w0[i]) * (w[i]-w0[i]);
			}
			error = Math.sqrt(error);
			if (error < tolerance) {
				FLAG = false;
			} else {
				for (int i = 0; i < state.length; i++) {
					w0[i] = w[i];
				}
				j++;
				if (j> maxIter) {
					System.out.println("Error, máximo número de iteraciones");
					return 0;
				}
			}
		}
		for (int i = 0; i < state.length; i++) {
			state[i] = w[i];
		}
		return time+deltaTime;
	}
	

}
