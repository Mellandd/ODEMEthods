package methods;

import java.util.Arrays;

import ode.InitialValueProblem;

public class FixedStepTrapezoidalMethod extends FixedStepMethod{
	
	private double tol;
	private int maxIter;

	public FixedStepTrapezoidalMethod(InitialValueProblem problem, double step, double tol, int maxIter) {
		super(problem, step);
		this.tol = tol;
		this.maxIter = maxIter;
	}

	@Override
	public double doStep(double deltaTime, double time, double[] state) {
		double[] k1 = new double[state.length];
		double h = deltaTime/2.0;
		double[] k = problem.getDerivative(time, state);
        super.addToEvaluationCounter(1);
		for (int i = 0; i < state.length; i++) {
			k1[i] = state[i] + h*k[i];
		}
		double[] w0 = Arrays.copyOf(k1, state.length);
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
				coc = (w0[i]-h*f[i]-k1[i])/(1.-h*fy[i]);
				w[i]=w0[i]-coc;
			}
			error = 0;
			for (int i = 0; i < state.length; i++) {
				error += (w[i]-w0[i]) * (w[i]-w0[i]);
			}
			error = Math.sqrt(error);
			if (error < tol) {
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
