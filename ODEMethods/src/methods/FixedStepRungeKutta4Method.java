package methods;

import ode.InitialValueProblem;

public class FixedStepRungeKutta4Method extends FixedStepMethod{
	protected double[] auxState;
	
    public FixedStepRungeKutta4Method(InitialValueProblem problem, double step) {
        super(problem,step);
        auxState = problem.getInitialState();
    }

	@Override
	public double doStep(double deltaTime, double time, double[] state) {
        super.addToEvaluationCounter(4);
        double h2 = deltaTime/2.0;
        double[] k1 = problem.getDerivative(time, state);
        for (int i=0; i<state.length; i++) {
            auxState[i] = state[i] + h2 * k1[i];
        }
        double[] k2 = problem.getDerivative(time+h2, auxState);
        for (int i=0; i<state.length; i++) {
            auxState[i] = state[i] + h2 * k2[i];
        }
        double[] k3 = problem.getDerivative(time+h2, auxState);
        for (int i=0; i<state.length; i++) {
            auxState[i] = state[i] + deltaTime * k3[i];
        }
        double[] k4 = problem.getDerivative(time+deltaTime, auxState);
        double h6 = deltaTime/6;
        for (int i=0; i<state.length; i++) {
            state[i] += h6 * (k1[i]+2*k2[i]+2*k3[i]+k4[i]);
        }
        return time+deltaTime;
	}
}
