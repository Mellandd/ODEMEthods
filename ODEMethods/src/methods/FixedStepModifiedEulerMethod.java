package methods;

import ode.InitialValueProblem;

public class FixedStepModifiedEulerMethod extends FixedStepMethod{
	
    private double[] auxState;
    
    public FixedStepModifiedEulerMethod(InitialValueProblem problem, double step) {
        super(problem,step);
        auxState = problem.getInitialState();
    }

	@Override
	public double doStep(double deltaTime, double time, double[] state) {
        super.addToEvaluationCounter(2);
        double h2 = deltaTime/2.0;
        double[] derivative = problem.getDerivative(time, state);
        for (int i=0; i<state.length; i++) {
            auxState[i] = state[i] + deltaTime * derivative[i];
        }
        double[] derivative2 = problem.getDerivative(time+deltaTime, auxState);
        for (int i=0; i<state.length; i++) {
            state[i] += h2 * (derivative[i]+derivative2[i]);
        }
        return time+deltaTime;
	}

}
