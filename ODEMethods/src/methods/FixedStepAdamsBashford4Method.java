package methods;

import ode.InitialValueProblem;

public class FixedStepAdamsBashford4Method extends FixedStepMethod{
    protected int lastStep=0;
    protected double[] auxState;
    protected double[] derivativeIm3,derivativeIm2,derivativeIm1, derivativeI;
    
    public FixedStepAdamsBashford4Method(InitialValueProblem problem, double step) {
        super(problem,step);
        auxState = problem.getInitialState();
    }

	@Override
	public double doStep(double deltaTime, double time, double[] state) {
        super.addToEvaluationCounter(1);
        switch(lastStep) {
            case 0 : 
                derivativeIm3 = problem.getDerivative(time, state);
                time = rungeKuttaStep(deltaTime, time, state, derivativeIm3); 
                lastStep++;
                break;
            case 1 : 
                derivativeIm2 = problem.getDerivative(time, state);
                time = rungeKuttaStep(deltaTime, time, state, derivativeIm2); 
                lastStep++;
                break;
            case 2 : 
                derivativeIm1 = problem.getDerivative(time, state);
                time = rungeKuttaStep(deltaTime, time, state, derivativeIm1); 
                lastStep++;
                break;
            default :
                derivativeI = problem.getDerivative(time, state);
                time = adamsBashfordStep(deltaTime, time, state, state);
                System.arraycopy(derivativeIm2,0,derivativeIm3,0,derivativeIm2.length);
                System.arraycopy(derivativeIm1,0,derivativeIm2,0,derivativeIm2.length);
                System.arraycopy(derivativeI  ,0,derivativeIm1,0,derivativeIm2.length);
                break;
        }
        return time;
	}
	
    public double adamsBashfordStep(double deltaTime, double time, double[] state, double[] newState) {
        double h24 = deltaTime/24.0;
        for (int i=0; i<state.length; i++) {
            newState[i] = state[i] + h24 * ( 55*derivativeI[i] - 59*derivativeIm1[i] + 37*derivativeIm2[i] -9*derivativeIm3[i]);
        }
        return time+deltaTime;
    }
    
    protected double rungeKuttaStep(double deltaTime, double time, double[] state, double[] k1) {
        super.addToEvaluationCounter(3);
        double h2 = deltaTime/2.0;
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
