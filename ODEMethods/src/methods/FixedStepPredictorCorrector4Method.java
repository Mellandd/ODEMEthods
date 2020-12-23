package methods;

import ode.InitialValueProblem;

public class FixedStepPredictorCorrector4Method extends FixedStepAdamsBashford4Method{

	protected double[] predictorState;
	
	public FixedStepPredictorCorrector4Method(InitialValueProblem problem, double step) {
		super(problem, step);
		predictorState = problem.getInitialState();
	}
	
    public double doStep(double deltaTime, double time, double[] state) {
        super.addToEvaluationCounter(1);
        switch(lastStep) {
            case 0 : 
                derivativeIm3 = problem.getDerivative(time, state);
                time = super.rungeKuttaStep(deltaTime, time, state, derivativeIm3); 
                lastStep++;
                break;
            case 1 : 
                derivativeIm2 = problem.getDerivative(time, state);
                time = super.rungeKuttaStep(deltaTime, time, state, derivativeIm2); 
                lastStep++;
                break;
            case 2 : 
                derivativeIm1 = problem.getDerivative(time, state);
                time = super.rungeKuttaStep(deltaTime, time, state, derivativeIm1); 
                lastStep++;
                break;
            default :
                derivativeI = problem.getDerivative(time, state);
                adamsBashfordStep(deltaTime, time, state, predictorState);
                time = adamsMoultonStep(deltaTime, time, state, state);
                System.arraycopy(derivativeIm2,0,derivativeIm3,0,derivativeIm2.length);
                System.arraycopy(derivativeIm1,0,derivativeIm2,0,derivativeIm2.length);
                System.arraycopy(derivativeI  ,0,derivativeIm1,0,derivativeIm2.length);
                break;
        }
        return time;
    }
    
    public double adamsMoultonStep(double deltaTime, double time, double[] state, double[] newState) {
        super.addToEvaluationCounter(1);
        double[] derivativeIp1 = problem.getDerivative(time, predictorState);
        double h24 = deltaTime/24.0;
        for (int i=0; i<state.length; i++) {
            newState[i] = state[i] + h24 * ( 9*derivativeIp1[i] + 19*derivativeI[i] -5*derivativeIm1[i] + derivativeIm2[i]);
        }
        return time+deltaTime;
    }

}
