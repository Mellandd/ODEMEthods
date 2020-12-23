package methods;

import java.util.stream.IntStream;

import ode.InitialValueProblem;

public class AdaptiveStepEulerMethod  extends AdaptiveStepMethod{
    private double[] halfStepState;
    private double[] fullStepState; 
    
    public AdaptiveStepEulerMethod(InitialValueProblem problem, double step, double tolerance) {
    	super(problem, step, tolerance);
        halfStepState = problem.getInitialState();
        fullStepState = problem.getInitialState();
    }

	@Override
    public double doStep(double deltaTime, double time, double[] state) {
        while (Math.abs(currentStep)>=minimumStepAllowed) {
            double[] derivative = problem.getDerivative(time, state);
            super.addToEvaluationCounter(2);
            double halfStep = currentStep/2;
            for (int i=0; i<state.length; i++) {
                halfStepState[i] = state[i] + halfStep * derivative[i];
                fullStepState[i] = state[i] + currentStep * derivative[i];
            }
            derivative = problem.getDerivative(time+halfStep, halfStepState);
            double error = 0;
            for (int i=0; i<state.length; i++) {
                halfStepState[i] += halfStep * derivative[i];
                double errorInIndex = halfStepState[i]-fullStepState[i];
                error += errorInIndex*errorInIndex;
            }
            error = Math.sqrt(error);
            if (error<tolerance*Math.abs(currentStep)) {
            	IntStream.range(0,state.length).forEach(i->state[i] = 2*halfStepState[i] - fullStepState[i]);
                time += currentStep;
                if (error<tolerance) currentStep = 2*currentStep;
                else {
                    double q = 0.84*(tolerance*Math.abs(currentStep))/error;
                    currentStep *= q;
                }
                return time;
            }
            double q = 0.84*(tolerance*Math.abs(currentStep))/error;
            currentStep *= q;
            System.out.println ("REJECTED: t = "+time+ " New step is "+currentStep+ " error = "+error);
        }
        return Double.NaN; 
    }
}
