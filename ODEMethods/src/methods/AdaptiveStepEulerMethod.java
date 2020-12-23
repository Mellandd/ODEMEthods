/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package methods;

import ode.InitialValueProblem;

public class AdaptiveStepEulerMethod extends AdaptiveStepMethod {
    private double[] halfStepState;
    private double[] fullStepState; 
    
    public AdaptiveStepEulerMethod(InitialValueProblem problem, double step, double tolerance) {
        super(problem,step,tolerance);
        halfStepState = problem.getInitialState();
        fullStepState = problem.getInitialState();
    }

    /**
     * Extrapolated Euler method implementation
     * @param deltaTime the step to take
     * @param time the current time
     * @param state the current state
     * @return the value of time of the step taken, state will contain the updated state
     */
    public double doStep(double deltaTime, double time, double[] state) {
        while (currentStep>=minimumStepAllowed) {
            double[] derivative = problem.getDerivative(time, state);
            super.addToEvaluationCounter(2);
            double halfStep = currentStep/2;
            for (int i=0; i<state.length; i++) {
                halfStepState[i] = state[i] + halfStep     * derivative[i];
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
            if (error<tolerance*currentStep) {
                for (int i=0; i<state.length; i++) {
                    //state[i] = mHalfStepState[i]; 
                    state[i] = 2*halfStepState[i] - fullStepState[i];
                }
                time += currentStep;
                // Adapt step
                if (error<1.0e-10) currentStep = 2*currentStep;
                else {
                    double q = 0.84*(tolerance*currentStep)/error;
                    currentStep *= q;
                }
                //System.out.println ("ACCEPTED: t = "+time+ " New step is "+mCurrentStep+ " error = "+error);
                return time;
            }
            // Try a new smaller step
            double q = 0.84*(tolerance*currentStep)/error;
            currentStep *= q;
            System.out.println ("REJECTED: t = "+time+ " New step is "+currentStep+ " error = "+error);
        }
        // Was not able to reach tolerance before going below mMinimumStepAllowed
        return Double.NaN; 
    }
    
}
