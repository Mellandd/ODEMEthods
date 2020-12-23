/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package methods;

import ode.InitialValueProblem;

public class AdaptiveStepRK4Method extends AdaptiveStepMethod {
    private double[] halfStepState;
    private double[] halfStepCompleteState;
    private double[] fullStepState; 
    
    public AdaptiveStepRK4Method(InitialValueProblem problem, double step, double tolerance) {
        super(problem,step,tolerance);
        halfStepState = problem.getInitialState();
        halfStepCompleteState = problem.getInitialState();
        fullStepState = problem.getInitialState();
    }

    public double doStep(double deltaTime, double time, double[] state) {
        while (Math.abs(currentStep)>=minimumStepAllowed) {
            double halfStep = currentStep/2;
            oneStep(currentStep, time,          state,          fullStepState);
            oneStep(halfStep,     time,          state,          halfStepState);
            oneStep(halfStep,     time+halfStep, halfStepState, halfStepCompleteState);
            
            double error = 0;
            for (int i=0; i<state.length; i++) {
                double errorInIndex = halfStepCompleteState[i]-fullStepState[i];
                error += errorInIndex*errorInIndex;
            }
            error = 16.0*Math.sqrt(error)/15.0;
            if (error<tolerance*Math.abs(currentStep)) {
                for (int i=0; i<state.length; i++) {
                    //state[i] = mHalfStepCompleteState[i]; 
                    state[i] = (16.0*halfStepCompleteState[i] - fullStepState[i])/15.0;
                }
                time += currentStep;
                stepList.add(currentStep);
                // Adapt step
                if (error<tolerance) currentStep = 2*currentStep;
                else {
                    double q = Math.pow((tolerance*Math.abs(currentStep))/(2.0*error),0.25);
                    q = Math.min(4, Math.max(q, 0.1));
                    currentStep *= q;
                }
                //System.out.println ("ACCEPTED: t = "+time+ " New step is "+mCurrentStep+ " error = "+error);
                return time;
            }
            // Try a new smaller step
            double q = Math.pow((tolerance*Math.abs(currentStep))/(2.0*error),0.25);
            q = Math.min(4, Math.max(q, 0.1));
            currentStep *= q;
            //System.out.println ("REJECTED: t = "+time+ " New step is "+currentStep+ " error = "+error);
        }
        // Was not able to reach tolerance before going below mMinimumStepAllowed
        return Double.NaN; 
    }
    
    
    private double oneStep(double deltaTime, double time, double[] state, double[] finalState) {
        super.addToEvaluationCounter(4);
        double h2 = deltaTime/2.0;
        double[] k1 = problem.getDerivative(time, state);
        for (int i=0; i<state.length; i++) {
            finalState[i] = state[i] + h2 * k1[i];
        }
        double[] k2 = problem.getDerivative(time+h2, finalState);
        for (int i=0; i<state.length; i++) {
            finalState[i] = state[i] + h2 * k2[i];
        }
        double[] k3 = problem.getDerivative(time+h2, finalState);
        for (int i=0; i<state.length; i++) {
            finalState[i] = state[i] + deltaTime * k3[i];
        }
        double[] k4 = problem.getDerivative(time+deltaTime, finalState);
        h2 = deltaTime/6;
        for (int i=0; i<state.length; i++) {
            finalState[i] = state[i] + h2 * (k1[i]+2*k2[i]+2*k3[i]+k4[i]);
        }
        return time+deltaTime;
    }
    
}
