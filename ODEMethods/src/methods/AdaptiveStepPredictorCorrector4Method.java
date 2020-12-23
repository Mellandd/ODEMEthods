package methods;

import ode.InitialValueProblem;

public class AdaptiveStepPredictorCorrector4Method extends AdaptiveStepMethod{
    static public final int STEPS = 4;

    protected boolean mustRestart=true;
    protected double[] predictorState, correctorState;
    protected double[] auxState; // Required by the RK starter
    protected double[]   times       = new double[STEPS-1];   // Times taken at restart
    protected double[][] states      = new double[STEPS-1][]; // ordered 2 = (i-2), 1 = (i-1) , 0 = i
    protected double[][] derivatives = new double[STEPS][];   // ordered 3 = (i-3), 2 = (i-2), 1 = (i-1) , 0 = i


    public AdaptiveStepPredictorCorrector4Method(InitialValueProblem problem, double step, double tolerance) {
        super(problem,step,tolerance);
        currentStep = step;
        minimumStepAllowed = step/1.0e6;
        predictorState = problem.getInitialState();
        correctorState = problem.getInitialState();
        for (int i=0; i<states.length; i++) states[i] = problem.getInitialState();
        auxState = problem.getInitialState();
    }


	@Override
    public double doStep(double deltaTime, double time, double[] state) {
        while (Math.abs(currentStep)>=minimumStepAllowed) {
            double h24 = currentStep/24.0;
            double currentTime=time;
            double[] currentState=state;
            if (mustRestart) {
                restartMethod(time, state);
                currentTime  = times[0];
                currentState = states[0];
            }
            // Predictor: 4-steps Adams-Bashford 
            super.addToEvaluationCounter(1);
            derivatives[0] = problem.getDerivative(currentTime, currentState);
            for  (int i=0; i<state.length; i++) {
                predictorState[i] = currentState[i] + h24 * ( 55*derivatives[0][i] - 59*derivatives[1][i] + 37*derivatives[2][i] -9*derivatives[3][i]);
            }
            // Corrector: 3-steps Adams-Moulton 
            super.addToEvaluationCounter(1);
            double[] derivativeIp1 = problem.getDerivative(currentTime+currentStep, predictorState);
            for (int i=0; i<state.length; i++) {
                correctorState[i] = currentState[i] + h24 * ( 9*derivativeIp1[i] + 19*derivatives[0][i] -5*derivatives[1][i] + derivatives[2][i]);
            }
            double norm = 0;
            for (int i=0; i<state.length; i++) {
                double diffInIndex = correctorState[i]-predictorState[i];
                norm += diffInIndex*diffInIndex;
            }
            norm = Math.sqrt(norm);
            double error = 19.0*norm/270.0;
            double maxErrorAllowed = tolerance*Math.abs(currentStep);
            if (error<maxErrorAllowed) {
                time = currentTime + currentStep;
                System.arraycopy(correctorState,0,state,0,state.length);
                stepList.add(currentStep);
                if (mustRestart) { // Add the starting steps
                    // This is somewhat unique. The first doStep() method that adds points to the solution by itself
                    for (int i=states.length-1; i>=0; i--) getSolution().add(times[i], states[i]);
                    //System.out.println ("FOUR POINTS ADDED from  t = "+mTimes[2]+ " to t =  "+time);
                }
                
                if (error < maxErrorAllowed*0.1) { // error is really small --> adapt step
                    if (norm < 1.0e-16) { // Prevent division by zero
                        currentStep = 2 * currentStep;
                    } 
                    else {
                        double q = 1.5*Math.pow(maxErrorAllowed/norm, 0.25);
                        q = Math.min(4, q); // Do not grow too much
                        //System.out.print ("ACCEPTED: t = "+time+ " Old step is "+mCurrentStep+ " error = "+error);
                        currentStep *= q;
                        //System.out.println ("  New step is "+mCurrentStep+" state= "+state[0]);
                    }   
                    mustRestart = true;
                }
                else {
                    //System.out.println ("ACCEPTED: t = "+time+ " with step "+mCurrentStep+ " error = "+error);
                    for (int i=derivatives.length-1; i>0; i--) // Prepare next step
                        System.arraycopy(derivatives[i-1],0,derivatives[i],0,state.length);
                    mustRestart = false;
                }
                return time;
            }
            // Try a new smaller step
            double q = 1.5*Math.pow(maxErrorAllowed/norm, 0.25);
            q = Math.max(q, 0.1); // Do not shrink too much
            currentStep *= q;
            //System.out.println ("REJECTED: t = "+time+ " New step is "+mCurrentStep+ " error = "+error);
            mustRestart = true;
        }
        // Was not able to reach tolerance before going below mMinimumStepAllowed
        return Double.NaN; 
    }
	
    protected void restartMethod(double time, double[] state) {
        //System.out.println ("Restarting RK: t = "+time+ " with step "+mCurrentStep+" state= "+state[0]);
        derivatives[3] = problem.getDerivative(time, state);
        times[2] = rungeKuttaStep(currentStep, time, state, states[2], derivatives[3]);
        
        derivatives[2] = problem.getDerivative(times[2], states[2]);
        times[1] = rungeKuttaStep(currentStep, times[2], states[2], states[1], derivatives[2]);
        
        derivatives[1] = problem.getDerivative(times[1], states[1]);
        times[0] = rungeKuttaStep(currentStep, times[1], states[1], states[0], derivatives[1]);
        // Yes, one could write this using a for loop...
    }
    
    protected double rungeKuttaStep(double deltaTime, double time, double[] state, double[] newState, double[] k1) {
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
            newState[i] = state[i] + h6 * (k1[i]+2*k2[i]+2*k3[i]+k4[i]);
        }
        return time+deltaTime;
    }
}
