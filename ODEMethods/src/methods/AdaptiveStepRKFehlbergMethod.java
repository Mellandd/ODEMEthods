package methods;

import ode.InitialValueProblem;

public class AdaptiveStepRKFehlbergMethod extends AdaptiveStepMethod{

    private double[] mRK4; 
    private double[] mRK5; 
    private double[] mAux; 
    
    public AdaptiveStepRKFehlbergMethod(InitialValueProblem problem, double step, double tolerance) {
        super(problem,step,tolerance);
        mRK4 = problem.getInitialState();
        mRK5 = problem.getInitialState();
        mAux = problem.getInitialState();
    }

	@Override
	public double doStep(double deltaTime, double time, double[] state) {
        double[] k1 = problem.getDerivative(time, state);
        super.addToEvaluationCounter(1);
        while (Math.abs(currentStep)>=minimumStepAllowed) {
            oneStep(time, state, k1);
            
            double error = 0;
            for (int i=0; i<state.length; i++) {
                double errorInIndex = mRK5[i]-mRK4[i];
                error += errorInIndex*errorInIndex;
            }
            error = Math.sqrt(error);
            if (error<tolerance*currentStep) {
                for (int i=0; i<state.length; i++) {
                    state[i] = mRK5[i];
                }
                time += currentStep;
                stepList.add(currentStep);
                // Adapt step
                if (error<1.0e-10) currentStep = 2*currentStep;
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
            //System.out.println ("REJECTED: t = "+time+ " New step is "+mCurrentStep+ " error = "+error);
        }
        // Was not able to reach tolerance before going below minimumStepAllowed
        return Double.NaN; 
	}
	
    private void oneStep(double time, double[] state, double[] k1) {
        super.addToEvaluationCounter(5);
        for (int i=0; i<state.length; i++) {
            mAux[i] = state[i] + currentStep * (1.0/4.0 * k1[i]);
        }
        double[] k2 = problem.getDerivative(time+currentStep/4.0, mAux);
        
        for (int i=0; i<state.length; i++) {
            mAux[i] = state[i] + currentStep * (3.0/32.0 * k1[i] + 9.0/32.0 * k2[i]);
        }
        double[] k3 = problem.getDerivative(time+3.0/8.0*currentStep, mAux);
        
        for (int i=0; i<state.length; i++) {
            mAux[i] = state[i] + currentStep * (1932.0/2197.0 * k1[i] - 7200.0/2197.0 * k2[i] + 7296.0/2197.0 * k3[i]);
        }
        double[] k4 = problem.getDerivative(time+12.0/13.0*currentStep, mAux);

        for (int i=0; i<state.length; i++) {
            mAux[i] = state[i] + currentStep * (439.0/216.0 * k1[i] - 8.0 * k2[i] + 3680.0/513.0 * k3[i] - 845.0/4104.0 * k4[i]);
        }
        double[] k5 = problem.getDerivative(time+currentStep, mAux);

        for (int i=0; i<state.length; i++) {
            mAux[i] = state[i] + currentStep * ( - 8.0/27.0 * k1[i] + 2.0 * k2[i] - 3544.0/2565.0 * k3[i] + 1859.0/4104.0 * k4[i] - 11.0/40.0 * k5[i]);
        }
        double[] k6 = problem.getDerivative(time+1.0/2.0*currentStep, mAux);

        for (int i=0; i<state.length; i++) {
            mRK4[i] = state[i] + currentStep  * (25.0/216.0 * k1[i] + 1408.0/ 2565.0 * k3[i] + 2197.0 / 4104.0 * k4[i] - 1.0/5.0 * k5[i]);
            mRK5[i] = state[i] + currentStep  * (16.0/135.0 * k1[i] + 6656.0/12825.0 * k3[i] + 28561.0/56430.0 * k4[i] - 9.0/50.0 * k5[i] + 2.0/55.0  * k6[i]);
        }
    }
}
