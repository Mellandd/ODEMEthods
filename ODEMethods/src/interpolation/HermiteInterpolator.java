package interpolation;

import ode.InitialValueProblem;
import ode.NumericalSolutionPoint;

public class HermiteInterpolator implements StateFunction{
    private double timeA, timeB, deltaTime;
    private double[] stateA, derivativeA;
    private double[] stateB, derivativeB;
    
    public HermiteInterpolator(InitialValueProblem problem, NumericalSolutionPoint pointA, 
            NumericalSolutionPoint pointB) {
        timeA = pointA.getTime();
        stateA = pointA.getState();
        derivativeA = problem.getDerivative(timeA, stateA);
        timeB = pointB.getTime();
        stateB = pointB.getState();
        derivativeB = problem.getDerivative(timeB, stateB);
        deltaTime = timeB - timeA;
    }
    
    public double getState(double time, int index) {
        double theta = (time - timeA) / deltaTime;
        double minus1 = theta - 1;
        double prod1 = theta * minus1;
        double prod2 = prod1 * (1 - 2 * theta);
        double coefX0 = -minus1 - prod2;
        double coefX1 = theta + prod2;
        double coefF0 = prod1 * minus1 * deltaTime;
        double coefF1 = prod1 * theta * deltaTime;
        return coefX0 * stateA[index] + coefX1 * stateB[index] + coefF0 * derivativeA[index] + coefF1 * derivativeB[index];
    }
    
    public double[] getState(double time) {
        double theta = (time - timeA) / deltaTime;
        double minus1 = theta - 1;
        double prod1 = theta * minus1;
        double prod2 = prod1 * (1 - 2 * theta);
        double coefX0 = -minus1 - prod2;
        double coefX1 = theta + prod2;
        double coefF0 = prod1 * minus1 * deltaTime;
        double coefF1 = prod1 * theta * deltaTime;
        
        double[] interpolation = new double[stateA.length];
        for (int i = 0; i < stateA.length; i++) {
            interpolation[i] = coefX0 * stateA[i] + coefX1 * stateB[i] + coefF0 * derivativeA[i] + coefF1 * derivativeB[i];
        }
        return interpolation;
    }
}
