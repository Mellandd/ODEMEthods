package problems;

import java.util.Arrays;

import interpolation.StateFunction;
import methods.AdaptiveStepMethod;
import methods.FixedStepBackwardEulerMethod;
import methods.FixedStepMethod;
import methods.FixedStepRungeKutta4Method;
import methods.FixedStepTrapezoidalMethod;
import ode.InitialValueProblem;
import ode.SolutionObject;
import tools.DisplaySequence;
import tools.DisplaySolution;


@SuppressWarnings("unused")
public class Rigid1D implements InitialValueProblem {

    // ------------------
    // Implementation of InitialValueProblem
    // ------------------

    public double getInitialTime() { 
        return 0; 
    }
    
    public double[] getInitialState() { // x,vx, y,vy 
        return new double[] { -1.0 };
    } 
    
    public double[] getDerivative(double t, double[] x) {
        return new double[] { 5*Math.exp(5*t)*(x[0]-t)*(x[0]-t) + 1 };
    }
    
    public double[] getPartialDerivative(double t, double[] x) {
    	return new double[] { 10*Math.exp(5*t)*(x[0]-t)*getDerivative(t,x)[0] };
    }

    // ------------------
    // End of implementation of InitialValueProblem
    // ------------------

    static private class TrueSol implements StateFunction {
            
        public double[] getState(double time) {
            return new double[] { time - Math.exp(-5*time) };
        }
            
        public double getState(double time, int index) {
            return time - Math.exp(-5*time);
        }
                
    }


    public static void main(String[] args) {
        double maxTime = 1; // Paso fijo: probar 0.2 y 0.24 para t=6 y 8. RKF : probar 2.81 y 2.82, AdapPC4: 3.6 y 3.7
        InitialValueProblem problem = new Rigid1D();

        double hStep = 0.1; //
        double tolerance = 1.0e-4;
        int maxIter = 100;
        FixedStepMethod method = new FixedStepTrapezoidalMethod(problem, hStep, tolerance, maxIter);
        SolutionObject solution = method.solveInterval(0,maxTime);
        if (solution == null) {
            System.out.println ("Method broke!");
            return;
        }
        // MUESTRA DE QUE LA SALIDA DENSA FUNCIONA.
        
        System.out.println("x[0] en punto intermedio 0.35: " + solution.getState(0.35)[0]);
        
        DisplaySolution.listError(method.getSolution(), new TrueSol(), new int[]{0});
        if (method instanceof AdaptiveStepMethod) DisplaySequence.plot(((AdaptiveStepMethod) method).getStepList());
        System.out.println ("Evaluations ="+method.getEvaluationCounter());


    }
}