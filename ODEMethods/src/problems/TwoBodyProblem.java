package problems;

import java.util.Arrays;

import interpolation.HermiteInterpolator;
import interpolation.StateFunction;
import methods.AdaptiveStepEulerMethod;
import methods.AdaptiveStepMethod;
import methods.AdaptiveStepPredictorCorrector4Method;
import methods.AdaptiveStepRK4Method;
import methods.AdaptiveStepRKFehlbergMethod;
import methods.FixedStepAdamsBashford4Method;
import methods.FixedStepBackwardEulerMethod;
import methods.FixedStepMethod;
import methods.FixedStepModifiedEulerMethod;
import methods.FixedStepPredictorCorrector4Method;
import methods.FixedStepRungeKutta4Method;
import ode.Event;
import ode.InitialValueProblem;
import ode.NumericalSolutionPoint;
import ode.SolutionObject;
import tools.BisectionMethod;
import tools.DisplaySequence;
import tools.DisplaySolution;


public class TwoBodyProblem implements InitialValueProblem {
    static private double sG = 8.6498928e-4;

    private double mM1 = 1988.5, mM2 = 0.0059724;
    private double[] initState = new double[] { 152.100533, 0.0 , 0.0, 0.105444 }; // x,vx,y,vy
    
    private double mConstant = sG * (mM1 + mM2);
    
    // ------------------
    // Implementation of InitialValueProblem
    // ------------------

    public double getInitialTime() { 
        return 0; 
    }
    
    public double[] getInitialState() { // x,vx, y,vy 
        return Arrays.copyOf(initState, initState.length);
    } 
    
    public double[] getDerivative(double t, double[] x) {
        double div  = Math.pow(x[0]*x[0]+x[2]*x[2],1.5);
        return new double[] { x[1], -mConstant * x[0] / div, 
                              x[3], -mConstant * x[2] / div };
    }

    // ------------------
    // End of implementation of InitialValueProblem
    // ------------------
    
    static private class Event1 extends Event {
    	
    	private double lastYearStart;
    	private double currentYear;
    	private double maxYears;

		public Event1(double tolerance, SolutionObject crossingFunction) {
			super(tolerance, crossingFunction);
			lastYearStart = 0;
			currentYear = 0;
			maxYears = 10;
		}

		@Override
		public void action(double time) {
			String label = function.getState(time, 3)>0 ? "Aphelium  " : "Perihelium";
            System.out.println (label+" at t="+time+", x = "+function.getState(time, 0));
			if (function.getState(time, 3)>0) {
	            double currentYearHours = time-lastYearStart;
	            currentYear++;
	            lastYearStart = time;
	            int days  = (int) Math.floor(currentYearHours/24.);
	            double hours = (currentYearHours - days*24.);
	            System.out.println ("====================================");
	            System.out.println ("YEAR: "+currentYear);
	            System.out.println ("Days : "+days+ " : Hours = " +hours);
	            System.out.println ("Days : "+currentYearHours/24.);
	            System.out.println ("Hours : "+currentYearHours);
	            System.out.println ("Total Hours : "+time);
	            System.out.println ("====================================");    
			}
		}

		@Override
		protected double crossingFunction(double time) {
			return function.getState(time, 2);
		}
    	
    }
    
    
    public static void main(String[] args) {
        double hStep = 1;
        double tolerance = 1.0e-10;

        InitialValueProblem problem = new TwoBodyProblem();
        FixedStepMethod method = new FixedStepAdamsBashford4Method(problem,1);
//        FixedStepMethod method = new AdaptiveStepRKFehlbergMethod(problem,10, tolerance);

//        method = new FixedStepPredictorCorrector4Method(problem,10);
//          FixedStepMethod method = new AdaptiveStepPredictorCorrector4Method(problem,hStep, tolerance);
//        FixedStepMethod method = new AdaptiveStepRKFehlbergMethod(problem,hStep, tolerance);

        
        SolutionObject solution = method.solveInterval(1, 100000);
        
        if (solution == null) {
        	System.out.println("Error, can't calculate the solution.");
        	return;
        }
        
        Event1 years = new Event1(tolerance, solution);
        double[] myTime = solution.getState(4365);
        System.out.println("Me devuelve x= " + myTime[0]);
        myTime = solution.getState(5000);
        System.out.println("Me devuelve en 4990 x= "+myTime[0] );
        
        
        years.calculateCrossing();
        
        
        System.out.println ("Evaluations ="+method.getEvaluationCounter());

        DisplaySolution.statePlot(method.getSolution(), 0, 2);
        if (method instanceof AdaptiveStepMethod) 
            DisplaySequence.plot(((AdaptiveStepMethod) method).getStepList());
    }
}