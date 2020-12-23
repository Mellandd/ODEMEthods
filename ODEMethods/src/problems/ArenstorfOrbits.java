/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package problems;

import java.util.Arrays;

import interpolation.HermiteInterpolator;
import interpolation.StateFunction;
import methods.AdaptiveStepMethod;
import methods.AdaptiveStepPredictorCorrector4Method;
import methods.AdaptiveStepRK4Method;
import methods.AdaptiveStepRKFehlbergMethod;
import methods.FixedStepMethod;
import methods.FixedStepPredictorCorrector4Method;
import methods.FixedStepRungeKutta4Method;
import ode.Event;
import ode.InitialValueProblem;
import ode.NumericalSolutionPoint;
import ode.SolutionObject;
import tools.BisectionMethod;
import tools.DisplaySequence;
import tools.DisplaySolution;

/**
 * https://www.johndcook.com/blog/2020/02/08/arenstorf-orbit/
 */
public class ArenstorfOrbits implements InitialValueProblem {
    static private double sMu = 0.012277471;
    static private double sMuPrime = 1-sMu;
    static private double sPeriod = 17.0652165601579625588917206249;

    private double[] initState = new double[] { 0.994, 0.0 , 0.0, -2.00158510637908252240537862224 }; // x,vx,y,vy
        
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
        double D1 = Math.pow((x[0]+sMu)*(x[0]+sMu) + x[2]*x[2],1.5);
        double D2 = Math.pow((x[0]-sMuPrime)*(x[0]-sMuPrime) + x[2]*x[2],1.5);
        return new double[] { 
            x[1], 
            x[0] + 2*x[3] - sMuPrime*(x[0]+sMu)/D1 - sMu*(x[0]-sMuPrime)/D2,
            x[3], 
            x[2] - 2*x[1] - sMuPrime*x[2]/D1 - sMu*x[2]/D2,
        };
    }

    // ------------------
    // End of implementation of InitialValueProblem
    // ------------------

    static private double computeCrossing (InitialValueProblem problem, FixedStepMethod method,
            NumericalSolutionPoint fromPoint, NumericalSolutionPoint toPoint, 
            double tolerance, int index) {
//        StateFunction interpolator = new EulerMethodInterpolator(problem, fromPoint);
//        StateFunction interpolator = new FixedStepMethodInterpolator(method, fromPoint);
        StateFunction interpolator = new HermiteInterpolator(problem, fromPoint, toPoint);
        double zeroAt = BisectionMethod.findZero (interpolator, fromPoint.getTime(), toPoint.getTime(), tolerance, index);
        if (Double.isNaN(zeroAt)) {
            System.out.print ("Zero not found!!!");
        }
        else {
            double[] zeroState = interpolator.getState(zeroAt);
            System.out.println ("Zero at t="+zeroAt+", x="+zeroState[0]+", vx="+zeroState[1]+", y="+zeroState[2]+", vy="+zeroState[3]);
        }        
        return zeroAt;
    }
    
    static private class Event1 extends Event{

		public Event1(double tolerance, SolutionObject function) {
			super(tolerance, function);
		}

		@Override
		public void action(double time) {
			double[] zero = function.getState(time);
            System.out.println ("Zero at t="+time+", x="+zero[0]+", vx="+zero[1]+", y="+zero[2]+", vy="+zero[3]);			
		}

		@Override
		protected double crossingFunction(double time) {
			return function.getState(time, 2);
		}
    	
    }
    
    public static void main(String[] args) {
        double hStep = 1.0e-3;
        double tolerance = 1.0e-6;
        InitialValueProblem problem = new ArenstorfOrbits();
//        FixedStepMethod method = new FixedStepPredictorCorrector4Method(problem,hStep);
//        method = new AdaptiveStepPredictorCorrector4Method(problem,hStep, tolerance);
        FixedStepMethod method = new FixedStepRungeKutta4Method(problem,hStep);
        
//        FixedStepMethod method = new FixedStepModifiedEulerMethod(problem,hStep);
        
        
        SolutionObject solution = method.solveInterval(0, 20);
        if (solution == null) {
        	System.out.println("Error, can't calculate the solution.");
        	return;
        }
        
        Event1 event = new Event1(tolerance, solution);
        
        event.calculateCrossing();
        
//        NumericalSolutionPoint previousPoint, currentPoint;
//        
//        previousPoint = currentPoint = method.getSolution().getLastPoint();
//        double time = problem.getInitialTime();
//        while (time<sPeriod*2.3) {
//            previousPoint = currentPoint;
//            currentPoint = method.step();
//            if (currentPoint==null) {
//                System.out.println ("Method failed at t="+previousPoint.getTime()+" !!!");
//                System.exit(2);
//            }
//            if (currentPoint.getState(2)*previousPoint.getState(2)<0) {
//                computeCrossing(problem, method, previousPoint, currentPoint, 1.0e-6, 2);
//                // Crossed axis
//                //if (currentPoint.getTime()>sPeriod) break;
//            }
//            time = currentPoint.getTime();
//        }
//        previousPoint.print();
//        currentPoint.print();
        
        
        System.out.println ("Evaluations ="+method.getEvaluationCounter());

        DisplaySolution.statePlot(method.getSolution(), 0, 2,(int) Math.floor(1.0e-2/hStep));
        if (method instanceof AdaptiveStepMethod) 
            DisplaySequence.plot(((AdaptiveStepMethod) method).getStepList());
    }
}