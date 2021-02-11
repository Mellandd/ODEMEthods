package problems;

import methods.AdaptiveStepMethod;
import methods.AdaptiveStepRK4Method;
import methods.AdaptiveStepRKFehlbergMethod;
import methods.FixedStepEulerMethod;
import methods.FixedStepMethod;
import methods.FixedStepModifiedEulerMethod;
import methods.FixedStepRungeKutta4Method;
import ode.Event;
import ode.InitialValueProblem;
import ode.SolutionObject;
import tools.DisplaySolution;

public class LotkaVolterra implements InitialValueProblem{
	
	private double a = 0.1;
	private double b = 0.2;
	private double c = 0.2;
	private double d = 0.05;

	@Override
	public double getInitialTime() {
		return 0;
	}

	@Override
	public double[] getInitialState() {
		double f = 0;
		double x0 = (1-f)*0.25;
		double y0 = (1-f)*0.15;
		return new double[] { x0 , a*x0-b*x0*y0, y0,-c*y0+d*x0*y0  };
	}

	@Override
	public double[] getDerivative(double t, double[] x) {
        return new double[] { x[1], 
        		a*x[1] - b*(x[1]*x[2] + x[0]*x[3]), //Derivamos x'(t)
                x[3],
                -c*x[3] + d*(x[1]*x[2]+ x[0]*x[3]) // Derivamos y'(t)
        };
	}
	
    static public double calculateIntegral(SolutionObject solution, double initialTime, double finalTime, double step, int index) {
    	double valor = 0;
    	double actualValue = solution.getState(initialTime, index);
    	double time = initialTime;
    	while(time < finalTime) {
    		valor += actualValue*step;
    		time += step;
    		actualValue = solution.getState(time, index);
    	}
    	return valor;
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
			return function.getState(time, 1);
		}
    	
    }
	
	public static void main(String[] args) {
		double tolerance = 1.0e-6;
		InitialValueProblem problem = new LotkaVolterra();
		FixedStepMethod method = new FixedStepModifiedEulerMethod(problem, 1.0e-2);
		FixedStepMethod method2 = new FixedStepRungeKutta4Method(problem, 1.0e-2);
		FixedStepMethod method3 = new FixedStepEulerMethod(problem, 1.0e-2);
		AdaptiveStepMethod method4 = new AdaptiveStepRKFehlbergMethod(problem, 1.0e-2, tolerance);

		SolutionObject solution = method.solveInterval(0, 200);
		SolutionObject solution2 = method2.solveInterval(0, 200);
		SolutionObject solution3 = method3.solveInterval(0, 200);
		SolutionObject solution4 = method4.solveInterval(0, 200);

	
        System.out.println("Evaluaciones: "+ method.getEvaluationCounter());
        System.out.println("Evaluaciones: "+ method2.getEvaluationCounter());
        System.out.println("Evaluaciones: "+ method3.getEvaluationCounter());
        System.out.println("Evaluaciones: "+ method4.getEvaluationCounter());


		Event1 event = new Event1(tolerance, solution2);
		
		event.calculateCrossing();
		// Obtenemos T = 66.842
		
		double t = 66.842;
		
		//Calculamos las dos integrales.

		double int1 = (1./t)*LotkaVolterra.calculateIntegral(solution, 0, t, 1e-3, 0);
		double int2 = (1./t)*LotkaVolterra.calculateIntegral(solution, 0, t, 1e-3, 2);
		
		System.out.println("Primera integral: " +int1);
		System.out.println("Segunda integral: " + int2);
		
		System.out.println("Valor en int1 "+solution2.getState(int1, 0));
		System.out.println("Valor en 2*int1 " + solution2.getState(int1, 0));
		
		System.out.println("Valor en int2 "+solution2.getState(int2, 2));
		System.out.println("Valor en 2*int2 " + solution2.getState(int2, 2));
		
        DisplaySolution.timePlot(method4.getSolution(),new int[]{0});

        
        // 
	}
}