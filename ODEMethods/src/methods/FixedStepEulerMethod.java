package methods;

import java.util.Iterator;
import java.util.stream.IntStream;

import ode.InitialValueProblem;
import ode.NumericalSolution;
import ode.NumericalSolutionPoint;

public class FixedStepEulerMethod extends FixedStepMethod{

	public FixedStepEulerMethod(InitialValueProblem problem, double step) {
		super(problem, step);
	}
	
	@Override
	public double doStep(double deltaTime, double time, double[] state) {
        double[] derivative = problem.getDerivative(time, state);
        super.addToEvaluationCounter(1);
        IntStream.range(0, state.length).forEach(i->state[i] += deltaTime*derivative[i]);
        return time+deltaTime;	
    }
	
    static public NumericalSolution extrapolate (NumericalSolution fullStep, NumericalSolution halfStep) {
        NumericalSolution extrapolatedSolution = new NumericalSolution();
        Iterator<NumericalSolutionPoint> iteratorFull = fullStep.iterator();
        Iterator<NumericalSolutionPoint> iteratorHalf = halfStep.iterator();
        while (iteratorFull.hasNext() && iteratorHalf.hasNext()) {
            NumericalSolutionPoint pointFull = iteratorFull.next();
            double[] stateFull = pointFull.getState();
            double[] stateHalf = iteratorHalf.next().getState();
            IntStream.range(0, stateFull.length)
            	.forEach(i ->stateFull[i] = (2*stateHalf[i]-stateFull[i]));
            extrapolatedSolution.add(pointFull.getTime(), stateFull);
            if (!iteratorHalf.hasNext()) return extrapolatedSolution;
            iteratorHalf.next();
        }
        return extrapolatedSolution;
    }
    
    /**
     * Uses Richardson extrapolation for Euler method
     * @param problem
     * @param maxTime
     * @param tolerance
     * @param initialStep
     * @param minStepAllowed
     * @return 
     */
    static public NumericalSolution extrapolateToTolerance(InitialValueProblem problem, 
            double maxTime, double tolerance, 
            double initialStep, double minStepAllowed) {
        
        double h = initialStep;
        FixedStepEulerMethod methodFull = new FixedStepEulerMethod(problem,h);
        methodFull.solve(maxTime);
        NumericalSolution solutionFull = methodFull.getSolution();
        while (Math.abs(h)>Math.abs(minStepAllowed)) {
            System.out.println ("Trying for h = "+h+"...");
            FixedStepEulerMethod methodHalf = new FixedStepEulerMethod(problem,h/2);
            methodHalf.solve(maxTime);
            NumericalSolution solutionHalf = methodHalf.getSolution();
            double maxError = maxHalfStepError(solutionFull,solutionHalf);
            System.out.println ("- Error (for h/2) ~= "+maxError);
            if (maxError<tolerance) {
                System.out.println ("Tolerance reached for h = "+h+"\n");
                return extrapolate(solutionFull,solutionHalf);
            }
            h /= 2;
            solutionFull = solutionHalf;
        }
        return null;
    }
	
	

}
