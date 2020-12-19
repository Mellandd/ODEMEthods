package interpolation;

import ode.InitialValueProblem;
import ode.NumericalSolutionPoint;

public class EulerMethodInterpolator {
    private double time;
    private double[] state, derivative;

    public EulerMethodInterpolator(InitialValueProblem problem, NumericalSolutionPoint point) {
        time = point.getTime();
        state = point.getState();
        derivative = problem.getDerivative(time, state);
    }
    
    public EulerMethodInterpolator(double time, double[] state, double[] derivative) {
        this.time  = time;
        this.state = state;
        this.derivative = derivative;
    }
    
    public double getState(double time, int index) {
        double step = time - this.time;
        return state[index] + step*derivative[index];
    }
    
    public double[] getState(double time) {
        double[] interpolation = new double[state.length];
        double step = time - this.time;
        for (int i=0; i<state.length; i++) {
            interpolation[i] = state[i] + step*derivative[i];
        }
        return interpolation;
    }
}
