package interpolation;

import java.util.Arrays;

import methods.FixedStepMethod;
import ode.NumericalSolutionPoint;

public class FixedStepMethodInterpolator implements StateFunction{
	private FixedStepMethod method;
	private double time;
	private double[] state;
	
    public FixedStepMethodInterpolator(FixedStepMethod method, NumericalSolutionPoint point) {
        this.method = method;
        time   = point.getTime();
        state  = point.getState();
    }

	@Override
	public double[] getState(double time) {
        double[] interpolation = Arrays.copyOf(state,state.length);;
        method.doStep(time-this.time, time, interpolation);
        return interpolation;
	}

	@Override
	public double getState(double time, int index) {
        double[] interpolation = Arrays.copyOf(state,state.length);;
        method.doStep(time-this.time, time, interpolation);
        return interpolation[index];
	}

}
