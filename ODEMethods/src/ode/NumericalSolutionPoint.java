package ode;

import java.util.Arrays;
import java.util.stream.IntStream;

public class NumericalSolutionPoint {
	private double time;
	private double[] state;
	
	public NumericalSolutionPoint(double time, double[] state) {
		this.time = time;
		this.state = Arrays.copyOf(state, state.length);
	}
	
	public double getTime() {
		return time;
	}
	
	public double[] getState() {
		return Arrays.copyOf(state, state.length);
	}
	
	public double getState(int index) {
		return state[index];
	}
	
	public void print() {
		System.out.println("Point at t="+time+" = (" + state[0]);
		IntStream.range(1, state.length)
			.forEach(s -> System.out.println(", "+ state[s]));
	}

}
