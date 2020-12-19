package ode;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class NumericalSolution {
	private List<NumericalSolutionPoint> pointList;
	
	public NumericalSolution() {
		pointList = new ArrayList<>();
	}
	
	public NumericalSolution(InitialValueProblem problem) {
		pointList = new ArrayList<>();
		pointList.add(new NumericalSolutionPoint(problem.getInitialTime(), problem.getInitialState()));
	}
	
	public NumericalSolutionPoint add(double time, double state[]) {
		NumericalSolutionPoint point = new NumericalSolutionPoint(time, state);
		if (pointList.add(point)) return point;
		return null;
	}
	
	public double getInitialTime() {
		return getFirstPoint().getTime();
	}
	
	public double getLastTime() {
		return getLastPoint().getTime();
	}
	
	public NumericalSolutionPoint getFirstPoint() {
		return pointList.get(0);
	}
	
	public NumericalSolutionPoint getLastPoint() {
		return pointList.get(pointList.size() - 1);
	}
	
	public double getFirstStep() {
		if (pointList.size() <= 1 ) return Double.NaN;
		NumericalSolutionPoint secondPoint = pointList.get(1);
		return secondPoint.getTime() - getInitialTime();
	}
	
	public void removeLast(int n) {
		int size = pointList.size();
		n = Math.min(n,size);
		pointList.subList(size - n, size).clear();
	}
	
	public Iterator<NumericalSolutionPoint> iterator() {
		return pointList.iterator();
	}
	
	public Iterator<NumericalSolutionPoint> iterator(int n){
		int size = pointList.size();
		return pointList.subList(size - n, size).iterator();
	}
	

}
