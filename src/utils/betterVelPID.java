package utils;

import java.util.LinkedList;
import java.util.Queue;

public class betterVelPID {
	public double p,i,d,integral=0,lastErr=0,sumVel=0;
	public long lastTime;
	Queue<Double> vel;
	public betterVelPID (double p, double i, double d) {
		this.p = p;
		this.i = i;
		this.d = d;
		lastTime = (long) (System.nanoTime()-1E6);
		vel = new LinkedList();
		for (int q = 0; q < 10; q ++) {
			vel.add(0.0);
		}
	}
	public double update(double err) {
		long currTime = System.nanoTime();
		double loop = (currTime-lastTime)/1.0E9;
		double deriv = (err-lastErr)/loop;
		sumVel += deriv;
		sumVel -= vel.remove();
		vel.add(deriv);
		deriv=sumVel/vel.size();
		integral += err*loop;
		lastTime = currTime;
		return p*err + i*integral + d*deriv;
	}
}
