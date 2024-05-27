package utils;

public class PID {
	public double p,i,d,integral = 0,lastErr = 0;
	public long lastTime;
	public PID (double p, double i, double d) {
		this.p = p;
		this.i = i;
		this.d = d;
		lastTime = (long) (System.nanoTime()-1E6);
	}
	public double update(double err) {
		long currTime = System.nanoTime();
		double loop = (currTime-lastTime)/1.0E9;
		double deriv = (err-lastErr)/loop;
		integral += err*loop;
		lastTime = currTime;
		return p*err + i*integral + d*deriv;
	}
}
