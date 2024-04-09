package localizer;

import utils.Pose2d;
import utils.cubicSpline;

public class ConstAccelLocalizer extends Localizer {
	public static final double fidelity = 1E-10;
	public ConstAccelLocalizer(cubicSpline p,int n) {//path with n subdivisions
		super(p,n);
		double t1 = 0;
		Pose2d last = l.get(0);
		double h1 = last.heading;
		double lastRX = 0;
		double lastRY = 0;
		double lastRH = 0;
		for (int i = 0; i < n; i++) {
			double t2 = ((double)i+1.0)/((double) n);
			
			double rx = p.getRelX(t2, t1);
			double ry = p.getRelY(t2, t1);
			double rh = p.getPose2d(t2).heading-h1;
			
			//derive the equations for deltaX and deltaY
			double vrx = (rx + lastRX)/2;
			double arx = 2*(rx-vrx);
			//v_x = vrx + arx*t
			double vry = (ry + lastRY)/2;
			double ary = 2*(ry-vry);
			//v_y = vry + ary*t
			double vrh = (rh + lastRH)/2;
			double arh = (rh-vrh);
			//h = h1 + vry*t + ary*t^2
			AdaptiveQuaderature x = new AdaptiveQuaderature(new double[] {vrx,arx},new double[] {h1,vrh,arh});
			AdaptiveQuaderature y = new AdaptiveQuaderature(new double[] {vry,ary},new double[] {h1,vrh,arh});
			
			Pose2d next = new Pose2d(
					last.x + x.evaluateCos(fidelity, 0, 1, 0) - y.evaluateSin(fidelity, 0, 1, 0),
					last.y + y.evaluateCos(fidelity, 0, 1, 0) + x.evaluateSin(fidelity, 0, 1, 0),
					h1 + rh
					);
			
			int xcosIndex = 0,ycosIndex = 0, xsinIndex = 0, ysinIndex = 0;
			for (double a = t1; a < t2; a += 1E-3) {
				double m = (a-t1)/(t2-t1);
				for (;x.cos.get(xcosIndex).time < m && xcosIndex < x.cos.size();xcosIndex ++) {}
				for (;x.sin.get(xsinIndex).time < m && xsinIndex < x.sin.size();xsinIndex ++) {}
				for (;y.cos.get(ycosIndex).time < m && ycosIndex < y.cos.size();ycosIndex ++) {}
				for (;y.sin.get(ysinIndex).time < m && ysinIndex < y.sin.size();ysinIndex ++) {}
				l.add(new Pose2d(
						last.x + x.cos.get(xcosIndex).val - y.sin.get(ysinIndex).val,
						last.y + y.cos.get(ycosIndex).val + x.sin.get(xsinIndex).val,
						h1 + rh
						));
				t.add(a);
			}
			
			l.add(next);
			t.add(t2);
			last = next;
			h1 = last.heading;
			lastRX = rx;
			lastRY = ry;
			lastRH = rh;
			t1 = t2;
		}
	}
	
	public static void main(String[] args) {
		/*
		AdaptiveQuaderature x = new AdaptiveQuaderature(
				new double[] {0.0682264334614,0.000408020108956},
				new double[] {0.0581560285465,-0.0182506740405,-0.000196256872469}
				);
		System.out.println(x.evaluateCos(1E-10, 0, 1,0));
		*/
		cubicSpline s = new cubicSpline(new Pose2d(100,155,Math.toRadians(0)),new Pose2d(300,205,Math.toRadians(0)));
		for (int i = 1; i < 1024; i *= 2) {
			Localizer l = new ConstAccelLocalizer(s,i);
			System.out.println(i + ", " + l.l.get(l.l.size()-1).getDist(s.getPose2d(1)));
		}
		//This shows that arc localization is about O(h^3)
	}
}
