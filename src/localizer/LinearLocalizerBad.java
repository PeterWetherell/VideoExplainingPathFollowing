package localizer;

import utils.Pose2d;
import utils.CubicSpline;

public class LinearLocalizerBad extends Localizer {
	public void update(CubicSpline p,int n) {//path with n subdivisions
		super.update(p,n);
		double t1 = 0;
		Pose2d last = l.get(0);
		for (int i = 0; i < n; i++) {
			double t2 = ((double)i+1.0)/((double) n);
			double rx = p.getRelX(t2, t1);
			double ry = p.getRelY(t2, t1);
			double h1 = last.heading;
			double h2 = p.getPose2d(t2).heading;
			Pose2d next = new Pose2d(
					last.x + rx*Math.cos(h1) - ry*Math.sin(h1),
					last.y + ry*Math.cos(h1) + rx*Math.sin(h1),
					h2
					);
			last = next;
			l.add(next);
			t.add(t2);
			t1 = t2;
		}
	}
	
	public static void main(String[] args) {
		CubicSpline s = new CubicSpline(new Pose2d(100,155,Math.toRadians(0)),new Pose2d(300,205,Math.toRadians(0)));
		double last = 0;
		Localizer l = new LinearLocalizerBad();
		for (int i = 1; i <= 1024; i *= 2) {
			l.update(s, i);
			double curr = l.l.get(l.l.size()-1).getDist(s.getPose2d(1));
			System.out.println(i + ", " + last/curr + ", " + curr);
			last = curr;
		}
		//This shows that linear localization is about O(h^2)
	}
}
