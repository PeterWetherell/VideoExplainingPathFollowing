package localizer;

import utils.Pose2d;
import utils.cubicSpline;

public class LinearLocalizer extends Localizer {
	public LinearLocalizer(cubicSpline p,int n) {//path with n subdivisions
		super(p,n);
		double t1 = 0;
		Pose2d last = l.get(0);
		for (int i = 0; i < n; i++) {
			double t2 = ((double)i+1.0)/((double) n);
			double rx = p.getRelX(t2, t1);
			double ry = p.getRelY(t2, t1);
			double h1 = last.heading;
			double h2 = p.getPose2d(t2).heading;
			double h15 = (h1 + h2)/2.0;
			Pose2d next = new Pose2d(
					last.x + rx*Math.cos(h15) - ry*Math.sin(h15),
					last.y + ry*Math.cos(h15) + rx*Math.sin(h15),
					h2
					);
			last = next;
			l.add(next);
			t.add(t2);
			t1 = t2;
		}
	}
	
	public static void main(String[] args) {
		cubicSpline s = new cubicSpline(new Pose2d(100,155,Math.toRadians(0)),new Pose2d(300,205,Math.toRadians(0)));
		for (int i = 1; i < 1024; i *= 2) {
			Localizer l = new LinearLocalizer(s,i);
			System.out.println(i + ", " + l.l.get(l.l.size()-1).getDist(s.getPose2d(1)));
		}
		//This shows that linear localization is about O(h^2)
	}
}
