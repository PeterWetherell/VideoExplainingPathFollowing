package localizer;

import utils.Pose2d;
import utils.CubicSpline;

public class ConstVelLocalizer extends Localizer {
	@Override
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
			double rh = h2-h1;
			if (Math.abs(rh)/(t2-t1) < 1E-4) {
				h1 = (h1+h2)/2.0;
				l.add(new Pose2d(
						last.x + rx*Math.cos(h1) - ry*Math.sin(h1),
						last.y + ry*Math.cos(h1) + rx*Math.sin(h1),
						h2
						));
				t.add(t1);
			}
			else {
				double r1 = rx/rh;
				double r2 = ry/rh;
				for (double a = t1; a < t2; a += 1E-3) {
					double m = (a-t1)/(t2-t1);
					rx = r1*Math.sin(m*(h2-h1)) - r2*(1-Math.cos(m*(h2-h1)));
					ry = r2*Math.sin(m*(h2-h1)) + r1*(1-Math.cos(m*(h2-h1)));
					l.add(new Pose2d(
							last.x + rx*Math.cos(h1) - ry*Math.sin(h1),
							last.y + ry*Math.cos(h1) + rx*Math.sin(h1),
							h2
							));
					t.add(a);
				}
				rx = r1*Math.sin(rh) - r2*(1-Math.cos(rh));
				ry = r2*Math.sin(rh) + r1*(1-Math.cos(rh));
			}
			Pose2d next = new Pose2d(
					last.x + rx*Math.cos(h1) - ry*Math.sin(h1),
					last.y + ry*Math.cos(h1) + rx*Math.sin(h1),
					h2
					);
			last = next;
			t1 = t2;
		}
		l.add(last);
		t.add(t1);
	}
	
	public static void main(String[] args) {
		CubicSpline s = new CubicSpline(new Pose2d(100,155,Math.toRadians(0)),new Pose2d(300,205,Math.toRadians(0)));
		Localizer l = new ConstVelLocalizer();
		for (int i = 1; i < 1024; i *= 2) {
			l.update(s, i);
			System.out.println(i + ", " + l.l.get(l.l.size()-1).getDist(s.getPose2d(1)));
		}
		//This shows that arc localization is also about O(h^2)
	}
}
