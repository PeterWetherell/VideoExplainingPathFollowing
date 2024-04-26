package localizer;

import utils.Pose2d;
import utils.cubicSpline;

public class ConstJerkLocalizer extends Localizer {
	public static final double fidelity = 1E-10;
	
	@Override
	public void update(cubicSpline p,int n) {//path with n subdivisions
		super.update(p,n);
		double t1 = 0;
		Pose2d last = l.get(0);
		double h1 = last.heading;
		double l1RH = 0, l1RX = 0, l1RY = 0;
		double l2RH = 0, l2RX = 0, l2RY = 0;
		for (int i = 0; i < n; i++) {
			double t2 = ((double)i+1.0)/((double) n);
			
			double rx = p.getRelX(t2, t1);
			double ry = p.getRelY(t2, t1);
			double rh = p.getPose2d(t2).heading-h1;
			
			//derive the equations for deltaX and deltaY
			double vrx = l2RX/-6.0+5.0/6.0*l1RX+rx/3.0;
			double arx = (-l2RX-l1RX+8.0*rx-6.0*vrx)/12.0;
			double jrx = (rx-vrx-arx);
			//v_x = vrx + arx*t
			double vry = l2RY/-6.0+5.0/6.0*l1RY+ry/3.0;
			double ary = (-l2RY-l1RY+8.0*ry-6.0*vry)/12.0;
			double jry = (ry-vry-ary);
			//v_y = vry + ary*t
			double vrh = l2RH/-6.0+5.0/6.0*l1RH+rh/3.0;
			double arh = (-l2RH-l1RH+8.0*rh-6.0*vrh)/12.0;
			double jrh = (rh-vrh-arh);
			//h = h1 + vry*t + ary*t^2
			
			//System.out.println(vrx+2*arx+3*jrx-rx);
			AdaptiveQuaderature x = new AdaptiveQuaderature(new double[] {vrx,2.0*arx,3.0*jrx},new double[] {h1,vrh,arh,jrh});
			AdaptiveQuaderature y = new AdaptiveQuaderature(new double[] {vry,2.0*ary,3.0*jry},new double[] {h1,vrh,arh,jrh});
			
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
			l2RX = l1RX; l2RY = l1RY; l2RH = l1RH;
			l1RX = rx; l1RY = ry; l1RH = rh;
			t1 = t2;
		}
	}
	
	public static void main(String[] args) {
		cubicSpline s = new cubicSpline(new Pose2d(100,155,Math.toRadians(0)),new Pose2d(300,205,Math.toRadians(0)));
		Localizer l = new ConstJerkLocalizer();
		for (int i = 1; i < 1024; i *= 2) {
			l.update(s, i);
			System.out.println(i + ", " + l.l.get(l.l.size()-1).getDist(s.getPose2d(1)));
		}
		//This shows that arc localization is about O(h^4)
	}
}
