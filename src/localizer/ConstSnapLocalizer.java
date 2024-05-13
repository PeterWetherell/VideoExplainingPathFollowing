package localizer;

import utils.Pose2d;
import utils.CubicSpline;

public class ConstSnapLocalizer extends Localizer {
	public static final double fidelity = 1E-10;
	
	static double[] times = {-3,-2,-1,0,1};
	static double[] q = {24,-6,4,-6,24};
	static double[] j = {
			2/q[0],
			3/q[1],
			4/q[2],
			5/q[3],
			6/q[4]
	};
	static double[] a = {
			-1/q[0],
			-1/q[1],
			1/q[2],
			5/q[3],
			11/q[4]
			
	};
	static double[] v = {
			-2/q[0],
			-3/q[1],
			-6/q[2],
			-5/q[3],
			6/q[4]
			
	};
	
	@Override
	public void update(CubicSpline p,int n) {//path with n subdivisions
		super.update(p,n);
		double t1 = 0;
		Pose2d last = l.get(0);
		double h1 = last.heading;
		double l1RH = 0, l1RX = 0, l1RY = 0;
		double l2RH = 0, l2RX = 0, l2RY = 0;
		double l3RH = 0, l3RX = 0, l3RY = 0;
		
		for (int i = 0; i < n; i++) {
			double t2 = ((double)i+1.0)/((double) n);
			
			double rx = p.getRelX(t2, t1);
			double ry = p.getRelY(t2, t1);
			double rh = p.getPose2d(t2).heading-h1;
			//derive the equations for deltaX and deltaY
			double[] rxv = {-1*(l3RX+l2RX+l1RX), -1*(l2RX+l1RX), -1*(l1RX), 0, rx};
			double vrx = (rxv[0]*v[0]+rxv[1]*v[1]+rxv[2]*v[2]+rxv[3]*v[3]+rxv[4]*v[4]);
			double arx = (rxv[0]*a[0]+rxv[1]*a[1]+rxv[2]*a[2]+rxv[3]*a[3]+rxv[4]*a[4]);
			double jrx = (rxv[0]*j[0]+rxv[1]*j[1]+rxv[2]*j[2]+rxv[3]*j[3]+rxv[4]*j[4]);
			double srx = (rx-vrx-arx-jrx);
			//double srx = rxv[0]/q[0]+rxv[1]/q[1]+rxv[2]/q[2]+rxv[3]/q[3]+rxv[4]/q[4];
			//v_x = vrx + arx*t
			double[] ryv = {-1*(l3RY+l2RY+l1RY), -1*(l2RY+l1RY), -1*(l1RY), 0, ry};
			double vry = (ryv[0]*v[0]+ryv[1]*v[1]+ryv[2]*v[2]+ryv[3]*v[3]+ryv[4]*v[4]);
			double ary = (ryv[0]*a[0]+ryv[1]*a[1]+ryv[2]*a[2]+ryv[3]*a[3]+ryv[4]*a[4]);
			double jry = (ryv[0]*j[0]+ryv[1]*j[1]+ryv[2]*j[2]+ryv[3]*j[3]+ryv[4]*j[4]);
			double sry = (ry-vry-ary-jry);
			//double sry = ryv[0]/q[0]+ryv[1]/q[1]+ryv[2]/q[2]+ryv[3]/q[3]+ryv[4]/q[4];
			//v_y = vry + ary*t
			double[] rhv = {-1*(l3RH+l2RH+l1RH), -1*(l2RH+l1RH), -1*(l1RH), 0, rh};
			double vrh = (rhv[0]*v[0]+rhv[1]*v[1]+rhv[2]*v[2]+rhv[3]*v[3]+rhv[4]*v[4]);
			double arh = (rhv[0]*a[0]+rhv[1]*a[1]+rhv[2]*a[2]+rhv[3]*a[3]+rhv[4]*a[4]);
			double jrh = (rhv[0]*j[0]+rhv[1]*j[1]+rhv[2]*j[2]+rhv[3]*j[3]+rhv[4]*j[4]);
			double srh = (rh-vrh-arh-jrh);
			//double srh = rhv[0]/q[0]+rhv[1]/q[1]+rhv[2]/q[2]+rhv[3]/q[3]+rhv[4]/q[4];
			//h = h1 + vry*t + ary*t^2
			
			//System.out.println(rxv[0] + " " + rxv[1] + " " + rxv[2] + " " + rxv[3]+ " " + rxv[4]);
			//System.out.println(vrx + " " + arx + " " + jrx + " " + srx);
			AdaptiveQuaderature x = new AdaptiveQuaderature(new double[] {vrx,2.0*arx,3.0*jrx,4.0*srx},new double[] {h1,vrh,arh,jrh,srh});
			AdaptiveQuaderature y = new AdaptiveQuaderature(new double[] {vry,2.0*ary,3.0*jry,4.0*sry},new double[] {h1,vrh,arh,jrh,srh});
			
			Pose2d next = new Pose2d(
					last.x + x.evaluateCos(fidelity, 0, 1, 0) - y.evaluateSin(fidelity, 0, 1, 0),
					last.y + y.evaluateCos(fidelity, 0, 1, 0) + x.evaluateSin(fidelity, 0, 1, 0),
					h1 + rh
					);
			
			int xcosIndex = 0,ycosIndex = 0, xsinIndex = 0, ysinIndex = 0;
			for (double z = t1; z < t2; z += 1E-3) {
				double m = (z-t1)/(t2-t1);
				for (;x.cos.get(xcosIndex).time < m && xcosIndex < x.cos.size();xcosIndex ++) {}
				for (;x.sin.get(xsinIndex).time < m && xsinIndex < x.sin.size();xsinIndex ++) {}
				for (;y.cos.get(ycosIndex).time < m && ycosIndex < y.cos.size();ycosIndex ++) {}
				for (;y.sin.get(ysinIndex).time < m && ysinIndex < y.sin.size();ysinIndex ++) {}
				l.add(new Pose2d(
						last.x + x.cos.get(xcosIndex).val - y.sin.get(ysinIndex).val,
						last.y + y.cos.get(ycosIndex).val + x.sin.get(xsinIndex).val,
						h1 + rh
						));
				t.add(z);
			}
			
			l.add(next);
			t.add(t2);
			last = next;
			h1 = last.heading;
			l3RX = l2RX; l3RY = l2RY; l3RH = l2RH;
			l2RX = l1RX; l2RY = l1RY; l2RH = l1RH;
			l1RX = rx; l1RY = ry; l1RH = rh;
			t1 = t2;
		}
	}
	
	public static void main(String[] args) {

		double[] test = {-5, -6, -2, 0, 3};
		double testa = (test[0]*a[0]+test[1]*a[1]+test[2]*a[2]+test[3]*a[3]+test[4]*a[4]);
		double testj = (test[0]*j[0]+test[1]*j[1]+test[2]*j[2]+test[3]*j[3]+test[4]*j[4]) * -1;
		double tests = (test[0]/q[0]+test[1]/q[1]+test[2]/q[2]+test[3]/q[3]+test[4]/q[4]);
		double testv = (test[4]-tests-testa-testj);
		System.out.println(testv + " " + testa + " " + testj + " " + tests);
		
		
		CubicSpline s = new CubicSpline(new Pose2d(100,155,Math.toRadians(0)),new Pose2d(300,205,Math.toRadians(0)));
		Localizer l = new ConstSnapLocalizer();
		for (int i = 1; i < 1024; i *= 2) {
			l.update(s, i);
			System.out.println(i + ", " + l.l.get(l.l.size()-1).getDist(s.getPose2d(1)));
		}
		//This shows that arc localization is about O(h^4)
	}
}
