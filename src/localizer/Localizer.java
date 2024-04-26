package localizer;

import java.awt.Graphics;
import java.util.ArrayList;

import utils.Pose2d;
import utils.CubicSpline;

public class Localizer {
	public ArrayList<Double> t;
	public ArrayList<Pose2d> l;
	public ArrayList<Pose2d> relativeMovement;
	
	public Localizer() {
		l = new ArrayList<Pose2d>();
		t = new ArrayList<Double>();
		relativeMovement = new ArrayList<Pose2d>();
	}
	
	public Localizer(CubicSpline p,int n) {//path with n subdivisions
		l = new ArrayList<Pose2d>();
		t = new ArrayList<Double>();
		relativeMovement = new ArrayList<Pose2d>();
		l.add(p.getPose2d(0.0));
		t.add(0.0);
	}
	
	public void update(CubicSpline p,int n) {
		l.clear();
		t.clear();
		relativeMovement.clear();
		l.add(p.getPose2d(0.0));
		t.add(0.0);
	}
	
	public void adjust(Pose2d p) {
		for (int i = 0; i < l.size(); i ++) {
			l.set(i, l.get(i).add(p));
		}
	}
	public void draw(int height, Graphics g) {
		int x1 = (int)l.get(0).x;
		int y1 = (int)l.get(0).y;
		for (int i = 1; i < l.size(); i ++) {
			int x2 = (int)l.get(i).x;
			int y2 = (int)l.get(i).y;
			g.drawLine(x1, height-y1, x2, height-y2);
			x1 = x2;
			y1 = y2;
		}
	}
	public Pose2d delta(double t1) {
		int min = 0;
		int max = t.size()-1;
		while (min <= max) {
			int mid = (min + max)/2;
			if (t.get(mid) == t1) {
				return l.get(mid);
			}
			else if (max == min) {
				break;
			}
			else if (t.get(mid) > t1) {
				max = mid-1;
			}
			else {
				min = mid+1;
			}
		}
		//cannot find the exact value;
		//we must do a linear approximation
		if (t.get(max) > t1) {
			min --;
		}
		else {
			max ++;
		}
		double a = t.get(min);
		double b = t.get(max);
		double w1 = (t1-a)/(b-a);
		double w2 = (b-t1)/(b-a);
		return new Pose2d(
				w1*l.get(min).x + w2*l.get(max).x,
				w1*l.get(min).y + w2*l.get(max).y,
				w1*l.get(min).heading + w2*l.get(max).heading
				);
	}
}
