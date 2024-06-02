package pathFollowers;

import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.swing.JFrame;

import utils.CubicSpline;
import utils.DrawUtil;
import utils.PID;
import utils.Pose2d;
import utils.Robot;

public class ConstantRadiusPurePursuitArcs extends PathFollower {
	Robot r;
	ArrayList<Pose2d> p;
	int lookAheadTarget;
	double pixelsPerPoint = 8;
	double radius = 200;
	
	PID fwd = new PID(0.03,0,0);
	PID str = new PID(0.06,0,0);
	PID turn = new PID(20, 0.0,0.0);
	
	@Override
	public void setup(ArrayList<CubicSpline> s, Pose2d startPose, JFrame f) {
		r = new Robot(startPose, f);
		p = new ArrayList<Pose2d>();
		p.add(s.get(0).getPose2d(0));
		for (int j = 0; j < s.size(); j ++) {
			for (double i = 0; i < 1; i += 0.01) { 
				if (s.get(j).getPose2d(i).getDist(p.get(p.size()-1)) > pixelsPerPoint) {
					p.add(s.get(j).getPose2d(i));
				}
			}
			p.add(s.get(j).getPose2d(1));
		}
	}

	boolean finishedPath = false;
	@Override
	public void update(Graphics2D g, JFrame j) {
		while (lookAheadTarget < p.size() && p.get(lookAheadTarget).getDist(r.p) < radius) {
			lookAheadTarget ++;
		}
		Pose2d target = p.get(Math.min(lookAheadTarget,p.size()-1)).clone();
		
		g.drawOval((int)target.x-10, (int)(j.getHeight() - target.y)-10, 20, 20);
		
		Pose2d error = target.sub(r.p);
		error.rotate(-r.p.heading);
		error.heading = Math.atan2(error.y,error.x);
		if (lookAheadTarget == p.size() && (p.get(p.size()-1).getDist(r.p) < radius/5.0 || finishedPath)) {
			error.heading = target.heading - r.p.heading;
			finishedPath = true;
		}
		while (Math.abs(error.heading) > Math.PI) {
			error.heading -= 2.0 * Math.PI * Math.signum(error.heading);
		}
		double fwdPow = fwd.update(error.x);
		double turnPow = turn.update(error.heading);
		
		r.setPowers(fwdPow, 0, turnPow);
		r.drawRobot(g);
		DrawUtil.drawLines(p,g,j);
	}
}
