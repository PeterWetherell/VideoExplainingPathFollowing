package utils;

import java.util.ArrayList;

public class Path {
	public ArrayList<cubicSpline> splines;
	public Path () {
		splines.add(new cubicSpline(new Pose2d(100,175,Math.toRadians(0)),new Pose2d(300,225,Math.toRadians(0))));
		splines.add(new cubicSpline(new Pose2d(300,225,Math.toRadians(0)), new Pose2d(450,425,Math.toRadians(75))));
		splines.add(new cubicSpline(new Pose2d(450,425,Math.toRadians(75)), new Pose2d(650,525,Math.toRadians(65))));
		splines.add(new cubicSpline(new Pose2d(650,525,Math.toRadians(65)),new Pose2d(800,675,Math.toRadians(90))));
		splines.add(new cubicSpline(new Pose2d(800,675,Math.toRadians(90)),new Pose2d(600,825,Math.toRadians(135))));
	}
	
	public Pose2d getPose(double t) {
		if (t == splines.size()) {
			return splines.get(splines.size()-1).getPose2d(1);
		}
		double time = t - (int)t;
		return splines.get((int)t).getPose2d(time);
	}
}
