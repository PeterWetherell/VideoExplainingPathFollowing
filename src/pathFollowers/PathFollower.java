package pathFollowers;

import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.swing.JFrame;

import utils.CubicSpline;
import utils.Pose2d;

public abstract class PathFollower {
	abstract public void setup(ArrayList<CubicSpline> s, Pose2d startPose, JFrame f);
	abstract public void update(Graphics2D g, JFrame j);
}
