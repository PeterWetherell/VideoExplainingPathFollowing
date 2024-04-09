package utils;

import localizer.Localizer;

public class Follower {
	Path p = new Path();
	Localizer l;
	
	enum robotCase{
		waitAtStart,
		followTraj,
		drawRelativeLocalization,
		drawLocalization,
		increaseFidelity
	};
	
	public Follower(Localizer l) {
		this.l = l;
	}
	
	public void update() {
		
	}
}
