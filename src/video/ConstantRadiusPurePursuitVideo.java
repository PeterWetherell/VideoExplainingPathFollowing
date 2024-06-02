package video;

import pathFollowers.ConstantRadiusPurePursuit;
import pathFollowers.PathFollower;

public class ConstantRadiusPurePursuitVideo {
	public static void main(String[] args) {
		PathFollower p = new ConstantRadiusPurePursuit();
		PathFollowerVideo vid = new PathFollowerVideo(p);
	}
}
