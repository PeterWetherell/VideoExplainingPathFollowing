package utils;

public class Pose2d {
	public double x, y, heading, velX, velY;
	
	public Pose2d (double x, double y, double heading, double velX, double velY) {
		this.x = x;
		this.y = y;
		this.heading = heading;
		this.velX = velX;
		this.velY = velY;
	}
	public Pose2d (double x, double y, double heading) {
		this(x,y,heading,0,0);
	}
	public Pose2d(double x, double y) {
		this(x,y,0);
	}
	public Pose2d clone() {
		return new Pose2d(x,y,heading,0,0);
	}
	public Pose2d sub(Pose2d p) {
		return new Pose2d(this.x-p.x,this.y-p.y,this.heading-p.heading);
	}
	public Pose2d add(Pose2d p) {
		return new Pose2d(this.x+p.x,this.y+p.y,this.heading+p.heading);
	}
	public double getDist(Pose2d p2) {
		return Math.sqrt(Math.pow(p2.x-x, 2)+Math.pow(p2.y-y, 2));
	}
	public void norm(double d) {
		double l = length();
		x *= d/l;
		y *= d/l;
	}
	public double length() {
		return Math.sqrt(x*x+y*y);
	}
	public void norm() {
		norm(1.0);
	}
	public String toString() {
		return "Pose2d[" + x + ", " + y + ", " + heading + "]";
	}
}
