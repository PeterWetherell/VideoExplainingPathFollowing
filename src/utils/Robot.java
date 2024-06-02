package utils;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;

import javax.swing.JFrame;

public class Robot {
	static boolean scalePower = true;
	//public Pose2d p, v, maxVel = new Pose2d(480,310,6), maxAccel = new Pose2d(1280,820,16);
	public Pose2d p, v, maxVel = new Pose2d(480,30,6), maxAccel = new Pose2d(1280,2000,32); // Tank
	public double scale = 1;
	long lastLoop = System.nanoTime();
	JFrame frame;
	
	public Robot(Pose2d Start, JFrame f) {
		frame = f;
		p = Start;
		p.heading = Math.toRadians(45);
		v = new Pose2d(0,0,0);
	}
	
	public void setPowers(double fwd, double str, double turn) {
		long currTime = System.nanoTime();
		double loopTime = (currTime-lastLoop)/1.0E9;
		lastLoop = currTime;
		
		double[] motorPowers = {fwd+str+turn,fwd-str+turn,fwd-str-turn,fwd+str-turn};
		if (!scalePower) {
			for (int i = 0; i < 4; i ++) {
				motorPowers[i] = Math.min(Math.max(motorPowers[i], -1), 1);
			}
		}
		else {
			double max = 1;
			for (int i = 0; i < 4; i ++) {
				max = Math.max(Math.abs(motorPowers[i]), max);
			}
			for (int i = 0; i < 4; i ++) {
				motorPowers[i] /= max;
			}
		}
		fwd = (motorPowers[0]+motorPowers[1]+motorPowers[2]+motorPowers[3])/4.0;
		str = (motorPowers[0]-motorPowers[1]-motorPowers[2]+motorPowers[3])/4.0;
		turn = (motorPowers[0]+motorPowers[1]-motorPowers[2]-motorPowers[3])/4.0;
		
		//System.out.println(fwd + " " + str + " " + turn);
		//System.out.println(loopTime);
		
		v.rotate(-p.heading); //make it a relative velocity
		
		v.heading -= p.heading;
		
		//System.out.println("Velocity: " + v);
		
		Pose2d accel = new Pose2d(
				maxAccel.x*fwd,
				maxAccel.y*str,
				maxAccel.heading*turn
				);
		
		v = v.add(accel.mult(loopTime));
		
		Pose2d friction = new Pose2d(
				 - maxAccel.x*Math.pow(v.x/maxVel.x,1),
				 - maxAccel.y*Math.pow(v.y/maxVel.y,1),
				 - maxAccel.heading*Math.pow(v.heading/maxVel.heading,1)
				);
		friction = friction.mult(loopTime);
		if (Math.abs(friction.x) > Math.abs(v.x)) {
			friction.x = -v.x;
		}
		if (Math.abs(friction.y) > Math.abs(v.y)) {
			friction.y = -v.y;
		}
		if (Math.abs(friction.heading) > Math.abs(v.heading)) {
			friction.heading = -v.heading;
		}
		
		v = v.add(friction);
		
		v.rotate(p.heading); //make it a global velocity
		v.heading += p.heading;
		
		p = p.add(v.mult(loopTime));
		if (Math.abs(p.heading) > Math.PI) {
			p.heading -= Math.signum(p.heading) * Math.PI * 2;
		}
	}
	
	public void update(Graphics g, JFrame f) {
		drawRobot(g);
	}
	
	public void drawRobot(Graphics g) {

		double robotWidth = 100;
		
		Graphics2D g2 = (Graphics2D) g;
		Stroke s = new BasicStroke((float)(scale*robotWidth/25.0),BasicStroke.CAP_SQUARE,BasicStroke.JOIN_MITER,10.0f);
		g2.setStroke(s);
		g2.setColor(Color.DARK_GRAY);

		double wheelWidth = scale*robotWidth/10;
		double wheelHeight = wheelWidth * 2;
		double wheelPosX = scale * robotWidth - wheelHeight - wheelWidth * 0.5;
		double wheelPosY = scale * robotWidth - wheelWidth * 1.5;
		drawRectangle(p,scale * robotWidth,scale * robotWidth,g2);
		double a = 1, b = 1;
		for (int i = 0; i < 4; i ++) {
			switch(i) {
			case(0): a = 1; b = 1; break;
			case(1): a = 1; b = -1; break;
			case(2): a = -1; b = -1; break;
			case(3): a = -1; b = 1; break;
			}
			drawRectangle(
				new Pose2d(
					p.x + a * wheelPosX/2 * Math.cos(p.heading) - b * wheelPosY/2 * Math.sin(p.heading),
					p.y + b * wheelPosY/2 * Math.cos(p.heading) + a * wheelPosX/2 * Math.sin(p.heading),
					p.heading
				),
				wheelWidth,
				wheelHeight,
				g2
			);
			drawOdo(g2,scale * robotWidth,wheelWidth);
		}
	}
	public void drawOdo(Graphics g, double robotWidth, double wheelWidth) {
		double a = Math.cos(p.heading) * wheelWidth/2.0;
		double b = Math.sin(p.heading) * wheelWidth/2.0;
		
		double r = (robotWidth - wheelWidth * 1.5)/2.0;
		
		Pose2d leftOdo = new Pose2d(
				p.x + (0) * Math.cos(p.heading) - r * Math.sin(p.heading),
				p.y + r * Math.cos(p.heading) + (0) * Math.sin(p.heading)
			);
		g.drawLine((int)(leftOdo.x - a),frame.getHeight()-(int)(leftOdo.y - b),(int)(leftOdo.x + a),frame.getHeight()-(int)(leftOdo.y + b));
		
		Pose2d rightOdo = new Pose2d(
				p.x + (0) * Math.cos(p.heading) - -1 * r * Math.sin(p.heading),
				p.y + -1 * r * Math.cos(p.heading) + (0) * Math.sin(p.heading)
			);
		g.drawLine((int)(rightOdo.x - a),frame.getHeight()-(int)(rightOdo.y - b),(int)(rightOdo.x + a),frame.getHeight()-(int)(rightOdo.y + b));
		
		Pose2d backOdo = new Pose2d(
				p.x + -1 * r * Math.cos(p.heading) - (0) * Math.sin(p.heading),
				p.y + (0) * Math.cos(p.heading) + -1 * r * Math.sin(p.heading)
			);
		g.drawLine((int)(backOdo.x + b),frame.getHeight()-(int)(backOdo.y - a),(int)(backOdo.x - b),frame.getHeight()-(int)(backOdo.y + a));
	}
	public void drawRectangle(Pose2d p, double width, double heignt, Graphics g) {
		double a = 1, b = 1, c = 1, d = 1;
		for (int i = 0; i < 4; i ++) {
			switch(i) {
			case(0): a = 1; b = 1; c = -1; d = 1; break;
			case(1): a = 1; b = -1; c = 1; d = 1;  break;
			case(2): a = -1; b = -1; c = 1; d = -1; break;
			case(3): a = -1; b = 1; c = -1; d = -1; break;
			}
			int x1 = (int) (p.x + a * heignt/2 * Math.cos(p.heading) - b * width/2 * Math.sin(p.heading));
			int y1 = (int) (p.y + b * width/2 * Math.cos(p.heading) + a * heignt/2 * Math.sin(p.heading));
			int x2 = (int) (p.x + c * heignt/2 * Math.cos(p.heading) - d * width/2 * Math.sin(p.heading));
			int y2 = (int) (p.y + d * width/2 * Math.cos(p.heading) + c * heignt/2 * Math.sin(p.heading));
			g.drawLine(x1,frame.getHeight()-y1,x2,frame.getHeight()-y2);
		}
	}
}
