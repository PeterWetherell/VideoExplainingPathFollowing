package video;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

import localizer.ConstVelLocalizer;
import localizer.Localizer;
import utils.Pose2d;
import utils.Robot;
import utils.cubicSpline;

public class ConstVelMath extends JPanel implements MouseListener, ActionListener, KeyListener{

	Timer timer;
	Font big = new Font("Courier New", 1, 50);
	Font small = new Font("Courier New", 1, 30);
	Font biggest = new Font("Courier New", 1, 90);
	JFrame frame;

	enum robotCase{
		waitAtStart,
		followTraj,
		trajDrawWait,
		drawLocalization;
	}
	
	int i = 0, n = 1;
	Robot r;
	Pose2d currentPose;
	ArrayList<Pose2d> poseHistory, odoHistory;
	cubicSpline[] path = new cubicSpline[5];
	long start = System.currentTimeMillis();
	robotCase rc;
	int fidelity = 1;
	boolean startPlay = false;
	double dLT, lastT = 0, t = 0;
	
	double currPoseTimer = 0;
	
	double globalError = 0, oneTimeError = 0;
	
	Localizer l;
	
	
	public static void main(String[] args) {
		ConstVelMath drive = new ConstVelMath();
	}
	
	public ConstVelMath() {
		frame = new JFrame("VideoExplanation");
		frame.setSize(1600, 934);
		frame.add(this);

		path[0] = new cubicSpline(new Pose2d(100,155,Math.toRadians(0)),new Pose2d(300,205,Math.toRadians(0)));
		path[1] = new cubicSpline(new Pose2d(300,205,Math.toRadians(0)), new Pose2d(450,405,Math.toRadians(75)));
		path[2] = new cubicSpline(new Pose2d(450,405,Math.toRadians(75)), new Pose2d(650,505,Math.toRadians(65)));
		path[3] = new cubicSpline(new Pose2d(650,505,Math.toRadians(65)),new Pose2d(800,655,Math.toRadians(90)));
		path[4] = new cubicSpline(new Pose2d(800,655,Math.toRadians(90)),new Pose2d(600,805,Math.toRadians(135)));
		
		r = new Robot(path[0].getPose2d(0));
		
		rc = robotCase.waitAtStart;
		
		start = System.nanoTime();
		
		poseHistory = new ArrayList<>();
		currentPose = path[0].getPose2d(0);
		odoHistory = new ArrayList<>();
		odoHistory.add(currentPose);
		
		timer = new Timer(15,this);
		timer.start();
		
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setVisible(true);
	}
	
	public void paint(Graphics g) {
		super.paintComponent(g);
		
		Graphics2D g2 = (Graphics2D) g;
		Stroke s = new BasicStroke((float)(4),BasicStroke.CAP_SQUARE,BasicStroke.JOIN_MITER,10.0f);
		g2.setStroke(s);

		drawAxis(g2);
		
		g2.setColor(Color.LIGHT_GRAY);
		drawLines(poseHistory,g2);

		t = (System.nanoTime() - start)/1.0E9;
		if (i != path.length) {
			switch (rc){
				case waitAtStart:
					if (t > 5) {
						reset();
						rc = robotCase.followTraj;
						break;
					}
					break;
				case followTraj:
					if (t > 3) {
						reset();
						rc = robotCase.trajDrawWait;
						l = new ConstVelLocalizer(path[i],n);
						currPoseTimer = 0;
						dLT = 0.25 + (l.l.get(0).getDist(odoHistory.get(odoHistory.size()-1)))/200.0;
						currentPose = path[i].getPose2d(1);
						poseHistory.add(currentPose);
						break;
					}
					if (t/3.0-currPoseTimer > 0.01) {
						currPoseTimer = t/3.0;
						currentPose = path[i].getPose2d(t/3.0);
						poseHistory.add(currentPose);
					}
					g2.setColor(Color.magenta);
					l = new ConstVelLocalizer(new cubicSpline(path[i],t/3.0),n);
					l.draw(frame.getHeight(),g2);
					oneTimeError = currentPose.getDist(l.l.get(l.l.size()-1));
					break;
				case trajDrawWait:
					g2.setColor(Color.magenta);
					l.draw(frame.getHeight(),g2);
					if (t > 0.25) {
						reset();
						rc = robotCase.drawLocalization;
					}
					break;
				case drawLocalization:
					Pose2d error = odoHistory.get(odoHistory.size()-1).sub(l.l.get(0));
					if (t > dLT) {
						reset();
						rc = robotCase.followTraj;
						i ++;
						l.adjust(error);
						append(odoHistory,l.l);
						break;
					}
					if (error.length() >= 1) {
						if (error.length() > 10) {
							error.norm(error.length()*(t-lastT)/(dLT-t));
						}
						l.adjust(error);
					}
					g2.setColor(Color.magenta);
					l.draw(frame.getHeight(),g2);
					globalError = currentPose.getDist(l.l.get(l.l.size()-1));
					break;
			}
		}
		else {
			if (t > 0.5 && n < 100) {
				n *= 2;
				reset();
				odoHistory.clear();
				odoHistory.add(path[0].getPose2d(0));
				for (int k = 0; k < path.length; k ++) {
					ConstVelLocalizer f = new ConstVelLocalizer(path[k],n);
					Pose2d error = odoHistory.get(odoHistory.size()-1).sub(f.l.get(0));
					f.adjust(error);
					append(odoHistory,f.l);
				}
				globalError = currentPose.getDist(odoHistory.get(odoHistory.size()-1));
			}
		}
		g2.setFont(small);
		g2.setColor(Color.black);
		int titleSize = 50;
		int fontSize = 30;
		int dist = 30;
		g2.drawString("Global Error: " + globalError, dist, titleSize);
		if (i < path.length) {
			g2.drawString("One Time Error: " + oneTimeError, dist, titleSize + fontSize);
			g2.drawString("Number of Itteratios: " + n, dist, titleSize + 2*fontSize);
		}
		else {
			g2.drawString("Number of Itteratios: " + n, dist, titleSize + fontSize);
		}
		
		
		lastT = t;
		g2.setColor(Color.magenta);
		drawLines(odoHistory,g2);
		g2.setColor(Color.black);
		r.update(frame.getHeight(),currentPose,g);
	}
	
	public void reset() {
		start = System.nanoTime();
		lastT = 0.0;
		t = 0.0;
	}
	
	public void append(ArrayList<Pose2d> p , ArrayList<Pose2d> q) {
		for (int i = 1; i < q.size(); i ++) {
			p.add(q.get(i));
		}
	}

	public void drawAxis(Graphics2D g2) {
		double yAxisLength = 500;
		double xAxisLength = 1300;
		Pose2d center = path[0].getPose2d(0);
		double size = 6;
		g2.setColor(Color.GRAY);
		g2.drawLine((int)(center.x), (int)(frame.getHeight()-center.y), (int)(center.x), (int)(frame.getHeight()-center.y - yAxisLength));
		Polygon p2 = new Polygon();
		p2.addPoint((int)(center.x - size), (int)(frame.getHeight()-center.y - yAxisLength));
		p2.addPoint((int)(center.x + size), (int)(frame.getHeight()-center.y - yAxisLength));
		p2.addPoint((int)(center.x), (int)(frame.getHeight()-center.y - yAxisLength - 2 * size));
		g2.fillPolygon(p2);
		
		g2.drawLine((int)(center.x), (int)(frame.getHeight()-center.y), (int)(center.x + xAxisLength), (int)(frame.getHeight()-center.y));
		Polygon p1 = new Polygon();
		p1.addPoint((int)(center.x + xAxisLength), (int)(frame.getHeight()-center.y - size));
		p1.addPoint((int)(center.x + xAxisLength), (int)(frame.getHeight()-center.y + size));
		p1.addPoint((int)(center.x + xAxisLength + 2 * size), (int)(frame.getHeight()-center.y));
		g2.fillPolygon(p1);
	}
	
	public void drawLines(ArrayList<Pose2d> p, Graphics2D g2) {
		for (int i = 1; i < p.size(); i ++) {
			g2.drawLine((int)p.get(i-1).x,(int)(frame.getHeight()-p.get(i-1).y),(int)p.get(i).x,(int)(frame.getHeight()-p.get(i).y));
		}
	}
	
	@Override
	public void actionPerformed(ActionEvent arg0) {
		repaint();
	
	}
	@Override
	public void mouseClicked(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void mousePressed(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void mouseReleased(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyTyped(KeyEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyPressed(KeyEvent e) {
		System.out.println(e.getKeyChar());
		if (e.getKeyChar() == ' ') {
			startPlay = true;
		}
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyReleased(KeyEvent e) {
		// TODO Auto-generated method stub
		
	}

}
