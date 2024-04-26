package localizerDemonstrationVideo;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
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

import localizer.ConstAccelLocalizer;
import localizer.Localizer;
import utils.Pose2d;
import utils.Robot;
import utils.CubicSpline;
import utils.DrawUtil;

public class LocalizerDemostrationVideo extends JPanel implements MouseListener, ActionListener, KeyListener{

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
	CubicSpline[] path = new CubicSpline[5];
	long start = System.currentTimeMillis();
	robotCase rc;
	int fidelity = 1;
	boolean startPlay = false;
	double dLT, lastT = 0, t = 0;
	double currPoseTimer = 0;
	double globalError = 0, oneTimeError = 0;
	
	Localizer l;
	
	public LocalizerDemostrationVideo(Localizer l) {
		frame = new JFrame("VideoExplanation");
		frame.setSize(1600, 934);
		frame.add(this);
		
		Pose2d[] arr = {
				new Pose2d(100,155,Math.toRadians(0)),
				new Pose2d(300,205,Math.toRadians(0)),
				new Pose2d(450,405,Math.toRadians(75)),
				new Pose2d(650,445,Math.toRadians(65)),
				new Pose2d(800,555,Math.toRadians(90)),
				new Pose2d(600,705,Math.toRadians(135))
				};
		for (int i = 0; i < path.length; i ++) {
			path[i] = new CubicSpline(arr[i],arr[i+1]);
		}
		
		this.l = l;
		
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
        g2.setRenderingHint(RenderingHints.KEY_ALPHA_INTERPOLATION, RenderingHints.VALUE_ALPHA_INTERPOLATION_QUALITY);
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_COLOR_RENDERING, RenderingHints.VALUE_COLOR_RENDER_QUALITY);
        g2.setRenderingHint(RenderingHints.KEY_DITHERING, RenderingHints.VALUE_DITHER_ENABLE);
        g2.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
        g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g2.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
        g2.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
		
		Stroke s = new BasicStroke((float)(4),BasicStroke.CAP_SQUARE,BasicStroke.JOIN_MITER,10.0f);
		g2.setStroke(s);

		DrawUtil.drawAxis(g2,path[0].getPose2d(0),frame);
		
		g2.setColor(Color.LIGHT_GRAY);
		DrawUtil.drawLines(poseHistory,g2,frame);

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
					if (t > 6) {
						reset();
						rc = robotCase.trajDrawWait;
						l.update(path[i],n);
						currPoseTimer = 0;
						dLT = 0.5 + (l.l.get(0).getDist(odoHistory.get(odoHistory.size()-1)))/100.0;
						currentPose = path[i].getPose2d(1);
						poseHistory.add(currentPose);
						break;
					}
					if (t/6.0-currPoseTimer > 0.01) {
						currPoseTimer = t/6.0;
						currentPose = path[i].getPose2d(t/6.0);
						poseHistory.add(currentPose);
					}
					g2.setColor(Color.magenta);
					l.update(new CubicSpline(path[i],t/6.0),n);
					l.draw(frame.getHeight(),g2);
					oneTimeError = currentPose.getDist(l.l.get(l.l.size()-1));
					break;
				case trajDrawWait:
					g2.setColor(Color.magenta);
					l.draw(frame.getHeight(),g2);
					if (t > 0.5) {
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
			if (t > 1.5 && n < 100) {
				n *= 2;
				reset();
				odoHistory.clear();
				odoHistory.add(path[0].getPose2d(0));
				for (int k = 0; k < path.length; k ++) {
					l.update(path[k],n);
					Pose2d error = odoHistory.get(odoHistory.size()-1).sub(l.l.get(0));
					l.adjust(error);
					append(odoHistory,l.l);
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
		DrawUtil.drawLines(odoHistory,g2,frame);
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
