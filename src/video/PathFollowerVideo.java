package video;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

import pathFollowers.PathFollower;
import utils.Pose2d;
import utils.CubicSpline;

public class PathFollowerVideo extends JPanel implements ActionListener{

	Timer timer;
	Font big = new Font("Courier New", 1, 50);
	Font small = new Font("Courier New", 1, 30);
	Font biggest = new Font("Courier New", 1, 90);
	JFrame frame;
	
	ArrayList<CubicSpline> path;
	long start = System.currentTimeMillis();
	PathFollower p;
	
	public PathFollowerVideo(PathFollower p) {
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
		path = new ArrayList<CubicSpline>();
		for (int i = 0; i < arr.length -1; i ++) {
			path.add(new CubicSpline(arr[i],arr[i+1]));
		}
		
		
		start = System.nanoTime();
		
		timer = new Timer(5,this);
		timer.start();
		
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setVisible(true);
		
		this.p = p;
		this.p.setup(path,arr[0], frame);
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
		g2.setColor(Color.LIGHT_GRAY);
		p.update(g2, frame);
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

}
