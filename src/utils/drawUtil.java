package utils;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.Stroke;
import java.util.ArrayList;

import javax.swing.JFrame;

public class drawUtil {
	
	public static void drawAxis(Graphics2D g2, Pose2d center, JFrame frame) {
		double yAxisLength = 600;
		double xAxisLength = 700;
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
	
	public static void drawLines(ArrayList<Pose2d> p, Graphics2D g2, JFrame frame) {
		if (p.size() == 0) {
			return;
		}
		Stroke small = new BasicStroke(6.0f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_MITER,10.0f);
		g2.setStroke(small);
		Pose2d lastDraw = p.get(0);
		for (int i = 1; i < p.size(); i ++) {
			if (lastDraw.getDist(p.get(i)) > 5 || i == p.size()-1) {
				g2.drawLine((int)lastDraw.x,(int)(frame.getHeight()-lastDraw.y),(int)p.get(i).x,(int)(frame.getHeight()-p.get(i).y));
				lastDraw = p.get(i);
			}
		}
		
	}
}
