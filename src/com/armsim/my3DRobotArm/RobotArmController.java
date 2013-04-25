package com.armsim.my3DRobotArm;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.LinkedList;
import java.util.Random;
import java.util.Vector;

import javax.swing.Timer;

public class RobotArmController {
	
	public RobotArm arm;
	public LinkedList<double[][]> waypoints = new LinkedList<double[][]>();
	public double jointSpeed = .01;
	public Random rand = new Random();
	
	public RobotArmController(RobotArm arm, int timeStep) {
		this.arm = arm;
	}
		
	/*
	public Vector<double[][]> aStar(double[][] start, double[][] stop) {
		
		Vector<double[][]> points = new Vector<double[][]>();
		for(int i=0;i<1000;i++) {
			points.add(randomPose());
		}
		points.add(start);
		points.add(stop);
		
		Hashtable<double[][],Double> g_score = new Hashtable<double[][],Double>();
		Hashtable<double[][],Double> h_score = new Hashtable<double[][],Double>();
		Hashtable<double[][],Double> f_score = new Hashtable<double[][],Double>();
		Hashtable<double[][],double[][]> came_from = new Hashtable<double[][],double[][]>();

		g_score.put(start, 0.0);
		h_score.put(start, angleDistance(start, stop));
		f_score.put(start, h_score.get(start));
		
		HashSet<double[][]> openSet = new HashSet<double[][]>();
		HashSet<double[][]> closedSet = new HashSet<double[][]>();
		openSet.add(start);
		
		while(openSet.size() > 0) {
			double min = Double.MAX_VALUE;
			double[][] x = null;
			for(double[][] w : openSet) {
				Double fScore = f_score.get(w);
				if(fScore != null && fScore < min) {
					min = fScore;
					x = w;
				}
			}
			
			if(x == null) {
				return null;
			}
			
			if(x.equals(stop)) {
				return reconstruct_path(came_from, x);
			}
			
			openSet.remove(x);
			closedSet.add(x);
			
			for(double[][] y : points) {
				if(closedSet.contains(y)) {
					continue;
				}
				double tentative_g_score = g_score.get(x) + angleDistance(x,y);
				boolean tentative_is_better = false;
				
				if(!openSet.contains(y)) {
					openSet.add(y);
					tentative_is_better = true;
				} else if(tentative_g_score < g_score.get(y)) {
					tentative_is_better = true;
				} else {
                    tentative_is_better = false;
				}

				if(tentative_is_better) {
	                came_from.put(y,x);
	                g_score.put(y, tentative_g_score);
	                h_score.put(y, angleDistance(y,stop));
	                f_score.put(y, g_score.get(y) + h_score.get(y));
				}
			}
		}
		
		return null;
	}
	
	public Vector<double[][]> reconstruct_path(Hashtable<double[][],double[][]> came_from, double[][] current_node) {
		if(came_from.containsKey(current_node)) {
			Vector<double[][]> p = reconstruct_path(came_from, came_from.get(current_node));
			p.add(current_node);
			return p;
		} else {
			Vector<double[][]> p = new Vector<double[][]>();
			p.add(current_node);
			return p;
		}
	}
	*/
}
