package com.armsim.my3DRobotArm;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.Vector;


public class AStar {
		
	public static Vector<AStarObject> aStar(AStarObject start, AStarObject stop, int verbosity) {
		
		Hashtable<AStarObject,Double> g_score = new Hashtable<AStarObject,Double>();
		Hashtable<AStarObject,Double> h_score = new Hashtable<AStarObject,Double>();
		Hashtable<AStarObject,Double> f_score = new Hashtable<AStarObject,Double>();
		Hashtable<AStarObject,AStarObject> came_from = new Hashtable<AStarObject,AStarObject>();

		g_score.put(start, 0.0);
		h_score.put(start, start.distance(stop));
		f_score.put(start, h_score.get(start));
		
		HashSet<AStarObject> openSet = new HashSet<AStarObject>();
		HashSet<AStarObject> closedSet = new HashSet<AStarObject>();
		openSet.add(start);
		
		while(openSet.size() > 0) {

			if(verbosity >= 8) { System.out.println("Starting round."); }
			if(verbosity >= 10) { System.out.println("Goal " + stop); }
			
			double min = Double.MAX_VALUE;
			AStarObject x = null;
			for(AStarObject w : openSet) {
				Double fScore = f_score.get(w);
				if(fScore != null && fScore < min) {
					min = fScore;
					x = w;
				}
			}
			
			if(verbosity >= 5) { System.out.println("Min f of " + min + " found for object " + x); }
			
			double newDistance = x.distance(stop);
			if(verbosity >= 3) { System.out.println("Distance to goal of selected point " + newDistance); }
			
			if(x == null) {
				return null;
			}
			
			if(x.equals(stop)) {
				if(verbosity >= 4) { System.out.println("At goal"); }
				return reconstruct_path(came_from, x);
			}
			
			openSet.remove(x);
			closedSet.add(x);
			
			for(AStarObject y : x.linksTo()) {
				
				if(verbosity >= 5) { System.out.println("Testing link out " + y); }
				
				if(closedSet.contains(y)) {
					continue;
				}
				
				double tentative_g_score = g_score.get(x) + .1;
				boolean tentative_is_better = false;
				
				//if(verbosity >= 5) { System.out.println("Tentative g score " + tentative_g_score); }
				
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
	                
					double newDistance2 = y.distance(stop);
					if(verbosity >= 5) { System.out.println("Distance to goal " + newDistance2); }
					

	                h_score.put(y, newDistance2);
	                f_score.put(y, g_score.get(y) + h_score.get(y));
				}
			}
		}
		
		return null;
	}
	
	private static Vector<AStarObject> reconstruct_path(Hashtable<AStarObject,AStarObject> came_from, AStarObject current_node) {
		if(came_from.containsKey(current_node)) {
			Vector<AStarObject> p = reconstruct_path(came_from, came_from.get(current_node));
			p.add(current_node);
			return p;
		} else {
			Vector<AStarObject> p = new Vector<AStarObject>();
			p.add(current_node);
			return p;
		}
	}
}
