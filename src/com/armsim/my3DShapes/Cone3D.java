package com.armsim.my3DShapes;

import com.armsim.my3DCore.Material;

/**
 * A Cone object.
 * @author bkievitk
 */

public class Cone3D extends Object3D {
	
	private static final long serialVersionUID = 92347860003L;

	public Cone3D(double[] center, double radius, double height, int divs, Material material) {
		addCone(center, radius, height, divs, material);
	}
	
	public void addCone(double[] center, double radius, double height, int divs, Material material) {
		int start = points.size();
		
		// Add points.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = center[0];
			newPoint[0] = center[1];
			newPoint[1] = -height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = Math.sin(Math.PI * 2 / divs * i);
			newPointNormal[0] = Math.cos(Math.PI * 2 / divs * i);
			newPointNormal[1] = -1;
			ptNormals.add(newPointNormal);
		}
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI * 2 / divs * i) * radius + center[0];
			newPoint[0] = Math.cos(Math.PI * 2 / divs * i) * radius + center[1];
			newPoint[1] = height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = Math.sin(Math.PI * 2 / divs * i);
			newPointNormal[0] = Math.cos(Math.PI * 2 / divs * i);
			newPointNormal[1] = -1;
			ptNormals.add(newPointNormal);
		}
		
		// Add top middle.
		double[] newPoint = new double[3];
		newPoint[2] = center[0];
		newPoint[0] = center[1];
		newPoint[1] = height / 2 + center[2];
		points.add(newPoint);
		ptNormals.add(null);
		
		// Add triangle sides.
		for(int i=0;i<divs;i++) {
			addTriangle(start,(i+1)%divs,i,i+divs,material);
			addTriangle(start,divs+(i+1)%divs,(i+1)%divs,i+divs,material);
		}
		
		// Add triangle top and bottom.
		for(int i=0;i<divs;i++) {
			addTriangle(start,2*divs,divs+(i+1)%divs,(i+divs),material);
		}
		
	}
}
