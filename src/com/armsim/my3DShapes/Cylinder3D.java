package com.armsim.my3DShapes;

import com.armsim.my3DCore.Material;

/**
 * Creat a cylinder.
 * @author bkievitk
 */

public class Cylinder3D extends Object3D {

	private static final long serialVersionUID = 92347860005L;

	/**
	 * Create a cylinder.
	 * @param center	Center of cylinder.
	 * @param radius	Radius.
	 * @param height	Height
	 * @param divs		Number of divisions.
	 * @param material	Material.
	 */
	public Cylinder3D(double[] center, double radius, double height, int divs, Material material) {
		addCylinder(center, radius, height, divs, material);
	}
	
	public Cylinder3D(double x, double y, double z, double radius, double height, int divs, Material material, boolean onEnd) {
		double[] center = {x,y,z};
		addCylinder(center,radius,height,divs,material);		
	}
	
	public void addCylinder(double[] center, double radius, double height, int divs, Material material) {
		int start = points.size();
		
		// Add points.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI * 2 / divs * i) * radius + center[0];
			newPoint[0] = Math.cos(Math.PI * 2 / divs * i) * radius + center[1];
			newPoint[1] = height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = Math.sin(Math.PI * 2 / divs * i) * radius;
			newPointNormal[0] = Math.cos(Math.PI * 2 / divs * i) * radius;
			newPointNormal[1] = 0;
			ptNormals.add(newPointNormal);
		}
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI * 2 / divs * i) * radius + center[0];
			newPoint[0] = Math.cos(Math.PI * 2 / divs * i) * radius + center[1];
			newPoint[1] = -height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = Math.sin(Math.PI * 2 / divs * i) * radius;
			newPointNormal[0] = Math.cos(Math.PI * 2 / divs * i) * radius;
			newPointNormal[1] = 0;
			ptNormals.add(newPointNormal);
		}
		
		// Add top middle.
		double[] newPoint = new double[3];
		newPoint[2] = center[0];
		newPoint[0] = center[1];
		newPoint[1] = height / 2 + center[2];
		points.add(newPoint);
		ptNormals.add(null);

		// Add bottom middle.
		newPoint = new double[3];
		newPoint[2] = center[0];
		newPoint[0] = center[1];
		newPoint[1] = -height / 2 + center[2];
		points.add(newPoint);
		ptNormals.add(null);
		
		// Add triangle sides.
		for(int i=0;i<divs;i++) {
			addTriangle(start,i+divs,i,(i+1)%divs,material);
			addTriangle(start,i+divs,(i+1)%divs,divs+(i+1)%divs,material);
		}
		
		// Add triangle top and bottom.
		for(int i=0;i<divs;i++) {
			addTriangle(start,i,2*divs,(i+1)%divs,material);
			addTriangle(start,(i+divs),divs+(i+1)%divs,2*divs+1,material);
		}
		
	}
}
