package com.armsim.my3DShapes;

import com.armsim.my3DCore.Material;

/**
 * A square swiped through a half arc.
 * @author bkievitk
 */

public class Handle3D extends Object3D {

	private static final long serialVersionUID = 92347860007L;

	/**
	 * Create a handle object.
	 * @param center	Center of the inscribed circle.
	 * @param radius	Radius of the circle.
	 * @param height	Height of the handle.
	 * @param divs		Number of divisions.
	 * @param material	Material to make out of.
	 */
	public Handle3D(double[] center, double radius, double height, int divs, Material material) {
		addCylinder(center, radius, height, divs, material);
	}
	
	public void addCylinder(double[] center, double radius, double height, int divs, Material material) {
		int start = points.size();
		
		// Add points to top outer rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 2 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius + center[1];
			newPoint[1] = height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 2;
			newPointNormal[0] = Math.cos(Math.PI / (divs-1) * i) * radius;
			newPointNormal[1] = 0;
			ptNormals.add(newPointNormal);
		}
		
		// Add points to bottom outer rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 2 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius + center[1];
			newPoint[1] = -height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 2;
			newPointNormal[0] = Math.cos(Math.PI / (divs-1) * i) * radius;
			newPointNormal[1] = 0;
			ptNormals.add(newPointNormal);
		}

		// Add points to top inner rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 1.5 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius  * .5 + center[1];
			newPoint[1] = height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = -Math.sin(Math.PI / (divs-1) * i) * radius * 1.5;
			newPointNormal[0] = -Math.cos(Math.PI / (divs-1) * i) * radius * .5;
			newPointNormal[1] = 0;
			ptNormals.add(newPointNormal);
		}
		
		// Add points to bottom inner rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 1.5 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius * .5 + center[1];
			newPoint[1] = -height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = -Math.sin(Math.PI / (divs-1) * i) * radius * 1.5;
			newPointNormal[0] = -Math.cos(Math.PI / (divs-1) * i) * radius * .5;
			newPointNormal[1] = 0;
			ptNormals.add(newPointNormal);
		}
		
		// Add points to top outer rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 2 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius + center[1];
			newPoint[1] = height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = 0;
			newPointNormal[0] = 0;
			newPointNormal[1] = 1;
			ptNormals.add(newPointNormal);
		}
		
		// Add points to bottom outer rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 2 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius + center[1];
			newPoint[1] = -height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = 0;
			newPointNormal[0] = 0;
			newPointNormal[1] = -1;
			ptNormals.add(newPointNormal);
		}

		// Add points to top inner rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 1.5 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius  * .5 + center[1];
			newPoint[1] = height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = 0;
			newPointNormal[0] = 0;
			newPointNormal[1] = 1;
			ptNormals.add(newPointNormal);
		}
		
		// Add points to bottom inner rim.
		for(int i=0;i<divs;i++) {
			double[] newPoint = new double[3];
			newPoint[2] = Math.sin(Math.PI / (divs-1) * i) * radius * 1.5 + center[0] - radius;
			newPoint[0] = Math.cos(Math.PI / (divs-1) * i) * radius * .5 + center[1];
			newPoint[1] = -height / 2 + center[2];
			points.add(newPoint);
			
			double[] newPointNormal = new double[3];
			newPointNormal[2] = 0;
			newPointNormal[0] = 0;
			newPointNormal[1] = -1;
			ptNormals.add(newPointNormal);
		}
		
		for(int deg=0;deg<=1;deg++) {
			for(int scale = 1;scale<=2;scale++) {
				for(int hSign=-1;hSign<=1;hSign+=2) {
					double[] newPoint = new double[3];
					newPoint[2] = Math.sin(Math.PI * deg) * radius * 2 * scale / 2 + center[0] - radius; 
					newPoint[0] = Math.cos(Math.PI * deg) * radius * scale / 2 + center[1];
					newPoint[1] = hSign * height / 2 + center[2];
					points.add(newPoint);
					
					double[] newPointNormal = new double[3];
					newPointNormal[2] = -1;
					newPointNormal[0] = 0;
					newPointNormal[1] = 0;
					ptNormals.add(newPointNormal);
				}
			}
		}
		
		// Add sides
		addTriangle(start,divs*8+0,divs*8+1,divs*8+2,material);
		addTriangle(start,divs*8+3,divs*8+2,divs*8+1,material);
		addTriangle(start,divs*8+6,divs*8+5,divs*8+4,material);
		addTriangle(start,divs*8+5,divs*8+6,divs*8+7,material);
		
		// Add triangle outer sides.
		for(int i=0;i<divs-1;i++) {
			addTriangle(start,i+divs,i,(i+1),material);
			addTriangle(start,i+divs,i+1,divs+(i+1),material);
		}
		
		// Add triangle inner sides.
		for(int i=0;i<divs-1;i++) {
			addTriangle(start,(i+1)+divs*2,i+divs*2,i+divs+divs*2,material);
			addTriangle(start,divs+(i+1)+divs*2,(i+1)+divs*2,i+divs+divs*2,material);
		}
		
		
		// Add bottom triangles/
		for(int i=0;i<divs-1;i++) {
			addTriangle(start,i+divs*4,i+divs*6,(i+1)+divs*4,material);
			addTriangle(start,i+divs*6,(i+1)+divs*6,(i+1)+divs*4,material);
		}
		
		// Add top triangles/
		for(int i=0;i<divs-1;i++) {
			addTriangle(start,(i+1)+divs*5,i+divs*7,i+divs*5,material);
			addTriangle(start,(i+1)+divs*5,(i+1)+divs*7,i+divs*7,material);
		}

	}
}
