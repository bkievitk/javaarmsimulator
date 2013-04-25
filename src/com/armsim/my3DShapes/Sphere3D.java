package com.armsim.my3DShapes;

import com.armsim.my3DCore.Material;

/**
 * Create a sphere object.
 * @author bkievitk
 */

public class Sphere3D extends Object3D {

	private static final long serialVersionUID = 92347860011L;

	/**
	 * Define sphere parameters.
	 * @param center	Center of sphere.
	 * @param radius	Radius of sphere.
	 * @param divsX		Divisions on the X axis.
	 * @param divsY		Divisions on the y axis.
	 * @param material	Material to make from.
	 */
	public Sphere3D(double[] center, double radius, int divsX, int divsY, Material material) {
		addSphere(center, radius, divsX, divsY, material);
	}
	
	public void addSphere(double[] center, double radius, int divsX, int divsY, Material material) {
		int start = points.size();
		
		// Add top middle.
		double[] newPoint = new double[3];
		newPoint[0] = center[0];
		newPoint[1] = center[1];
		newPoint[2] = radius + center[2];
		points.add(newPoint);
		
		double[] pointNormal = new double[3];
		pointNormal[0] = 0;
		pointNormal[1] = 0;
		pointNormal[2] = radius;
		ptNormals.add(pointNormal);

		// Add bottom middle.
		newPoint = new double[3];
		newPoint[0] = center[0];
		newPoint[1] = center[1];
		newPoint[2] = -radius + center[2];
		points.add(newPoint);
		
		pointNormal = new double[3];
		pointNormal[0] = 0;
		pointNormal[1] = 0;
		pointNormal[2] = -radius;
		ptNormals.add(pointNormal);
		
		// For each strip.
		for(int y=0;y<divsY;y++) {
			
			// For each part of the strip.
			for(int x=0;x<divsX;x++) {
				newPoint = new double[3];
				
				double z = (radius * 2) / (divsY + 1) * (y + 1) - radius;
				double len = Math.sqrt(radius * radius - z * z);
				newPoint[0] = Math.sin(Math.PI * 2 / divsX * x) * len + center[0];
				newPoint[1] = Math.cos(Math.PI * 2 / divsX * x) * len + center[1];
				newPoint[2] = z + center[2];
				points.add(newPoint);
				
				pointNormal = new double[3];
				pointNormal[0] = Math.sin(Math.PI * 2 / divsX * x) * len;
				pointNormal[1] = Math.cos(Math.PI * 2 / divsX * x) * len;
				pointNormal[2] = z;
				ptNormals.add(pointNormal);
			}
		}
				
		// For each strip.
		for(int x=0;x<divsX;x++) {
			
			// Add bottom and top.			
			addTriangle(	start,
							2 + x,
							2+((x + 1) % divsX),
							1,
							material);
			addTriangle(	start,
							2 + x + (divsY - 1) * divsX,
							0,
							2 + ((x + 1) % divsX) + (divsY - 1) * divsX,
							material);
			
			
			for(int y=0;y<divsY-1;y++) {
				addTriangle(	start,
								2 + x + (divsX) * (y),
								2 + ((x+1)%divsX) + (divsX) * (y+1),
								2 + ((x+1)%divsX) + (divsX) * (y),
								material);
				addTriangle(	start,
								2 + x + divsX * (y),
								2 + x + divsX * (y+1),
								2 + ((x+1)%divsX) + divsX * (y+1),
								material);
			}
		}
	}
}
