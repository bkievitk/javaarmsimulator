package com.armsim.my3DShapes;

import java.awt.image.BufferedImage;

import com.armsim.my3DCore.Material;



/**
 * Create a zero thickness wall.
 * @author bkievitk
 */

public class Wall3D extends Object3D {
	
	private static final long serialVersionUID = 92347860012L;

	/**
	 * Generate a wall that is broken into pieces.
	 * @param ax		Starting point X value.
	 * @param ay		Starting point Y value.
	 * @param bx		Ending point X value.
	 * @param by		Ending point Y value.
	 * @param height	Height of wall.
	 * @param parts		Number of division to make wall up of.
	 * @param material	Material to make wall of.
	 */
	public Wall3D(double ax, double ay, double bx, double by, double height, int parts, Material material) {
		
		// Build each wall piece.
		for(int i=0;i<parts;i++) {
			double newAX = ax + (bx - ax) * i / parts;
			double newBX = ax + (bx - ax) * (i + 1) / parts;
			
			double newAY = ay + (by - ay) * i / parts;
			double newBY = ay + (by - ay) * (i + 1) / parts;
			
			addWall(newAX,newAY,newBX,newBY,height,material);
		}
	}
	
	/**
	 * Generate a single block of wall.
	 * @param ax		Starting point X value.
	 * @param ay		Starting point Y value.
	 * @param bx		Ending point X value.
	 * @param by		Ending point Y value.
	 * @param height	Height of wall.
	 * @param material	Material to make wall of.
	 */
	public Wall3D(double ax, double ay, double bx, double by, double height, Material material) {
		addWall(ax, ay, bx, by, height, material);
	}
	
	/**
	 * Generate a wall that is broken into pieces.
	 * @param a			Starting point values as array pair.
	 * @param b			Ending point values as array pair.
	 * @param height	Height of wall.
	 * @param parts		Number of division to make wall up of.
	 * @param material	Material to make wall of.
	 */
	public Wall3D(double[] a, double[] b, double height, Material material, BufferedImage img) {
		addWall(a[0],a[1],b[0],b[1],height,material);
	}
	
	/**
	 * Actually generate a single block of wall.
	 * @param ax		Starting point X value.
	 * @param ay		Starting point Y value.
	 * @param bx		Ending point X value.
	 * @param by		Ending point Y value.
	 * @param height	Height of wall.
	 * @param material	Material to make wall of.
	 */
	public void addWall(double ax, double ay, double bx, double by, double height, Material material) {
		int start = points.size();

		double[] newPoint = new double[3];
		newPoint[0] = ay;
		newPoint[1] = -height;
		newPoint[2] = ax;
		points.add(newPoint);
		ptNormals.add(null);

		newPoint = new double[3];
		newPoint[0] = ay;
		newPoint[1] = 0;
		newPoint[2] = ax;
		points.add(newPoint);
		ptNormals.add(null);

		newPoint = new double[3];
		newPoint[0] = by;
		newPoint[1] = -height;
		newPoint[2] = bx;
		points.add(newPoint);
		ptNormals.add(null);

		newPoint = new double[3];
		newPoint[0] = by;
		newPoint[1] = 0;
		newPoint[2] = bx;
		points.add(newPoint);
		ptNormals.add(null);
		

		addTriangle(start,0,1,3,material.newImage(true,true,false,true));
		addTriangle(start,3,2,0,material.newImage(false,false,false,true));		
		addTriangle(start,3,1,0,material.newImage(false,false,false,false));
		addTriangle(start,0,2,3,material.newImage(false,false,false,false));		
	}
	
}
