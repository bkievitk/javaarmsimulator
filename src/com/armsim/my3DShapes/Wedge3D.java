package com.armsim.my3DShapes;

import com.armsim.my3DCore.Material;

/**
 * A wedge shaped object.
 * @author bkievitk
 */

public class Wedge3D extends Object3D {
	
	private static final long serialVersionUID = 92347860013L;

	public Wedge3D(double[] center, double radius, Material material) {
		double[] size = {radius, radius, radius};
		addWedge(center,size,material);
	}

	public Wedge3D(double[] center, double[] size, Material material) {
		addWedge(center,size,material);
	}
		
	/**
	 * Add a wedge to this object context.
	 * @param center	Center of the wedge.
	 * @param size		Size of the wedge.
	 * @param material	Material to make out of.
	 */
	public void addWedge(double[] center, double[] size, Material material) {
		
		// Define all points for a cube since this is easier.
		int start = points.size();
		for(int x=0;x<=1;x++) {
			for(int y=0;y<=1;y++) {
				for(int z=0;z<=1;z++) {
					double[] newPoint = new double[3];
					newPoint[0] = center[0] + (x - .5) * size[0];
					newPoint[1] = center[1] + (y - .5) * size[1];
					newPoint[2] = center[2] + (z - .5) * size[2];
					points.add(newPoint);
					ptNormals.add(null);
				}
			}
		}
				
		// Define triangles for a wedge.
		addTriangle(start,2,6,0,material);		
		addTriangle(start,1,7,3,material);		
		addTriangle(start,0,3,2,material);
		addTriangle(start,0,1,3,material);		
		addTriangle(start,2,3,7,material);
		addTriangle(start,2,7,6,material);
		addTriangle(start,0,6,7,material);
		addTriangle(start,0,7,1,material);
	}
}
