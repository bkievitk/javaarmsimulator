package com.armsim.my3DShapes;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.util.HashSet;
import java.util.Vector;

import com.armsim.my3DCore.Material;
import com.armsim.my3DCore.MatrixMath;
import com.armsim.my3DCore.TransformMy3D;
import com.armsim.my3DCore.Triangle3D;
import com.armsim.my3DCore.TriangleCollision;
import com.armsim.my3DCore.Universe;

import java.io.Serializable;

public abstract class Object3D implements Serializable {
	
	private static final long serialVersionUID = 1300466461134542262L;

	// Distance from the user to the screen.
	// This is used for perspective warp.
	public static final double SCREEN_DISTANCE = -50.0;
	
	// Axis to normalize to.
	private static final double[][] AXIS = {{0,0,1},{1,0,0},{1,1,1}};
	
	// Transform for this object.
	public TransformMy3D transform = new TransformMy3D();
	
	// Children objects take this transform and then their own.
	private Vector<Object3D> children = new Vector<Object3D>();
	private Object3D parent = null;
	
	// The points defining each triangle.
	protected Vector<int[]> triangles = new Vector<int[]>();
	protected Vector<Material> triangleMaterial = new Vector<Material>();
	
	// Points and their norms.
	protected Vector<double[]> points = new Vector<double[]>();
	protected Vector<double[]> ptNormals = new Vector<double[]>();

	// Define light.
	private double[] distLight_World = {.5,-.4,.5};
	private double[] distLight = new double[3];
		
	public void flipNormals() {
		//for(double[] ptNormal : ptNormals) {
		//	if(ptNormal != null) {
		//		for(int i=0;i<ptNormal.length;i++) {
		//			ptNormal[i] = -ptNormal[i];
		//		}
		//	}
		//}
		for(int[] triangle : triangles) {
			int tmp = triangle[0];
			triangle[0] = triangle[1];
			triangle[1] = tmp;
		}
	}

	/**
	 * Recursively show structure.
	 */
	public String toString() {
		return toString(0);
	}
	
	public void removeObject() {
		parent.children.remove(this);
	}

	public String showPoints() {
		String ret = "points (" + points.size() + "): ";
		for(double[] point : points) {
			ret += "[";
			for(double pt : point) {
				ret += "(" + pt + ")";
			}
			ret += "]";
		}
		ret += "\ntriangles (" + triangles.size() + "):";
		for(int[] triangle : triangles) {
			ret += "[";
			for(int tri : triangle) {
				ret += "(" + tri + ")";
			}
			ret += "]";
		}
		ret += "\n";
		return ret;
	}

	/**
	 * Show the object and it's descendents.
	 * @param depth
	 * @return
	 */
	public String toString(int depth) {
		String ret = "";
		for(int i=0;i<depth;i++) {
			ret += " ";
		}
		ret += "object\r\n";
		for(Object3D child : children) {
			ret += child.toString(depth+1);
		}
		return ret;
	}
	
	/**
	 * Get the set of all points given the current transform.
	 * @return
	 */
	public Vector<double[]> getPointsInSpace() {
		return getTransform().apply(points);
	}

	/**
	 * Apply a transform to the actual point set.
	 * @param transform
	 */
	public void permApplyTransform(TransformMy3D transform) {
		points = transform.apply(points);
		ptNormals = transform.applyNoShift(ptNormals);
	}

	/**
	 * Add a new point to the 3D object.
	 * @param point
	 * @param ptNormal
	 */
	public void addPoint(double[] point, double[] ptNormal) {
		points.add(point);
		ptNormals.add(ptNormal);
	}

	/**
	 * Add a new triangle to the 3D object.
	 * @param triangle
	 * @param material
	 */
	public void addTriangle(int[] triangle, Material material) {
		triangles.add(triangle);
		triangleMaterial.add(material);
	}

	/**
	 * Add a polygon to the 3D object.
	 * This will only parse appropriately for flat, convex hulls.
	 * @param points
	 * @param material
	 */
	public void addPolygon(int[] points, Material material) {
		for(int i=1;i<points.length-1;i++) {
			int[] tri = {points[0],points[i],points[i+1]};
			triangles.add(tri);
			triangleMaterial.add(material);
		}
	}	

	/**
	 * Test if this point is on the inside of the triangle.
	 * @param pt
	 * @param triangle
	 * @return
	 */
	public boolean inside(double[] pt, int triangle) {
		return inside(pt,triangles.get(triangle));
	}

	/**
	 * Test if this point is on the inside of the triangle.
	 * @param pt
	 * @param triangle
	 * @return
	 */
	public boolean inside(double[] pt, int[] triangle) {
		
		for(int triID : triangle) {
			if(MatrixMath.equals(pt,points.get(triID))) {
				return true;
			}
		}		
		
		double[] normal = getNormal(points.get(triangle[0]),points.get(triangle[1]),points.get(triangle[2]));
		double[] toPoint = MatrixMath.sub(pt,points.get(triangle[0]));
		if(MatrixMath.dotProduct(normal, toPoint) > 0) {
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Get the linear object inscribed by moving this object from the first to the second transform.
	 * @param t1
	 * @param t2
	 * @return
	 */
	public Object3D getTraversal(TransformMy3D t1, TransformMy3D t2) {
		
		Vector<double[]> points1 = t1.apply(points);
		Vector<double[]> ptNormals1 = t1.applyNoShift(ptNormals);
		
		Vector<double[]> points2 = t2.apply(points);
		Vector<double[]> ptNormals2 = t2.applyNoShift(ptNormals);
		
		Object3D newObject = new NullObject3D();
		newObject.points.addAll(points1);
		newObject.points.addAll(points2);
		newObject.ptNormals.addAll(ptNormals1);
		newObject.ptNormals.addAll(ptNormals2);
		
		// Add first triangles.
		for(int i=0;i<triangles.size();i++) {
			newObject.triangles.add(MatrixMath.copy(triangles.get(i)));
			newObject.triangleMaterial.add(triangleMaterial.get(i).copy());
		}
		
		// Add second triangles.
		for(int i=0;i<triangles.size();i++) {
			// Use this size offset.
			newObject.triangles.add(MatrixMath.add(triangles.get(i),points1.size()));
			newObject.triangleMaterial.add(triangleMaterial.get(i).copy());
		}
		
		// Then add the connections between.
		for(int i=0;i<triangles.size();i++) {
			int[] tri1 = MatrixMath.copy(triangles.get(i));
			int[] tri2 = MatrixMath.add(triangles.get(i),points1.size());
			
			for(int j=0;j<tri1.length;j++) {
				int[] newTri1 = {tri1[j],tri1[(j+1)%tri1.length],tri2[j]};
				newObject.triangles.add(newTri1);
				newObject.triangleMaterial.add(triangleMaterial.get(i).copy());
				
				int[] newTri2 = {tri2[j],tri1[(j+1)%tri1.length],tri2[(j+1)%tri1.length]};
				newObject.triangles.add(newTri2);
				newObject.triangleMaterial.add(triangleMaterial.get(i).copy());
			}
		}
		
		// Get list of internal points to remove.
		
		/*
		// For each point.
		for(int i=0;i<newObject.points.size();i++) {
			// For each shell triangle.
			boolean keep = false;
			for(int j=triangles.size()*2;j<newObject.triangles.size();j++) {
				if(!newObject.inside(newObject.points.get(i), j)) {
					keep = true;
					System.out.println("Here");
					break;
				}
			}
			if(!keep) {
				System.out.println("Remove " + i);
			}
		}
		*/
		return newObject;		
	}

	/**
	 * Remove these points from the object.
	 * Removes all triangles containing any of these points.
	 * Adjusts the triangle point values.
	 * @param toRemove
	 */
	public void removePoints(HashSet<Integer> toRemove) {
		// Build transfer table first.
		int[] transferTable = new int[points.size()];
		int index = 0;
		for(int i=0;i<transferTable.length;i++) {
			transferTable[i] = index;
			if(!toRemove.contains(i)) {
				index++;
			}
		}
		
		// Remove points and normals.
		Vector<double[]> newPoints = new Vector<double[]>();
		Vector<double[]> newNormals = new Vector<double[]>();
		for(int i=0;i<points.size();i++) {
			if(!toRemove.contains(i)) {
				newPoints.add(points.get(i));
				newNormals.add(ptNormals.get(i));
			}
		}
		points = newPoints;
		ptNormals = newNormals;
		
		// Remove and relabel triangles.
		Vector<int[]> newTriangles = new Vector<int[]>();
		Vector<Material> newTriangleMaterial = new Vector<Material>();
		for(int i=0;i<triangles.size();i++) {
			int[] triangle = triangles.get(i);
			boolean contains = false;
			for(int j : triangle) {
				if(toRemove.contains(j)) {
					contains = true;
					break;
				}
			}
			// Only add triangle if it does not contain a deleted point.
			if(!contains) {
				int[] newTriangle = new int[triangle.length];
				for(int j=0;j<newTriangle.length;j++) {
					newTriangle[j] = transferTable[triangle[j]];
				}
				newTriangles.add(triangle);
				newTriangleMaterial.add(triangleMaterial.get(i));
			}
		}
		triangles = newTriangles;
		triangleMaterial = newTriangleMaterial;
	}

	/**
	 * Find the average of the points.
	 * @return
	 */
	public double[] findCenter() {
		double[] pointSum = new double[3];
		for(double[] point : points) {
			for(int i=0;i<point.length;i++) {
				pointSum[i] += point[i];
			}
		}
		for(int i=0;i<pointSum.length;i++) {
			pointSum[i] /= points.size();
		}
		return pointSum;
	}

	/**
	 * Add an object as a child.
	 * @param child
	 */
	public void addChild(Object3D child) {
		children.add(child);
		child.parent = this;
	}

	/**
	 * Set the material for all triangles in the object.
	 * @param m
	 */
	public void setAllMaterial(Material m) {
		for(int i=0;i<triangleMaterial.size();i++) {
			triangleMaterial.set(i, m);
		}
	}

	/**
	 * Just change the color for all triangles in the object.
	 * @param color
	 */
	public void setAllMaterialColor(Color color) {
		for(Material m : triangleMaterial) {
			m.color = color;
		}
	}

	/**
	 * Set the color of all triangles for this object and all descendants.
	 * @param color
	 */
	public void setAllMaterialColorRecursive(Color color) {
		setAllMaterialColorRecursive(color,this);
	}

	/**
	 * Actually does the recursive work.
	 * @param color
	 * @param thisObject
	 */
	private void setAllMaterialColorRecursive(Color color, Object3D thisObject) {
		if(thisObject != null) {
			for(Material m : triangleMaterial) {
				m.color = color;
			}
			for(Object3D child : children) {
				setAllMaterialColorRecursive(color,child);
			}
		}
	}

	/**
	 * List of all children objects.
	 * @return
	 */
	public Vector<Object3D> getChildren() {
		return children;
	}

	/**
	 * Apply the full transform heiarchy of parents to find the world relative transform of this object.
	 * @return
	 */
	public TransformMy3D getTransform() {
		if(parent == null) {
			return transform;
		}
		return parent.getTransform().combineNew(transform);
	}
	
	/**
	 * Apply perspective transform to all of these points.
	 * This also centers the points in the window frame and scales them appropriately.
	 * @param points
	 * @param width
	 * @param height
	 * @return
	 */
	public Vector<double[]> applyPerspective(Vector<double[]> points, int width, int height) {
		Vector<double[]> newPoints = new Vector<double[]>();
		for(double[] point : points) {
			newPoints.add(applyPerspective(point,width,height));
		}
		return newPoints;
	}
	
	/**
	 * Test of this object or its descendants collides with the given object.
	 * @param object
	 * @return
	 */
	public boolean collisionRecursive(Object3D object) {
		return collisionRecursive(object, this);
	}

	/**
	 * Test the collisions.
	 * @param object
	 * @param thisObject
	 * @return
	 */
	private boolean collisionRecursive(Object3D object, Object3D thisObject) {
		if(thisObject == null) {
			return false;
		}
		
		if(thisObject.collision(object)) {
			return true;
		}
		
		for(Object3D child : children) {
			if(collisionRecursive(object,child)) {
				return true;
			}
		}
		
		return false;
	}

	/**
	 * Test if this object is colliding with the given object.
	 * @param object
	 * @return
	 */
	public boolean collision(Object3D object) {
				
		Vector<double[]> o1Pts = getTransform().apply(points);
		Vector<double[]> o2Pts = object.getTransform().apply(object.points);
		
		if(!boundingBoxesCollide(getBoundingBox(o1Pts), getBoundingBox(o2Pts))) {
			return false;
		}
		
		for(int[] triangle1 : triangles) {
			double[][] t1 = {o1Pts.get(triangle1[0]),o1Pts.get(triangle1[1]),o1Pts.get(triangle1[2])};
			for(int[] triangle2 : object.triangles) {
				double[][] t2 = {o2Pts.get(triangle2[0]),o2Pts.get(triangle2[1]),o2Pts.get(triangle2[2])};
				if(collision(t1,t2)) {
					return true;
				}
			}
		}
		
		return false;
	}
	
	/**
	 * Test collision between two sets of points.
	 * @param t1
	 * @param t2
	 * @return
	 */
	private boolean collision(double[][] t1, double[][] t2) {
		return TriangleCollision.tri_tri_intersect(t1[0],t1[1],t1[2], t2[0],t2[1],t2[2]);
	}
	
	/**
	 * Test if the bounding boxes of two objects collide.
	 * @param b1
	 * @param b2
	 * @return
	 */
	private boolean boundingBoxesCollide(double[][] b1, double[][] b2) {
		for(int i=0;i<3;i++) {
			if(b1[i][1] < b2[i][0] || b1[i][0] > b2[i][1]) {
				return false;
			}
		}
		return true;
	}
	
	/**
	 * Get the bounding box for this set of points.
	 * @param points
	 * @return
	 */
	private static double[][] getBoundingBox(Vector<double[]> points) {
		
		double[][] ret = new double[3][2];
		for(int i=0;i<3;i++) {
			ret[i][0] = Double.MAX_VALUE;
			ret[i][1] = Double.MIN_VALUE;
		}
		
		for(double[] pt : points) {
			for(int i=0;i<3;i++) {
				ret[i][0] = Math.min(ret[i][0], pt[i]);
				ret[i][1] = Math.max(ret[i][1], pt[i]);
			}
		}
		
		return ret;
	}
	
	/**
	 * Apply perspective transform.
	 * @param point
	 * @param width
	 * @param height
	 * @return
	 */
	public double[] applyPerspective(double[] point, int width, int height) {
		double x = point[0];
		double y = point[1];
		double z = point[2];
		
		double multiplier = SCREEN_DISTANCE / (z + SCREEN_DISTANCE);
		double dx = multiplier * x;
		double dy = multiplier * y;
		
		double[] newPoint = new double[3];
		newPoint[0] = dx * width / 80 + width / 2;
		newPoint[1] = dy * width / 80 + height / 2;
		newPoint[2] = z * width / 80;
		
		return newPoint;
	}
	
	/**
	 * Draw a single point in three space.
	 * @param zBuffer
	 * @param x
	 * @param y
	 * @param z
	 * @param g
	 * @param color
	 */
	public void drawZPoint(double[][] zBuffer, int x, int y, double z, Graphics g, Color color) {
		if(x >= 0 && y >= 0 && x < zBuffer.length && y <zBuffer[0].length) {
			zBuffer[x][y] = z;
			g.setColor(color);
			g.drawLine(x, y, x, y);
		}
	}
	
	/**
	 * http://www.cs.unc.edu/~mcmillan/comp136/Lecture6/Lines.html
	 * @param zBuffer
	 * @param x0
	 * @param y0
	 * @param x1
	 * @param y1
	 * @param z
	 */
	public void drawZLine(double[][] zBuffer, int x0, int y0, int x1, int y1, double z, Graphics g, Color color) {
	
        int dy = y1 - y0;
        int dx = x1 - x0;
        int stepx, stepy;

        if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
        if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }

        drawZPoint(zBuffer, x0, y0, z, g, color);
        drawZPoint(zBuffer, x1, y1, z, g, color);
        
        if (dx > dy) {
            int length = (dx - 1) >> 2;
            int extras = (dx - 1) & 3;
            int incr2 = (dy << 2) - (dx << 1);
            if (incr2 < 0) {
                int c = dy << 1;
                int incr1 = c << 1;
                int d =  incr1 - dx;
                for (int i = 0; i < length; i++) {
                    x0 += stepx;
                    x1 -= stepx;
                    if (d < 0) {														// Pattern:
                    	drawZPoint(zBuffer, x0, y0, z, g, color);						//
                    	drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);				//  x o o
                    	drawZPoint(zBuffer, x1, y1, z, g, color);						//
                    	drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);
                        d += incr1;
                    } else {
                        if (d < c) {													// Pattern:
                        	drawZPoint(zBuffer, x0, y0, z, g, color);					//      o
                        	drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);	//  x o
                        	drawZPoint(zBuffer, x1, y1, z, g, color);					//
                        	drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                        } else {
                        	drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);			// Pattern:
                        	drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);			//    o o 
                        	drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);			//  x
                        	drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);			//
                        }
                        d += incr2;
                    }
                }
                if (extras > 0) {
                    if (d < 0) {
                    	drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);
                    } else
                    if (d < c) {
                    	drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);
                    } else {
                    	drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                    }
                }
            } else {
                int c = (dy - dx) << 1;
                int incr1 = c << 1;
                int d =  incr1 + dx;
                for (int i = 0; i < length; i++) {
                    x0 += stepx;
                    x1 -= stepx;
                    if (d > 0) {
                    	drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);				// Pattern:
                    	drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);		//      o
                    	drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);				//    o
                    	drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);		//  x
                        d += incr1;
                    } else {
                        if (d < c) {
                        	drawZPoint(zBuffer, x0, y0, z, g, color);					// Pattern:
                        	drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color); //      o
                        	drawZPoint(zBuffer, x1, y1, z, g, color);                   //  x o
                        	drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color); //
                        } else {
                        	drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);			// Pattern:
                        	drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);			//    o o
                        	drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);			//  x
                        	drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);			//
                        }
                        d += incr2;
                    }
                }
                if (extras > 0) {
                    if (d > 0) {
                    	drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                    } else
                    if (d < c) {
                    	drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);
                    } else {
                    	drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                        if (extras > 2) {
                            if (d > c)
                            	drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                            else
                            	drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);
                        }
                    }
                }
            }
        } else {
            int length = (dy - 1) >> 2;
            int extras = (dy - 1) & 3;
            int incr2 = (dx << 2) - (dy << 1);
            if (incr2 < 0) {
                int c = dx << 1;
                int incr1 = c << 1;
                int d =  incr1 - dy;
                for (int i = 0; i < length; i++) {
                    y0 += stepy;
                    y1 -= stepy;
                    if (d < 0) {
                        drawZPoint(zBuffer, x0, y0, z, g, color);
                        drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                        drawZPoint(zBuffer, x1, y1, z, g, color);
                        drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);
                        d += incr1;
                    } else {
                        if (d < c) {
                            drawZPoint(zBuffer, x0, y0, z, g, color);
                            drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                            drawZPoint(zBuffer, x1, y1, z, g, color);
                            drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                        } else {
                            drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                            drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                            drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);
                            drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);
                        }
                        d += incr2;
                    }
                }
                if (extras > 0) {
                    if (d < 0) {
                        drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);
                    } else
                    if (d < c) {
                        drawZPoint(zBuffer, stepx, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);
                    } else {
                        drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                    }
                }
            } else {
                int c = (dx - dy) << 1;
                int incr1 = c << 1;
                int d =  incr1 + dy;
                for (int i = 0; i < length; i++) {
                    y0 += stepy;
                    y1 -= stepy;
                    if (d > 0) {
                        drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                        drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        drawZPoint(zBuffer, x1 -= stepy, y1, z, g, color);
                        drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                        d += incr1;
                    } else {
                        if (d < c) {
                            drawZPoint(zBuffer, x0, y0, z, g, color);
                            drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                            drawZPoint(zBuffer, x1, y1, z, g, color);
                            drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                        } else {
                            drawZPoint(zBuffer, x0 += stepx, y0, z, g, color);
                            drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                            drawZPoint(zBuffer, x1 -= stepx, y1, z, g, color);
                            drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);
                        }
                        d += incr2;
                    }
                }
                if (extras > 0) {
                    if (d > 0) {
                        drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                    } else
                    if (d < c) {
                        drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 2) drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);
                    } else {
                        drawZPoint(zBuffer, x0 += stepx, y0 += stepy, z, g, color);
                        if (extras > 1) drawZPoint(zBuffer, x0, y0 += stepy, z, g, color);
                        if (extras > 2) {
                            if (d > c)
                                drawZPoint(zBuffer, x1 -= stepx, y1 -= stepy, z, g, color);
                            else
                                drawZPoint(zBuffer, x1, y1 -= stepy, z, g, color);
                        }
                    }
                }
            }
        }
    }
	
	public void renderTriangle(BufferedImage img, double[][] zBuffer, Object[][] objBuffer, double[][] n, double[][] t, double[][] p, Material material, int renderType) {
				
		if(renderType == Universe.RENDER_POINTS) {
			
			Graphics g = img.getGraphics();
			//g.setColor(triangleMaterial.get(t).color);
			int color;
			
			
			color = Math.min(Math.max(0, (int)(p[0][2]/4)),255);
			g.setColor(new Color(color,color,color)); 
			g.drawLine((int)p[0][0], (int)p[0][1], (int)p[0][0], (int)p[0][1]);
			
			color = Math.min(Math.max(0, (int)(p[1][2]/4)),255);
			g.setColor(new Color(color,color,color)); 
			g.drawLine((int)p[1][0], (int)p[1][1], (int)p[1][0], (int)p[1][1]);
			
			color = Math.min(Math.max(0, (int)(p[2][2]/4)),255);
			g.setColor(new Color(color,color,color)); 
			g.drawLine((int)p[2][0], (int)p[2][1], (int)p[2][0], (int)p[2][1]);
			
			
		} else if(renderType == Universe.RENDER_WIREFRAME) {
			Graphics g = img.getGraphics();
			
			g.setColor(material.color);					
			//drawZLine(zBuffer, (int)p[0][0], (int)p[0][1], (int)p[1][0], (int)p[1][1], Double.MAX_VALUE,g,material.color);
			//drawZLine(zBuffer, (int)p[1][0], (int)p[1][1], (int)p[2][0], (int)p[2][1], Double.MAX_VALUE,g,material.color);
			//drawZLine(zBuffer, (int)p[2][0], (int)p[2][1], (int)p[0][0], (int)p[0][1], Double.MAX_VALUE,g,material.color);
			g.drawLine((int)p[0][0], (int)p[0][1], (int)p[1][0], (int)p[1][1]);
			g.drawLine((int)p[1][0], (int)p[1][1], (int)p[2][0], (int)p[2][1]);
			g.drawLine((int)p[2][0], (int)p[2][1], (int)p[0][0], (int)p[0][1]);
			
		} else {
			
			if(renderType == Universe.RENDER_WIREFRAME_FLAT) {
				Graphics g = img.getGraphics();
				g.setColor(Color.BLACK);						
				drawZLine(zBuffer, (int)p[0][0], (int)p[0][1], (int)p[1][0], (int)p[1][1], Double.MAX_VALUE,g,Color.BLACK);
				drawZLine(zBuffer, (int)p[1][0], (int)p[1][1], (int)p[2][0], (int)p[2][1], Double.MAX_VALUE,g,Color.BLACK);
				drawZLine(zBuffer, (int)p[2][0], (int)p[2][1], (int)p[0][0], (int)p[0][1], Double.MAX_VALUE,g,Color.BLACK);
			}
			
			double[] normal = getNormal(t[0], t[1], t[2]);
			double theta = getTheta(getNormal(p[0], p[1], p[2]));
			
			if(theta <= Math.PI / 2) {	
				Material triMaterial = material;
				double[] triNormal = normal;				
				Triangle3D triangle = new Triangle3D(triMaterial, t, n, p, triNormal);				
				fasterFill(zBuffer, objBuffer, img, triangle, renderType);
			}
		}
	}
	
	/**
	 * 
	 * @param pt1
	 * @param pt2
	 * @return
	 */
	public double[] planeIntercept(double[] pt1, double[] pt2) {
		double zIntercept = -SCREEN_DISTANCE - .05;
		double a = (zIntercept - pt1[2]) / (pt2[2] - pt1[2]);
		double x = pt1[0] + a * (pt2[0] - pt1[0]);
		double y = pt1[1] + a * (pt2[1] - pt1[1]);
		double[] intercept = {x,y,zIntercept};		
		return intercept;
	}
	
	public double planeInterceptFraction(double[] pt1, double[] pt2) {
		double zIntercept = -SCREEN_DISTANCE - .05;
		double a = (zIntercept - pt1[2]) / (pt2[2] - pt1[2]);
		return a;
	}

	/**
	 * The second and third point are behind the view scene.
	 * @param n
	 * @param t
	 * @param p
	 * @param img
	 * @param zBuffer
	 * @param objBuffer
	 * @param transformOld
	 * @param renderType
	 * @param material
	 */
	public void renderTwoBehind(double[][] n, double[][] t, double[][] p, BufferedImage img, double[][] zBuffer, Object[][] objBuffer, TransformMy3D transformOld, int renderType, Material material) {
		
		double[] t12 = planeIntercept(t[0], t[1]);
		double[] t13 = planeIntercept(t[0], t[2]);
		double[] p12 = applyPerspective(t12, img.getWidth(), img.getHeight());
		double[] p13 = applyPerspective(t13, img.getWidth(), img.getHeight());
		double a12 = planeInterceptFraction(t[0], t[1]);//(t12[2] - t[0][2]) / (t[1][2] - t[0][2]);
		double a13 = planeInterceptFraction(t[0], t[2]);//(t13[2] - t[0][2]) / (t[2][2] - t[0][2]);
		double[] n12;
		double[] n13;

		if(n[0] == null || n[2] == null) {
			n12 = null;
		} else {
			n12 = MatrixMath.add(MatrixMath.multiply(n[1], 1-a12),MatrixMath.multiply(n[0], a12));
		}
		
		if(n[1] == null || n[2] == null) {
			n13 = null;
		} else {
			n13 = MatrixMath.add(MatrixMath.multiply(n[2], 1-a13),MatrixMath.multiply(n[0], a13));
		}
		
		double[][] t1N = {n12,n13,n[0]};
		double[][] t1T = {t12,t13,t[0]};
		double[][] t1P = {p12,p13,p[0]};
				
		renderTriangle(img, zBuffer, objBuffer, t1N, t1T, t1P, material, renderType);
	}
	
	/**
	 * The third point is behind the view scene.
	 * @param n
	 * @param t
	 * @param p
	 * @param img
	 * @param zBuffer
	 * @param objBuffer
	 * @param transformOld
	 * @param renderType
	 * @param material
	 */
	public void renderOneBehind(double[][] n, double[][] t, double[][] p, BufferedImage img, double[][] zBuffer, Object[][] objBuffer, TransformMy3D transformOld, int renderType, Material material) {
		
		double[] t13 = planeIntercept(t[0], t[2]);
		double[] t23 = planeIntercept(t[1], t[2]);
		double[] p13 = applyPerspective(t13, img.getWidth(), img.getHeight());
		double[] p23 = applyPerspective(t23, img.getWidth(), img.getHeight());
		double a13 = planeInterceptFraction(t[0], t[2]);//(t13[2] - t[0][2]) / (t[2][2] - t[0][2]);
		double a23 = planeInterceptFraction(t[1], t[2]);//(t23[2] - t[1][2]) / (t[2][2] - t[1][2]);
		double[] n13;
		double[] n23;
				
		if(n[0] == null || n[2] == null) {
			n13 = null;
		} else {
			n13 = MatrixMath.add(MatrixMath.multiply(n[2], 1-a13),MatrixMath.multiply(n[0], a13));
		}
		
		if(n[1] == null || n[2] == null) {
			n23 = null;
		} else {
			n23 = MatrixMath.add(MatrixMath.multiply(n[2], 1-a23),MatrixMath.multiply(n[1], a23));
		}	
		
		double[][] t1N = {n13,n[0],n23};
		double[][] t1T = {t13,t[0],t23};
		double[][] t1P = {p13,p[0],p23};
		
		renderTriangle(img, zBuffer, objBuffer, t1N, t1T, t1P, material, renderType);
		
		double[][] t2N = {n[0],n[1],n23};
		double[][] t2T = {t[0],t[1],t23};
		double[][] t2P = {p[0],p[1],p23};
		
		renderTriangle(img, zBuffer, objBuffer, t2N, t2T, t2P, material, renderType);
		
	}
	
	public void render(BufferedImage img, double[][] zBuffer, Object[][] objBuffer, TransformMy3D transformOld, int renderType) {
		
		TransformMy3D transform = transformOld.combineNew(this.transform);
		
		Vector<double[]> transformed = transform.apply(points);
		Vector<double[]> transformedNormals = transform.applyNoShift(ptNormals);
		Vector<double[]> perspective = applyPerspective(transformed, img.getWidth(), img.getHeight());
		
		distLight = distLight_World;		
		distLight = normalize(distLight);
		
		double zCuttoff = SCREEN_DISTANCE;
		
		for(int tID=0;tID<triangles.size();tID++) {

			double[] t1 = transformed.get(triangles.get(tID)[0]);
			double[] t2 = transformed.get(triangles.get(tID)[1]);
			double[] t3 = transformed.get(triangles.get(tID)[2]);
						
			double[] p1 = perspective.get(triangles.get(tID)[0]);
			double[] p2 = perspective.get(triangles.get(tID)[1]);
			double[] p3 = perspective.get(triangles.get(tID)[2]);
						
			double[] n1 = transformedNormals.get(triangles.get(tID)[0]);
			double[] n2 = transformedNormals.get(triangles.get(tID)[1]);
			double[] n3 = transformedNormals.get(triangles.get(tID)[2]);
			
			if(t1[2] < -zCuttoff && t2[2] < -zCuttoff && t3[2] < -zCuttoff) {
				// All in front of viewer.
				double[][] n = {n1,n2,n3};
				double[][] t = {t1,t2,t3};
				double[][] p = {p1,p2,p3};
				renderTriangle(img, zBuffer, objBuffer, n, t, p, triangleMaterial.get(tID), renderType);				
			} else if(t1[2] >= -zCuttoff && t2[2] >= -zCuttoff && t3[2] >= -zCuttoff) {
				// All behind viewer.
			} else {		
				
				// Some in front and some behind.
				if(t1[2] < -zCuttoff) {
					if(t2[2] < -zCuttoff) {
						// 3 behind
						double[][] n = {n1,n2,n3};
						double[][] t = {t1,t2,t3};
						double[][] p = {p1,p2,p3};
						renderOneBehind(n, t, p, img, zBuffer, objBuffer, transformOld, renderType, triangleMaterial.get(tID));
					} else if(t3[2] < -zCuttoff) {
						// 2 behind
						double[][] n = {n3,n1,n2};
						double[][] t = {t3,t1,t2};
						double[][] p = {p3,p1,p2};
						renderOneBehind(n, t, p, img, zBuffer, objBuffer, transformOld, renderType, triangleMaterial.get(tID));
					} else {
						// 2 and 3 behind.
						double[][] n = {n1,n2,n3};
						double[][] t = {t1,t2,t3};
						double[][] p = {p1,p2,p3};
						renderTwoBehind(n, t, p, img, zBuffer, objBuffer, transformOld, renderType, triangleMaterial.get(tID));
					}
				} else if(t2[2] < -zCuttoff) {
					if(t3[2] < -zCuttoff) {
						// 1 behind
						double[][] n = {n2,n3,n1};
						double[][] t = {t2,t3,t1};
						double[][] p = {p2,p3,p1};
						renderOneBehind(n, t, p, img, zBuffer, objBuffer, transformOld, renderType, triangleMaterial.get(tID));
					} else {
						// 1 and 3 behind
						double[][] n = {n2,n3,n1};
						double[][] t = {t2,t3,t1};
						double[][] p = {p2,p3,p1};
						renderTwoBehind(n, t, p, img, zBuffer, objBuffer, transformOld, renderType, triangleMaterial.get(tID));
					}
				} else {
					// 1 and 2 behind
					double[][] n = {n3,n1,n2};
					double[][] t = {t3,t1,t2};
					double[][] p = {p3,p1,p2};
					renderTwoBehind(n, t, p, img, zBuffer, objBuffer, transformOld, renderType, triangleMaterial.get(tID));
				}
				
			}
		}
		
		for(Object3D p : children) {
			p.render(img, zBuffer,objBuffer,transform,renderType);
		}		
	}
	
	public double[] avg(double[] p1, double[] p2, double[] p3) {
		double[] avg = {	(p1[0] + p2[0] + p3[0]) / 3,
							(p1[1] + p2[1] + p3[1]) / 3,
							(p1[2] + p2[2] + p3[2]) / 3};
		return avg;
	}
	
	public double[] getNormal(double[] p1, double[] p2, double[] p3) {
		
		double ax = p1[0] - p2[0];
		double ay = p1[1] - p2[1];
		double az = p1[2] - p2[2];
		
		double bx = p3[0] - p2[0];
		double by = p3[1] - p2[1];
		double bz = p3[2] - p2[2];
		
		// A is the cross product.					
		double[] normal = {	ay * bz - az * by,
							az * bx - ax * bz,
							ax * by - ay * bx};
		
		normal = normalize(normal);
		
		return normal;
	}
	
	public double getAngle(double[] pt1, double[] pt2) {
		double len1 = Math.sqrt(pt1[0] * pt1[0] + pt1[1] * pt1[1] + pt1[2] * pt1[2]);
		double len2 = Math.sqrt(pt2[0] * pt2[0] + pt2[1] * pt2[1] + pt2[2] * pt2[2]);
		double dot = pt2[0] * pt1[0] + pt2[1] * pt1[1] + pt2[2] * pt1[2];
		double angle = dot / (len1 * len2);
		return Math.acos(angle);
	}
	
	public double getColor(double[] normal, Material material) {
		double lAngleN = getAngle(normal,distLight);
		
		double LOnN = lAngleN;
		double LOnNx = normal[0] * LOnN;
		double LOnNy = normal[1] * LOnN;
		double LOnNz = normal[2] * LOnN;
	
		// Reflection.
		double Rx = LOnNx + LOnNx - distLight[0];
	 	double Ry = LOnNy + LOnNy - distLight[1];
	 	double Rz = LOnNz + LOnNz - distLight[2];
	 	
	 	double rDotV = -(Rx * 0 + Ry * 0 + Rz * 1);
	 	
		int s =  material.s;  // Is a shininess constant for this material, which decides how "evenly" light is reflected from a shiny spot.
		double ia = material.ia; // Ambiant intensity.
		double id = material.id;  //3 Diffusion intensity of light. 
		double is = material.is;  // Specular intensity of light. 		 			
	 	
		//return ia + lAngleN * id + Math.pow(rDotV,s) * is;
		return ia + lAngleN * id + intPower(rDotV,s) * is;
	}
	
	public double intPower(double base, int power) {
		double sum = 1;
		for(int i=0;i<power;i++) {
			sum *= base;
		}
		return sum;
		
	}
	
	public double dotProduct(Point.Double p1, Point.Double p2) {
		return p1.x * p2.x + p1.y * p2.y;
	}
	
	public Point.Double sub(Point.Double p1, Point.Double p2) {
		return new Point.Double(p1.x-p2.x,p1.y-p2.y);
	}

	public double crossProduct(Point.Double p1, Point.Double p2) {
		return p1.x * p2.y - p1.y * p2.x;
	}	

	public double[] crossProduct(double[] p1, double[] p2) {
		if(p1.length == 3) {
			double[] ret = {p1[1] * p2[2] - p1[2] * p2[1],
							p1[2] * p2[0] - p1[0] * p2[2],
							p1[0] * p2[1] - p1[1] * p2[0]};
			return ret;
		} else {
			return null;
		}
	}
	
	public boolean sameSide(Point.Double p1, Point.Double p2, Point.Double a, Point.Double b) {
		double cp1 = crossProduct(sub(b,a), sub(p1,a));
		double cp2 = crossProduct(sub(b,a), sub(p2,a));
		return (cp1 * cp2 >= 0);
	}

	public boolean pointInTriangle(Point.Double p, Point.Double a, Point.Double b, Point.Double c) {
		/*
		if(crossProduct(sub(b,a), sub(p,a)) > 0) {
			if(crossProduct(sub(c,b), sub(p,b)) > 0) {
				if(crossProduct(sub(a,c), sub(p,c)) > 0) {
					return true;
				}
			}
		}
		return false;
		*/
		

		
		return sameSide(p,a, b,c) && sameSide(p,b, a,c) && sameSide(p,c, a,b);
	}

    
	public void fasterFill(double[][] zBuffer, Object[][] objBuffer, BufferedImage img, Triangle3D triangle3D, int renderType) {
		
		double[] p1 = triangle3D.perspective[0];
		double[] p2 = triangle3D.perspective[1];
		double[] p3 = triangle3D.perspective[2];
				
		double[] n1 = triangle3D.transformedNormals[0];
		double[] n2 = triangle3D.transformedNormals[1];
		double[] n3 = triangle3D.transformedNormals[2];
		
		int minX = (int)Math.floor(Math.min(Math.min(p1[0],p2[0]),p3[0]));
		int minY = (int)Math.floor(Math.min(Math.min(p1[1],p2[1]),p3[1]));
		int maxX = (int)Math.ceil(Math.max(Math.max(p1[0],p2[0]),p3[0]));
		int maxY = (int)Math.ceil(Math.max(Math.max(p1[1],p2[1]),p3[1]));
		
		minX = Math.max(minX, 0);
		minY = Math.max(minY, 0);
		maxX = Math.min(maxX, img.getWidth());
		maxY = Math.min(maxY, img.getHeight());
		
		if(maxX == minX || maxY == minY) {
			return;
		}
				
		// Calculate transform matrix.
		double[][] m3t = {{p1[0],p2[0],p3[0]},{p1[1],p2[1],p3[1]},{1,1,1}};
		double[][] m3ti = MatrixMath.invert(m3t);
		double[][] m2t = MatrixMath.multiply(m3ti,AXIS);
		double[][] m2 = MatrixMath.transpose(m2t);

		Color faceColor = triangle3D.material.color;
		
		// Default color for this face.
		double col = 0;
		col = getColor(triangle3D.normal, triangle3D.material);

		// Used for checking if in triangle.
		Point.Double pt1 = new Point.Double(p1[0],p1[1]);
		Point.Double pt2 = new Point.Double(p2[0],p2[1]);
		Point.Double pt3 = new Point.Double(p3[0],p3[1]);
		
		// For every point within.
		for(int y=minY;y<=maxY;y++) {
			for(int x=minX;x<=maxX;x++) {

				// In triangle and in screen.
				if(x >= 0 && y >= 0 && x < zBuffer.length && y < zBuffer[0].length && pointInTriangle(new Point.Double(x,y),pt1,pt2,pt3)) {
					
					// Calculate a,b and c from the transform matrix.
					double c = x * m2[0][0] + y * m2[1][0] + m2[2][0];
					double a = x * m2[0][1] + y * m2[1][1] + m2[2][1];
					double b = 1 - c - a;

					// Calculate z.
					double z = a * p1[2] + b * p2[2] + c * p3[2];

					// Closer than closest point.
					if(zBuffer[x][y] < z) {
						
						if(renderType == Universe.RENDER_DEAPTH) {

							// Color is a function of depth.
							int color = (int)(z/2);
							color = Math.min(Math.max(0, color),255);
							int rgb = new Color(color,color,color).getRGB(); 
							
							img.setRGB(x, y, rgb);
							objBuffer[x][y] = triangle3D.material.object;
							zBuffer[x][y] = z;
							
							
						} else {
							Color base;
							
							if(renderType == Universe.RENDER_FLAT || renderType == Universe.RENDER_WIREFRAME_FLAT) {
								
								// Use flat color.
								base = triangle3D.material.color;
							} else {
								
								// Get light from new normal.
								if(n1 != null && n2 != null && n3 != null) {
									
									double[] newNormal = {	-a * n1[0] - b * n2[0] - c * n3[0],
															-a * n1[1] - b * n2[1] - c * n3[1],
															-a * n1[2] - b * n2[2] - c * n3[2]};
									
									triangle3D.normal = normalize(newNormal);
								 	col = getColor(triangle3D.normal, triangle3D.material);
								}

								// Use face color or image color.
								base = triangle3D.material.getColor(a, c); 
							}
							
							int red = (int)(base.getRed() * col);
							int green = (int)(base.getGreen() * col);
							int blue = (int)(base.getBlue() * col);
							
							Color old = new Color(img.getRGB(x, y));
							int alpha = triangle3D.material.color.getAlpha();
							red = (red * alpha + old.getRed() * (255 - alpha)) / 255;
							green = (green * alpha + old.getGreen() * (255 - alpha)) / 255;
							blue = (blue * alpha + old.getBlue() * (255 - alpha)) / 255;
														
							red = Math.max(Math.min(red, 255),0);
							green = Math.max(Math.min(green, 255),0);
							blue = Math.max(Math.min(blue, 255),0);
							faceColor = new Color(red,green,blue);
													
							img.setRGB(x, y, faceColor.getRGB());
							objBuffer[x][y] = triangle3D.material.object;
							zBuffer[x][y] = z;
						}
					}
				}
			}
		}
	}	

	public double getTheta(double ax, double ay, double az) {
		double adb = -az;
		double lenB = Math.sqrt(ax * ax + ay * ay + az * az);
		return Math.acos(adb/lenB);
	}
	
	public double getTheta(double[] p) {
		double adb = -p[2];
		double lenB = getLen(p);
		return Math.acos(adb/lenB);
	}
	
	public double getTheta(double[] p1, double[] p2) {
		double adb = p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2];
		double len1 = Math.sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2]);
		double len2 = Math.sqrt(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2]);
		return Math.acos(adb/(len1*len2));
	}
	
	public double getDot(double[] p1, double[] p2) {
		return p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2];
	}
	
	public double getLen(double[] p) {
		return Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
	}
	
	public double[] normalize(double[] p) {
		double len  = getLen(p);
		double[] norm = new double[3];
		norm[0] = p[0] / len;
		norm[1] = p[1] / len;
		norm[2] = p[2] / len;
		return norm;
	}
	
	public double[][] getABC(double[] p1, double[] p2, double p3[], double x, double y) {
		double[][] m = {{1,p1[0],p1[1]},{1,p2[0],p2[1]},{1,p3[0],p3[1]}};
		double[][] minv = MatrixMath.invert(m);
		double[][] m2 = {{1,x,y}};
		return MatrixMath.multiply(minv, m2);
	}
	
	public BufferedImage rotate(BufferedImage img) {
		BufferedImage ret = new BufferedImage(img.getWidth(),img.getHeight(),BufferedImage.TYPE_INT_BGR);
		for(int x=0;x<img.getWidth();x++) {
			for(int y=0;y<img.getHeight();y++) {
				ret.setRGB(x, y, img.getRGB(img.getWidth()-x-1, img.getHeight()-y-1));
			}
		}
		return ret;
	}
	
	public BufferedImage flip(BufferedImage img) {
		BufferedImage ret = new BufferedImage(img.getWidth(),img.getHeight(),BufferedImage.TYPE_INT_BGR);
		for(int x=0;x<img.getWidth();x++) {
			for(int y=0;y<img.getHeight();y++) {
				ret.setRGB(x, y, img.getRGB(img.getWidth()-x-1, y));
			}
		}
		return ret;
	}
	
	public void addTriangle(int start, int a, int b, int c, Material material) {
		int[] newT = new int[3];
		newT[0] = a + start;
		newT[1] = b + start;
		newT[2] = c + start;
		triangles.add(newT);
		triangleMaterial.add(material);
	}
}
