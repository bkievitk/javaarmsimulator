package com.armsim.my3DCore;

public class Triangle3D {
	public Material material;
	public double[][] transformed = new double[3][];
	public double[][] transformedNormals = new double[3][];
	public double[][] perspective = new double[3][];
	public double[] normal;
	
	public Triangle3D(Material material, double[][] transformed, double[][] transformedNormals, double[][] perspective, double[] normal) {
		this.material = material;
		this.transformed = transformed;
		this.transformedNormals = transformedNormals;
		this.perspective = perspective;
		this.normal = normal;
	}
}
