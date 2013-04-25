package com.armsim.my3DShapes;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Vector;

import com.armsim.my3DCore.Material;

/**
 * A 3D bezier surface.
 * @author bkievitk
 */

public class Bezier3D extends Object3D {

	private static final long serialVersionUID = 92347860002L;

	// Load from file.
	public Bezier3D(File f, Material material) {
		loadBezier(f, 4, material);
	}
	
	public void loadBezier(File f, int divs, Material material) {
		try {
			
			Vector<double[]> patchPoints = new Vector<double[]>();
			Vector<int[][]> patches = new Vector<int[][]>();
			
			// Read file.
			BufferedReader r = new BufferedReader(new FileReader(f));
			String line;
			while((line = r.readLine()) != null) {
				
				if(line.charAt(0) == '#') {
					// Comment.
				}
				else if(line.charAt(0) == 'V') {
					// Point.
					String[] parts = line.split(" +");
					double[] newPoint = new double[3];
					newPoint[0] = Double.parseDouble(parts[1]);
					newPoint[1] = Double.parseDouble(parts[2]);
					newPoint[2] = Double.parseDouble(parts[3]);
					patchPoints.add(newPoint);
				}
				else if(line.charAt(0) == 'P') {
					// Bezier Patch.
					String[] parts = line.split(" +");
					int[][] patch = new int[4][4];
					for(int x=0;x<4;x++) {
						for(int y=0;y<4;y++) {
							patch[x][y] = Integer.parseInt(parts[x*4+y+1]);
						}
					}
					patches.add(patch);
				}
			}
			
			for(int[][] patch : patches) {
				int u,v;

				double[][][] newPatch = new double[divs][divs][3];
				double[][][] newNorm = new double[divs][divs][3];
				
				// For all u by all v.
				for(u=0;u<divs;u++) {
					for(v=0;v<divs;v++) {
						
						double du = u/(double)(divs-1);
					    double dv = v/(double)(divs-1);
					       
					    double[] p = {	(1-du) * (1-du) * (1-du),
					    				3 * du * (1-du) * (1-du),
					    				3 * du * du * (1-du),
					    				du * du * du};
					   	
					   	double[] q = {	(1-dv) * (1-dv) * (1-dv),
					   					3 * dv * (1-dv) * (1-dv),
					   					3 * dv * dv * (1-dv),
					   					dv * dv * dv};
					   	
					   	for(int l=0;l<3;l++) {
							newPatch[u][v][l] = 0;
							for(int i=0;i<4;i++) {	
								double sum = 0;
								for(int j=0;j<4;j++) {
									sum += p[j] * patchPoints.get(patch[j][i]-1)[l];
								}
								newPatch[u][v][l] += q[i] * sum;
							}
					   	}
		
					   	double[] deru = {	-3 * (-1 + du) * (-1 + du),
					   						3 - 12 * du + 9 * du * du,
					   						3 * (2 - 3 * du) * du,
					   						3 * du * du};
		
					   	double[] derv = {	-3 * (-1 + dv) * (-1 + dv),
					   						3 - 12 * dv + 9 * dv * dv,
					   						3 * (2 - 3 * dv) * dv,
					   						3 * dv * dv};
					   						   	
					   	double[] derivitiveU = new double[3];
					   	for(int l=0;l<3;l++) {
					   		derivitiveU[l] = 0;
							for(int i=0;i<4;i++) {	
								double sum = 0;
								for(int j=0;j<4;j++) {
									sum += deru[j] * patchPoints.get(patch[j][i]-1)[l];
								}
								derivitiveU[l] += q[i] * sum;
							}
					   	}
					   	
					   	double[] derivitiveV = new double[3];
					   	for(int l=0;l<3;l++) {
					   		derivitiveV[l] = 0;
							for(int i=0;i<4;i++) {	
								double sum = 0;
								for(int j=0;j<4;j++) {
									sum += p[j] * patchPoints.get(patch[j][i]-1)[l];
								}
								derivitiveV[l] += derv[i] * sum;
							}
					   	}
					   	
					   	newNorm[u][v][0] = -((derivitiveU[1] * derivitiveV[2]) - (derivitiveU[2] * derivitiveV[1]));
					   	newNorm[u][v][1] = -((derivitiveU[2] * derivitiveV[0]) - (derivitiveU[0] * derivitiveV[2]));
					   	newNorm[u][v][2] = -((derivitiveU[0] * derivitiveV[1]) - (derivitiveU[1] * derivitiveV[0]));
							
					   	int start = points.size();
						
						for(int x=0;x<divs;x++) {
							for(int y=0;y<divs;y++) {
								points.add(newPatch[x][y]);
								ptNormals.add(newNorm[x][y]);
							}
						}
								
						for(int x=0;x<divs-1;x++) {
							for(int y=0;y<divs-1;y++) {
								int p00 = x * divs + y;
								int p01 = x * divs + (y+1) % divs;
								int p10 = ((x+1) % divs) * divs + y;
								int p11 = ((x+1) % divs) * divs + (y+1)%divs;
								addTriangle(start, p11, p10, p00, material);
								addTriangle(start, p00, p01, p11, material);
							}
						}
					}
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
