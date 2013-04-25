package com.armsim.my3DRobotArm;

import java.io.File;
import java.util.Vector;

import com.armsim.my3DCore.MatrixMath;
import com.armsim.my3DCore.Universe;

public class RobotArmGrid {
	
	public static File armXML = new File("com/armsim/arm2.xml");
		
	public static boolean nextPoint(double[] point, Range[] ranges) {
		for(int i=0;i<point.length;i++) {
			point[i] += ranges[i].step; // Add step.
			if(point[i] < ranges[i].max) { // Not past the last step.
				return true;
			} else {
				point[i] = ranges[i].min; // Reset to min.
			}
		}
		return false;
	}
	
	public static void main(String[] args) {
	
		if(args.length > 0 && args[0].equals("-run")) {
			
			Universe u = new Universe();
			
			RobotArm robotArm = new RobotArm(true, armXML, u, -1);
			
			Range[] ranges = new Range[(args.length-1)/3];
			for(int i=1;i<args.length;i+=3) {
				ranges[(i-1)/3] = new Range(Double.parseDouble(args[i]),
											Double.parseDouble(args[i+1]),
											Double.parseDouble(args[i+2]));
			}
			
			int product = 1;
			for(int i=0;i<ranges.length;i++) {
				product *= Math.ceil((ranges[i].max - ranges[i].min) / ranges[i].step);
			}
			
			System.out.println("There are " + product + " points to test.");
			
			double[] point = new double[ranges.length];
			for(int i=0;i<point.length;i++) {
				point[i] = ranges[i].min; // Start at min.
			}
						
			do {
				// Test this point.
				double[][] pointFull = robotArm.convertRelevantJoints(point);
				robotArm.setJoints(pointFull, false);

				if(!robotArm.armCollision()) {
					double[] pointCopy = MatrixMath.copy(point);
					
					// Only bother with neighbors if this is a valid point.
					for(int i=0;i<point.length;i++) {
						pointCopy[i] = point[i] - ranges[i].step;
						if(pointCopy[i] >= ranges[i].min) {
							// Test this connection.
							double[][] pointCopyFull = robotArm.convertRelevantJoints(pointCopy);
							if(robotArm.testPath(pointFull, pointCopyFull)) {
								System.out.print(MatrixMath.show(point));
								System.out.println(MatrixMath.show(pointCopy));
							}
						}
						pointCopy[i] = point[i] + ranges[i].step;
						if(pointCopy[i] <= ranges[i].max) {
							// Test this connection.
							double[][] pointCopyFull = robotArm.convertRelevantJoints(pointCopy);
							
							if(robotArm.testPath(pointFull, pointCopyFull)) {
								System.out.print(MatrixMath.show(point));
								System.out.println(MatrixMath.show(pointCopy));
							}
						}
						pointCopy[i] = point[i];
					}
				}
			} while(nextPoint(point, ranges));
			
			
		} else if(args.length > 0 && args[0].equals("-range")) {
			Universe u = new Universe();
			
			
			RobotArm robotArm = new RobotArm(true, armXML, u, -1);
			System.out.println(robotArm.getRelevantJointRanges());
		} else {
			System.out.println("Format:");
			System.out.println("  -run min1 max1 step1 min2 max2 step2...");
			System.out.println("  -range");
		}
	}
	
	
}
