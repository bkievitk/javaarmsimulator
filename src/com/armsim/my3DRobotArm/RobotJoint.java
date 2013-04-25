package com.armsim.my3DRobotArm;

import com.armsim.my3DCore.TransformMy3D;
import com.armsim.my3DShapes.Object3D;


public class RobotJoint {
	
	public Object3D shape;
	public String jointName;
	public double[] centerOfRotationParent;
	public double[] centerOfRotationChild;
	public double[] rotation;
	public double[] rotationMin;
	public double[] rotationMax;
	public double[] rotationZero;
	public boolean[] rotationPositive = null;
		
	public RobotJoint() {
	}
	
	public RobotJoint(Object3D shape, double[] centerOfRotationParent, double[] centerOfRotationChild, final double[] rotation, double[] rotationMin, double[] rotationMax, String jointName) {
		this.shape = shape;
		this.centerOfRotationParent = centerOfRotationParent;
		this.centerOfRotationChild = centerOfRotationChild;
		this.rotation = rotation;
		this.rotationMin = rotationMin;
		this.rotationMax = rotationMax;
		this.jointName = jointName;			
	}
	
	public boolean validAngles() {
		for(int j=0;j<rotation.length;j++) {
			if(rotation[j] < rotationMin[j] || rotation[j] > rotationMax[j]) {
				return false;
			}
		}
		return true;
	}
	
	public void setTransform() {
		shape.transform = new TransformMy3D();
		
		// There seems to be an occasional issue with collision detection when they are in exactly the same plane.
		// This offset seems to counter that problem.
		shape.transform.combine(TransformMy3D.translate(-centerOfRotationParent[0], -centerOfRotationParent[1], -centerOfRotationParent[2]));
		shape.transform.combine(TransformMy3D.translate(centerOfRotationChild[0], centerOfRotationChild[1], centerOfRotationChild[2]));

		if(rotationPositive != null && !rotationPositive[0]) {
			shape.transform.combine(TransformMy3D.rotateX(rotationMax[0]-rotation[0] + rotationMin[0] - rotationZero[0]));
		} else {
			shape.transform.combine(TransformMy3D.rotateX(rotation[0] - rotationZero[0]));
		}

		if(rotationPositive != null && !rotationPositive[1]) {
			shape.transform.combine(TransformMy3D.rotateY(rotationMax[1]-rotation[1] + rotationMin[1] - rotationZero[1]));
		} else {
			shape.transform.combine(TransformMy3D.rotateY(rotation[1] - rotationZero[1]));
		}

		if(rotationPositive != null && !rotationPositive[2]) {
			shape.transform.combine(TransformMy3D.rotateZ(rotationMax[2]-rotation[2] + rotationMin[2] - rotationZero[2]));
		} else {
			shape.transform.combine(TransformMy3D.rotateZ(rotation[2] - rotationZero[2]));
		}

		shape.transform.combine(TransformMy3D.translate(-centerOfRotationChild[0], -centerOfRotationChild[1], -centerOfRotationChild[2]));
	}

	public boolean movableJoint(int axis) {
		return rotationMin[axis] != rotationMax[axis];
	}
}
