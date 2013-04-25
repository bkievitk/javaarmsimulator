package com.armsim.my3DRobotArm;

import java.awt.Point;
import java.util.*;
import java.io.*;
import com.armsim.my3DCore.*;
import com.armsim.my3DShapes.*;
import com.mds.MDSJoint;

public class RobotArmMDS extends RobotArm {
	
	public static Hashtable<MDSJoint,Integer> jointIDs = getJointID();

	public RobotArmMDS(boolean visualization, File armXML, Universe u, int updateMS) {
		super(visualization, armXML, u, updateMS);
	}

    public static Hashtable<MDSJoint,Integer> getJointID() {
    	Hashtable<MDSJoint,Integer> hash = new Hashtable<MDSJoint,Integer>();
        hash.put(MDSJoint.RUpperArmRoll,0);
        hash.put(MDSJoint.RElbow,1);
        hash.put(MDSJoint.RShoulderAbduct,2);
        hash.put(MDSJoint.RShoulderPitch,3);
        hash.put(MDSJoint.RWristRoll,4);
        hash.put(MDSJoint.RWristFlex,5);
        return hash;
    }

	protected synchronized int getJointID(Object j) {
		int id = super.getJointID(j);
		if(id >= 0) {
			return id;
		}

		if(j instanceof MDSJoint) {
			Integer idI = jointIDs.get(((MDSJoint)j));
			if(idI == null) {
				return -1;
			}
			return idI;
		}

		return -1;
	}	
	
    //@Override
    public float getAngle(Object j) {
    	int id = getJointID(j);
	if(id >= 0) {
		    
		    Vector<Double> joints = getRelevantJoints();
		    Vector<Double> offsets = getRelevantJointOffsets();
		    return joints.get(id).floatValue() + offsets.get(id).floatValue();
    	}
		
    	return -1;
    }

    //@Override
    public float getUpperLimit(Object j) {
    	int id = getJointID(j);
		if(id >= 0) {
    		return (float)(joints.get(id).rotationMax[getRelevantAxis(id)] - joints.get(id).rotationMin[getRelevantAxis(id)]) / 2;
    	}
    	return -1;
    }

    //@Override
    public float getLowerLimit(Object j) {
    	int id = getJointID(j);
		if(id >= 0) {
    		return (float)(joints.get(id).rotationMin[getRelevantAxis(id)] - joints.get(id).rotationMax[getRelevantAxis(id)]) / 2;
    	}
    	return -1;
    }

    //@Override
    public synchronized void setAngle(Object j, float a) {
    	setAngle(j,a,1);
    }

    //@Override
    public void setAngle(Object j, float a, float v) {
    	int id = getJointID(j);
		if(id >= 0) {
		    // Set velocity to move toward the desired angle based on a linear angle set.
			v = Math.abs(v);
			if(a < (float)joints.get(id).rotation[getRelevantAxis(id)]) {
				v = -v;
			}
	
			// Set desired velocity.
			desiredVelocity[id][getRelevantAxis(id)] = (double)v;
	
			// Set desired pose.
			if(desiredPose == null) {
				desiredPose = getPose();
			}
			desiredPose[id][getRelevantAxis(id)] = a  - (joints.get(id).rotationMax[getRelevantAxis(id)] - joints.get(id).rotationMin[getRelevantAxis(id)]) / 2;
		}
    }
	
	/**
	 * A list of joint offsets for the MDS.
	 * @return
	 */
	public Vector<Double> getRelevantJointOffsets() {
		Vector<Double> values = new Vector<Double>();
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.rotationMin[j] != joint.rotationMax[j]) {
					values.add((joint.rotationMin[j] - joint.rotationMax[j]) / 2);
				}
			}
		}
		return values;
	}
	
	public ArrayList<Object> getJoints() {
		return new ArrayList(jointIDs.keySet());
	}

    //@Override
    public synchronized void setVelocity(Object j, float v) {
    	int id = getJointID(j);
	//System.out.println("(MDS)SETVELO: JOINT ID IS: " + id + "   |  Object is: " + j.toString());//REV:
    	if(id >= 0) {
			desiredVelocity[id][getRelevantAxis(id)] = (double)v;
			desiredPose = null;
	}
    }	
}
