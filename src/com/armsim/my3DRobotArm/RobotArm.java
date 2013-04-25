package com.armsim.my3DRobotArm;

import java.awt.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

import javax.swing.*;
import javax.swing.Timer;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.event.*;

import com.armsim.my3DCore.*;
import com.armsim.my3DShapes.*;
import com.mds.JointControl;

import com.mds.MDSJoint;

public class RobotArm implements JointControl {
	
	public static Random rand = new Random();
	public static final double ACCELERATION = 1.25; // rad/sec^2

	/**
	 * Robot arm specifictions.
	 **/

	// List of all robot joints.
	public Vector<RobotJoint> joints = new Vector<RobotJoint>();

	// The first part of the robot arm attached to the body.
	public Object3D armRoot;

	// Object that represents the robots hand.
	public Object3D endEffector;

	// 3D Universe to implement in.
	public Universe universe;

	// Object pairs that are not able to collide with eachother.
	private HashSet<ObjectPair> nonColliding = new HashSet<ObjectPair>();

	/**
	 * Robot arm stored goals and physics.
	 **/

	// Robot velocity.
	protected double[][] desiredPose = null;
	protected double[][] desiredVelocity;
	protected double[][] velocity;
	
	// Goal system.
	protected boolean moveToGoal = false;
	protected double[] goal;
	protected Object3D goalObject;

	/**
	 * Simulation information.
	 **/

	// Real-time timer.
	private Timer timer;

	// Speed of joint for gradient descent.
	public double jointSpeed = .05;

	// Simulator speed up in proportion to real-time.
	private double simulatorSpeedup = 1.0;
	
	// How verbose to be.
	public int verbosity = 0;

	// Root object that the arm is attached to.
	public NullObject3D root;

	/**
	 * Starts a real-time robot arm instance with visualization.
	 **/
	public static void main(String[] args) {
		// Create the universe.
		Universe u = new Universe();
		File armXML = new File("arm2.xml");
		new RobotArm(true, armXML, u, 10);
	}	
	
	public void addBody() {
		Object3D upperBody = new Cube3D(0, 0, -.5, .6, .6, 1.2, new Material(Color.LIGHT_GRAY,"upper body",null));
		Object3D lowerBody = new Cube3D(0, 0, -1.8, 1, .8, 1.2, new Material(Color.LIGHT_GRAY,"lower body",null));
		//double[] headCenter = {0,0,.9};
		//Object3D head = new Sphere3D(headCenter,.4,7,7,new Material(Color.LIGHT_GRAY,"head",null));
		double[] headConeCenter = {0,0,-.5};
		Object3D headCone = new Cone3D(headConeCenter, .6, .6, 7, new Material(Color.LIGHT_GRAY,"head cone",null));
		headCone.permApplyTransform(TransformMy3D.rotateX(Math.PI*1.5));
		headCone.permApplyTransform(TransformMy3D.stretch(1, .5, 1));
		NullObject3D bodyScaller = new NullObject3D();
		
		bodyScaller.transform.combine(TransformMy3D.stretch(1.09/3, 1.09/3, 1.09/3));
		bodyScaller.addChild(upperBody);
		bodyScaller.addChild(lowerBody);
		//bodyScaller.addChild(head);
		bodyScaller.addChild(headCone);
		universe.root.addChild(bodyScaller);
		addNonColliding(joints.get(0).shape, upperBody);
		//addNonColliding(joints.get(0).shape, head);
	}

	/**
	 * Initialize robot in the universe.
	 * @param universe
	 */
	public RobotArm(boolean visualization, File armXML, Universe u, int updateMS) {
		universe = u;
		boolean rightArm = false;
		double robotArmScaller = (9.6 / 100) / .088;
				
		// Create the robot arm.
		try {
			XMLParseRobotArm.parseXML(new BufferedReader(new FileReader(armXML)),this);
			velocity = getZeroVelocity();
			desiredVelocity = getZeroVelocity();
		} catch (IOException e) {
			e.printStackTrace();
		}

		// The root of the universe will contain the robot arm and the scene.
		root = new NullObject3D();		
		NullObject3D robotScaller = new NullObject3D();

		if(rightArm) {
			robotScaller.transform.combine(TransformMy3D.stretch(robotArmScaller, robotArmScaller, robotArmScaller));		
		} else {
			robotScaller.transform.combine(TransformMy3D.stretch(-robotArmScaller, robotArmScaller, robotArmScaller));		
			for(RobotJoint joint : joints) {
				joint.shape.flipNormals();

				// Flip positive direction for the left arm.
				//if(joint.rotationPositive == null) {
				//	joint.rotationPositive = new boolean[joint.rotation.length];
				//}
				//for(int i=0;i<joint.rotationPositive.length;i++) {
				//	joint.rotationPositive[i] = !joint.rotationPositive[i];
				//}
			}
		} 
		root.addChild(robotScaller);
		robotScaller.addChild(armRoot);
		
		// Set root and view.
		u.root.addChild(root);	
					
		if(visualization) {

			// Control for user.
			buildControlPanelSetPose();
			buildControlPanelMoveToPose();
			buildControlPanelSetGoal();

			// Add the center sphere.
			//double[] center2 = {0,0,0};
			//double radius2 = .1;
			//Material material2 = new Material(Color.LIGHT_GRAY,"object",null);
			//Sphere3D centerSphere = new Sphere3D(center2,radius2,10,10,material2);
			//addNonColliding(centerSphere, armRoot);
			//u.root.addChild(centerSphere);

			// Visualization.
			final JPanel3DChangeObject jpanel3D = new JPanel3DChangeObject(u);
			u.changeListeners.add(new SceneChangeListener() {
				public void sceneChanged() {
					jpanel3D.repaint();
				}			
			});

			// Create frame.
			JFrame frame = new JFrame();
			frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			frame.setSize(1000,700);
			frame.add(jpanel3D);
			frame.setVisible(true);			
			
		}
		
		// Build real-time timer.
		timer = new Timer(updateMS,new ActionListener() {
	        public void actionPerformed(ActionEvent arg0) {
	            tick(timer.getDelay());
	        }
	    });

		// If an update greater than 0 ms is given, then start the timer.
		if(updateMS > 0) {
		    timer.start();
		}
	}

	/**
	 * Set a goal for the gradient descent system.
	 * Also place object in the world.
	 **/
	public void setGoal(double[] goal) {
		this.goal = goal;

		if(goalObject != null) {
			goalObject.removeObject();
		}
		
		goalObject = new Sphere3D(goal, .1, 7, 7, new Material(Color.GREEN,null,null));
		universe.root.addChild(goalObject);
		universe.sceneChanged();
	}

	/**
	 * Return object representing the goal.
	 **/
	public Object3D getGoalObject() {
		return goalObject;
	}

	/**
	 * Return the 3D location that is set as the goal.
	 **/
	public double[] getGoal() {
		return goal;
	}	

	/**
	 * Start the simulator running in real-time.
	 **/
	public void startRealTime() {
		timer.start();
	}

	/**
	 * Pause the simulator running in real-time.
	 **/
	public void stopRealTime() {
		timer.stop();
	}

	/**
	 * Set the time between simulator updates.
	 * This is given in real time in the world.
	 **/
	public void setRealTimeSpeed(int updateMS) {
		timer.setDelay(updateMS);
	}

	/**
	 * Set how much faster or slower the simulator is running virtual-time than real-time is passing.
	 **/
	public void setRealTimeSpeedup(double simulatorSpeedup) {
		this.simulatorSpeedup = simulatorSpeedup;
	}
	
	/**
	 * Move simulation forward in time this amount.
	 * Action priority:
	 *   Gradient descent - if 'goal' set
	 *   Move to pose - if 'desiredPose' set
	 *   Move to desired velocity
	 * @param universe
	 */
	public synchronized void tick(int ms) {

		//System.out.println(getGoalAngle());
		
		if(goal != null && goalObject != null && moveToGoal) {
			performGradientDescentStep(goal, goalObject, 0);
			return;
		}
		
		// Calculate the maximum acceleration in this time step.
		double acceleration = ACCELERATION / 1000 * ms * simulatorSpeedup;

		// Check if you are still moving joints.
		boolean stillMovingToLocation = false;

		if(verbosity > 5) {
			System.out.println("Tick");
		}
		
		// Update velocity.
		for(int jointID=0;jointID<joints.size();jointID++) {
			RobotJoint joint = joints.get(jointID);

			for(int axis=0;axis<joint.rotation.length;axis++) {
				if(joint.movableJoint(axis)) {

					// For each movable joint.
					double currentVelocity = velocity[jointID][axis];
					double desiredVelocity = this.desiredVelocity[jointID][axis];
					double dv = (desiredVelocity - currentVelocity); 

					if(verbosity > 6) {
						System.out.println("Moving joint " + jointID);
						System.out.println(" Desired velocity (" + desiredVelocity + ") actual velocity (" + currentVelocity + ") dv (" + dv + ")");
					}
					
					// If no pose is specified, then just try to accelerate to the desired velocity.
					if(desiredPose == null) {


						if(dv < -acceleration) {
							velocity[jointID][axis] -= acceleration;
							if(verbosity > 7) {
								System.out.println("  Accelerating " + acceleration + " (" + velocity[jointID][axis] + ")");
							}
						} else if(dv < acceleration) {
							// If you could travel there in this tick then go there, not past.
							velocity[jointID][axis] = desiredVelocity;
							if(verbosity > 7) {
								System.out.println("  Maintaining velocity (" + velocity[jointID][axis] + ")");
							}
						} else {
							velocity[jointID][axis] += acceleration;
							if(verbosity > 7) {
								System.out.println("  Accelerating " + acceleration + " (" + velocity[jointID][axis] + ")");
							}
						}
					} else {
						
						// A pose has been specified.
						double currentAngle = (float)joints.get(jointID).rotation[getRelevantAxis(jointID)];
						double desiredAngle = desiredPose[jointID][getRelevantAxis(jointID)];

						// Change in angle.
						double da =  desiredAngle - currentAngle;

						if(verbosity > 6) {
							System.out.println(" angle:    " + currentAngle + " "  + desiredAngle + " " + da);
							System.out.println(" velocity: " + currentVelocity + " " + desiredVelocity + " " + dv);
						}
						
						// Get difference in velocity from where you are and where you want to be.
						double timeToStop = Math.abs(currentVelocity / ACCELERATION); // seconds.
						double distanceToStop = timeToStop * currentVelocity / 2;


						if(verbosity > 6) {
							System.out.println("  Distance to stop: " + distanceToStop);
						}
						
						// Check if you are going to reach/overshoot if you start decelerating.
						if((da > 0 && distanceToStop >= da) || (da > 0 && desiredVelocity < 0)) {
							// Need to start slowing down.
							velocity[jointID][axis] -= acceleration;

							// But not past stopped.
							if(velocity[jointID][axis] < 0) {
								velocity[jointID][axis] = 0;
								if(verbosity > 7) {
									System.out.println("  Stopped");
								}
							} else {
								stillMovingToLocation = true;
								if(verbosity > 7) {
									System.out.println("  Deccelerating at " + (-acceleration));
								}
							}
						} else if((da < 0 && distanceToStop <= da) || (da < 0 && desiredVelocity > 0)) {
							// Need to start slowing down.
							velocity[jointID][axis] += acceleration;

							// But not past stopped.
							if(velocity[jointID][axis] > 0) {
								if(verbosity > 7) {
									System.out.println("  Stopped");
								}
								velocity[jointID][axis] = 0;
							} else {
								if(verbosity > 7) {
									System.out.println("  Deccelerating at " + acceleration);
								}
								stillMovingToLocation = true;
							}
						} else {

							// Still accelerating or at constant velocity.
							if(dv < -acceleration) {
								velocity[jointID][axis] -= acceleration;
								if(verbosity > 7) {
									System.out.println("  Accelerating at " + (-acceleration));
								}
								stillMovingToLocation = true;
							} else if(dv < acceleration) {
								// If you could travel there in this tick then go there, not past.
								velocity[jointID][axis] = desiredVelocity;
								if(desiredVelocity == 0) {
									if(verbosity > 7) {
										System.out.println("  Maintaining velocity of zero");
									}
								} else {
									if(verbosity > 7) {
										System.out.println("  Maintaining velocity");
									}
									stillMovingToLocation = true;
								}
							} else {
								velocity[jointID][axis] += acceleration;
								if(verbosity > 7) {
									System.out.println("  Accelerating at " + acceleration);
								}
								stillMovingToLocation = true;
							}
						}
					}
				}
			}
		}

		// If at the desired location and you were moving to a location, then stop setting a desiredPose and set velocity to zero.
		if(!stillMovingToLocation && desiredPose != null) {
			desiredPose = null;
			desiredVelocity = getZeroVelocity();
		}

		if(limitVelocityOnCollision(velocity,simulatorSpeedup)) {
			if(verbosity > 2) {
				System.out.println("  Arm has exceded a joint limit.");

				for(int jointID=0;jointID<joints.size();jointID++) {
					RobotJoint joint = joints.get(jointID);
					for(int axis=0;axis<joint.rotation.length;axis++) {
						if(joint.movableJoint(axis)) {
							System.out.println(jointID + ") " + velocity[jointID][axis]);
						}
					}
				}
			}
		}



		// Update position.
		// If this needs to be more accurate we can take the average of the before and after acceleration instead of the after acceleration.
		setJoints(MatrixMath.scale(velocity,simulatorSpeedup), ms, true);
		

		if(verbosity > 5) {
			System.out.println();
		}
	}

	/**
	 * Convert a joint object into a joint id.
	 **/
	protected int getJointID(Object j) {
		if(j instanceof Integer) {
			return (Integer)j;
		} else if(j instanceof RobotJoint) {
			for(int i=0;i<joints.size();i++) {
				if(joints.get(i) == j) {
					return i;
				}
			}
		}
		return -1;
	}

    //@Override
    public synchronized void setVelocity(Object j, float v) {
    	int id = getJointID(j);
	//System.out.println("SETVELO: JOINT ID IS: " + id + "   |  Object is: " + j.toString()); //REV:
    	if(id >= 0) {
			desiredVelocity[id][getRelevantAxis(id)] = (double)v;
			desiredPose = null;
		}
    }
    
    //@Override
    public float getAngle(Object j) {
    	int id = getJointID(j);
    	if(id >= 0) {
	         return (float)joints.get(id).rotation[getRelevantAxis(id)];
    	}
    	return -1;
    }

    //@Override
    public float getUpperLimit(Object j) {
    	int id = getJointID(j);
    	if(id >= 0) {
    		return (float)joints.get(id).rotationMax[getRelevantAxis(id)];
    	}
    	return -1;
    }

    //@Override
    public float getLowerLimit(Object j) {
    	int id = getJointID(j);
    	if(id >= 0) {
    		return (float)joints.get(id).rotationMin[getRelevantAxis(id)];
    	}
    	return -1;
    }

    //@Override
    public void setAngle(Object j, float a) {
    	setAngle(j, a, 0.25f);
    }

    //@Override
    public synchronized void setAngle(Object j, float a, float v) {
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
			desiredPose[id][getRelevantAxis(id)] = (double)a;
    	}
    }

	//@Override
    public synchronized ArrayList<Object> getJoints() {
		return new ArrayList(joints);
	}

	//@Override
    public synchronized void updateJoints() {
		updateTransform(true);
	}

    /**
     * Given a joint, this finds the relevent axis number for that joint.
     * Assuming only one axis.
     * @param joint
     * @return
     */
    protected int getRelevantAxis(int joint) {
    	RobotJoint j = joints.get(joint);
    	for(int i=0;i<j.rotation.length;i++) {
    		if(j.rotationMax[i] != j.rotationMin[i]) {
    			return i;
    		}
    	}
    	return -1;
    }

    /**
     * Given a relevent joint id, this returns a point where x is the jointID and y is the axisID.
     * @param id
     * @return
     */
    protected Point getReleventID(int id) {
    	int count = 0;
    	for(int joint=0;joint<joints.size();joint++) {
    		RobotJoint j = joints.get(joint);
    		for(int axis=0;axis<j.rotation.length;axis++) {
    			if(j.rotationMax[axis] != j.rotationMin[axis]) {
        			if(id == count) {
        				return new Point(joint,axis);
        			} else {
        				count++;
        			}
        		}
    		}
    	}
    	return null;
    }
    
	/**
	 * Set velocity.
	 */
	public synchronized void setJointVelocities(double[][] velocity) {
		this.velocity = velocity;
	}

	/**
	 * Set arm moving for a given time.
	 * @param values Speed in radians per second.
	 * @param ms Time in ms
	 */
	public synchronized void setJoints(double[][] velocity, int ms, boolean showUpdate) {
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.movableJoint(j)) {
					joint.rotation[j] = normAngle(joint.rotation[j] + velocity[i][0] / 1000 * ms);
				}
			}
		}
		updateTransform(showUpdate);
	}	

	/**
	 * Set the joints in joint space values.
	 * Assumes joints are zero centered.
	 * @param values
	 * @param showUpdate
	 */
	public synchronized void setJointsZeroCentered(double[][] values, boolean showUpdate) {
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				joint.rotation[j] = normAngle(values[i][j] + (joint.rotationMax[j]  - joint.rotationMin[j]) / 2);
			}
		}
		updateTransform(showUpdate);
	}

	/**
	 * Set the joints in joint space values.
	 * @param values
	 * @param showUpdate
	 */
	public synchronized void setJoints(double[][] values, boolean showUpdate) {
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				joint.rotation[j] = normAngle(values[i][j]);
			}
		}
		updateTransform(showUpdate);
	}
	
	/**
	 * Show each joint and it's angles.
	 */
	public String toString() {
		String ret = "";
		double[][] pose = getPose();
		for(int i=0;i<pose.length;i++) {
			ret += "[" + i + ",";
			for(int j=0;j<pose[i].length;j++) {
				ret += "," + pose[i][j];
			}
			ret += "]";
		}
		return ret;
	}
	
	/**
	 * A list of joints that can be changed.
	 * @return
	 */
	public Vector<Double> getRelevantJoints() {
		Vector<Double> values = new Vector<Double>();
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.rotationMin[j] != joint.rotationMax[j]) {
					values.add(joint.rotation[j]);
				}
			}
		}
		return values;
	}


	/**
	 * A list of joints that can be changed.
	 * Converts from internal 0-n representation into -n/2-n/2 representation. 
	 * @return
	 */
	public Vector<Double> getRelevantJointsToZeroCentered() {
		Vector<Double> values = new Vector<Double>();
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.rotationMin[j] != joint.rotationMax[j]) {
					values.add(joint.rotation[j] - (joint.rotationMax[j]  - joint.rotationMin[j]) / 2);
				}
			}
		}
		return values;
	}

	/**
	 * Convert a list of joints that can be changed into a full joint specification.
	 * @param values
	 * @return
	 */
	public double[][] convertRelevantJoints(double[] values) {
		double[][] ret = new double[joints.size()][];
		int jointID = 0;
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			ret[i] = new double[joint.rotation.length];
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.rotationMin[j] != joint.rotationMax[j]) {
					ret[i][j] = values[jointID];
					jointID++;
				} else {
					ret[i][j] = joint.rotationMin[j];
				}
			}
		}
		return ret;
	}

	/**
	 * Convert a list of joints that can be changed into a full joint specification.
	 * @param values
	 * @return
	 */
	public double[][] convertRelevantJoints(Vector<Double> values) {
		double[][] ret = new double[joints.size()][];
		int jointID = 0;
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			ret[i] = new double[joint.rotation.length];
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.rotationMin[j] != joint.rotationMax[j]) {
					ret[i][j] = values.get(jointID);
					jointID++;
				} else {
					ret[i][j] = joint.rotationMin[j];
				}
			}
		}
		return ret;
	}
	
	/**
	 * Convert a list of full joint specifications into only relevent ones.
	 * @param values
	 * @return
	 */
	public double[] unconvertRelevantJoints(double[][] values) {
		Vector<Double> ret = new Vector<Double>();
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.movableJoint(j)) {
					ret.add(values[i][j]);
				}
			}
		}
		double[] ret2 = new double[ret.size()];
		for(int i=0;i<ret.size();i++) {
			ret2[i] = ret.get(i);
		}
		return ret2;
	}

	/**
	 * Return as a string, the relevent joint ranges.
	 * This is for the grid.
	 * @return
	 */
	public String getRelevantJointRanges() {
		String ret = "";		
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.rotationMin[j] != joint.rotationMax[j]) {
					ret += "joint " + i + " axis " + j + " minimum " + joint.rotationMin[j] + " maximum " + joint.rotationMax[j] + "\n";
				} else {
					ret += "joint " + i + " axis " + j + " can not move\n";
				}
			}
		}	
		
		ret += "Full ranges in min1 max1 step1 min2 max2 step2 ... format:\n-run ";
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				if(joint.rotationMin[j] != joint.rotationMax[j]) {
					ret += joint.rotationMin[j] + " " + joint.rotationMax[j] + " .1 ";
				}
			}
		}
		ret += "\n";
		
		return ret;
	}
	
	/**
	 * List the joint space pose.
	 * @return
	 */
	public double[][] getPose() {
		double[][] values = new double[joints.size()][];
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			values[i] = new double[joint.rotation.length];
			for(int j=0;j<joint.rotation.length;j++) {
				values[i][j] = joint.rotation[j];
			}
		}
		return values;
	}
	

	/**
	 * Specify a goal to move to.
	 **/
	public void buildControlPanelSetGoal() {
		JPanel goalPanel = new JPanel(new GridLayout(5,0));
		final JTextField xAxis = new JTextField();
		final JTextField yAxis = new JTextField();
		final JTextField zAxis = new JTextField();

		JButton setGoal = new JButton("set goal");
		setGoal.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				try {
					double[] point = {	Double.parseDouble(xAxis.getText()),
										Double.parseDouble(yAxis.getText()),
										Double.parseDouble(zAxis.getText())};
					setGoal(point);
				} catch(NumberFormatException e) {
				}
			}
		});

		final JCheckBox moveToGoalCheckBox = new JCheckBox("move to goal");
		moveToGoalCheckBox.setSelected(false);
		moveToGoalCheckBox.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				moveToGoal = moveToGoalCheckBox.isSelected();
			}
		});
		
		JPanel xAxisPanel = new JPanel(new BorderLayout());
		JPanel yAxisPanel = new JPanel(new BorderLayout());
		JPanel zAxisPanel = new JPanel(new BorderLayout());

		xAxisPanel.add(new JLabel("X axis"),BorderLayout.WEST);
		xAxisPanel.add(xAxis,BorderLayout.CENTER);
		yAxisPanel.add(new JLabel("Y axis"),BorderLayout.WEST);
		yAxisPanel.add(yAxis,BorderLayout.CENTER);
		zAxisPanel.add(new JLabel("Z axis"),BorderLayout.WEST);
		zAxisPanel.add(zAxis,BorderLayout.CENTER);

		goalPanel.add(xAxisPanel);
		goalPanel.add(yAxisPanel);
		goalPanel.add(zAxisPanel);
		goalPanel.add(setGoal);
		goalPanel.add(moveToGoalCheckBox);
		
		// Create frame.
		JFrame goalFrame = new JFrame("Select a goal.");
		goalFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		goalFrame.setSize(200,200);
		goalFrame.add(goalPanel);
		goalFrame.setVisible(true);	
	}


	/**
	 * Specify a pose to move to.
	 **/
	public void buildControlPanelMoveToPose() {

		JFrame frame = new JFrame("Select point to move to.");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setSize(250,800);
		Container container = frame.getContentPane();
		container.setLayout(new FlowLayout());
		
		String[] axis = {"X","Y","Z"};

		final Vector<JSlider> sliders = new Vector<JSlider>();
		
		// For each joint.
		for(RobotJoint joint : joints) {
			
			// Joint panel.
			JPanel jointPanel = new JPanel(new FlowLayout());
			container.add(jointPanel);
			
			jointPanel.setBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.LIGHT_GRAY, 1), joint.jointName));
			
			// For each axis.
			for(int i=0;i<joint.rotation.length;i++) {
				if(joint.rotationMax[i] != joint.rotationMin[i]) {
					JSlider slider = new JSlider();
					slider.setMinimum((int)(joint.rotationMin[i] * 100));
					slider.setMaximum((int)(joint.rotationMax[i] * 100));
					slider.setValue((int)(joint.rotation[i] * 100));
					slider.setMajorTickSpacing(100);
					slider.setPaintTicks(true);
					slider.setPaintLabels(true);
					sliders.add(slider);
					
					JPanel labelPanel = new JPanel(new BorderLayout());
					labelPanel.add(slider,BorderLayout.CENTER);
					labelPanel.add(new JLabel(axis[i]),BorderLayout.NORTH);
					jointPanel.add(labelPanel);
				}
			}
		}
		
		// Request movement to that pose.
		JButton start = new JButton("start");
		start.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				System.out.println("Setting joint desired angles.");
				for(int i=0;i<sliders.size();i++) {
					float a = sliders.get(i).getValue() / 100.0f;
					System.out.println("Setting joint " + i + " to angle " + a);
					setAngle(new Integer(i), a);
				}
				System.out.println();
			}
		});
		container.add(start);
		
		frame.setVisible(true);
	}
	
	/**
	 * A control panel that allows manipulation of the joints through sliders.
	 */
	public void buildControlPanelSetPose() {
				
		JFrame frame = new JFrame("Move joints directly.");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setSize(250,800);
		Container container = frame.getContentPane();
		container.setLayout(new FlowLayout());
		
		String[] axis = {"X","Y","Z"};

		// For each joint.
		for(RobotJoint joint : joints) {
			
			// Joint panel.
			JPanel jointPanel = new JPanel(new FlowLayout());
			container.add(jointPanel);
			
			jointPanel.setBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.LIGHT_GRAY, 1), joint.jointName));
			
			// For each axis.
			for(int i=0;i<joint.rotation.length;i++) {
				if(joint.rotationMax[i] != joint.rotationMin[i]) {
					final RobotJoint activeJoint = joint;
					final JSlider slider = new JSlider();
					final int sliderID = i;
					slider.setMinimum((int)(joint.rotationMin[i] * 100));
					slider.setMaximum((int)(joint.rotationMax[i] * 100));
					slider.setValue((int)(joint.rotation[i] * 100));
					slider.setMajorTickSpacing(100);
					slider.setPaintTicks(true);
					slider.setPaintLabels(true);
					slider.addChangeListener(new ChangeListener() {
						public void stateChanged(ChangeEvent arg0) {
							activeJoint.rotation[sliderID] = slider.getValue() / 100.0;
							updateTransform(true);
						}				
					});
					
					JPanel labelPanel = new JPanel(new BorderLayout());
					labelPanel.add(slider,BorderLayout.CENTER);
					labelPanel.add(new JLabel(axis[sliderID]),BorderLayout.NORTH);
					jointPanel.add(labelPanel);
				}
			}
		}
		
		frame.setVisible(true);
	}
	
	/**
	 * This is just a pair of objects joined together.
	 * This is primarily used for collision ignoring.
	 * @author bkievitk
	 */
	class ObjectPair {
		
		// Two objects.
		public Object3D object1;
		public Object3D object2;

		public ObjectPair(Object3D object1, Object3D object2) {
			this.object1 = object1;
			this.object2 = object2;
		}
		
		/**
		 * The hash is the xor of both.
		 */
		public int hashCode() {
			return object1.hashCode() ^ object2.hashCode();
		}

		/**
		 * Either order matches.
		 */
		public boolean equals(Object o) {
			if(o instanceof ObjectPair) {
				ObjectPair o2 = (ObjectPair)o;
				if(	(object1.equals(o2.object1) && object2.equals(o2.object2)) ||
					(object1.equals(o2.object2) && object2.equals(o2.object1))) {
					return true;
				}
			}
			return false;
		}
	}
	
	/**
	 * Add a pair of objects that are not allowed to collide.
	 * @param o1	Object 1
	 * @param o2	Object 2
	 */
	public void addNonColliding(Object3D o1, Object3D o2) {
		nonColliding.add(new ObjectPair(o1,o2));
	}

	/**
	 * Test if two objects are on the non-collision list.
	 * @param o1
	 * @param o2
	 * @return
	 */
	private boolean canCollide(Object3D o1, Object3D o2) {
		return !nonColliding.contains(new ObjectPair(o1,o2));
	}

	/**
	 * Apply one step of gradient descent search.
	 * Use global verbosity level.
	 * @param goal		Location in three space that you want the hand to be in.
	 * @param goalObj	The object that you want to reach.
	 */
	public boolean performGradientDescentStep(double[] goal, Object3D goalObj) {
		return performGradientDescentStep(goal,goalObj,verbosity);
	}

	/**
	 * Apply one step of gradient descent search.
	 * @param goal		Location in three space that you want the hand to be in.
	 * @param goalObj	The object that you want to reach.
	 */
	public boolean performGradientDescentStep(double[] goal, Object3D goalObj, int verbosity) {
		
		if(!moveToGoal) {
			return true;
		}

		if(verbosity >= 5) { System.out.println("Running Descent"); }
		
		// Store your original location.
		double[][] originalPose = getPose();
				
		// Find the best location to go to.
		double bestFitness = fitness(goal,goalObj);
		int bestJoint = -1;
		int bestAxis = -1;
		boolean bestDirectionPlus = true;
		
		if(verbosity >= 5) { System.out.println("Current Fitness " + bestFitness); }
		
		// Maximum angle that the joint can move in this time step.
		double step = .05;
		
		// Try every joint and angle.
		for(int jointID=0;jointID<originalPose.length;jointID++) {
			RobotJoint joint = joints.get(jointID);
			
			for(int axis=0;axis<originalPose[jointID].length;axis++) {

				// If this joint can move in this axis.
				if(joint.rotationMax[axis] != joint.rotationMin[axis]) {
					
					// This is the pose to test.
					double[][] pose = MatrixMath.copy(originalPose);
					pose[jointID][axis] = normAngle(pose[jointID][axis] + step);
					setJoints(pose,false);
					
					// Get fitness and see if it is the best so far.
					double fitness = fitness(goal,goalObj, verbosity);
					if(fitness > bestFitness) {
						bestFitness = fitness;
						bestJoint = jointID;
						bestAxis = axis;
						bestDirectionPlus = true;
					}
					
					// Try moving this joint in the other direction.
					pose = MatrixMath.copy(originalPose);
					pose[jointID][axis] = normAngle(pose[jointID][axis] - step);
					setJoints(pose,false);
	
					// Get fitness and see if it is the best so far.
					fitness = fitness(goal,goalObj, verbosity);
					if(fitness > bestFitness) {
						bestFitness = fitness;
						bestJoint = jointID;
						bestAxis = axis;
						bestDirectionPlus = false;
					}
				}
			}
		}
		
		// If you can move somewhere better than you are...
		if(bestJoint >= 0) {
			
			if(verbosity >= 5) { System.out.println("Best fitness found at joint(" + bestJoint + ") axis(" + bestAxis + ") with fitness of " + bestFitness); }

			// Make the movement.
			double[][] pose = MatrixMath.copy(originalPose);
			if(bestDirectionPlus) {
				pose[bestJoint][bestAxis] += step;
			} else {
				pose[bestJoint][bestAxis] -= step;
			}
			setJoints(pose,true);
			return true;
			
		} else {
			if(verbosity >= 5) { System.out.println("Unable to find better location."); }
			
			// Otherwise stay where you were.
			setJoints(originalPose,false);
			return false;
		}
	}
	
	/**
	 * Get the centroid of the gripper that is the last joint.
	 * @return
	 */
	public double[] getHandCentroid() {
		Vector<double[]> pts = endEffector.getPointsInSpace();
		return MatrixMath.average(pts);
	}

	/**
	 * Get the centroid of every robot joint.
	 * @return
	 */
	public Vector<double[]> getPartsCentroid() {
		Vector<double[]> centroids = new Vector<double[]>();
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			Vector<double[]> pts = joint.shape.getPointsInSpace();
			centroids.add(MatrixMath.average(pts));
		}
		return centroids;
	}
	
	/**
	 * Get the fitness of the given arm location.
	 * Sets to new location and then returns back before exiting.
	 * @param goal
	 * @param goalObj
	 * @return
	 */
	public double fitness(double[] goal, double[][] pose, Object3D goalObj) {
		return fitness(goal,pose,goalObj,verbosity);
	}

	/**
	 * Get the fitness of the given arm location.
	 * Sets to new location and then returns back before exiting.
	 * @param goal
	 * @param goalObj
	 * @return
	 */
	public synchronized double fitness(double[] goal, double[][] pose, Object3D goalObj, int verbosity) {
		double[][] oldPose = getPose();
		setJoints(pose,false);
		double fitness = fitness(goal,goalObj,verbosity);
		setJoints(oldPose,false);
		return fitness;
	}

	/**
	 * Get the fitness of this arm location.
	 * Use global verbosity level.
	 * @param goal
	 * @param goalObj
	 * @return
	 */
	public double fitness(double[] goal, Object3D goalObj) {
		return fitness(goal,goalObj,verbosity);
	}
	
	public double getGoalAngle() {
		TransformMy3D t = endEffector.getTransform();
		double[] unit = {0,0,1};
		double[] goalAngle = {-0.8226824387888697,0.568410932652049,0.010129982725835502};
		double[] unitTransformed = t.apply(unit);
		double[] newUnit = MatrixMath.normalize(unitTransformed);
		double cos = MatrixMath.cos(newUnit, goalAngle);
		System.out.println(newUnit[0] + "," + newUnit[1] + "," + newUnit[2]);
		return cos;
	}
	
	/**
	 * Get the fitness of this arm location.
	 * @param goal
	 * @param goalObj
	 * @return
	 */
	public double fitness(double[] goal, Object3D goalObj, int verbosity) {
		
		double fitness = 0;
		double collisionWeight = 0.0;

		double objectStrength = 0;

		// Get distance to goal.
		double[] handCentroid = getHandCentroid();
		double distToGoal = MatrixMath.dist(handCentroid, goal);
		fitness -= distToGoal;
		
		TransformMy3D t = endEffector.getTransform();
		double[] unit = {0,0,1};
		double[] goalAngle = {-0.8226824387888697,0.568410932652049,0.010129982725835502};
		double[] unitTransformed = t.apply(unit);
		double[] newUnit = MatrixMath.normalize(unitTransformed);
		double cos = MatrixMath.cos(newUnit, goalAngle);
		
		//fitness -= cos;
		
		if(verbosity >= 9) { System.out.println("Distance to goal: " + distToGoal); }
		if(verbosity >= 9) { System.out.println("Distance to goal angle: " + cos); }
		
		// Strongly penalize collision.
		if(armCollision()) {
			return -10000;
		}
		
		// Get point distances to obstructions.
		
		// This is a list of the centroids of all pieces of the arm.
		Vector<double[]> armCentroids = getPartsCentroid();
		
		// This is a list of all objects in the world as we find them.
		LinkedList<Object3D> objects = new LinkedList<Object3D>();
		objects.add(universe.root);
		
		// Try all objects.
		while(objects.size() > 0) {
			
			// Get object.
			Object3D object = objects.removeFirst();
			
			// Do not care if it is yourself or the goal.
			// Only stay away from other objects.
			if(object != joints.get(0).shape && object != goalObj) {
				for(Object3D child : object.getChildren()) {
					objects.add(child);
				}
								
				// Get the centroid of this object.
				Vector<double[]> pts = object.getPointsInSpace();
				double[] objCentroid = MatrixMath.average(pts);

				// Make sure there is actually something here.
				// The NullObject3D will return null.
				if(objCentroid != null) {
					
					// Fitness gets penalized for every distance similarity between every arm piece and every element in the world.
					for(double[] armCentroid : armCentroids) {
						double distToObject = MatrixMath.dist(armCentroid, objCentroid);
						fitness -= (1 / distToObject) * collisionWeight;
					}
				}
			}
		}
		

		if(verbosity >= 9) { System.out.println("Fitness: " + fitness); }

		// Penalize for arm angles collision.
		return fitness;
	}
	
	/**
	 * This object represents a pose of the robot.
	 * It is used for A* planning.
	 **/
	class AStarObjectPose extends AStarObject {

		// Actual pose.
		public double[] pose;

		// Number of steps over that each pose is.
		public int[] intPose;

		/**
		 * Two poses are equal if their step pose is equal.
		 **/
		public boolean equals(Object o) {
			if(o instanceof AStarObjectPose) {
				AStarObjectPose other = (AStarObjectPose)o;
				for(int i=0;i<pose.length;i++) {
					if(intPose[i] != other.intPose[i]) {
						return false;
					}
				}
				return true;
			}
			return false;
		}
		
		/**
		 * The hash code is based on the integer pose code so equal objects will have the same hash value.
		 **/
		public int hashCode() {
			int hash = 0;
			for(int i=0;i<intPose.length;i++) {
				int doubleHash = (new Integer(intPose[i])).hashCode();				
				doubleHash = (doubleHash << i) | (doubleHash >>> (32-i));
				hash ^= doubleHash;
			}
			return 0;
		}
		
		/**
		 * Show each actual pose angle.
		 **/
		public String toString() {
			String ret = "";
			for(int i=0;i<pose.length;i++) {
				ret += "[" + pose[i] + "]";
			}
			return ret;
		}

		/**
		 * In initialization, calculate the step pose.
		 **/		
		public AStarObjectPose(double[] pose) {
			this.pose = pose;
			
			intPose = new int[pose.length];
			for(int i=0;i<pose.length;i++) {
				intPose[i] = (int)((pose[i] - joints.get(i).rotationMin[getRelevantAxis(i)]) / .1);
			}
		}
		
		/**
		 * Using city block distance.
		 * This is since A* is only planning connections between directly ajacent poses in this implementation.
		 **/
		public double distance(AStarObject other) {
			if(other instanceof AStarObjectPose) {
				double sum = 0;
				for(int i=0;i<pose.length;i++) {
					sum += Math.abs(pose[i] - ((AStarObjectPose)other).pose[i]);
				}
				return sum;
			} else {
				return -1;
			}
		}
		
		/**
		 * Get all valid moves out of this pose.
		 **/
		public Vector<AStarObject> linksTo() {
			
			// Test this point.
			double[][] pointFull = convertRelevantJoints(pose);
			setJoints(pointFull, false);

			Vector<AStarObject> goTo = new Vector<AStarObject>();
			
			if(!armCollision()) {
				
				double[] pointCopy = MatrixMath.copy(pose);
				
				// Only bother with neighbors if this is a valid point.
				for(int i=0;i<pose.length;i++) {
					pointCopy[i] = pose[i] - .1;
					if(pointCopy[i] >= joints.get(i).rotationMin[getRelevantAxis(i)]) {
						// Test this connection.
						double[][] pointCopyFull = convertRelevantJoints(MatrixMath.copy(pointCopy));
						if(testPath(pointFull, pointCopyFull)) {
							AStarObjectPose aStarPose = new AStarObjectPose(MatrixMath.copy(pointCopy));
							//System.out.println(aStarPose);
							goTo.add(aStarPose);
						}
					}
					pointCopy[i] = pose[i] + .1;
					if(pointCopy[i] <= joints.get(i).rotationMax[getRelevantAxis(i)]) {
						// Test this connection.
						double[][] pointCopyFull = convertRelevantJoints(pointCopy);
						
						if(testPath(pointFull, pointCopyFull)) {
							AStarObjectPose aStarPose = new AStarObjectPose(MatrixMath.copy(MatrixMath.copy(pointCopy)));
							//System.out.println(aStarPose);
							goTo.add(aStarPose);
						}
					}
					pointCopy[i] = pose[i];
				}
			}
			return goTo;
		}
	}

	/**
	 * Given this goal, get the path of points to travel to using A*.
	 **/
	public Vector<double[]> getAStarPath(double[][] goal) {
		
		// Get start and goal pose.
		AStarObject start = new AStarObjectPose(unconvertRelevantJoints(this.getPose()));
		AStarObject stop = new AStarObjectPose(unconvertRelevantJoints(goal));
		
		// Solve path.
		Vector<AStarObject> path = AStar.aStar(start, stop, verbosity);
		
		// Show path.
		for(AStarObject pose : path) {
			System.out.println(pose);
		}
		
		Vector<double[]> pathRet = new Vector<double[]>();
		for(AStarObject object : path) {
			pathRet.add(((AStarObjectPose)object).pose);
		}
		
		return pathRet;
	}

	/**
	 * Checks if you are at a given pose within the speed tolerance.
	 * @param from	Your location.
	 * @param to	The goal location.
	 * @return
	 */
	private boolean atWaypoint(double[][] from, double[][] to) {
		for(int i=0;i<from.length;i++) {
			double diffX = Math.abs(from[i][0] - to[i][0]);
			double diffY = Math.abs(from[i][1] - to[i][1]);
			if(diffX > jointSpeed || diffY > jointSpeed) {
				return false;
			}
		}
		return true;
	}
	
	/**
	 * Test to see if every step of the path is a non-colliding point.
	 * @param start	Start pose.
	 * @param stop	Ending pose.
	 * @return
	 */
	public synchronized boolean testPath(double[][] start, double[][] stop) {
				
		double[][] storePath = getPose();
		setJoints(start, false);

		// Start bad.
		if(armCollision()) {
			setJoints(storePath, false);
			return false;
		}
		
		int maxSteps = 0;
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<joint.rotation.length;j++) {
				int steps = (int)(Math.abs(joint.rotation[j] - stop[i][j]) / jointSpeed + 1);
				maxSteps = Math.max(steps, maxSteps);
			}
		}

		// Keep moving.
		for(;maxSteps>=0;maxSteps--) {
			double[][] atJoint = getPose();
			double[][] newAngle = moveToward(atJoint, stop, jointSpeed);
						
			setJoints(newAngle, false);
			
			if(armCollision()) {
				setJoints(storePath, false);
				return false;
			}
			
			if(atWaypoint(newAngle, stop)) {
				setJoints(storePath, false);
				return true;
			}
		}

		System.out.println("Error in reaching the testing point.");
		return false;
	}	

	/**
	 * Move the robot pose one tick in the correct direction.
	 * @param from		Starting pose.
	 * @param to		Desired pose.
	 * @param maxAngle	Angle turn speed.
	 * @return			New pose.
	 */
	private double[][] moveToward(double[][] from, double[][] to, double maxAngle) {
		double[][] newAngle = new double[from.length][];
		for(int i=0;i<from.length;i++) {
			newAngle[i] = new double[from.length];
			for(int j=0;j<from[i].length;j++) {
				newAngle[i][j] = moveAngle(from[i][j],to[i][j],maxAngle);
			}
		}
		return newAngle;
	}	

	/**
	 * Move between angles in the quickest direction possible.
	 * @param from	Starting angle.
	 * @param to	Stopping angle.
	 * @param speed	Maximum speed.
	 * @return		New angle.
	 */
	private double moveAngle(double from, double to, double speed) {

		// Get the distance in radians between the points.
		double dist = Math.abs(to - from);
		
		// Cycle distance.
		if(dist > Math.PI) {
			dist = Math.PI * 2 - dist;
		}
		
		// If you can get there in one turn then just do it exactly.
		if(dist < speed) {
			return to;
		}
		
		// Move in the correct direction.
		if(to > from) {
			if(to - from > Math.PI) {
				return normAngle(from - speed);
			} else {
				return normAngle(from + speed);
			}
		} else {
			if(from - to > Math.PI) {
				return normAngle(from + speed);
			} else {
				return normAngle(from - speed);
			}
		}
	}
	
	/**
	 * Normalize and angle between -2PI - 4PI into the range 0 - 2PI
	 * @param angle	Starting angle.
	 * @return		Normalized angle.
	 */
	private static double normAngle(double angle) {
		if(angle > Math.PI * 2) {
			return angle - Math.PI * 2;
		} else if(angle < 0) {
			return angle + Math.PI * 2;
		} else {
			return angle;
		}
	}
	
	/**
	 * Get a random pose that is valid for the arm angles.
	 * @return
	 */
	public double[][] getRandomPose() {
		double[][] pose = new double[getPose().length][];
		for(int i=0;i<pose.length;i++) {
			RobotJoint joint = joints.get(i);
			pose[i] = new double[joint.rotation.length];
			for(int j=0;j<joint.rotation.length;j++) {
				pose[i][j] = rand.nextDouble() * (joint.rotationMax[j] - joint.rotationMin[j]) + joint.rotationMin[j];
			}
		}
		return pose;
	}
	
	/**
	 * Get a random velocity for each joint in the range [-max,max].
	 * Do not give velocity to joints with no movement.
	 * @param max
	 * @return
	 */
	public double[][] getRandomVelocity(double max) {
		double[][] pose = new double[getPose().length][];
		for(int i=0;i<pose.length;i++) {
			RobotJoint joint = joints.get(i);
			pose[i] = new double[joint.rotation.length];
			for(int j=0;j<joint.rotation.length;j++) {
				
				// Only set velocity on joint if it can move.
				if(joint.rotationMin[j] == joint.rotationMax[j]) {
					pose[i][j] = 0;
				} else {
					pose[i][j] = rand.nextDouble() * max * 2 - max;
				}
			}
		}
		return pose;
	}

	/**
	 * Get a random velocity for each joint in the range [-max,max].
	 * Do not give velocity to joints with no movement.
	 * @param max
	 * @return
	 */
	public double[][] getZeroVelocity() {
		double[][] pose = new double[getPose().length][];
		for(int i=0;i<pose.length;i++) {
			RobotJoint joint = joints.get(i);
			pose[i] = new double[joint.rotation.length];
		}
		return pose;
	}
	
	/**
	 * Test for collision of arm.
	 * @return
	 */
	public boolean armCollision() {
		for(RobotJoint joint : joints) {
			joint.shape.setAllMaterialColor(Color.LIGHT_GRAY);
		}
		
		boolean collision = false;
		for(int i=0;i<joints.size();i++) {
			if(armCollision(i,universe.root)) {
				collision = true;
			}
		}
		
		// Check joint collisions.
		for(int i=0;i<joints.size();i++) {
			if(	!joints.get(i).validAngles() ) {
				joints.get(i).shape.setAllMaterialColor(Color.RED);
				if(i > 0) {
					joints.get(i-1).shape.setAllMaterialColor(Color.RED);
				}
				collision = true;
			}
		}
		
		return collision;
	}	

	/**
	 * If there will be a collision at any of the joints, then set the velocity of that joint to zero.
	 **/
	public boolean limitVelocityOnCollision(double[][] velocity, double simulatorSpeedup) {
		boolean hadCollision = false;
		for(int i=0;i<joints.size();i++) {
			RobotJoint joint = joints.get(i);
			for(int j=0;j<velocity[i].length;j++) {
				if(joint.movableJoint(j)) {
					double newAngle = normAngle(joint.rotation[j] + velocity[i][j] * simulatorSpeedup);
					if(newAngle < joint.rotationMin[j] || newAngle > joint.rotationMax[j]) {
					    
					    double distToMin = Math.abs(newAngle - joint.rotationMin[j]);
					    if(distToMin > Math.PI) {
						distToMin = Math.PI * 2 - distToMin;
					    }

					    double distToMax = Math.abs(newAngle - joint.rotationMax[j]);
					    if(distToMax > Math.PI) {
						distToMax = Math.PI * 2 - distToMax;
					    }

						/*
						// Force joint to the limit.
						// For some reason, this seems to cause skips.

					    if(distToMax > distToMin) {
							joint.rotation[j] = joint.rotationMin[j];
					    } else {
							joint.rotation[j] = joint.rotationMax[j];
					    }

						if(newAngle < joint.rotationMin[j]) {
							joint.rotation[j] = joint.rotationMin[j];
						} else {
							joint.rotation[j] = joint.rotationMax[j];
						}
					    */
						velocity[i][j] = 0;
						hadCollision = true;
					}
				} 
			}
		}
		return hadCollision;
	}
	
	/**
	 * Test joint for collision with object.
	 * @param jointID
	 * @param object
	 * @return
	 */
	private boolean armCollision(int jointID, Object3D object) {

		// End if no object.
		if(object == null) {
			return false;
		}
		
		Object3D thisObject = joints.get(jointID).shape;		
		if(	canCollide(object,thisObject) && object != joints.get(jointID).shape) {
			RobotJoint joint = joints.get(jointID);
			if(joint.shape.collision(object)) {
				joint.shape.setAllMaterialColor(Color.RED);
				return true;
			}
		}

		for(Object3D child : object.getChildren()) {
			if(armCollision(jointID,child)) {
				return true;
			}
		}
		
		return false;		
	}
	
	/**
	 * Apply rotation transforms, check for collision, update scene if applicable.
	 * @param showUpdate
	 */
	public boolean updateTransform(boolean showUpdate) {

		for(int i=0;i<joints.size();i++) {			
			joints.get(i).setTransform();
		}
		
		// check external collisions.
		boolean collision = armCollision();

		if(showUpdate) {
			universe.sceneChanged();
		}
		
		return collision;
	}
	
}
