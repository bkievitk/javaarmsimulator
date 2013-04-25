package com.armsim.my3DRobotArm;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.IOException;
import java.util.Hashtable;
import java.util.Vector;

import com.armsim.my3DCore.Material;
import com.armsim.my3DCore.TransformMy3D;
import com.armsim.my3DShapes.*;

public class XMLParseRobotArm {
	
	// Just an editable number.
	private static class LineNumber {
		public int number;		
		public LineNumber() {
			number = 0;
		}
	}

	public static void showError(LineNumber lineNum, String message) {
		System.out.println("Error on line " + lineNum.number + ": " + message);
	}
	
	/**
	 * Parse a robot arm object from a reader.
	 * @param r
	 * @return
	 * @throws IOException
	 */
	public static boolean parseXML(BufferedReader r, RobotArm robotArm) throws IOException {
		
		String line;
		LineNumber lineNum = new LineNumber();
		Hashtable<String,RobotJoint> joints = new Hashtable<String,RobotJoint>();
		
		while((line = r.readLine()) != null) {
			lineNum.number ++;
			line = line.trim();
			
			if(line.startsWith("#")) {
				// Comment.
			} else if(line.startsWith("<joint>")) {
				RobotJoint joint = parseXMLJoint(r,robotArm, lineNum);				
				if(joint == null) {
					showError(lineNum, "Loading joint " + joints.size());
					return false;
				}			
				if(joints.containsKey(joint.jointName)) {
					showError(lineNum, "The joint name [" + joint.jointName + "] can only be set for one joint.");
					return false;
				}
				joints.put(joint.jointName, joint);
				robotArm.joints.add(joint);
			} else if(line.startsWith("<connect>")) {				
				if(!parseXMLConnection(r, joints, lineNum)) {
					return false;
				}
			} else if(line.startsWith("<noCollision>")) {
				if(!parseXMLCollision(r, joints, robotArm, lineNum)) {
					return false;
				}
			} else if(line.startsWith("<root>")) {
				String name = removeTags(line);
				RobotJoint j = joints.get(name);
				if(j == null) {
					showError(lineNum, "Joint name [" + name + "] is unknown and can not be set as root.");
					return false;
				} else {
					robotArm.armRoot = j.shape;
				}
			} else if(line.startsWith("<endEffector>")) {				
				String name = removeTags(line);
				RobotJoint j = joints.get(name);
				if(j == null) {
					showError(lineNum, "Joint name [" + name + "] is unknown and can not be set as end effector.");
					return false;
				} else {
					robotArm.endEffector = j.shape;
				}
			}
		}
		
		if(robotArm.armRoot == null) {
			System.out.println("Error global: No arm root specified.");
			return false;
		} if(robotArm.endEffector == null) {
			System.out.println("Error global: End effector not specified.");
			return false;
		}
		
		robotArm.updateTransform(true);	
		return true;
	}

	public static boolean parseXMLCollision(BufferedReader r, Hashtable<String,RobotJoint> joints, RobotArm robotArm, LineNumber lineNum) throws IOException {
		String line;	

		RobotJoint first = null;
		RobotJoint second = null;
		
		while((line = r.readLine()) != null) {
			lineNum.number++;
			line = line.trim();
			
			if(line.startsWith("#")) {
				// Comment.
			} else if(line.startsWith("<first>")) {
				String name = removeTags(line);
				first = joints.get(name);
				if(first == null) {
					showError(lineNum, "First collision segment [" + name + "] is unknown and can not be set as a collision element.");
					return false;
				}
			} else if(line.startsWith("<second>")) {	
				String name = removeTags(line);
				second = joints.get(name);			
				if(second == null) {
					showError(lineNum, "Second collision segment [" + name + "] is unknown and can not be set as a collision element.");
					return false;
				}
			} else if(line.startsWith("</noCollision>")) {				
				if(first == null) {
					showError(lineNum, "First collision object not set.");
					return false;
				} if(second == null) {
					showError(lineNum, "Second collision object not set.");
					return false;
				}
				
				robotArm.addNonColliding(first.shape, second.shape);
				return true;
			}
		}
		
		showError(lineNum, "File ended prematurely.");
		return false;
	}
	
	public static boolean parseXMLConnection(BufferedReader r, Hashtable<String,RobotJoint> joints, LineNumber lineNum) throws IOException {
		String line;	

		RobotJoint parent = null;
		RobotJoint child = null;
		
		while((line = r.readLine()) != null) {
			lineNum.number++;
			line = line.trim();
			
			if(line.startsWith("#")) {
				// Comment.
			} else if(line.startsWith("<parent>")) {
				String name = removeTags(line);
				parent = joints.get(name);
				if(parent == null) {
					showError(lineNum, "Parent segment [" + name + "] is unknown and can not be set as a parent element.");
					return false;
				}
			} else if(line.startsWith("<child>")) {
				String name = removeTags(line);
				child = joints.get(name);			
				if(child == null) {
					showError(lineNum, "Child segment [" + name + "] is unknown and can not be set as a child element.");
					return false;
				}
			} else if(line.startsWith("</connect>")) {	
				
				if(parent == null) {
					showError(lineNum, "Parent object not set.");
					return false;
				} if(child == null) {
					showError(lineNum, "Child object not set.");
					return false;
				}
				
				parent.shape.addChild(child.shape);
				return true;
			}
		}

		showError(lineNum, "File ended prematurely.");
		return false;
	}
	
	public static RobotJoint parseXMLJoint(BufferedReader r, RobotArm robotArm, LineNumber lineNum) throws IOException {
		
		String line = null;
		RobotJoint robotJoint = new RobotJoint();
		
		try {
			while((line = r.readLine()) != null) {
				lineNum.number++;
				line = line.trim();
				
				if(line.startsWith("#")) {
				} else if(line.startsWith("<object>")) {
					robotJoint.shape = parseXMLObject(r,lineNum);
					// Error loading shape.
					if(robotJoint.shape == null) {
						return null;
					}
				} else if(line.startsWith("<rotationPositive>")) {
					String positivesString = removeTags(line);
					positivesString = positivesString.trim().toLowerCase();
					if(positivesString.length() != 3) {
						showError(lineNum, "Positive line must contain only + or - signs and there must be three of them.");
						return null;
					}
					boolean[] positives = {positivesString.charAt(0) == '+',positivesString.charAt(1) == '+',positivesString.charAt(2) == '+'};
					robotJoint.rotationPositive = positives;
				} else if(line.startsWith("<transform>")) {
					TransformMy3D transform = parseXMLTransform(r,lineNum);
					
					// Error loading shape.
					if(transform == null) {
						return null;
					} else if(robotJoint.shape == null) {
						showError(lineNum, "Attempt to set transform before shape has been set.");
						return null;
					}				
					robotJoint.shape.permApplyTransform(transform);				
				} else if(line.startsWith("<centerOfRotationParent>")) {
					robotJoint.centerOfRotationParent = parseEquasions(removeTags(line));
					if(robotJoint.centerOfRotationParent == null) {
						return null;
					}
				} else if(line.startsWith("<centerOfRotationChild>")) {
					robotJoint.centerOfRotationChild = parseEquasions(removeTags(line));
					if(robotJoint.centerOfRotationChild == null) {
						return null;
					}
				} else if(line.startsWith("<rotation>")) {
					robotJoint.rotation = parseEquasions(removeTags(line));
					if(robotJoint.rotation == null) {
						return null;
					}
				} else if(line.startsWith("<rotationZero>")) {
					robotJoint.rotationZero = parseEquasions(removeTags(line));
					if(robotJoint.rotationZero == null) {
						return null;
					}
				} else if(line.startsWith("<rotationMin>")) {
					robotJoint.rotationMin = parseEquasions(removeTags(line));
					if(robotJoint.rotationMin == null) {
						return null;
					}
				} else if(line.startsWith("<rotationMax>")) {
					robotJoint.rotationMax = parseEquasions(removeTags(line));
					if(robotJoint.rotationMax == null) {
						return null;
					}
				} else if(line.startsWith("<name>")) {
					robotJoint.jointName = removeTags(line);
				} else if(line.startsWith("</joint>")) {
	
					if(robotJoint.shape == null) {
						showError(lineNum, "No shape specified.");
						return null;
					} else if(robotJoint.centerOfRotationParent == null) {
						showError(lineNum, "No center of rotation for parent specified.");
						return null;
					} else if(robotJoint.centerOfRotationChild == null) {
						showError(lineNum, "No center of rotation for child specified.");
						return null;
					} else if(robotJoint.rotation == null) {
						showError(lineNum, "No rotation specified.");
						return null;
					} else if(robotJoint.rotationMin == null) {
						showError(lineNum, "No rotation min specified.");
						return null;
					} else if(robotJoint.rotationMax == null) {
						showError(lineNum, "No rotation max specified.");
						return null;
					} else if(robotJoint.jointName == null) {
						showError(lineNum, "No joint name specified.");
						return null;
					}
					
					if(robotJoint.rotationZero == null) {
						robotJoint.rotationZero = new double[3];
					}
					
					return robotJoint;
				}
			}
		} catch(Exception e) {
			showError(lineNum, "Number parsing error [" + line + "].");
			return null;
		}

		showError(lineNum, "File ended prematurely.");
		return null;
	}
	
	public static TransformMy3D parseXMLTransform(BufferedReader r, LineNumber lineNum) throws IOException {
		
		String line = null;
		TransformMy3D transform = new TransformMy3D();

		try {
			while((line = r.readLine()) != null) {
				lineNum.number ++;
				line = line.trim();
	
				if(line.startsWith("#")) {
				} else if(line.startsWith("<translate>")) {
					double[] data = parseEquasions(removeTags(line));
					if(data == null) {
						return null;
					}
					transform.combine(TransformMy3D.translate(data[0], data[1], data[2]));
				} else if(line.startsWith("<rotateX>")) {
					transform.combine(TransformMy3D.rotateX(parseEquasion(removeTags(line))));
				} else if(line.startsWith("<rotateY>")) {
					transform.combine(TransformMy3D.rotateY(parseEquasion(removeTags(line))));
				} else if(line.startsWith("<rotateZ>")) {
					transform.combine(TransformMy3D.rotateZ(parseEquasion(removeTags(line))));
				} else if(line.startsWith("<scale>")) {
					double[] data = parseEquasions(removeTags(line));
					transform.combine(TransformMy3D.stretch(data[0], data[1], data[2]));
				} else if(line.startsWith("</transform>")) {
					return transform;
				}
			}
		} catch(Exception e) {
			showError(lineNum, "Number parsing error [" + line + "].");
			return null;
		}
		
		showError(lineNum, "File ended prematurely.");
		return null;
	}
	
	public static boolean parseXMLObjects(BufferedReader r, Object3D root) throws IOException {
		LineNumber lineNum = new LineNumber();
		return parseXMLObjects(r,root,lineNum);
	}
	
	public static boolean parseXMLObjects(BufferedReader r, Object3D root, LineNumber lineNum) throws IOException {
		String line;
		while((line = r.readLine()) != null) {
			lineNum.number ++;
			line = line.trim();
			if(line.startsWith("<object>")) {
				Object3D object = XMLParseRobotArm.parseXMLObject(r,lineNum);
				if(object == null) {
					return false;
				}
				root.addChild(object);
			}
		}
		return true;
	}
	
	public static Object3D parseXMLObject(BufferedReader r, LineNumber lineNum) throws IOException {
		
		String line;		
		Hashtable<String,String> keys = new Hashtable<String,String>();
		Vector<Object3D> children = new Vector<Object3D>();
		TransformMy3D transform = null;
		
		while((line = r.readLine()) != null) {
			lineNum.number++;
			line = line.trim();	
						
			if(line.startsWith("#")) {
			} else if(line.startsWith("<transform>")) {
				transform = XMLParseRobotArm.parseXMLTransform(r, lineNum);
				if(transform == null) {
					showError(lineNum, "Returned transform null.");
					return null;
				}
			} else if(line.startsWith("<object>")) {
				Object3D object = parseXMLObject(r,lineNum);
				if(object == null) {
					showError(lineNum, "Returned object null.");
					return null;
				}
				children.add(object);
			} else if(line.startsWith("</object>")) {
				String shape = keys.get("shape");
				
				if(shape == null) {
					showError(lineNum, "Shape not specified.");
					return null;
				}
				
				Object3D object = null;
				
				try {
				
					if(shape.equals("cone")) {
						//double[] center, double radius, double height, int divs, Material material
						double[] center = parseEquasions(keys.get("center"));
						double radius = parseEquasion(keys.get("radius"));
						double height = parseEquasion(keys.get("height"));
						int divs = Integer.parseInt(keys.get("divs"));
						Color color = parseColor(keys.get("color"));
						String name = keys.get("name");
						object = new Cone3D(center,radius,height,divs,new Material(color,name,null));
					} else if(shape.equals("cube")) {
						//double[] center, double[] size, Material material
						double[] center = parseEquasions(keys.get("center"));
						double[] size = parseEquasions(keys.get("size"));
						Color color = parseColor(keys.get("color"));
						String name = keys.get("name");
						object = new Cube3D(center,size,new Material(color,name,null));
					} else if(shape.equals("cylinder")) {
						//double[] center, double radius, double height, int divs, Material material
						double[] center = parseEquasions(keys.get("center"));
						double radius = parseEquasion(keys.get("radius"));
						double height = parseEquasion(keys.get("height"));
						int divs = Integer.parseInt(keys.get("divs"));
						Color color = parseColor(keys.get("color"));
						String name = keys.get("name");
						object = new Cylinder3D(center,radius,height,divs,new Material(color,name,null));
					} else if(shape.equals("handle")) {
						//double[] center, double radius, double height, int divs, Material material
						double[] center = parseEquasions(keys.get("center"));
						double radius = parseEquasion(keys.get("radius"));
						double height = parseEquasion(keys.get("height"));
						int divs = Integer.parseInt(keys.get("divs"));
						Color color = parseColor(keys.get("color"));
						String name = keys.get("name");
						object = new Handle3D(center,radius,height,divs,new Material(color,name,null));
					} else if(shape.equals("null")) {
						object = new NullObject3D();
					} else if(shape.equals("sphere")) {
						//double[] center, double radius, int divsX, int divsY, Material material
						double[] center = parseEquasions(keys.get("center"));
						double radius = parseEquasion(keys.get("radius"));
						int divsX = Integer.parseInt(keys.get("divsX"));
						int divsY = Integer.parseInt(keys.get("divsY"));
						Color color = parseColor(keys.get("color"));
						String name = keys.get("name");
						object = new Sphere3D(center,radius,divsX,divsY,new Material(color,name,null));
					} else if(shape.equals("wall")) {
						//double ax, double ay, double bx, double by, double height, int parts, Material material
						double ax = parseEquasion(keys.get("ax"));
						double ay = parseEquasion(keys.get("ay"));
						double bx = parseEquasion(keys.get("bx"));
						double by = parseEquasion(keys.get("by"));
						double height = parseEquasion(keys.get("height"));
						int parts = Integer.parseInt(keys.get("parts"));
						Color color = parseColor(keys.get("color"));
						String name = keys.get("name");
						object = new Wall3D(ax,ay,bx,by,height,parts,new Material(color,name,null));			
					} else {
						showError(lineNum, "Unrecognized shape [" + shape + "]");
						return null;
					}
				} catch(Exception e) {
					e.printStackTrace();
					
					showError(lineNum, "Number format exception.");
					return null;
				}
				
				for(Object3D child : children) {
					object.addChild(child);
				}
						
				if(object != null && transform != null) {
					object.permApplyTransform(transform);
				}
				return object;
				
			} else {
				if(line.length() > 0) {
					keys.put(getTag(line), removeTags(line));
				}
			}
		}

		showError(lineNum, "File ended prematurely.");
		return null;
	}
	
	public static String getTag(String line) {
		int start = line.indexOf('<') + 1;
		int stop = line.indexOf('>', start);
		return line.substring(start,stop);
	}
	
	public static String removeTags(String line) {
		int start = line.indexOf('>') + 1;
		int stop = line.indexOf('<', start);
		return line.substring(start,stop);
	}
	
	/*
	public static double[] parseArray(String line) {
		String data = removeTags(line);
		try {
			double[] array = parseEquasions(data);
		} catch(Exception e) {
			
		}
	}
	*/
	
	public static double[] parseEquasions(String eqs) throws Exception {
		String[] parts = eqs.split(",");
		double[] doubles = new double[parts.length];
		for(int i=0;i<parts.length;i++) {
			doubles[i] = parseEquasion(parts[i]);
		}
		return doubles;
	}
	
	public static double parseEquasion(String eq) throws Exception {
		eq = eq.replaceAll(" ", "");
		eq = eq.replaceAll("PI", Math.PI + "");
		
		Vector parsed = new Vector();
		String partial = null;
		boolean negative = false;
		boolean wasSymbol = true;
		
		for(int i=0;i<eq.length();i++) {
			if((eq.charAt(i) >= '0' && eq.charAt(i) <= '9') || eq.charAt(i) == '.') {
				if(partial == null) {
					partial = "";
				}
				partial += eq.charAt(i);
				wasSymbol = false;
			} else {
				if(wasSymbol && eq.charAt(i) == '-') {
					negative = true;
				} else {
					if(partial != null) {
						if(negative) {
							parsed.add(-Double.parseDouble(partial));
						} else {
							parsed.add(Double.parseDouble(partial));
						}
						negative = false;
						partial = null;
					}				
					parsed.add(eq.charAt(i));
					wasSymbol = true;
				}
			}
		}
		if(partial != null) {
			if(negative) {
				parsed.add(-Double.parseDouble(partial));
			} else {
				parsed.add(Double.parseDouble(partial));
			}
		}	
		
		reduceEq(parsed, 0, parsed.size());
		return (Double)parsed.get(0);
	}
	
	private static void reduceEq(Vector parsed, int start, int stop) {
		try {
			// Parentheses first.
			int count = 0;
			int parenStart = -1;
			for(int i=start;i<stop;i++) {
				if(parsed.get(i) instanceof Character) {
					char c = (Character)parsed.get(i);
					if(c == '(') {
						if(count == 0) {
							parenStart = i;
						}
						count ++;
					} else if(c == ')') {
						count --;
						if(count == 0) {
							reduceEq(parsed,parenStart+1,i);
							int removedSize = (i - parenStart);						
							parsed.remove(parenStart);
							parsed.remove(parenStart+1);
							
							i -= removedSize;
							stop -= removedSize;
						}
					}
				}
			}
			
			// Multiplication and division.
			for(int i=start;i<stop;i++) {
				if(parsed.get(i) instanceof Character) {
					char c = (Character)parsed.get(i);
					if(c == '*') {
						double v1 = (Double)parsed.get(i-1);
						double v2 = (Double)parsed.get(i+1);
						double result = v1 * v2;
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.add(i-1,result);
						i-=2;
						stop -= 2;
					} else if(c == '/') {
						double v1 = (Double)parsed.get(i-1);
						double v2 = (Double)parsed.get(i+1);
						double result = v1 / v2;
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.add(i-1,result);
						i--;
						stop -= 2;
					}
				}
			}
			
			// Addition and subtraction.
			for(int i=start;i<stop;i++) {
				if(parsed.get(i) instanceof Character) {
					char c = (Character)parsed.get(i);
					if(c == '+') {
						double v1 = (Double)parsed.get(i-1);
						double v2 = (Double)parsed.get(i+1);
						double result = v1 + v2;
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.add(i-1,result);
						i--;
						stop -= 2;
					} else if(c == '-') {
						double v1 = (Double)parsed.get(i-1);
						double v2 = (Double)parsed.get(i+1);
						double result = v1 - v2;
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.remove(i-1);
						parsed.add(i-1,result);
						i--;
						stop -= 2;
					}
				}
			}
		} catch(Exception e) {
		}
	}
	
	public static Color parseColor(String line) {
		String[] parts = line.split(",");
		return new Color(	Integer.parseInt(parts[0]),
							Integer.parseInt(parts[1]),
							Integer.parseInt(parts[2]));
	}
	
}

