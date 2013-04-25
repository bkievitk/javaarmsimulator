package com.armsim.my3DRobotArm;
import java.util.*;
public abstract class AStarObject {
	public abstract double distance(AStarObject other);
	public abstract Vector<AStarObject> linksTo();
}
