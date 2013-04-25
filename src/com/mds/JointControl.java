/**
 * ADE 1.0
 * Copyright 1997-2011 HRILab (http://hrilab.org/)
 *
 * All rights reserved.  Do not copy and use without permission.
 * For questions contact Matthias Scheutz at mscheutz@cs.tufts.edu
 *
 * JointControl.java
 *
 * @author Paul Schermerhorn
 */
package com.mds;

import java.util.ArrayList;

/** <code>JointControl</code>.  Control and query joints.
 */
public interface JointControl {
    public void setVelocity(Object j, float v);
    public float getAngle(Object j);
    public float getUpperLimit(Object j);
    public float getLowerLimit(Object j);
    public void setAngle(Object j, float a);
    public void setAngle(Object j, float a, float v);
    public ArrayList<Object> getJoints();
    public void updateJoints();
}
// vi:ai:smarttab:expandtab:ts=8 sw=4
