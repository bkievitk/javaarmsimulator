/**
 * ADE 1.0
 * Copyright 1997-2010 HRILab (http://hrilab.org/)
 *
 * All rights reserved.  Do not copy and use without permission.
 * For questions contact Matthias Scheutz at mscheutz@indiana.edu
 *
 * MDSJoint.java
 *
 * @author Paul Schermerhorn
 * 
 */

package com.mds;

/**
 * Enumeration for specifying Nao joints.
 */
public enum MDSJoint {
    // probably don't need to allow manipulation of fan speed
    //BodyFans(0X0044,0),
    //HeadFans(0X0047,0),
    Logo(0X0052,1.0f,1,0.0f),
    Body(0X0058,0.8212f,32595,1.0f),

    HeadRoll(0x0030,0.5962f,4074,1.0f),
    HeadPitch(0x0038,0.7407f,4074,1.0f),
    HeadPan(0x003A,1.2733f,4074,1.0f),
    NeckPitch(0x0039,0.6656f,4074,1.0f),
    JawPitch(0x0033,0.3640f,4319,1.0f),
    JawRoll(0x003B,0.5814f,2730,1.0f),
    JawJut(0X0043,1.5752f,2730,1.0f),

    EyePitch(0x0032,0.5138f,4319,2.0f),
    LEyePan(0x0031,1.2136f,2730,2.0f),
    REyePan(0x003F,1.2437f,2730,2.0f),

    LUpperEyelid(0x0034,1.4950f,2730,2.0f),
    RUpperEyelid(0x003C,1.5887f,2730,2.0f),
    LLowerEyelid(0X0042,0.9510f,2730,2.0f),
    RLowerEyelid(0x003D,0.8949f,2730,2.0f),
    LEyelidRoll(0X0041,0.4301f,2730,2.0f),
    REyelidRoll(0x0036,0.4557f,2730,2.0f),

    LEyebrowPitch(0x0035,0.2344f,2730,2.0f),
    REyebrowPitch(0x0037,0.2443f,2730,2.0f),
    LEyebrowRoll(0X0040,1.1301f,2730,2.0f),
    REyebrowRoll(0x003E,1.1473f,2730,2.0f),

    LThumbRotate(0X004C,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    RThumbRotate(0X004D,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    LThumb(0X0050,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    RThumb(0X0051,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    LIndex(0X004E,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    RIndex(0X004F,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    LMiddle(0X004A,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    RMiddle(0X004B,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    LPinky(0X0048,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),
    RPinky(0X0049,(float)(Math.PI*0.5),(int)(2608*2.5),1.0f),

    LWristFlex(0X0056,1.5038f,4074,1.0f),
    RWristFlex(0X0057,1.4932f,4074,1.0f),
    LWristRoll(0X0059,1.5899f,4074,1.0f),
    RWristRoll(0X0053,1.5831f,4074,1.0f),

    LShoulderPitch(0X005A,3.1065f,32595,1.0f),
    RShoulderPitch(0X005B,3.0960f,32595,1.0f),
    LShoulderAbduct(0X005E,2.9648f,39114,1.0f),
    RShoulderAbduct(0X005F,2.9504f,39114,1.0f),

    LElbow(0X0054,2.6139f,4074,1.0f),
    RElbow(0X0055,2.6139f,4074,1.0f),
    LUpperArmRoll(0X005C,2.5994f,4074,1.0f),
    RUpperArmRoll(0X005D,2.6006f,4074,1.0f);

    private int address; // CAN address
    private float range; // (nominal) range in radians
    private int scale; // encoder counts/radian
    private float max; // max velocity

    private MDSJoint(int a, float r, int s, float m) {
        address = a;
        range = r;
        scale = s;
        max = m;
        //System.out.println(this+" range: "+r+", "+(r*2*s));
    }
    
    public int getAddress() {
        return address;
    }
    
    public float getRange() {
        return range;
    }
    
    public int getScale() {
        return scale;
    }

    public float getMaxVelocity() {
        return max;
    }
}
// vi:ai:smarttab:expandtab:ts=8 sw=4
