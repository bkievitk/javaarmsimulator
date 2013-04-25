package com.armsim.my3DCore;

import java.awt.Color;
import java.awt.image.BufferedImage;

import java.io.Serializable;

public class Material implements Serializable {

	private static final long serialVersionUID = 92347860014L;

	public Color color;
	public Object object;
	public transient BufferedImage img;
	
	public int s =  2;  	// Is a shininess constant for this material, which decides how "evenly" light is reflected from a shiny spot.
	public double ia = .1; 	// Ambiant intensity.
	public double id = .2;  // Diffusion intensity of light. 
	public double is = 0; 	// Specular intensity of light. 

	public boolean flipX = false;		// Image flipped along the x axis.
	public boolean flipY = false;		// Image flipped along the y axis.
	public boolean flipDir = true;		// X and Y axis reversed.
	public boolean renderImg = true;	// True to render the image.

	public boolean renderWireframe = false;
			
	public Material(Color color, Object object, BufferedImage img, int s, double ia, double id, double is) {
		this.color = color;
		this.object = object;
		this.img = img;
		this.s = s;
		this.ia = ia;
		this.id = id;
		this.is = is;
	}
	
	public Material(Color color, Object object, BufferedImage img) {
		this.color = color;
		this.object = object;
		this.img = img;
	}
		
	public Material newImage(boolean flipX, boolean flipY, boolean flipDir, boolean renderImg) {
		Material m = copy();
		m.flipX = flipX;
		m.flipY = flipY;
		m.flipDir = flipDir;
		m.renderImg = renderImg;
		return m;
	}
	
	public Material copy() {
		Material m = new Material(color,object,img);
		m.s = s;
		m.ia = ia;
		m.id = id;
		m.is = is;
		return m;
	}
	
	public Color getColor(double a, double c) {
	
		if(img == null || !renderImg) {
			return color;
		}
		
		int px,py;
		double a1,c1;
		
		if(flipDir) {
			a1 = c;
			c1 = a;
		} else {
			a1 = a;
			c1 = c;
		}
		
		if(flipX) {
			px = (int)((1 - c1) * img.getWidth());
		} else {
			px = (int)(c1 * img.getWidth());
		}
		
		if(flipY) {
			py = (int)((1 - a1) * img.getHeight());
		} else {
			py = (int)(a1 * img.getHeight());
		}
		
		if(px < 0) { px = 0; }
		if(py < 0) { py = 0; }
		if(px >= img.getWidth()) { px = img.getWidth() - 1; }
		if(py >= img.getHeight()) { py = img.getHeight() - 1; }
		
		return new Color(img.getRGB(px, py));		
	}
	
}
