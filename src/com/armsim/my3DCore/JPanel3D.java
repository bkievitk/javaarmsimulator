package com.armsim.my3DCore;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import javax.swing.JPanel;

import com.armsim.my3DShapes.Object3D;



public abstract class JPanel3D extends JPanel implements MouseListener, MouseMotionListener, ComponentListener, KeyListener {
	
	private static final long serialVersionUID = 767985053073837254L;
	
	// The 3D data.
	public Universe u = new Universe();

	// Track mouse movement.
	protected Point mouseDown = null;
	
	// Image first rendered to this.
	protected BufferedImage img = new BufferedImage(500,500, BufferedImage.TYPE_INT_RGB);
	public int renderType = Universe.RENDER_NORMAL;

	/**
	 * Render to BufferedImage.
	 * @param p
	 * @return
	 */
	public static BufferedImage renderObject(Object3D p) {
		BufferedImage background = new BufferedImage(50,50,BufferedImage.TYPE_INT_RGB);
		Graphics g = background.getGraphics();
		g.setColor(Color.BLACK);
		g.fillRect(0,0,50,50);
		
		BufferedImage img = new BufferedImage(50,50,BufferedImage.TYPE_INT_RGB);
		Universe u = new Universe();
		u.background = background;
		u.root = p;
		
		u.view = TransformMy3D.translate(0,0,-Object3D.SCREEN_DISTANCE - 10);
		u.view.combine(TransformMy3D.rotateX(-.5));
		u.view.combine(TransformMy3D.rotateZ(.1));
		u.view.combine(TransformMy3D.stretch(4,4,4));
		
		u.render(img,Universe.RENDER_NORMAL);
		return img;
	}
		
	public JPanel3D(Universe u) {
		// Use the test perspective.
		setupPanel(u);
	}
	
	public void setupPanel(Universe u) {

		// Add listeners.
		addMouseListener(this);
		addMouseMotionListener(this);
		addComponentListener(this);
		addKeyListener(this);
		setFocusable(true);
		
		// Set perspective.
		this.u = u;
				
		// This performs the render.
		setView();
	}
	
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		
		// Render to image and then to screen.
		u.render(img,renderType);
		g.drawImage(img,0,0,this);
	}

	public void mouseClicked(MouseEvent arg0) {
		
		// Make sure the object buffer exists.
		if(u.objBuffer != null) {
			
			// Retrieve the object from the object buffer.
			Object clicked = u.objBuffer[arg0.getX()][arg0.getY()];
			System.out.println(clicked);
		}
	}

	public void mousePressed(MouseEvent arg0) {
		
		// This is the active object.
		this.requestFocus();
		
		// Track that the mouse button was pressed.
		mouseDown = arg0.getPoint();
	}

	/**
	 * Render given the current location and rotation.
	 */
	public abstract void setView();

	public void componentResized(ComponentEvent arg0) {
		// If window has changed, then resize the bitmap and rebuild.
		int width = Math.max(getWidth(), 50);
		int height = Math.max(getHeight(), 50);
		img = new BufferedImage(width,height, BufferedImage.TYPE_INT_RGB);
		setView();
	}
	
	public void mouseMoved(MouseEvent arg0) {}
	public void componentHidden(ComponentEvent arg0) {} 
	public void componentMoved(ComponentEvent arg0) {}
	public void componentShown(ComponentEvent arg0) {}
	public void mouseEntered(MouseEvent arg0) {}
	public void mouseExited(MouseEvent arg0) {}
	public void mouseReleased(MouseEvent arg0) {}
	public void keyReleased(KeyEvent arg0) {}
	public void keyTyped(KeyEvent arg0) {}
	
}
