package com.armsim.my3DCore;

import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import javax.swing.JFrame;

import com.armsim.my3DShapes.Object3D;



public class JPanel3DMoveWorld extends JPanel3D {

	private static final long serialVersionUID = 767985053073837254L;
	
	// Rotation and position in world.
	public double tX = 0;	// Rotation around x (left, right)
	public double tY = 0;	// Rotation around y (up, down)
	public double px = 0;	// Position in world plane
	public double py = -10;	// Position in world plane
	
	// Check if you move relative to the world or yourself.
	public boolean moveInWorldCoords = false;
	
	public static void main(String[] args) {
		// Create frame.
		JFrame frame = new JFrame();
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(1000,700);
		frame.add(new JPanel3DMoveWorld(null));
		frame.setVisible(true);
	}
	
	public JPanel3DMoveWorld(Universe u) {
		super(u);
	}
	
	//public JPanel3DMoveWorld(Object3D p) {
	//	super(p);
	//}
	
	public void mouseDragged(MouseEvent arg0) {
		
		// Get difference in mouse position.
		// Use this to modify rotation.
		int dx = mouseDown.x - arg0.getX();
		int dy = mouseDown.y - arg0.getY();
		if(dx * dx + dy * dy > 4) {
			tX += .01 * dy;
			tX = Math.max(Math.min(tX, .3),-.3);
			tY -= .01 * dx;
			mouseDown = arg0.getPoint();
			setView();
		}
	}

	/**
	 * Render given the current location and rotation.
	 */
	public void setView() {
		// Start with identity.
		u.view = (TransformMy3D.translate(0, 2.5, 0));
		
		// To rotate, move to perspective point, rotate, and then move back.
		u.view.combine(TransformMy3D.translate(0,0,-Object3D.SCREEN_DISTANCE));
		u.view.combine(TransformMy3D.rotateX(tX));
		u.view.combine(TransformMy3D.rotateY(tY));
		u.view.combine(TransformMy3D.translate(0,0,Object3D.SCREEN_DISTANCE));
		
		// Move to your location and perspective point.
		u.view.combine(TransformMy3D.translate(px, 0, -Object3D.SCREEN_DISTANCE + py));

		// Now render.
		repaint();
	}
	

	public void keyPressed(KeyEvent arg0) {
		
		// Change render type.
		switch(arg0.getKeyChar()) {
			case '1': renderType = Universe.RENDER_WIREFRAME; break;
			case '2': renderType = Universe.RENDER_NORMAL; break;
			case '3': renderType = Universe.RENDER_DEAPTH; break;
			case '4': renderType = Universe.RENDER_FLAT; break;
		}
		
		if(moveInWorldCoords) {
			// Register key press for movement.
			switch(arg0.getKeyChar()) {
				case 'a': px += 1; break;
				case 'd': px -= 1; break;
				case 's': py += 1; break;
				case 'w': py -= 1; break;
			}
		} else {
			switch(arg0.getKeyChar()) {
				case 'a': 
					px += Math.cos(tY); 
					py += Math.sin(tY); 
					break;
				case 'd':
					px -= Math.cos(tY); 
					py -= Math.sin(tY); 
					break;
				case 's':
					px += Math.sin(tY); 
					py -= Math.cos(tY); 
					break;
				case 'w':
					px -= Math.sin(tY); 
					py += Math.cos(tY); 
					break;
			}
		}
		setView();
	}
}
