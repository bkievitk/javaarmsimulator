package com.armsim.my3DCore;

import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.util.Vector;

import javax.swing.JFrame;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.io.*;

import com.armsim.my3DShapes.*;



public class JPanel3DChangeObject extends JPanel3D {

	private static final long serialVersionUID = 8260415992743063710L;
	
	public double tX = Math.PI/2.0;	// Rotation around x (left, right)
	public double tY = 0;	// Rotation around y (up, down)

	private Vector<ChangeListener> viewChange = new Vector<ChangeListener>();
		
	public JPanel3DChangeObject(Universe u) {
		super(u);
	}
	
	public void addViewChangeListener(ChangeListener cl) {
		viewChange.add(cl);
	}
	
	public static void main(String[] args) {
		
		Universe u = new Universe();
		
		double[] center = {0,0,0};
		double[] size = {2,3,1};
		Material material = new Material(Color.RED,null,null);
		
		NullObject3D root = new NullObject3D();

		try {
			ObjectInputStream reader = new ObjectInputStream(new FileInputStream(new File("table.info")));
			Object3D table = (Object3D)reader.readObject();
			root.addChild(table);
		} catch(IOException e) {
			e.printStackTrace();
		} catch(ClassNotFoundException e) {
			e.printStackTrace();
		}


		u.root = root;
		
		/*
		u.root = obj;
		double[] pt = {5,10,-1};
		Sphere3D sphere = new Sphere3D(pt, .1, 3, 3, new Material(Color.BLUE,null,null));
		u.root.addChild(sphere);
		System.out.println(obj.inside(pt, 0));
		*/
		
		// Create frame.
		JFrame frame = new JFrame();
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(1000,700);
		frame.add(new JPanel3DChangeObject(u));
		frame.setVisible(true);
	}
	
	public void setView() {
		u.view = TransformMy3D.translate(0,0,-Object3D.SCREEN_DISTANCE - 10);
		u.view.combine(TransformMy3D.rotateX(tX));
		u.view.combine(TransformMy3D.rotateY(tY));		
		u.view.combine(TransformMy3D.stretch(3, 3, 3));		
		
		if(viewChange != null) {
			for(ChangeListener cl : viewChange) {
				ChangeEvent ce = new ChangeEvent(this);
				cl.stateChanged(ce);
			}
		}
		repaint();
	}

	public void mouseDragged(MouseEvent arg0) {
		int dx = mouseDown.x - arg0.getX();
		int dy = mouseDown.y - arg0.getY();
		if(dx * dx + dy * dy > 4) {
			tX += .01 * dy;
			tY -= .01 * dx;
			mouseDown = arg0.getPoint();
			setView();
		}
	}

	public void keyPressed(KeyEvent arg0) {
		
		// Change render type.
		switch(arg0.getKeyChar()) {
			case '1': renderType = Universe.RENDER_WIREFRAME; break;
			case '2': renderType = Universe.RENDER_NORMAL; break;
			case '3': renderType = Universe.RENDER_DEAPTH; break;
			case '4': renderType = Universe.RENDER_FLAT; break;
		}
		setView();
	}

}
