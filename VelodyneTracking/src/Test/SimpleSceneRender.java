package Test;

import static javax.media.opengl.GL.GL_COLOR_BUFFER_BIT;
import static javax.media.opengl.GL.GL_DEPTH_BUFFER_BIT;
import static javax.media.opengl.GL.GL_DEPTH_TEST;
import static javax.media.opengl.GL.GL_LEQUAL;
import static javax.media.opengl.GL.GL_NICEST;
import static javax.media.opengl.GL2ES1.GL_PERSPECTIVE_CORRECTION_HINT;
import static javax.media.opengl.fixedfunc.GLLightingFunc.GL_SMOOTH;
import static javax.media.opengl.fixedfunc.GLMatrixFunc.GL_MODELVIEW;
import static javax.media.opengl.fixedfunc.GLMatrixFunc.GL_PROJECTION;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.glu.GLU;

import com.jogamp.opengl.util.gl2.GLUT;

public class SimpleSceneRender extends GLCanvas implements GLEventListener, KeyListener{

	private GLU glu;  // for the GL Utility
	private GLUT glut;
	private CameraControl cam = new CameraControl(30,30,50);
	
	@Override
	public void init(GLAutoDrawable drawable) {
		this.addGLEventListener(this);
		this.addKeyListener(this);
		
		GL2 gl = drawable.getGL().getGL2();      // get the OpenGL graphics context
		glu = new GLU();                         // get GL Utilities
		glut = new GLUT();                       // get GL Utilities Tool
		gl.glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // set background (clear) color
		gl.glClearDepth(1.0f);      // set clear depth value to farthest
		gl.glEnable(GL_DEPTH_TEST); // enables depth testing
		gl.glDepthFunc(GL_LEQUAL);  // the type of depth test to do
		gl.glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // best perspective correction
		gl.glShadeModel(GL_SMOOTH); // blends colors nicely, and smoothes out lighting
	}

	@Override
	public void dispose(GLAutoDrawable drawable) {
		
	}

	@Override
	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();  // get the OpenGL 2 graphics context
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear color and depth buffers
		
		gl.glLoadIdentity(); // reset
		
		this.glu.gluLookAt(this.cam.x, this.cam.y, this.cam.z, 0, 0, 0, 0, 0, 1);
		
		gl.glBegin(gl.GL_TRIANGLES); // draw using triangles
        gl.glVertex3f(0.0f, 10.0f, 0.0f);
        gl.glVertex3f(-10.0f, -1.0f, 0.0f);
        gl.glVertex3f(1.0f, -1.0f, 0.0f);
        gl.glEnd();
        
        System.out.println("update");
	}

	@Override
	public void reshape(GLAutoDrawable drawable, int x, int y, int width,
			int height) {
		GL2 gl = drawable.getGL().getGL2();  // get the OpenGL 2 graphics context

		if (height == 0) height = 1;   // prevent divide by zero
		float aspect = (float)width / height;

		// Set the view port (display area) to cover the entire window
		gl.glViewport(0, 0, width, height);

		// Setup perspective projection, with aspect ratio matches viewport
		gl.glMatrixMode(GL_PROJECTION);  // choose projection matrix
		gl.glLoadIdentity();             // reset projection matrix
		glu.gluPerspective(45.0, aspect, 0.1, 300.0); // fovy, aspect, zNear, zFar

		// Enable the model-view transform
		gl.glMatrixMode(GL_MODELVIEW);
		gl.glLoadIdentity(); // reset
	}

	@Override
	public void keyPressed(KeyEvent e) {
		int key = e.getKeyCode();
		switch(key){
		case KeyEvent.VK_ESCAPE:
			System.out.println("escape is pressed");
			break;
		case KeyEvent.VK_LEFT: 
			this.cam.rotAngle+=10;this.cam.update();break;
		case KeyEvent.VK_RIGHT: 
			this.cam.rotAngle-=10;this.cam.update();break;
		case KeyEvent.VK_UP: 
			this.cam.vertAngle+=10;this.cam.update();break;
		case KeyEvent.VK_DOWN: 
			this.cam.vertAngle-=10;this.cam.update();break;
		case KeyEvent.VK_O:
			this.cam.dist += 1;this.cam.update();break;
		case KeyEvent.VK_I:
			this.cam.dist -= 1;this.cam.update();break;
		}
	}

	@Override
	public void keyReleased(KeyEvent e) {
		
	}

	@Override
	public void keyTyped(KeyEvent e) {
		
	}
	

}
