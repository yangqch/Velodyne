package Test;

import java.awt.*;
import java.awt.event.*;
import java.io.File;
import java.io.IOException;
import java.nio.FloatBuffer;

import javax.swing.*;
import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.glu.GLU;


import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;

import calibration.FrameTransformer;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.util.FPSAnimator;
import static javax.media.opengl.GL.*;  // GL constants
import static javax.media.opengl.GL2.*; // GL2 constants
 
/**
 * JOGL 2.0 Program Template (GLCanvas)
 * This is a "Component" which can be added into a top-level "Container".
 * It also handles the OpenGL events to render graphics.
 */
@SuppressWarnings("serial")
public class LidarSingleFrameViewer extends GLCanvas implements GLEventListener, KeyListener{
   // Define constants for the top-level container
   private static String TITLE = "Overview";  // window's title
   private static final int CANVAS_WIDTH = 1000;  // width of the drawable
   private static final int CANVAS_HEIGHT = 1000; // height of the drawable
   private static final int FPS = 30; // animator's target frames per second
   private CameraControl cam = new CameraControl(2820.617f, -1313.578f, 65.280f+20, 30, 30, 50);
   /** The entry main() method to setup the top-level container and animator */
   public static void main(String[] args) {
      // Run the GUI codes in the event-dispatching thread for thread safety
      SwingUtilities.invokeLater(new Runnable() {
         @Override
         public void run() {
            // Create the OpenGL rendering canvas
            GLCanvas canvas = new LidarSingleFrameViewer();
            canvas.setPreferredSize(new Dimension(CANVAS_WIDTH, CANVAS_HEIGHT));
 
            // Create a animator that drives canvas' display() at the specified FPS.
            final FPSAnimator animator = new FPSAnimator(canvas, FPS, true);
 
            // Create the top-level container
            final JFrame frame = new JFrame(); // Swing's JFrame or AWT's Frame
            frame.getContentPane().add(canvas);
            frame.addWindowListener(new WindowAdapter() {
               @Override
               public void windowClosing(WindowEvent e) {
                  // Use a dedicate thread to run the stop() to ensure that the
                  // animator stops before program exits.
                  new Thread() {
                     @Override
                     public void run() {
                        if (animator.isStarted()) animator.stop();
                        System.exit(0);
                     }
                  }.start();
               }
            });
            frame.setTitle(TITLE);
            frame.pack();
            frame.setVisible(true);
            animator.start(); // start the animation loop
         }
      });
   }
 
   // Setup OpenGL Graphics Renderer
 
   private GLU glu;  // for the GL Utility
   private LidarFrameFactory lff;
   private LidarFrame[] lfs;
 
   /** Constructor to setup the GUI for this Component */
   public LidarSingleFrameViewer() {
      this.addGLEventListener(this);
      this.addKeyListener(this);
      
      try{
    	  this.lff = new LidarFrameFactory (new File("/home/qichi/Qichi_Velodyne/processed_data/mid_object_groundFrame.dat"));
      }catch(Exception e){
    	  System.out.println(e);
      }
   }
 
   // ------ Implement methods declared in GLEventListener ------
 
   /**
    * Called back immediately after the OpenGL context is initialized. Can be used
    * to perform one-time initialization. Run only once.
    */
   @Override
   public void init(GLAutoDrawable drawable) {
      GL2 gl = drawable.getGL().getGL2();      // get the OpenGL graphics context
      glu = new GLU();                         // get GL Utilities
      gl.glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // set background (clear) color
      gl.glClearDepth(1.0f);      // set clear depth value to farthest
      gl.glEnable(GL_DEPTH_TEST); // enables depth testing
      gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
      gl.glEnableClientState(GL2.GL_COLOR_ARRAY);
      gl.glDepthFunc(GL_LEQUAL);  // the type of depth test to do
      gl.glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // best perspective correction
      gl.glShadeModel(GL_SMOOTH); // blends colors nicely, and smoothes out lighting
 
      // ----- Your OpenGL initialization code here -----
      
      //read all the data and generate rendering commands
      int num = 100;
      this.lfs = new LidarFrame[num];
      try{
    	  for(int i=0; i<num; i++){
    		  lfs[i] = lff.getLidarFrame2();
    	  }
      }catch(IOException e){
    	  System.out.println(e);
      }
      
   }
 
   /**
    * Call-back handler for window re-size event. Also called when the drawable is
    * first set to visible.
    */
   @Override
   public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
      GL2 gl = drawable.getGL().getGL2();  // get the OpenGL 2 graphics context
 
      if (height == 0) height = 1;   // prevent divide by zero
      float aspect = (float)width / height;
 
      // Set the view port (display area) to cover the entire window
      gl.glViewport(0, 0, width, height);
 
      // Setup perspective projection, with aspect ratio matches viewport
      gl.glMatrixMode(GL_PROJECTION);  // choose projection matrix
      gl.glLoadIdentity();             // reset projection matrix
      glu.gluPerspective(45.0, aspect, 0.1, 100.0); // fovy, aspect, zNear, zFar
 
      // Enable the model-view transform
      gl.glMatrixMode(GL_MODELVIEW);
      gl.glLoadIdentity(); // reset
   }
 
   /**
    * Called back by the animator to perform rendering.
    */
   @Override
   public void display(GLAutoDrawable drawable) {
      GL2 gl = drawable.getGL().getGL2();  // get the OpenGL 2 graphics context
      gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear color and depth buffers
      gl.glLoadIdentity();  // reset the model-view matrix
      
      gl.glLoadIdentity(); // reset

      this.glu.gluLookAt(this.cam.x, this.cam.y, this.cam.z, this.cam.lx, this.cam.ly, this.cam.lz, 0, 0, 1);
      FrameTransformer trans = new FrameTransformer();
//      Point3D avg = trans.calcMean(lfs[10].transformToWorld(trans));
//      this.glu.gluLookAt(avg.x+10, avg.y+50, avg.z+50, avg.x, avg.y, avg.z, 0, 0, 1);
      
      for(LidarFrame lf: lfs){
    	  //transform
    	  this.renderLidarFrame(gl, lf.transformToWorld(trans), lf);
      }
   }
   
	private void renderLidarFrame(GL2 gl, Point3D[] points, LidarFrame lf){
		int pointNum = points.length;
		FloatBuffer vertexArray = Buffers.newDirectFloatBuffer(pointNum*3);
		FloatBuffer colorArray = Buffers.newDirectFloatBuffer(pointNum*3);
		for(int i=0; i<pointNum; i++){
			Point3D p = points[i];
			vertexArray.put((float)p.x);vertexArray.put((float)p.y);vertexArray.put((float)p.z);
			float intensity = lf.getIntensity(i);
			colorArray.put(intensity);colorArray.put(intensity);colorArray.put(intensity);
		}
		vertexArray.rewind();
		colorArray.rewind();
		gl.glColorPointer( 3, gl.GL_FLOAT, 0, colorArray);
		this.renderVextexArray(gl, vertexArray, pointNum, 3, GL2.GL_FLOAT, GL2.GL_POINTS);
	}
	
	private void renderVextexArray(GL2 gl, FloatBuffer vertexArray, int dataNum, int dataDim, int dataNumType, int dataGeoType){
		gl.glVertexPointer(dataDim, dataNumType, 0, vertexArray);
		gl.glDrawArrays(dataGeoType, 0, dataNum);
	}
 
   /**
    * Called back before the OpenGL context is destroyed. Release resource such as buffers.
    */
   @Override
   public void dispose(GLAutoDrawable drawable) { }

   @Override
   public void keyPressed(KeyEvent e) {
		int key = e.getKeyCode();
		switch(key){
		case KeyEvent.VK_LEFT: 
			this.cam.rotAngle+=10;this.cam.update();break;
		case KeyEvent.VK_RIGHT: 
			this.cam.rotAngle-=10;this.cam.update();break;
		case KeyEvent.VK_UP: 
			this.cam.vertAngle+=10;this.cam.update();break;
		case KeyEvent.VK_DOWN: 
			this.cam.vertAngle-=10;this.cam.update();break;
		case KeyEvent.VK_O:
			this.cam.dist = this.cam.fixPos ? this.cam.dist : this.cam.dist+1;this.cam.update();break;
		case KeyEvent.VK_I:
			this.cam.dist = this.cam.fixPos ? this.cam.dist : this.cam.dist-1;this.cam.update();break;
		case KeyEvent.VK_F:
			this.cam.changeMode();this.cam.update();break;
		}
   }

   @Override
   public void keyReleased(KeyEvent e) {
	   // TODO Auto-generated method stub

   }

   @Override
   public void keyTyped(KeyEvent e) {
	   // TODO Auto-generated method stub

}
}

class CameraControl{
	private static double DEG2RAD=Math.PI/180.0;
	private static double RAD2DEG=180.0/Math.PI;
	
	float rotAngle;//right-hand from negative x-axis 
	float vertAngle;//up: negative z-axis
	float dist;//distance to origin
	
	boolean fixPos=false;
	
	float x,y,z;//camera position w.r.t world frame
	float lx, ly, lz=0;
	
	public CameraControl(float rot, float vert, float dist) {
		this.rotAngle=rot;
		this.vertAngle=vert;
		this.dist=dist;
		this.update();
		lx=0;ly=0;lz=0;

	}
	
	public CameraControl(double x, double y, double z, float rot, float vert, float dist){
		this.x = (float)x;this.y = (float)y;this.z = (float)z;
		this.rotAngle=rot;
		this.vertAngle=vert;
		this.dist=dist;
		fixPos=true;
		this.update();
	}
	
	public void update(){
		this.vertAngle%=360;
		this.rotAngle%=360;
		if(!fixPos){
			z = lz + this.dist * (float)Math.sin(this.vertAngle*DEG2RAD);
			float xyDist = this.dist * (float)Math.cos(this.vertAngle*DEG2RAD);
			x = lx + xyDist * (float)Math.cos(this.rotAngle*DEG2RAD);
			y = ly + xyDist * (float)Math.sin(this.rotAngle*DEG2RAD);
		}else{
			lz = z - this.dist * (float)Math.sin(this.vertAngle*DEG2RAD);
			float xyDist = this.dist * (float)Math.cos(this.vertAngle*DEG2RAD);
			lx = x - xyDist * (float)Math.cos(this.rotAngle*DEG2RAD);
			ly = y - xyDist * (float)Math.sin(this.rotAngle*DEG2RAD);
		}
		
		System.out.printf("cam pos %.1f, %.1f, %.1f\n",x,y,z);
	}
	
	public void changeMode(){
		this.fixPos = !this.fixPos;
	}
	
	public void setPos(float x, float y, float z){
		this.x=x;this.y=y;this.z=z;
	}
}

