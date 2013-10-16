package VelodyneView;

import gridmap_generic.GridmapMatrix;

import java.awt.*;
import java.awt.event.*;
import java.io.EOFException;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.FloatBuffer;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.glu.GLU;
import javax.swing.filechooser.FileSystemView;

import prepocess.ConnCompFilter;
import prepocess.RangeFilter;
import velodyne2d.Point2D;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;
import calibration.GroundPlaneDetector;

import com.jogamp.opengl.util.gl2.GLUT;

import detection.VehicleModel;

import static javax.media.opengl.GL.*;  // GL constants
import static javax.media.opengl.GL2.*; // GL2 constants

public class LidarGLViewer extends GLCanvas implements GLEventListener, KeyListener{
	//for gl
	protected GLU glu;  // for the GL Utility
	protected GLUT glut;
	protected CameraControl cam;
	protected CameraControl orthoCam;
	//for viewport
	private int subWindow=0;
	private int width;
	private int height;
	//for frame control
	private LidarFrameProcessor lidarFrameProcessor;
	protected boolean framePause;
	protected boolean showRawData;
	protected boolean orthoview;
	
	private FrameTransformer mTrans;
	protected VehicleModel egoVehicle;
	protected CoordinateFrame localWorldFrame;
	
	private static final int rotMin=-180;
	private static final int rotMax=180;
	
	public LidarGLViewer(){}

	/** Constructor to setup the GUI for this Component */
	public LidarGLViewer(LidarFrameProcessor lidarFrameProcessor) {
		this.addGLEventListener(this);
		this.addKeyListener(this);
		this.framePause=false;
		this.showRawData=false;
		this.orthoview=false;
		this.cam=new CameraControl(0,-30,50);
		this.orthoCam = new CameraControl(0, 90, -50);
		this.lidarFrameProcessor = lidarFrameProcessor;
		
		this.mTrans = new FrameTransformer();
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
		glut = new GLUT();                       // get GL Utilities Tool
		gl.glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // set background (clear) color
		gl.glClearDepth(1.0f);      // set clear depth value to farthest
		gl.glEnable(GL_DEPTH_TEST); // enables depth testing
		gl.glDepthFunc(GL_LEQUAL);  // the type of depth test to do
		gl.glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // best perspective correction
		gl.glShadeModel(GL_SMOOTH); // blends colors nicely, and smoothes out lighting

		// ----- Your OpenGL initialization code here -----
		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
	}

	/**
	 * Call-back handler for window re-size event. Also called when the drawable is
	 * first set to visible.
	 */
	@Override
	public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
		GL2 gl = drawable.getGL().getGL2();  // get the OpenGL 2 graphics context
		
		this.width=width;
		this.height=height;
		
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

	/**
	 * Called back by the animator to perform rendering.
	 */
	@Override
	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();  // get the OpenGL 2 graphics context
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear color and depth buffers

		if (height == 0) height = 1;   // prevent divide by zero
		float aspect = (float)width / height;

		// Set the view port (display area) to cover the entire window
		gl.glViewport(0,0,width,height);

		// Setup perspective projection, with aspect ratio matches viewport
		gl.glMatrixMode(GL_PROJECTION);  // choose projection matrix
		gl.glLoadIdentity();             // reset projection matrix
		glu.gluPerspective(45.0, aspect, 0.1, 300.0); // fovy, aspect, zNear, zFar
		// Enable the model-view transform
		gl.glMatrixMode(GL_MODELVIEW);
		gl.glLoadIdentity(); // reset
		
		if(orthoview){
			this.glu.gluLookAt(this.orthoCam.x, this.orthoCam.y, this.orthoCam.z, this.orthoCam.lookat_x, this.orthoCam.lookat_y, 0, -1, 0, 0);
			//this.glu.gluLookAt(this.orthoCam.x, this.orthoCam.y, this.orthoCam.z, 10, 0, 0, 0, 0, 1);
		}else{
			this.glu.gluLookAt(this.cam.x, this.cam.y, this.cam.z, 0, 0, 0, 0, 0, -1);
		}
		
		//draw the scene
		renderLidarScene(gl);

		//draw ego vehicle and axis
		gl.glColor3f(1.0f, 1.0f, 1.0f);
		this.renderVehicle(gl);
		this.renderAxis(gl);
		
		gl.glColor3f(1.0f, 1.0f, 1.0f);
		this.renderCircle(gl, 5);
		
//		for(int i=0; i<2; i++){
//			if (height == 0) height = 1;   // prevent divide by zero
//			float aspect = (float)width / height;
//	
//			// Set the view port (display area) to cover the entire window
//			gl.glViewport(i==0?0:width/2, i==0?0:height*i/5, width/2, i==0?height:height/5);
//	
//			if(i==2){
//				// Setup perspective projection, with aspect ratio matches viewport
//				gl.glMatrixMode(GL_PROJECTION);  // choose projection matrix
//				gl.glLoadIdentity();             // reset projection matrix
//				glu.gluPerspective(45.0, aspect, 0.1, 300.0); // fovy, aspect, zNear, zFar
//				// Enable the model-view transform
//				gl.glMatrixMode(GL_MODELVIEW);
//				gl.glLoadIdentity(); // reset
//				//draw the scene
//				renderLidarScene(gl);
//			}else{
//				gl.glMatrixMode(GL_PROJECTION);  // choose projection matrix
//				gl.glLoadIdentity();             // reset projection matrix
//				glu.gluOrtho2D(width/2, width, height/5, 0);
//				gl.glColor3f(1.0f, 0.0f, 0.0f); 
//				gl.glBegin(GL_QUADS);
//				gl.glVertex2d(0, 100);
//				gl.glVertex2d(0, 0);
//				gl.glVertex2d(width, 0);
//				gl.glVertex2d(width, 100);
//				gl.glEnd();
//			}
//			
//		}
	}

	/**
	 * Called back before the OpenGL context is destroyed. Release resource such as buffers.
	 */
	@Override
	public void dispose(GLAutoDrawable drawable) { 
	}
	
	protected void renderLidarScene(GL2 gl){
		if(!this.framePause){
			try{
				this.lidarFrameProcessor.readNextFrame();
				this.lidarFrameProcessor.findConnComp(0.3, 5);
				//this.lidarFrameProcessor.filterWithHeightGridmap(0, 0.3);
			}catch(Exception e){
				e.printStackTrace();
				this.lidarFrameProcessor.stop();
				System.exit(-1);
			}
		}
		
//		BodyFrame egoFrame = this.lidarFrameProcessor.getCurFrame().getBodyFrame();
//		this.egoVehicle = new VehicleModel(egoFrame.getPosition().x, egoFrame.getPosition().y, egoFrame.getYaw(true)); 
//		this.localWorldFrame = this.lidarFrameProcessor.getCurFrame().getLocalWorldFrame();
		
		gl.glColor3f(1.0f, 1.0f, 1.0f);
		if(this.showRawData){
			this.renderLidarFrame(gl, this.lidarFrameProcessor.getCurFrame()); //show raw frame data
			//this.renderVirtualTable(gl, this.lidarFrameProcessor.getVirtualTable(), null); //show all virtual table data
			//this.renderVirtualTable(gl, this.lidarFrameProcessor.getVirtualTable(), this.lidarFrameProcessor.getMask1(), this.lidarFrameProcessor.getMask());
			//this.renderVirtualTable(gl, this.lidarFrameProcessor.getVirtualTable(), this.lidarFrameProcessor.getMask1());
		}else{
			this.renderVirtualTable(gl, this.lidarFrameProcessor.getVirtualTable(), null); //show all virtual table data
//			this.renderVirtualTable(gl, this.lidarFrameProcessor.getVirtualTable(), this.lidarFrameProcessor.getCompFilter().getCompMask());
			//this.renderVirtualTable(gl, this.lidarFrameProcessor.getVirtualTable(), this.lidarFrameProcessor.getHeightMapFilter().getMask()); //show all virtual table data
			//this.renderVirtualTable(gl, this.lidarFrameProcessor.getVirtualTable(), this.lidarFrameProcessor.getOccupyMapFilter().getMask());
		}
		//draw height map
		//renderPoints(gl, this.lidarFrameProcessor.getHeightMapFilter().getGroundDataBuffer(), this.lidarFrameProcessor.getHeightMapFilter().getDataNum());
		//draw occupancy map
		//renderPoints(gl, this.lidarFrameProcessor.getOccupyMapFilter().getGroundDataBuffer(), this.lidarFrameProcessor.getOccupyMapFilter().getDataNum());
	}
	
	protected void renderCircle(GL2 gl, double radiance){
		gl.glPushMatrix();
		gl.glBegin(GL_LINE_LOOP);
		for(double i=0;i<Math.PI*2;i+=0.1){
			gl.glVertex3d(radiance*Math.cos(i),radiance*Math.sin(i), 0);
		}
		gl.glEnd();
		gl.glPopMatrix();
	}
	
	protected void renderAxis(GL2 gl){
		gl.glLineWidth(1);
		gl.glBegin(GL_LINES);
		gl.glColor3f(1.0f, 0.0f, 0.0f);   // Red
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(5,0,0);//body_x-forward
		gl.glColor3f(0.0f, 1.0f, 0.0f);   // Green
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(0,5,0);//body_y-right
		gl.glColor3f(0.0f, 0.0f, 1.0f);   // Blue
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(0,0,5);//body_z-down
		gl.glEnd();
	}
	
	protected void renderVehicle(GL2 gl){
//		gl.glPushMatrix();
//		//gl.glTranslated(this.egoVehicle.x, this.egoVehicle.y, this.egoVehicle.z);
//		Point2D[] corners = this.egoVehicle.getCornerPoints(this.localWorldFrame, this.mTrans);
//		gl.glBegin(GL_LINE_LOOP);
//		for(Point2D p: corners){
//			gl.glVertex3d(p.x,p.y,0);
//		}
//		gl.glEnd();
//		Point2D[] arrows = this.egoVehicle.getArrowPoints(this.localWorldFrame, this.mTrans);
//		gl.glBegin(GL_LINE_LOOP);
//		for(Point2D p: arrows){
//			gl.glVertex3d(p.x,p.y,0);
//		}
//		gl.glEnd();
//		gl.glPopMatrix();
	}
	
	protected void renderLidarFrame(GL2 gl, LidarFrame lf){
		FloatBuffer vertexArray = lf.getDataBuffer();
		vertexArray.rewind();
		int dataNum = lf.getPointNum();
		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
		gl.glVertexPointer(3, GL2.GL_FLOAT, 0, vertexArray);
		gl.glDrawArrays(GL2.GL_POINTS, 0, dataNum);
		gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);
	}
	
	protected void renderVirtualTable(GL2 gl, VirtualTable vt, boolean[][] mask){
		FloatBuffer vertexArray = vt.getProcessedDataBuffer(rotMin, rotMax, mask);
		int dataNum = vt.getDataNum();
		gl.glPointSize(2);
		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
		gl.glVertexPointer(3, GL2.GL_FLOAT, 0, vertexArray);
		gl.glDrawArrays(GL2.GL_POINTS, 0, dataNum);
		gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);
	}
	
	protected void renderPoints(GL2 gl, FloatBuffer vertexArray, int dataNum){
		if(dataNum<=0) return;
		gl.glPointSize(1);
		gl.glColor3f(0, 0, 1);
		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
		gl.glVertexPointer(3, GL2.GL_FLOAT, 0, vertexArray);
		gl.glDrawArrays(GL2.GL_POINTS, 0, dataNum);
		gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);
	}
	
	/**
	 * show data in mask1 in one color
	 * show data in mask2 in another color
	 * mask2 will cover mask1
	 * @param gl
	 * @param vt
	 * @param mask1: green
	 * @param mask2: red
	 */
	protected void renderVirtualTable(GL2 gl, VirtualTable vt, boolean[][] mask1, boolean[][] mask2){		
		gl.glColor3f(0, 1, 0);
		renderVirtualTable(gl, vt, mask1);
		gl.glColor3f(1, 0, 0);
		renderVirtualTable(gl, vt, mask2);
	}
	
	private void renderProcessedVirtualTable(GL2 gl, VirtualTable vt, GroundPlaneDetector mDet){
		FloatBuffer vertexArray;
		int dataNum;
		gl.glPointSize(2);
		vertexArray = vt.getAllDataBuffer();
		dataNum = vt.getDataNum();
		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
		gl.glVertexPointer(3, GL2.GL_FLOAT, 0, vertexArray);
		gl.glDrawArrays(GL2.GL_POINTS, 0, dataNum);
		gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);
		
//		vertexArray = vt.getGroundDataBuffer(rotMin, rotMax, this.mDet);
//		dataNum = vt.getDataNum();
//		gl.glColor3f(1, 0, 0);
//		this.renderVextexArray(gl, vertexArray, dataNum, 3, GL2.GL_FLOAT, GL2.GL_POINTS);
		
//		vertexArray = mDet.getGroundFitPointsBuffer();
//		dataNum = mDet.getPointNum();
//		gl.glColor3f(1, 0, 0);
//		this.renderVextexArray(gl, vertexArray, dataNum, 3, GL2.GL_FLOAT, GL2.GL_POINTS);
//		
//		vertexArray = mDet.getPlaneGridDataBuffer();
//		dataNum = mDet.getGridPointNum();
//		gl.glColor3f(0, 0, 1);
//		this.renderVextexArray(gl, vertexArray, dataNum, 3, GL2.GL_FLOAT, GL2.GL_LINES);
//		
//		vertexArray = vt.getObjectDataBuffer(rotMin, rotMax, this.mDet);
//		dataNum = vt.getDataNum();
//		gl.glColor3f(1, 1, 1);
//		this.renderVextexArray(gl, vertexArray, dataNum, 3, GL2.GL_FLOAT, GL2.GL_POINTS);
		
//		vertexArray = vt.getBoundaryBuffer(rotMin, rotMax, this.mDet);
//		dataNum = vt.getDataNum();
//		gl.glColor3f(1, 1, 1);
//		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
//		gl.glVertexPointer(3, GL2.GL_FLOAT, 0, vertexArray);
//		gl.glDrawArrays(GL2.GL_LINE_LOOP, 0, dataNum);
//		gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);
		
//		vertexArray = vt.getBoundaryBuffer2(rotMin, rotMax, this.mDet);
//		dataNum = vt.getDataNum();
//		gl.glColor3f(0, 1, 0);
//		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
//		gl.glVertexPointer(3, GL2.GL_FLOAT, 0, vertexArray);
//		gl.glDrawArrays(GL2.GL_LINES, 0, dataNum);
//		gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);
	}
	
	protected void renderVextexArray(GL2 gl, FloatBuffer vertexArray, int dataNum, int dataDim, int dataNumType, int dataGeoType){
		gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
		gl.glVertexPointer(dataDim, dataNumType, 0, vertexArray);
		gl.glDrawArrays(dataGeoType, 0, dataNum);
		gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);
	}

	@Override
	public void keyPressed(KeyEvent e) {
		// TODO Auto-generated method stub
		int key = e.getKeyCode();
		switch(key){
		case KeyEvent.VK_ESCAPE:
			System.out.println("escape is pressed");
			new AnimatorStopper(this.getAnimator()).start();
			break;
		case KeyEvent.VK_LEFT:
			if(this.orthoview){
				this.orthoCam.y+=10;
				this.orthoCam.lookat_y+=10;
			}else{
				this.cam.rotAngle+=10;this.cam.update();
			}
			break;
		case KeyEvent.VK_RIGHT:
			if(this.orthoview){
				this.orthoCam.y-=10;
				this.orthoCam.lookat_y-=10;
			}else{
				this.cam.rotAngle-=10;this.cam.update();	
			}
			break;
		case KeyEvent.VK_UP:
			if(this.orthoview){
				this.orthoCam.x-=10;
				this.orthoCam.lookat_x-=9;
			}else{
				this.cam.vertAngle-=10;this.cam.update();	
			}
			break;
		case KeyEvent.VK_DOWN:
			if(this.orthoview){
				this.orthoCam.x+=10;
				this.orthoCam.lookat_x+=9;
			}else{
				this.cam.vertAngle+=10;this.cam.update();	
			}
			break;
		case KeyEvent.VK_O:
			if(this.orthoview){
				this.orthoCam.z += 1;	
			}else{
				this.cam.dist += 1;this.cam.update();
			}
			break;
		case KeyEvent.VK_I:
			if(this.orthoview){
				this.orthoCam.z -= 1;
			}else{
				this.cam.dist -= 1;this.cam.update();
			}
			break;
		case KeyEvent.VK_SPACE:
			this.framePause=!this.framePause;break;
		case KeyEvent.VK_K:
			this.showRawData=!this.showRawData;break;
		case KeyEvent.VK_L:
			this.lidarFrameProcessor.logVirtualTable();break;
		case KeyEvent.VK_V:
			this.orthoview = !this.orthoview; break;
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