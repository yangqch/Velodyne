package velodyne2d;

import java.awt.event.*;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.glu.GLU;

import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;
import VelodyneView.AnimatorStopper;
import VelodyneView.CameraControl;
import VelodyneView.LidarFrameProcessor;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.util.gl2.GLUT;

import detection.VehicleModel;

import static javax.media.opengl.GL.*;  // GL constants
import static javax.media.opengl.GL2ES1.GL_PERSPECTIVE_CORRECTION_HINT;
import static javax.media.opengl.fixedfunc.GLLightingFunc.GL_SMOOTH;
import static javax.media.opengl.fixedfunc.GLMatrixFunc.GL_MODELVIEW;
import static javax.media.opengl.fixedfunc.GLMatrixFunc.GL_PROJECTION;

public class LidarGLViewer extends GLCanvas implements GLEventListener, KeyListener{

	private GLU glu;  // for the GL Utility
	private GLUT glut;
	private int width;
	private int height;
	
	//scene
	protected LidarFramePorcessor2D lidarFrameProcessor;
	protected VehicleModel egoVehicle;
	protected CoordinateFrame2D localWorldFrame;
	protected FrameTransformer2D mTrans;
	
	protected VirtualTable preVirtualTable;
	
	//camera and frame control
	protected CameraControl cam;
	protected CameraControl orthoCam;
	protected boolean framePause;
	protected boolean orthoview;
	protected boolean show3D;
	
	/** Constructor to setup the GUI for this Component */
	public LidarGLViewer(LidarFrameProcessor processor) {
		this.addGLEventListener(this);
		this.addKeyListener(this);
		this.framePause=false;
		this.show3D=false;
		this.orthoview=true;
		this.cam=new CameraControl(0,-30,50);
		this.orthoCam = new CameraControl(0, 90, -50);
		
		this.lidarFrameProcessor = new LidarFramePorcessor2D(processor);
		this.mTrans = new FrameTransformer2D();
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
			//this.glu.gluLookAt(-1,0,0, 0, 0, 0, 0, 0, 1);
			//System.out.println(this.cam);
		}
		
		if(!readScan()) return;
		
		//draw ego vehicle and axis
		this.egoVehicle = this.lidarFrameProcessor.getEgoVehicle();
		if(this.show3D){
			this.renderVehicle3D(gl, this.egoVehicle, new float[]{1.0f, 1.0f, 1.0f});
		}else{
			this.renderVehicle(gl, this.egoVehicle, new float[]{1.0f, 1.0f, 1.0f});
		}
		
		this.renderAxis(gl);
		this.renderEgoAxis(gl);
		
		gl.glColor3f(0.2f, 0.2f, 0.2f);
		this.renderCircle(gl, 5);
		this.renderCircle(gl, 15);
		this.renderCircle(gl, 25);
		
		//draw the scene
		renderLidarScene(gl);
	}

	/**
	 * Called back before the OpenGL context is destroyed. Release resource such as buffers.
	 */
	@Override
	public void dispose(GLAutoDrawable drawable) { 
	}	
	
	private boolean readScan(){
		if(!this.framePause){
			try{
				preVirtualTable = this.lidarFrameProcessor.get3DProcessor().getVirtualTable();
				this.lidarFrameProcessor.readNextFrame();
			}catch(Exception e){
				e.printStackTrace();
				this.lidarFrameProcessor.stop();
				System.exit(-1);
			}
		}
		return this.lidarFrameProcessor.isReady();
	}
	
	protected void renderLidarScene(GL2 gl){
		this.localWorldFrame = this.lidarFrameProcessor.getScan().getLocalWorldFrame();
		if(this.show3D){
//			this.renderLidarFrame(gl, this.lidarFrameProcessor.get3DProcessor().getVirtualTable().getPoint3D(null, false), new float[] {1, 1, 1});
			this.renderLidarFrame(gl, preVirtualTable.getPoint3D(null, false), new float[] {1, 1, 1});
		}else{
			this.renderLidarFrame(gl, this.lidarFrameProcessor.getScan().getPoints2D(null), new float[] {1, 1, 1});
		}
	}
	
	private void renderCircle(GL2 gl, double radiance){
		gl.glBegin(GL_LINE_LOOP);
		for(double i=0;i<Math.PI*2;i+=0.1){
			gl.glVertex3d(radiance*Math.cos(i),radiance*Math.sin(i), 0);
		}
		gl.glEnd();
	}
	
	private void renderAxis(GL2 gl){
//		System.out.println("draw axis");
		gl.glLineWidth(1);
		gl.glBegin(GL_LINES);
		gl.glColor3f(1.0f, 0.0f, 0.0f);   // Red
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(5,0,0);//body_x-forward
		//Vector x = this.lidarFrameProcessor.bodyFrame.getBodyX2D();
		//gl.glVertex3f((float)x.x*5, (float)x.y*5, 0);
		gl.glColor3f(0.0f, 1.0f, 0.0f);   // Green
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(0,3,0);//body_y-right
		//Vector y = this.lidarFrameProcessor.bodyFrame.getBodyY2D();
		//gl.glVertex3f((float)y.x*5, (float)y.y*5, 0);
		gl.glColor3f(0.0f, 0.0f, 1.0f);   // blue
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(0,0,5);//body_z-down
		gl.glEnd();
	}
	
	private void renderEgoAxis(GL2 gl){
//		System.out.println("draw axis");
		gl.glLineWidth(1);
		gl.glBegin(GL_LINES);
		gl.glColor3f(1.0f, 0.0f, 0.0f);   // Red
		gl.glVertex3f(0,0,0);
		Vector x = this.lidarFrameProcessor.getScan().bodyFrame.getX();
		gl.glVertex3f((float)x.x*2.5f, (float)x.y*2.5f, 0);
		gl.glColor3f(0.0f, 1.0f, 0.0f);   // Green
		gl.glVertex3f(0,0,0);
		Vector y = this.lidarFrameProcessor.getScan().bodyFrame.getY();
		gl.glVertex3f((float)y.x*2.5f, (float)y.y*2.5f, 0);
		gl.glColor3f(0.0f, 0.0f, 1.0f);   // blue
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(0,0,5);//body_z-down
		gl.glEnd();
	}

	protected void renderVehicle(GL2 gl, VehicleModel vehicle, float[] color){
		gl.glPushMatrix();
		Point2D[] corners = vehicle.getCornerPoints(this.localWorldFrame, this.mTrans);
		gl.glColor3fv(color, 0);
		gl.glLineWidth(1);
		gl.glBegin(GL_LINE_LOOP);
		for(Point2D p: corners){
			gl.glVertex2d(p.x,p.y);
		}
		gl.glEnd();
		Point2D[] arrows = vehicle.getArrowPoints(this.localWorldFrame, this.mTrans);
		gl.glBegin(GL_LINE_LOOP);
		for(Point2D p: arrows){
			gl.glVertex2d(p.x,p.y);
		}
		gl.glEnd();
		gl.glPopMatrix();
	}
	
	protected void renderVehicle3D(GL2 gl, VehicleModel vehicle, float[] color){
		gl.glPushMatrix();
		Point2D[] corners = vehicle.getCornerPoints(this.localWorldFrame, this.mTrans);
		gl.glColor3fv(color, 0);
		gl.glLineWidth(1);
		gl.glBegin(GL_LINE_LOOP);
		for(Point2D p: corners){
			gl.glVertex3d(p.x,p.y, 0);
		}
		gl.glEnd();
		gl.glBegin(GL_LINE_LOOP);
		for(Point2D p: corners){
			gl.glVertex3d(p.x,p.y, 1.5);
		}
		gl.glEnd();
		gl.glBegin(GL_LINES);
		for(Point2D p: corners){
			gl.glVertex3d(p.x,p.y, 0);
			gl.glVertex3d(p.x,p.y, 1.5);
		}
		gl.glEnd();
		Point2D[] arrows = vehicle.getArrowPoints(this.localWorldFrame, this.mTrans);
		gl.glBegin(GL_LINE_LOOP);
		for(Point2D p: arrows){
			gl.glVertex2d(p.x,p.y);
		}
		gl.glEnd();
		gl.glPopMatrix();
	}

	protected void renderLidarFrame(GL2 gl, Point2D[] points, float[] color){
		int pointNum = points.length;
		FloatBuffer vertexArray = Buffers.newDirectFloatBuffer(pointNum*2);
		for(int i=0; i<pointNum; i++){
			Point2D p = points[i];
			vertexArray.put((float)p.x);vertexArray.put((float)p.y);
		}
		vertexArray.rewind();
		gl.glColor3fv(color, 0);
		gl.glPointSize(3);
		this.renderVextexArray(gl, vertexArray, pointNum, 2, GL2.GL_FLOAT, GL2.GL_POINTS);
	}
	
	protected void renderLidarFrame(GL2 gl, Point3D[] points, float[] color){
		int pointNum = points.length;
		FloatBuffer vertexArray = Buffers.newDirectFloatBuffer(pointNum*3);
		for(int i=0; i<pointNum; i++){
			Point3D p = points[i];
			vertexArray.put((float)p.x);vertexArray.put((float)p.y);vertexArray.put((float)p.z);
		}
		vertexArray.rewind();
		gl.glColor3fv(color, 0);
		gl.glPointSize(3);
		this.renderVextexArray(gl, vertexArray, pointNum, 3, GL2.GL_FLOAT, GL2.GL_POINTS);
	}

	protected void renderPoints(GL2 gl, Point2D[] points, float[] color, int size){
		int pointNum = points.length;
		if(pointNum==0) return;
		FloatBuffer vertexArray = Buffers.newDirectFloatBuffer(pointNum*2);
		for(int i=0; i<pointNum; i++){
			Point2D p = points[i];
			vertexArray.put((float)p.x);vertexArray.put((float)p.y);
		}
		vertexArray.rewind();
		gl.glColor3fv(color, 0);
		gl.glPointSize(size);
		this.renderVextexArray(gl, vertexArray, pointNum, 2, GL2.GL_FLOAT, GL2.GL_POINTS);
	}
	
	protected void renderPoints(GL2 gl, List<Point2D> points, float[] color, int size){
		int pointNum = points.size();
		if(pointNum==0) return;
		FloatBuffer vertexArray = Buffers.newDirectFloatBuffer(pointNum*2);
		for(int i=0; i<pointNum; i++){
			Point2D p = points.get(i);
			vertexArray.put((float)p.x);vertexArray.put((float)p.y);
		}
		vertexArray.rewind();
		gl.glColor3fv(color, 0);
		gl.glPointSize(size);
		this.renderVextexArray(gl, vertexArray, pointNum, 2, GL2.GL_FLOAT, GL2.GL_POINTS);
	}
	
	protected void renderPoint(GL2 gl, Point2D p, float[] color, int size){
		gl.glColor3fv(color, 0);
		gl.glPointSize(size);
		gl.glBegin(GL_POINTS);
		gl.glVertex2d(p.x, p.y);
		gl.glEnd();
	}
	
	protected void renderLine(GL2 gl, Line line, float[] color){
		gl.glColor3fv(color, 0);
		gl.glLineWidth(3);
		gl.glBegin(GL_LINES);
		gl.glVertex2d(line.p1.x, line.p1.y);
		gl.glVertex2d(line.p2.x, line.p2.y);
		gl.glEnd();
	}
	
	protected void renderTraj(GL2 gl, ArrayList<Point2D> traj, float[] color){
		gl.glColor3fv(color, 0);
		gl.glLineWidth(1);
		gl.glBegin(GL_LINE_STRIP);
		for(Point2D p: traj){
			gl.glVertex2d(p.x, p.y);
		}
		gl.glEnd();
	}
	
	protected void renderTraj(GL2 gl, Point2D[] traj, float[] color){
		gl.glColor3fv(color, 0);
		gl.glLineWidth(1);
		gl.glBegin(GL_LINE_STRIP);
		for(Point2D p: traj){
			gl.glVertex2d(p.x, p.y);
		}
		gl.glEnd();
	}
	
	protected void renderLines(GL2 gl, Line[] lines, float[] color){
		gl.glColor3fv(color, 0);
		gl.glLineWidth(3);
		gl.glBegin(GL_LINES);
		for(Line line : lines){
			gl.glVertex2d(line.p1.x, line.p1.y);
			gl.glVertex2d(line.p2.x, line.p2.y);
		}
		gl.glEnd();
	}

	protected void renderVextexArray(GL2 gl, FloatBuffer vertexArray, int dataNum, int dataDim, int dataNumType, int dataGeoType){
		gl.glVertexPointer(dataDim, dataNumType, 0, vertexArray);
		gl.glDrawArrays(dataGeoType, 0, dataNum);
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
			this.framePause=!this.framePause;
			break;
		case KeyEvent.VK_V:
			this.orthoview = !this.orthoview; break;
		case KeyEvent.VK_K:
			this.show3D = !this.show3D; break;
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