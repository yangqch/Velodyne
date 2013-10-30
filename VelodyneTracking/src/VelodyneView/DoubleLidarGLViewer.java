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
import velodyne2d.Vector;

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

public class DoubleLidarGLViewer extends LidarGLViewer implements GLEventListener, KeyListener{
	//for frame control
	private DoubleLidarFrameProcessor lidarFrameProcessor;
	
	private boolean[][] compMask=null;
	private boolean[][] dynamicMask=null;
	
	public DoubleLidarGLViewer(){}

	/** Constructor to setup the GUI for this Component */
	public DoubleLidarGLViewer(DoubleLidarFrameProcessor lidarFrameProcessor) {
		super(null);
		this.lidarFrameProcessor = lidarFrameProcessor;
	}
	
	@Override
	protected void renderLidarScene(GL2 gl){
//		this.glu.gluLookAt(this.cam.x, this.cam.y, this.cam.z, 0, 0, 0, 0, 0, 1);

		if(!this.framePause){
			try{
				this.lidarFrameProcessor.readNextFrame();
				compMask = this.lidarFrameProcessor.findConnComp(0.5, 10, null);
				//dynamicMask = this.lidarFrameProcessor.filterStaticPoints();
				//dynamicMask = this.lidarFrameProcessor.filterStaticComp(0.5, 10, 0.5);
			}catch(Exception e){
				e.printStackTrace();
				this.lidarFrameProcessor.stop();
				System.exit(-1);
			}
		}
		
		BodyFrame egoFrame = this.lidarFrameProcessor.getCurFrame().getBodyFrame();
		this.egoVehicle = new VehicleModel(egoFrame.getPosion2D(), egoFrame.getBodyY2D(), VehicleModel.default_width, VehicleModel.default_length, 0); 
		this.localWorldFrame = this.lidarFrameProcessor.getCurFrame().getLocalWorldFrame();
		
		if(this.showRawData){
			gl.glColor3f(0, 0, 1);
			this.renderVirtualTable(gl, this.lidarFrameProcessor.getMainTable(), null);
			gl.glColor3f(1, 0, 0);
			this.renderVirtualTable(gl, this.lidarFrameProcessor.getMainTable(), compMask); //show all virtual table data
			//this.renderVirtualTable(gl, this.lidarFrameProcessor.getMainTable(), dynamicMask);
		}else{
			gl.glColor3f(0, 1, 0);
			//this.renderVirtualTable(gl, this.lidarFrameProcessor.getMainTable(), dynamicMask); //show all virtual table data
			this.renderVirtualTable(gl, this.lidarFrameProcessor.getMainTable(), null); //show all virtual table data
			gl.glColor3f(0, 0, 1);
			this.renderVirtualTable(gl, this.lidarFrameProcessor.getMotionTable(), null); //show motion data
			
		}

	}
	
	protected void renderEgoAxis(GL2 gl){
//		System.out.println("draw axis");
		gl.glLineWidth(1);
		gl.glBegin(GL_LINES);
		gl.glColor3f(1.0f, 0.0f, 0.0f);   // Red
		gl.glVertex3f(0,0,0);
		Vector x = this.lidarFrameProcessor.getCurFrame().getBodyFrame().getBodyX2D();
		gl.glVertex3f((float)x.x*2.5f, (float)x.y*2.5f, 0);
		gl.glColor3f(0.0f, 1.0f, 0.0f);   // Green
		gl.glVertex3f(0,0,0);
		Vector y = this.lidarFrameProcessor.getCurFrame().getBodyFrame().getBodyY2D();
		gl.glVertex3f((float)y.x*2.5f, (float)y.y*2.5f, 0);
		gl.glColor3f(0.0f, 0.0f, 1.0f);   // blue
		gl.glVertex3f(0,0,0);
		gl.glVertex3f(0,0,5);//body_z-down
		gl.glEnd();
	}


}