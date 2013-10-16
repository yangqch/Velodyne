package velodyne2d;

import java.awt.event.*;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.glu.GLU;

import VelodyneDataIO.Point3D;
import VelodyneView.AnimatorStopper;
import VelodyneView.CameraControl;
import VelodyneView.LidarFrameProcessor;

import calibration.BodyFrame;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.util.gl2.GLUT;

import detection.RayMeas;
import detection.Track;
import detection.VehicleModel;

import static javax.media.opengl.GL.*;  // GL constants
import static javax.media.opengl.GL2ES1.GL_PERSPECTIVE_CORRECTION_HINT;
import static javax.media.opengl.fixedfunc.GLLightingFunc.GL_SMOOTH;
import static javax.media.opengl.fixedfunc.GLMatrixFunc.GL_MODELVIEW;
import static javax.media.opengl.fixedfunc.GLMatrixFunc.GL_PROJECTION;

public class LidarGLViewerForDetection extends LidarGLViewer{
	//app related control
	private int showFrame;//show 0-current frame only or 1-with previous or 2-next lidar frames
	private boolean showLines;//show raw lines extracted from segments with breakpoints
	private boolean showAdjLines;//show adjusted lines in initializer
	private boolean showCar;//show generated cars and their predictions
	private boolean showCarMeasurements;//show expected lidar points on car
	private boolean showPredictCar;//show predicted car in next frame, usually show with next frame
	//container for detected vehicles
	private List<VehicleModel> vehicles;
	
	/** Constructor to setup the GUI for this Component */
	public LidarGLViewerForDetection(LidarFrameProcessor processor) {
		super(processor);
		
		this.showFrame=0;
		this.showLines=false;
		this.showAdjLines=true;
		this.showCar=true;
		this.showCarMeasurements=false;
		this.showPredictCar=false;
		
		this.vehicles= new ArrayList<VehicleModel>();
	}

	protected void renderLidarScene(GL2 gl){
		//DRAW lidar point cloud in 2d
		//previous scan is blue
		//current segments are white
		//motion vectors are red
		//extracted lines are green
		//detected cars are white

		this.localWorldFrame = this.lidarFrameProcessor.getScan().getLocalWorldFrame();
		
		if(this.showFrame==1){//show previous frame
			this.renderLidarFrame(gl, mTrans.transform(this.lidarFrameProcessor.getPrevScan().getLocalWorldFrame(), this.localWorldFrame, this.lidarFrameProcessor.getPrevScan().getPoints2D(null)), new float[] {0,0,1});	
		}else if(this.showFrame==2){//show next frame
			this.renderLidarFrame(gl, mTrans.transform(this.lidarFrameProcessor.getNextScan().getLocalWorldFrame(), this.localWorldFrame, this.lidarFrameProcessor.getNextScan().getPoints2D(null)), new float[] {0,0,1});
		}else{
			this.renderLidarFrame(gl, this.lidarFrameProcessor.getScan().getPoints2D(null), new float[] {1, 1, 1});
		}
		
		//draw segments and motion in frame
		List<Segment> segments1 = this.lidarFrameProcessor.findMotionSegment();
		//System.out.println("==================================");
		if(this.showFrame!=0){
			for(Segment seg: segments1){
				//transform segment to prevScan frame
				Point2D[] points = seg.getPoints();
				this.renderLidarFrame(gl, points, new float[] {1, 1, 1});
				//get lines connect motion points and motion vectors
				Line[] lines = showFrame==1 ? seg.getMotion().getPrevMoveLines(points) : seg.getMotion().getNextMoveLines(points);
				this.renderLines(gl, lines, new float[]{1, 0, 0});
			}	
		}
		
		
		//draw extracted lines in frame
		List<Segment> segments2 = this.lidarFrameProcessor.extractLines(segments1);
		if(this.showLines){
			for(Segment seg: segments2){
				if(seg.getMotion().isMoving()==0) continue;	
				Line[] lines = seg.getLineExtractor().getLines();
				this.renderLines(gl, lines, new float[] {0, 1, 0});
				this.renderPoints(gl, seg.getLineExtractor().getBreakPoints(), new float[] {0, 1, 0}, 5);
			}	
		}
		
		//draw detected vehicles
		vehicles.clear();
		for(Segment seg: segments2){
			//initialize vehicle
			if(seg.getMotion().isMoving()==0) continue;	
			VehicleModel v = this.lidarFrameProcessor.getVehInitializer().initialize(seg);
			//draw adjusted lines in initializer, 
			if(this.lidarFrameProcessor.getVehInitializer().numOfLine>0 && this.showAdjLines){
				this.renderLine(gl, this.lidarFrameProcessor.getVehInitializer().getFrontLine(), new float[] {0, 1, 1});
				if(this.lidarFrameProcessor.getVehInitializer().numOfLine==2){
					this.renderLine(gl, this.lidarFrameProcessor.getVehInitializer().getSideLine(), new float[] {0, 1, 1});
				}
			}
			if(v!=null) vehicles.add(v);
		}
		//draw vehicles and measurements
		if(vehicles.size()>0){
			System.out.printf("~~~~~~~~~~~~~~~~~~~~~~~detect %d vehicle", vehicles.size());
			if(this.showCar){
				for(VehicleModel v : vehicles){
					System.out.printf("vehicle at %f, %f\n", v.center.x, v.center.y);
					this.renderVehicle(gl, v, new float[]{1.0f, 1.0f, 1.0f});
					
					if(this.showCarMeasurements){
						//draw predicted measurement on the car
						v.predictMeasurement(this.lidarFrameProcessor.getScan(), mTrans);
						//this.renderPoints(gl, v.getDetCorners(this.lidarFrameProcessor.getScan().getLocalWorldFrame(), mTrans), new float[] {1, 0, 0});
						Point2D[] points = v.getMeasurements();
						ArrayList<RayMeas> rayMeas = v.getRayMeas();
						for(int i=0; i<rayMeas.size(); i++){
							if(Math.abs(rayMeas.get(i).score)<0.1){
								this.renderPoint(gl, points[i], new float[] {0, 1, 0}, 5);
							}
							else if(rayMeas.get(i).score<0){
								this.renderPoint(gl, points[i], new float[] {1, 0, 0}, 5);
							}else{
								this.renderPoint(gl, points[i], new float[] {0, 0, 1}, 5);
							}
						}
					}
				}
			}			
			//draw predict vehicles
			if(this.showPredictCar){
				for(VehicleModel v : vehicles){
					System.out.printf("predict vehicle at %f, %f\n", v.center.x, v.center.y);
					this.renderVehicle(gl, v.reproduce(), new float[]{1f, 1f, 0f});
				}	
			}
		}
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
		case KeyEvent.VK_F:
			this.showFrame=(this.showFrame+1)%3;
			break;
		case KeyEvent.VK_L:
			this.showLines = !this.showLines;
			break;
		case KeyEvent.VK_A:
			this.showAdjLines = !this.showAdjLines;
			break;
		case KeyEvent.VK_C:
			this.showCar=!this.showCar;
			break;
		case KeyEvent.VK_M:
			this.showCarMeasurements=!this.showCarMeasurements;
			break;
		case KeyEvent.VK_P:
			this.showPredictCar = !this.showPredictCar;
			break;
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