package velodyne2d;

import java.awt.event.KeyEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.media.opengl.GL2;

import detection.TrackReader;
import detection.VehicleModel;

import VelodyneView.AnimatorStopper;
import VelodyneView.LidarFrameProcessor;

public class LiDARGLViewerForReplay extends LidarGLViewer{

//	private boolean show3D=false;
	private TrackReader reader;
	
	public LiDARGLViewerForReplay(LidarFrameProcessor processor, String trackFile) {
		super(processor);
		
		try{
			reader = new TrackReader(new File(trackFile));
		}catch(Exception e){
			e.printStackTrace();
			System.exit(-1);
		}
		
		show3D=false;
		
	}
	
	protected void renderLidarScene(GL2 gl){
		this.localWorldFrame = this.lidarFrameProcessor.getScan().getLocalWorldFrame();
		if(this.show3D){
//			this.renderLidarFrame(gl, this.lidarFrameProcessor.get3DProcessor().getVirtualTable().getPoint3D(null, false), new float[] {1, 1, 1});
			this.renderLidarFrame(gl, preVirtualTable.getPoint3D(null, false), new float[] {1, 1, 1});
		}else{
			this.renderLidarFrame(gl, this.lidarFrameProcessor.getScan().getPoints2D(null), new float[] {1, 1, 1});
		}
		
		try{
			reader.readNext(this.lidarFrameProcessor.timestamp);
			for(VehicleModel v: reader.getValidVehicles()){
				if(this.show3D){
					this.renderVehicle3D(gl, v, new float[] {1,0,0});
				}else{
					this.renderVehicle(gl, v, new float[] {1,0,0});
				}
			}
			for(VehicleModel v: reader.getInvalidVehicles()){
				if(this.show3D){
					this.renderVehicle3D(gl, v, new float[] {0,0,1});
				}else{
					this.renderVehicle(gl, v, new float[] {0,0,1});
				}
			}
			for(ArrayList<Point2D> traj : reader.getTrajectories()){
				this.renderTraj(gl, mTrans.transform(null, localWorldFrame, traj), new float[] {0, 1, 0});
			}
		}catch(Exception e){
			e.printStackTrace();
			System.exit(-1);
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
		case KeyEvent.VK_V:
			this.orthoview = !this.orthoview; break;
		case KeyEvent.VK_K:
			this.show3D = !this.show3D; break;
		}
	}
}
