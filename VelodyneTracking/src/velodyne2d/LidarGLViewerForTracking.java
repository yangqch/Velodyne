package velodyne2d;

import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import javax.media.opengl.GL2;

import detection.Track;
import detection.TrackManager;
import detection.VehicleModel;

import VelodyneView.AnimatorStopper;
import VelodyneView.LidarFrameProcessor;

public class LidarGLViewerForTracking extends LidarGLViewerForDetection{

	private boolean showPrior;//show particles/vehicles before resampling
	private boolean showCar;//show vehicles or particles
	private boolean slowMotion;
	
	private Collection<Track> tracks;//existed tracks
	private List<VehicleModel> newVehicles;//new detected vehicles
	
	private TrackManager manager;
	
	public LidarGLViewerForTracking(LidarFrameProcessor processor) {
		super(processor);
		showPrior = false;
		showCar = false;
		slowMotion = false;
		tracks = null;
		newVehicles = new ArrayList<VehicleModel>();
		
		try{
			manager = new TrackManager(TrackManager.logFileName);
		}catch(Exception e){
			e.printStackTrace();
			throw new RuntimeException();
		}
	}
	
	@Override
	protected void renderLidarScene(GL2 gl) {
		this.localWorldFrame = this.lidarFrameProcessor.getScan().getLocalWorldFrame();
		this.renderLidarFrame(gl, this.lidarFrameProcessor.getScan().getPoints2D(null), new float[] {0,0,1});
		if(!framePause){//update tracks and detections, pause when new track us added
			//update tracks
			manager.update(this.lidarFrameProcessor.getScan(), mTrans, this.lidarFrameProcessor.timestamp);
			//detection
			List<Segment> segments1 = this.lidarFrameProcessor.findMotionSegment();
			List<Segment> segments2 = this.lidarFrameProcessor.extractLines(segments1);
			for(Segment seg: segments2){
				if(seg.getMotion().isMoving()==0) continue;
				if(manager.isTracked(seg.getCenter(), this.localWorldFrame, mTrans)) continue;
				VehicleModel v = this.lidarFrameProcessor.getVehInitializer().initialize(seg);				
				if(v!=null) newVehicles.add(v);
			}
			//add to track manager
			manager.add(newVehicles, this.lidarFrameProcessor.timestamp);
			if(newVehicles.size()>0 || slowMotion){
				System.out.printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++new track is added\n");
				framePause=true;
				slowMotion=true;
				newVehicles.clear();
			}
			tracks = manager.getTracks();
			//visualize tracks
			for(Track track : tracks){
				//visualize average vehicle state
				this.renderVehicle(gl, track.getAvgVehicle(), new float[] {1, 0, 0});
				//particles
				Point2D[] particles = mTrans.transform(null, this.localWorldFrame, track.getPosterior());
				this.renderPoints(gl, particles, new float[] {1, 1, 1}, 1);
			}			
		}
		else{//draw the current tracks for debugging
			//visualize track
			for(Track t : tracks){
				//visualize average vehicle state
				this.renderVehicle(gl, t.getAvgVehicle(), new float[] {1, 0, 0});
				//visualize the particles before/after resampling
				if(showPrior){
					Point2D[] particles = mTrans.transform(null, this.localWorldFrame, t.getPrior(1));
					this.renderPoints(gl, particles, new float[] {1, 1, 1}, 1);
					particles = mTrans.transform(null, this.localWorldFrame, t.getPrior(0.01));
					this.renderPoints(gl, particles, new float[] {1, 0, 0}, 2);
				}else{
					Point2D[] particles = mTrans.transform(null, this.localWorldFrame, t.getPosterior());
					this.renderPoints(gl, particles, new float[] {1, 1, 1}, 1);
				}
				//visualize cars
				if(showCar){
					VehicleModel[] vehs = null;
					if(showPrior){
						vehs = t.getPriorVehicle(0.1); 
					}else{
						vehs = t.getPosteriorVehicle();
					}
					for(VehicleModel veh: vehs){
						float a=1;
						this.renderVehicle(gl, veh, new float[]{a, a, a});
					}
				}
			}
		}
	}
	
//	@Override
//	protected void renderLidarSceneForPF(GL2 gl) {
//		this.localWorldFrame = this.lidarFrameProcessor.getScan().getLocalWorldFrame();
//		this.renderLidarFrame(gl, this.lidarFrameProcessor.getScan().getPoints2D(null), new float[] {0,0,1});
//		
//		if(!framePause){
//			//add new detected vehicles
//			for(VehicleModel v : newVehicles){
//				tracks.add(new Track(v));
//				System.out.printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++new track is added\n");
//				framePause=false;
//				slowMotion = true;
//			}
//			newVehicles.clear();
//			//track predict, update and resample
//			int cnt=0;
//			for(Track t : tracks){
//				System.out.printf("track %d\n", cnt++);
//				t.diffuse();
//				t.update(this.lidarFrameProcessor.getScan(), mTrans);
//				t.resample();
//				//visualize average vehicle state
//				this.renderVehicle(gl, t.getAvgVehicle(), new float[] {1, 0, 0});
//				//visualize the particles after resampling
//				Point2D[] particles = mTrans.transform(null, this.localWorldFrame, t.getPosterior());
//				this.renderPoints(gl, particles, new float[] {1, 1, 1}, 1);
//			}
//			//detect and initialize vehicle far from tracks
//			List<Segment> segments1 = this.lidarFrameProcessor.findMotionSegment();
//			List<Segment> segments2 = this.lidarFrameProcessor.extractLines(segments1);
//			for(Segment seg: segments2){
//				if(seg.getMotion().isMoving()==0) continue;
//				boolean isTrack = false;
//				Point2D segCenter = seg.getCenter();
//				for(Track t : tracks){
//					if(segCenter.distance(mTrans.transform(null, this.localWorldFrame, t.getCenter()))<=10.0 ){
//						isTrack=true; break;
//					}
//				}
//				if(!isTrack){//if the segment is far from existed tracks, initialize vehicle
//					VehicleModel v = this.lidarFrameProcessor.getVehInitializer().initialize(seg);				
//					if(v!=null) newVehicles.add(v);
//				}
//			}
//			for(Iterator<Track> iter = tracks.iterator(); iter.hasNext(); ){
//				if(iter.next().isTerminate()){
//					iter.remove();
//					System.out.printf("-------------------------------------------------------------------------------------------one track is deleted\n");
//				}
//			}
//			System.out.printf("totally %d tracks\n", tracks.size());
//			if(tracks.size()>0 && slowMotion){
//				framePause=true;
//				showCar=false;
//				showPrior=false;
//			}
//		}else{
//			//visualize track
//			for(Track t : tracks){
//				//visualize average vehicle state
//				this.renderVehicle(gl, t.getAvgVehicle(), new float[] {1, 0, 0});
//				//visualize the particles before/after resampling
//				if(showPrior){
//					Point2D[] particles = mTrans.transform(null, this.localWorldFrame, t.getPrior(1));
//					this.renderPoints(gl, particles, new float[] {1, 1, 1}, 1);
//					particles = mTrans.transform(null, this.localWorldFrame, t.getPrior(0.01));
//					this.renderPoints(gl, particles, new float[] {1, 0, 0}, 2);
//				}else{
//					Point2D[] particles = mTrans.transform(null, this.localWorldFrame, t.getPosterior());
//					this.renderPoints(gl, particles, new float[] {1, 1, 1}, 1);
//				}
//				//visualize cars
//				if(showCar){
//					VehicleModel[] vehs = null;
//					if(showPrior){
//						vehs = t.getPriorVehicle(0.1); 
//					}else{
//						vehs = t.getPosteriorVehicle();
//					}
//					for(VehicleModel veh: vehs){
//						float a=1;
//						this.renderVehicle(gl, veh, new float[]{a, a, a});
//					}
//				}
//			}
//		}
//
//		
//	}
	
	private void logAllTracks(){
		this.manager.logAllTracks();
	}
	
	@Override
	public void keyPressed(KeyEvent e) {
		// TODO Auto-generated method stub
		int key = e.getKeyCode();
		switch(key){
		case KeyEvent.VK_ESCAPE:
			System.out.println("escape is pressed");
			this.logAllTracks();
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
		case KeyEvent.VK_P:
			this.showPrior=!this.showPrior;break;
		case KeyEvent.VK_C:
			this.showCar=!this.showCar;break;
		case KeyEvent.VK_S:
			this.slowMotion=!this.slowMotion;
			this.framePause=!this.framePause;
			break;
		case KeyEvent.VK_V:
			this.orthoview = !this.orthoview; break;
		}
	}
	
}
