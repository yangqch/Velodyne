package velodyne2d;

import java.awt.Color;
import java.awt.event.KeyEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;

import javax.media.opengl.GL2;
import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import detection.TrackReader;
import detection.VehicleModel;

import VelodyneView.AnimatorStopper;
import VelodyneView.LidarFrameProcessor;

public class LiDARGLViewerForReplay extends LidarGLViewer{

//	private boolean show3D=false;
	private TrackReader reader;
	
	private JFrame plotFrame;
	private Plot2DPanel plot;
	private HashMap<Integer, LinkedList<SpeedPoint>> speedTrajMap;
	static Color[] colors = {Color.black, Color.cyan, Color.darkGray, Color.gray, Color.green, Color.lightGray, Color.magenta, Color.orange, Color.pink, Color.yellow};
	
	public LiDARGLViewerForReplay(LidarFrameProcessor processor, String trackFile) {
		super(processor);
		
		try{
			reader = new TrackReader(new File(trackFile));
		}catch(Exception e){
			e.printStackTrace();
			System.exit(-1);
		}
		
		show3D=false;
		
		plot = new Plot2DPanel();
		plotFrame = new JFrame("speed plot");
		plotFrame.setContentPane(plot);
		plotFrame.setSize(600,600);
		plotFrame.setVisible(true);
		
		speedTrajMap = new HashMap<Integer, LinkedList<SpeedPoint>>();
		
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
			//read vehicles at the current time stamp 
			reader.readNext(this.lidarFrameProcessor.timestamp);
			//valid vehicles
			for(Entry<Integer, VehicleModel> ventry: reader.getValidVehiclesWithID().entrySet()){
				VehicleModel v = ventry.getValue();
				int id = ventry.getKey();
				if(this.show3D){
					this.renderVehicle3D(gl, v, new float[] {1,0,0});
				}else{
					this.renderVehicle(gl, v, new float[] {1,0,0});
				}
				if(this.framePause) continue;
				
				if(!speedTrajMap.containsKey(id)){
					speedTrajMap.put(id, new LinkedList<SpeedPoint>());
				}
				speedTrajMap.get(id).add(new SpeedPoint(this.lidarFrameProcessor.timestamp, v.speed));
			}
			//invalid vehicles
			for(VehicleModel v: reader.getInvalidVehicles()){
				if(this.show3D){
					this.renderVehicle3D(gl, v, new float[] {0,0,1});
				}else{
					this.renderVehicle(gl, v, new float[] {0,0,1});
				}
			}
			//trajectory vehciles
			for(ArrayList<Point2D> traj : reader.getTrajectories()){
				this.renderTraj(gl, mTrans.transform(null, localWorldFrame, traj), new float[] {0, 1, 0});
			}
			//update all queues if time evolve
			if(!this.framePause){
				double earliest = this.lidarFrameProcessor.timestamp - 600;
				ArrayList<Integer> badId = new ArrayList<Integer>();
				for(int id: speedTrajMap.keySet()){
					Queue<SpeedPoint> speeds = speedTrajMap.get(id);
					while(speeds.peek().time<earliest){
						speeds.poll();
					}
					if(speedTrajMap.get(id).isEmpty()) badId.add(id);
				}
				for(int id : badId){
					speedTrajMap.remove(id);
				}
			}
			//clear the panel first
			this.plot.removeAllPlots();
			//draw speed trajectories
			for(Entry<Integer, LinkedList<SpeedPoint>> entry: speedTrajMap.entrySet()){
				LinkedList<SpeedPoint> speeds = entry.getValue();
				double[] x=new double[speeds.size()];
				double[] y=new double[speeds.size()];
				for(int i=0; i<speeds.size(); i++){
					x[i]=speeds.get(i).time;
					y[i]=speeds.get(i).speed;
				}
				this.plot.addLinePlot(entry.getKey().toString(), colors[entry.getKey()%colors.length], x, y);
			}
			plot.setFixedBounds(1,0,3.0);
			
			
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

class SpeedPoint{
	double time;
	double speed;
	public SpeedPoint(double t, double s) {
		time = t;
		speed = s;
	}
}
