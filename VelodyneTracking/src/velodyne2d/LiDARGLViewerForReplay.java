package velodyne2d;

import java.awt.Color;
import java.awt.Font;
import java.awt.event.KeyEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import com.jogamp.opengl.util.awt.TextRenderer;

import detection.TrackReader;
import detection.VehicleModel;

import VelodyneView.AnimatorStopper;
import VelodyneView.LidarFrameProcessor;

public class LiDARGLViewerForReplay extends LidarGLViewer{

	static Color[] colors = {Color.black, Color.cyan, Color.gray, Color.green, Color.lightGray, Color.magenta, Color.orange, Color.pink, Color.yellow};
	static double showTimeRange = 30;
	static double MPS2MPH=2.23693629 * 15;
	
	private TrackReader reader;
	
	private JFrame plotFrame;
	private Plot2DPanel plot;
	private HashMap<Integer, LinkedList<SpeedPoint>> speedTrajMap;
	private LinkedList<SpeedPoint> egoSpeedTraj;
	private Point2D prevEgoPos;
	private double earliest = -1;
	private HashMap<Integer, Color> colorMap;
	private int nextColor;
	
	private TextRenderer text;
	
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
		nextColor = 0;
		colorMap = new HashMap<Integer, Color>();
		
		egoSpeedTraj = new LinkedList<SpeedPoint>();
		
		text = new TextRenderer(new Font("SansSerif", Font.BOLD, 12));
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
				if(this.show3D){
					this.renderVehicle3D(gl, v, new float[] {1,0,0});
				}else{
					this.renderVehicle(gl, v, new float[] {1,0,0});
				}
			}
			//render vehicle ID
			text.begin3DRendering();
			text.setColor(1, 1, 0, 1.0f);
			for(Entry<Integer, VehicleModel> ventry: reader.getValidVehiclesWithID().entrySet()){
				VehicleModel v = ventry.getValue();
				renderVehicleText(text, v, ventry.getKey().toString());
			}
			text.end3DRendering();
			
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
				//update ego
				if(prevEgoPos==null){
					prevEgoPos = this.egoVehicle.center; 
				}else{
					egoSpeedTraj.add(new SpeedPoint(this.lidarFrameProcessor.timestamp, this.egoVehicle.center.distance(prevEgoPos)));
					prevEgoPos = this.egoVehicle.center;
				}
				//update tracked vehicle speed
				for(Entry<Integer, VehicleModel> ventry: reader.getValidVehiclesWithID().entrySet()){
					VehicleModel v = ventry.getValue();
					int id = ventry.getKey();
					
					if(!speedTrajMap.containsKey(id)){
						speedTrajMap.put(id, new LinkedList<SpeedPoint>());
					}
					speedTrajMap.get(id).add(new SpeedPoint(this.lidarFrameProcessor.timestamp, v.speed));
				}
				
				//exceed 60 secs, truncate trajectories 
				if(earliest==-1) earliest = this.lidarFrameProcessor.timestamp;
				if(earliest < this.lidarFrameProcessor.timestamp - showTimeRange){
					earliest += showTimeRange/2;
					
					ArrayList<Integer> badId = new ArrayList<Integer>();
					for(int id: speedTrajMap.keySet()){
						Queue<SpeedPoint> speeds = speedTrajMap.get(id);
						while(speeds.peek()!=null && speeds.peek().time < earliest){
							speeds.poll();
						}
						if(speeds.isEmpty()){
							badId.add(id);
						}
					}
					for(int id : badId){
						speedTrajMap.remove(id);
						colorMap.remove(id);
					}
					while(egoSpeedTraj.peek()!=null && egoSpeedTraj.peek().time < earliest){
						egoSpeedTraj.poll();
					}
				}
			}
			
			this.plotSpeedTrajs();
			
		}catch(Exception e){
			e.printStackTrace();
			System.exit(-1);
		}
	}
	
	private void plotSpeedTrajs(){
		//clear the panel first
		this.plot.removeAllPlots();
		//draw speed trajectories
		ArrayList<Integer> ids = new ArrayList<Integer>();
		for(Integer id: speedTrajMap.keySet()){
			ids.add(id);
		}
		Collections.sort(ids);
		for(Integer id : ids){
			LinkedList<SpeedPoint> speeds = speedTrajMap.get(id);
			double[] x=new double[speeds.size()];
			double[] y=new double[speeds.size()];
			
			for(int i=0; i<speeds.size(); i++){
				double sum = 0; int cnt=0; int win=5;
				//moving average
				for(int j=i-win; j<i+win; j++){
					if(j<0 || j>=speeds.size()) continue;
					cnt++;
					sum += speeds.get(j).speed;
				}
				//set time and speed
				x[i]=speeds.get(i).time;
				y[i] = (sum/cnt) * MPS2MPH;
			}
			if(!colorMap.containsKey(id)){
				colorMap.put(id, colors[nextColor]);
				nextColor = (nextColor+1)%colors.length;
			}
			this.plot.addLinePlot(id.toString(), colorMap.get(id), x, y);
		}
		if(egoSpeedTraj.size()>0){
			double[] x=new double[egoSpeedTraj.size()];
			double[] y=new double[egoSpeedTraj.size()];
			for(int i=0; i<egoSpeedTraj.size(); i++){
				x[i]=egoSpeedTraj.get(i).time;
				y[i]=egoSpeedTraj.get(i).speed * MPS2MPH;
			}
			this.plot.addLinePlot("ego", Color.red, x, y);
		}
		plot.setFixedBounds(0,earliest, earliest+showTimeRange);
		plot.setFixedBounds(1,0,60);
		plot.addLegend("east");
		plot.setAxisLabels("time(secs)", "speed(mph)");
		plot.getAxis(0).setLabelFont(new Font("Courier", Font.BOLD, 20));
		plot.getAxis(0).setLabelPosition(0.5, -0.15);
		plot.getAxis(1).setLabelFont(new Font("Courier", Font.BOLD, 20));
		plot.getAxis(1).setLabelPosition(-0.15, 0.5);
		plot.getAxis(1).setLabelAngle(-Math.PI / 2);
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
