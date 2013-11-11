package detection;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import velodyne2d.Point2D;
import velodyne2d.Vector;

public class TrackReader {
	private BufferedReader reader;
	private List<VehicleState> vehicles;
	private String line="";
	private String[] data=null;
	
	private HashMap<Integer, ArrayList<Point2D>> trajectories;
//	private HashMap<Integer, VehicleState> vehicleMap;
	
	private double curTime=-1;
	private double nextTime=-1;
	
	public TrackReader(File f) throws IOException{
		reader = new BufferedReader(new FileReader(f));
		vehicles=new ArrayList<VehicleState>();
		trajectories = new HashMap<Integer, ArrayList<Point2D>>();
		this.readNextTime();
	}
	//read all vehicles in next time frame, store the first line of data in this.data
	private void readNextTime() throws IOException{
		HashMap<Integer, Boolean> updateMap = new HashMap<Integer, Boolean>();
		for(Integer id: this.trajectories.keySet()){
			updateMap.put(id, false);
		}
		
		vehicles.clear();
		if(data!=null && data.length>0){//make vehicle use remained data
			VehicleModel v = new VehicleModel(new Point2D(Double.parseDouble(data[2]), Double.parseDouble(data[3])), new Vector(Double.parseDouble(data[4]), Double.parseDouble(data[5])), 
					Double.parseDouble(data[7]), Double.parseDouble(data[8]), Double.parseDouble(data[6]));
			VehicleState state = new VehicleState(v, Integer.parseInt(data[1]), Double.parseDouble(data[9])==1 ? true : false);
			vehicles.add(state);
			
			//update trajectories
			if(state.isValid){
				if(!this.trajectories.containsKey(state.id)){
					this.trajectories.put(state.id, new ArrayList<Point2D>());
				}
				this.trajectories.get(state.id).add(v.center);
				updateMap.put(state.id, true);
			}
		}
		while((line = reader.readLine())!=null){
			
//			System.out.printf("cur %.3f, next %.3f\n", curTime, nextTime);
			
			data = line.split(",");
			double time = Double.parseDouble(data[0]);
			if(this.nextTime!=time){//reach new time frame
				this.curTime=this.nextTime;
				this.nextTime=time;
				break;
			}
			VehicleModel v = new VehicleModel(new Point2D(Double.parseDouble(data[2]), Double.parseDouble(data[3])), new Vector(Double.parseDouble(data[4]), Double.parseDouble(data[5])), 
					Double.parseDouble(data[7]), Double.parseDouble(data[8]), Double.parseDouble(data[6]));
			VehicleState state = new VehicleState(v, Integer.parseInt(data[1]), Double.parseDouble(data[9])==1 ? true : false);
			vehicles.add(state);
			
			//update trajectories
			if(state.isValid){
				if(!this.trajectories.containsKey(state.id)){
					this.trajectories.put(state.id, new ArrayList<Point2D>());
				}
				this.trajectories.get(state.id).add(v.center);
				updateMap.put(state.id, true);
			}
		}
		
		for(Entry<Integer, Boolean> e: updateMap.entrySet()){
			if(!e.getValue())
				this.trajectories.remove(e.getKey());
		}
	}
	
	public void readNext(double time) throws IOException{
		while(time - this.curTime > 0.01){
//			System.out.printf("query states for %.3f, current time of reader is %.3f\n", time, curTime);
			this.readNextTime();
//			System.out.printf("after read, curTime is %.3f\n", curTime);
			if(this.line==null){
				break;
			}
		}
		if(Math.abs(time - this.curTime)>0.01){
			this.vehicles.clear();
		}
		
	}
	
	public List<VehicleModel> getValidVehicles(){
		List<VehicleModel> vList = new ArrayList<VehicleModel>();
		for(VehicleState state : this.vehicles){
			if(state.isValid){
				vList.add(state.vehicle);
			}
		}
		return vList;
	}
	
	public Map<Integer, VehicleModel> getValidVehiclesWithID(){
		Map<Integer, VehicleModel> vmap = new HashMap<Integer, VehicleModel>();
		for(VehicleState state : this.vehicles){
			if(state.isValid){
				vmap.put(state.id, state.vehicle);
			}
		}
		return vmap;
	}
	
	public List<VehicleModel> getInvalidVehicles(){
		List<VehicleModel> vList = new ArrayList<VehicleModel>();
		for(VehicleState state : this.vehicles){
			if(!state.isValid){
				vList.add(state.vehicle);
			}
		}
		return vList;
	}
	
	public List<ArrayList<Point2D>> getTrajectories(){
		List<ArrayList<Point2D>> trajList = new ArrayList<ArrayList<Point2D>>();
//		System.out.printf("%d trajectories\n", this.trajectories.size());
		for(VehicleState state : this.vehicles){
			if(state.isValid){
				trajList.add(this.trajectories.get(state.id));
//				System.out.printf("length %d", this.trajectories.get(state.id).size());
			}
		}
		return trajList;
	}
	
	public static void main(String[] args){
		try{
			TrackReader rd = new TrackReader(new File("/home/qichi/Desktop/trajectories/CA215N_trip1_refined.log"));
			rd.readNext(30.150);
			List<VehicleModel> vl = rd.getValidVehicles();
			for(VehicleModel v: vl){
				System.out.println(v);
			}
			System.out.println("========================");
			rd.readNext(31.217);
			vl = rd.getValidVehicles();
			for(VehicleModel v: vl){
				System.out.println(v);
			}
			System.out.println("========================");
		}catch(Exception e){
			e.printStackTrace();
		}
		
	}

}

class VehicleState{
	VehicleModel vehicle;
	boolean isValid;
	int id;
	public VehicleState(VehicleModel v, int id, boolean isValid) {
		this.vehicle = v;
		this.isValid = isValid;
		this.id = id;
	}
}