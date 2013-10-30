package detection;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Properties;
import java.util.Set;

import velodyne2d.CoordinateFrame2D;
import velodyne2d.FrameTransformer2D;
import velodyne2d.Point2D;
import velodyne2d.Vector;
import velodyne2d.VirtualScan;

public class TrackManager {
	private HashMap<Integer, Track> trackMap;
	private HashMap<Integer, ArrayList<TrackState>> trackStateMap;
	
	int trackId;//always accumulate
	private PrintWriter logger;
	
	private double initDistFromTrack;
	private TrackConf trackConf;
	
	public TrackManager(Properties conf) throws IOException{
		trackMap = new HashMap<Integer, Track>();
		trackStateMap = new HashMap<Integer, ArrayList<TrackState>>();
		this.trackId = 0;
		
		this.logger = new PrintWriter(new File(conf.getProperty("trackLogFile")));
		this.initDistFromTrack = Double.parseDouble(conf.getProperty("initDistFromTrack"));
		this.trackConf = new TrackConf(conf);
	}
	
	public void add(List<VehicleModel> vehicles, double timestamp){
		synchronized (this) {
			for(VehicleModel v: vehicles){
				this.trackId++;
				Track track = new Track(v, trackConf);
				trackMap.put(this.trackId, track);
				//save track state
				this.putTrackState(this.trackId, track, timestamp);
			}
			//update the track's state and shape
		}
	}
	
	/**
	 * check if there is any track around this pos
	 * @param pos
	 */
	public boolean isTracked(Point2D pos, CoordinateFrame2D localWorldFrame, FrameTransformer2D trans){
		synchronized (this) {
			boolean isTracked = false;
			for(Entry<Integer, Track> entry: trackMap.entrySet()){
				Track track = entry.getValue();
//				if(pos.distance(trans.transform(null, localWorldFrame, track.getCenter()))<=10.0 ){
//				System.out.printf("vehicle pos %s\n", pos);
//				System.out.printf("track pos %s\n", track.getCenter());
				if(pos.distance(track.getCenter())<=initDistFromTrack ){
					isTracked=true;
					break;
				}
			}
			return isTracked;
		}
	}
	
	public void update(VirtualScan scan, FrameTransformer2D trans, double timestamp, boolean[] mask){
		synchronized (this) {
			ArrayList<Integer> lostTracks = new ArrayList<Integer>();
			//update tracks
			for(Entry<Integer, Track> entry: trackMap.entrySet()){
				Track track = entry.getValue();
				int trackId = entry.getKey();
				//update track
				track.diffuse();
				track.update(scan, trans, mask);
				track.resample();
				//save to trackStateMap
				this.putTrackState(trackId, track, timestamp);
				//check termination
				if(track.isTerminate()){
					lostTracks.add(trackId);
				}
			}
			//delete lost tracks and dump to log file
			for(Integer trackId: lostTracks){
				this.trackMap.remove(trackId);
				this.dumpTrack(trackId);
			}
		}
	}
	/**
	 * dump track to log file
	 * @param track
	 */
	private void dumpTrack(int trackId){
		ArrayList<TrackState> states = this.trackStateMap.get(trackId);
		if(states==null){
			return;
		}
		for(TrackState state: states){
			this.logger.println(state);
		}
		logger.flush();
		this.trackStateMap.remove(trackId);
	}
	
	public void dumpExternalTrack(VehicleModel v, double timestamp){
		this.logger.printf("%.3f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n", timestamp, -1, v.center.x, v.center.y, v.direction.x, v.direction.y, v.speed, v.shape.width, v.shape.length, 1);
	}
	
	/**
	 * add trackState to trackStateMap
	 * @param id
	 * @param track
	 * @param timestamp
	 */
	private void putTrackState(int id, Track track, double timestamp){
		TrackState state = new TrackState(track, id, timestamp);
		ArrayList<TrackState> list = this.trackStateMap.get(id);
		if(list==null){
			list = new ArrayList<TrackState>();
		}
		list.add(state);
		trackStateMap.put(id, list);
	}
	
	public Set<Integer> getIds(){
		Set<Integer> x;
		synchronized (this) {
			x = trackStateMap.keySet();
		}
		return x;
	}
	
	public ArrayList<Point2D> getTraj(int idx){
		ArrayList<Point2D> traj = new ArrayList<Point2D>();
		synchronized (this) {
			if(this.trackStateMap.containsKey(idx)){
				ArrayList<TrackState> list = this.trackStateMap.get(idx);
				for(TrackState t: list){
					traj.add(t.pos);
				}
			}
		}
		return traj;
	}
	
	public Collection<Track> getTracks(){
		Collection<Track> tracks = null;
		synchronized (this) {
			tracks = this.trackMap.values();
		}
		return tracks;
	}
	
	public void logAllTracks(){
		synchronized (this) {
			int[] ids = new int[trackStateMap.size()];
			int i=0;
			for(int id: trackStateMap.keySet()){
				ids[i++] = id;
			}
			for(int id: ids){
				System.out.printf("dump %d\n", id);
				this.dumpTrack(id);
			}
			logger.flush();
		}
	}
	
	public int getNumOfTrack(){
		int num = 0;
		synchronized (this) {
			num = this.trackMap.size();
		}
		return num;
	}
}

class TrackState{
	double timestamp;
	int trackId;
	Point2D pos;
	Vector direction;
	double speed;
	double width;
	double length;
	int lost;
	public TrackState(Track track, int trackId, double timestamp){
		this.timestamp = timestamp;
		this.trackId = trackId;
		VehicleModel vehicle = track.getAvgVehicle();
		pos = vehicle.center;
		direction = vehicle.direction;
		speed = vehicle.speed;
		width = vehicle.shape.width;
		length = vehicle.shape.length;
		
		lost=track.isLost() ? 0 : 1;
	}
	
	public String toString(){
		return String.format("%.3f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d", 
				timestamp, trackId, pos.x, pos.y, direction.x, direction.y, speed, width, length, lost);
	}
}