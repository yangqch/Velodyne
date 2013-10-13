package detection;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import velodyne2d.CoordinateFrame2D;
import velodyne2d.FrameTransformer2D;
import velodyne2d.Point2D;
import velodyne2d.Vector;
import velodyne2d.VirtualScan;

public class TrackManager {
	public static String logFileName="/home/qichi/tracks.log";
	
	private HashMap<Integer, Track> trackMap;
	private HashMap<Integer, ArrayList<TrackState>> trackStateMap;
	
	int trackId;//always accumulate
	private PrintWriter logger;
	
	public TrackManager(String logFileName) throws IOException{
		trackMap = new HashMap<Integer, Track>();
		trackStateMap = new HashMap<Integer, ArrayList<TrackState>>();
		this.trackId = 0;
		
		this.logger = new PrintWriter(new File(logFileName));
	}
	
	public void add(List<VehicleModel> vehicles, double timestamp){
		synchronized (this) {
			for(VehicleModel v: vehicles){
				this.trackId++;
				Track track = new Track(v);
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
				if(pos.distance(trans.transform(null, localWorldFrame, track.getCenter()))<=10.0 ){
					isTracked=true; 
					break;
				}
			}
			return isTracked;
		}
	}
	
	public void update(VirtualScan scan, FrameTransformer2D trans, double timestamp){
		synchronized (this) {
			ArrayList<Integer> lostTracks = new ArrayList<Integer>();
			//update tracks
			for(Entry<Integer, Track> entry: trackMap.entrySet()){
				Track track = entry.getValue();
				int trackId = entry.getKey();
				//update track
				track.diffuse();
				track.update(scan, trans);
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
}

class TrackState{
	double timestamp;
	int trackId;
	Point2D pos;
	Vector direction;
	double speed;
	double width;
	double length;
	
	public TrackState(Track track, int trackId, double timestamp){
		this.timestamp = timestamp;
		this.trackId = trackId;
		VehicleModel vehicle = track.getAvgVehicle();
		pos = vehicle.center;
		direction = vehicle.direction;
		speed = vehicle.speed;
		width = vehicle.shape.width;
		length = vehicle.shape.length;
	}
	
	public String toString(){
		return String.format("%.3f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", 
				timestamp, trackId, pos.x, pos.y, direction.x, direction.y, speed, width, length);
	}
}