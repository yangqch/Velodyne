package gridmap_generic;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.PriorityQueue;

import javax.management.RuntimeErrorException;

public class OccupyCell implements Cell{
	int freeCnt;
	int occupyCnt;
	transient float last_hit_time;//each cell can only be hit once in each time frame
	transient float cur_hit_time;//
	transient boolean hit;//false-free, true-occupied
//	transient boolean updated;
	
	public OccupyCell() {
		freeCnt=0;
		occupyCnt=0;
		this.reset();
	}
	
	public double getValue(){
		return (occupyCnt+freeCnt)==0 ? 0.5 : (double)occupyCnt/(double)(occupyCnt+freeCnt);
		//return (freeCnt>10 || freeCnt>=occupyCnt) ? 0 : 1;
		//return (occupyCnt+freeCnt==0 || freeCnt >= occupyCnt) ? 0 : 1;
	 }
	
	/**
	 * must be called before update() to tell the cell the new frame is coming 
	 * @param timestamp
	 */
	public void updateTime(float timestamp){
		cur_hit_time = timestamp;
	}
	
	/**
	 * value: 0-free, 1-occupy
	 */
	public void update(double value){
		if(last_hit_time > cur_hit_time){//shouldn't reach here, time is always increasing
			throw new RuntimeErrorException(new Error("time is getting smaller!!!"));
		}
		else if(last_hit_time < cur_hit_time){//new frame is coming, check in the measurement in previous frame
			if(last_hit_time!=-1){
				if(hit) occupyCnt++;
				else freeCnt++;
				hit=false;//initialize as false(free)
			}
			last_hit_time = cur_hit_time;
		}
		hit = hit || (value>0 ? true : false);//hit will be occupy if hit once by occupied
	}
	
	public void reset(){
		last_hit_time=-1;
		cur_hit_time=-1;
		hit=false;
	}
	
	public OccupyCell cloneCell(){
		OccupyCell c = new OccupyCell();
		c.freeCnt = freeCnt;
		c.occupyCnt = occupyCnt;
		reset();
		return c;
	}
	
	public String toString(){
		StringBuilder sb = new StringBuilder();
		sb.append("occupy: ");sb.append(occupyCnt);sb.append('\n');
		sb.append("free: ");sb.append(freeCnt);sb.append('\n');
		sb.append("last_hit_time: ");sb.append(last_hit_time);sb.append('\n');
		sb.append("cur_hit_time: ");sb.append(cur_hit_time);sb.append('\n');
		sb.append("hit: ");sb.append(hit);sb.append('\n');
		return sb.toString();
	}
	
	private void readObject(java.io.ObjectInputStream in) throws IOException, ClassNotFoundException{
		in.defaultReadObject();
		this.reset();
	}
	
	private synchronized void writeObject(java.io.ObjectOutputStream stream) throws IOException {
		if(last_hit_time!=-1){
			if(hit) occupyCnt++;
			else freeCnt++;
			hit=false;//initialize as false(free)	
		}
		stream.defaultWriteObject();
	}
	
	public static void main(String[] argv){
		OccupyCell cell = new OccupyCell();
		int[] hits={0, 0, 1, 1, 0, 1};
		
		for(int hit: hits){
			cell.updateTime(0);
			cell.update(hit);
		}
		System.out.println(cell);
		
		try{
			FileOutputStream out = new FileOutputStream(new File("./OccupyCell"));
			ObjectOutputStream os = new ObjectOutputStream(out);
			os.writeObject(cell);
			out.close(); os.close();
			
			FileInputStream in = new FileInputStream(new File("./OccupyCell"));
			ObjectInputStream is = new ObjectInputStream(in);
			cell=(OccupyCell)is.readObject();
			in.close(); is.close();
			
			for(int hit: hits){
				cell.updateTime(0);
				cell.update(hit);
			}
			
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			System.out.println(cell);
		}
	}
}
