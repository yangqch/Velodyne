package VelodyneDataIO;

import java.awt.geom.Point2D;
import java.io.File;
import java.nio.FloatBuffer;

import calibration.CoordinateFrame;

import com.jogamp.common.nio.Buffers;

public class VirtualScan2D {
	static final double RAD2DEG=180/Math.PI;
	static final double DEG2RAD=Math.PI/180;
	
	public static void main(String[] argv){

	}
	
	private float minRot=-180;
	private float maxRot=180;
	private float resolution=0.5f;
	
	private VirtualRay[] rays;
	private Point3D[] points;
	private CoordinateFrame groundFrame;
	private float time;
	private int rayNum;
	
	public VirtualScan2D(float minRot, float maxRot, float resoltion){
		this.minRot = minRot;
		this.maxRot = maxRot;
		this.resolution = resoltion;
		rayNum = (int)Math.ceil((maxRot-minRot)/resolution);
		rays = new VirtualRay[rayNum];
	}
	
	public VirtualScan2D(float timestampe, CoordinateFrame ground, int numOfPoints) {
		time = timestampe;
		groundFrame = ground;
		points = new Point3D[numOfPoints];
		rayNum = (int)Math.ceil((maxRot-minRot)/resolution);
		rays = new VirtualRay[rayNum];
	}
	
	public void setRay(int idx, float dist){
		if(dist < 0) return;//if dist<0, set to null
		
		if(rays[idx]!=null){
			rays[idx].setFreeDist((float)dist);
		}
		else
			rays[idx] = new VirtualRay((float)dist);
	}
	
	public int getRayNum(){
		return rayNum;
	}
	
	public void put(int idx, float x, float y){
		Point3D p = new Point3D(x, y, 0);
		points[idx] = p;
		this.addPointToRay(p, this.rays);
	}
	
	public VirtualRay[] getRays(){
		return this.rays;
	}
	
	private void addPointToRay(Point3D p, VirtualRay[] rays){
		double dist = Math.sqrt(p.x*p.x + p.y*p.y);
		double rot = Math.atan2(p.y, p.x)*RAD2DEG;
		int rayIdx = (int)Math.floor((rot-minRot)/resolution);
		this.setRay(rayIdx, (float)dist);
	}
	
	public void aggregate(Point3D[] points){
		rays = new VirtualRay[rayNum];
		for(Point3D p: points){
			this.addPointToRay(p, rays);
		}
	}
	
	public int getDistanceBuffer(float startDeg, float endDeg, FloatBuffer fb, RayDistType rayDistType){
		int validNum = 0;
		int start = rotDeg2Idx(startDeg);
		int end = rotDeg2Idx(endDeg);
		fb.rewind();
		for(int i=start; i<end; i++){
			Point3D p = this.ray2Point(i, rayDistType, false);
			if(p==null) continue;
			fb.put((float)p.x);fb.put((float)p.y);fb.put((float)p.z);
			validNum++;
		}
		fb.rewind();
		return validNum;
	}
	

	public int getRefForGridmap(Point3D[] points){
		if(points.length < this.rayNum){
			return 0;
		}
		for(int i=0; i < this.rayNum; i++){
			points[i] = this.ray2Point(i, RayDistType.free, true);
		}
		return rayNum;
	}
	
	
	private int rotDeg2Idx(float rotDeg){
		return (int)Math.floor((rotDeg-minRot)/resolution);
	}
	
	private float idx2rotDeg(int idx){
		return (float)((idx+0.5)*resolution+minRot);
	}
	
	//max_range guaranteed
	//return relative point position w.r.t body frame
	private Point3D ray2Point(int idx, RayDistType rayDistType, boolean replaceNull){
		VirtualRay ray = this.rays[idx];
		double dist=-1;
		if(replaceNull){
			if(ray==null)
				return new Point3D(0, 0, 0);
			else{
				dist = ray.getDistance(rayDistType);
//				if(dist>LidarFrame.MAX_RANGE){
//					return new Point3D(0, 0, 0);
//				}
				dist = dist>LidarFrame.MAX_RANGE ? LidarFrame.MAX_RANGE : dist;
				return new Point3D(dist * Math.cos(idx2rotDeg(idx)*DEG2RAD), dist * Math.sin(idx2rotDeg(idx)*DEG2RAD), 0);
			}
		}else{
			if(ray==null) return null;
			else dist = ray.getDistance(rayDistType);
			return new Point3D(dist * Math.cos(idx2rotDeg(idx)*DEG2RAD), dist * Math.sin(idx2rotDeg(idx)*DEG2RAD), 0);
		}
	}
}

class VirtualRay{
	static final float surface = 0.1f;
	RayDistType rayDistType;
	float[] distance = new float[2];//reflection distance and plus object surface thickness
	
	public VirtualRay(float freeDist) {
		distance[0] = freeDist;
		distance[1] = freeDist + surface;
	}
	
	public void setFreeDist(float freeDist){
		if(distance[0] > freeDist){
			distance[0] = freeDist;
			distance[1] = freeDist + surface;
		}
	}
	
	public float getDistance(RayDistType rayDistType){
		return distance[rayDistType.ordinal()];
	}
}
