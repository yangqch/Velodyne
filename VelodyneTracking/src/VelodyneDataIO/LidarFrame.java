package VelodyneDataIO;

import java.nio.FloatBuffer;
import java.util.ArrayList;

import javax.management.RuntimeErrorException;

import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;

import com.jogamp.common.nio.Buffers;

public class LidarFrame {
	public final static double MAX_RANGE=50.0;
	public final static double MIN_RANGE=3.0;
	
	float time;//system time in AidedINS
	//ArrayList<Point3D> dataPoints;//data points in x,y,z coordinates
	float [] dataPoints;
	Point3D[] dataPoints3D;
	float [] intensityArray;//
	int pointNum;//number of points in this frame
	CoordinateFrame localWorldFrame;//params of local world frame w.r.t world frame
	BodyFrame bodyFrame;//body of vehicle itself
	
	public LidarFrame(float time, BodyFrame bodyFrame, CoordinateFrame localWorldFrame, int pointNum) {
		this.time = time;
		this.localWorldFrame = localWorldFrame;
		this.bodyFrame = bodyFrame;
		this.pointNum = pointNum;
		dataPoints = new float[pointNum*3];
		intensityArray = new float[pointNum];
		dataPoints3D = null;
	}
	
	public void putData(int idx, float x, float y, float z, float intensity){
		//if(Math.sqrt(x*x+y*y+z*z)>=MAX_RANGE) return;
		dataPoints[idx*3]=x;dataPoints[idx*3+1]=y;dataPoints[idx*3+2]=z;
		intensityArray[idx]=intensity;
	}
//	/**
//	 * transform data points from body to new frame using transFrame
//	 * and set body to new frame
//	 * @param trans
//	 * @param transFrame: CoordinateFrame contains T and R from Lidar to the new frame, usually ground plane
//	 */
//	public void transfromBodyFrame(FrameTransformer trans, CoordinateFrame transFrame){
//		//change point from body to new(ground plane)
//		dataPoints3D = trans.transform4D(transFrame, dataPoints);
//		for(int i=0; i<dataPoints3D.length; i++){
//			this.dataPoints[i*3] = (float)this.dataPoints3D[i].x;
//			this.dataPoints[i*3+1] = (float)this.dataPoints3D[i].y;
//			this.dataPoints[i*3+2] = (float)this.dataPoints3D[i].z;
//		}
//		//change the localWorldFrame to groun plane frame
//		trans.transformBodyFrame(localWorldFrame, transFrame);
//	}
	
	/**
	 * transform data points from body to local world frame using transFrame
	 * and set body to local world frame
	 * local world frame has same rotation world frame(NED) and same position to body frame  
	 * @param trans
	 */
	public void transfromToLocalWorldFrame(FrameTransformer trans){
		if(this.localWorldFrame!=null){
			throw new RuntimeErrorException(new Error("lidarFrame already has local world frame, this function should only be called to lidarFrame from raw data"));
		}
		//change new body frame to local world frame
		this.localWorldFrame = this.bodyFrame.cloneFrame();
		this.localWorldFrame.copyRotation(trans.getWorldFrame().getRotation());
		//change point from body to local world frame(new body frame)
		dataPoints3D = trans.transform4D(this.bodyFrame, this.localWorldFrame, dataPoints);
		for(int i=0; i<dataPoints3D.length; i++){
			this.dataPoints[i*3] = (float)this.dataPoints3D[i].x;
			this.dataPoints[i*3+1] = (float)this.dataPoints3D[i].y;
			this.dataPoints[i*3+2] = (float)this.dataPoints3D[i].z;
		}
	}
	/**
	 * from localWorldFrame to world frame
	 * @param trans
	 * @return
	 */
	public Point3D[] transformToWorld(FrameTransformer trans){
		return trans.transform4D(localWorldFrame, null, this.dataPoints);
		//return trans.transform4D(null, null, this.dataPoints);
	}

	
	public FloatBuffer getDataBuffer(){
		return Buffers.newDirectFloatBuffer(dataPoints);
	}
	
	public float[] getDataArray(){
		return this.dataPoints;
	}
	
	public Point3D[] getDataPoints(boolean needMaxRange){
		//if (dataPoints3D!=null) return dataPoints3D;
		ArrayList<Point3D> points = new ArrayList<Point3D>();
		for(int idx=0; idx<this.pointNum; idx++){
			//System.out.printf("range %f", calcDist(dataPoints[idx*3], dataPoints[idx*3+1], dataPoints[idx*3+2]));
			if(!needMaxRange && calcDist(dataPoints[idx*3], dataPoints[idx*3+1], dataPoints[idx*3+2])>=MAX_RANGE){
				continue;
			}
			points.add(new Point3D(dataPoints[idx*3], dataPoints[idx*3+1], dataPoints[idx*3+2]));
		}
		
		return points.toArray(new Point3D[points.size()]);
	}
	
	public Point3D getDataPoint(int idx){
		return new Point3D(dataPoints[idx*3], dataPoints[idx*3+1], dataPoints[idx*3+2]);
	}
	
	public float getIntensity(int idx){
		return this.intensityArray[idx];
	}
	
	public int getPointNum(){
		return this.pointNum;
	}
	
	public int getDataNum(){
		return this.pointNum*3;
	}
	
	public CoordinateFrame getLocalWorldFrame(){
		return this.localWorldFrame;
	}
	
	public BodyFrame getBodyFrame(){
		return this.bodyFrame;
	}
	
	public float getTime(){
		return this.time;
	}
	
	private double calcDist(float x, float y, float z){
		return Math.sqrt((x*x+y*y+z*z));
	}
}