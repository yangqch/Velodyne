package velodyne2d;

import java.util.ArrayList;

/**
 * a 2d virtual scan, always initialized from a 3D VirtualTable in factory
 * @author qichi
 *
 */
public class VirtualScan {
	public static double DEG2RAD=Math.PI/180.0;
	public static double RAD2DEG=180.0/Math.PI;
	
	double minRot, maxRot;
	double rotRes;
	int numOfRays;

	double[] reflections;
	Point2D[] points;
	
	CoordinateFrame2D localWorldFrame;
	
	public VirtualScan(double minRot, double maxRot, int numOfRays, CoordinateFrame2D frame) {
		this.rotRes = (maxRot-minRot)/numOfRays;
		this.minRot = minRot;
		this.maxRot = maxRot;
		this.numOfRays = numOfRays;
		this.reflections = new double[numOfRays];
		this.points = new Point2D[numOfRays];
		this.localWorldFrame = frame;
	}
	
	public void put(int idx, double dist){
		dist = dist<0 ? 0 : dist;
		this.reflections[idx]= dist;
		this.points[idx]= this.polar2XY(new Polar(this.index2Angle(idx), dist));
		//System.out.printf("index %d, angle %.1f, dist %.1f, (%.1f, %.1f)\n", idx, this.index2Angle(idx), dist, this.points[idx].x, this.points[idx].y);
	}
	
	public int getColNum(){
		return points.length;
	}
	
	public double getDistance(int idx){
		return this.reflections[idx];
	}
	
	public Point2D polar2XY(Polar po){
		 return new Point2D(Math.cos(po.getRadius())*po.dist, Math.sin(po.getRadius())*po.dist);
	}
	
	public Polar XY2polar(Point2D p){
		return new Polar(Math.atan2(p.y, p.x)*RAD2DEG, Math.sqrt(p.x*p.x + p.y*p.y));
	}
	
	/**
	 * 
	 * @param idx
	 * @return angle in degree
	 */
	public double index2Angle(int idx){
		return minRot + idx*rotRes + rotRes/2;
	}
	
	/**
	 * 
	 * @param angle, in degree
	 * @return
	 */
	public int angle2Index(double angle){
		return (int)Math.floor( (angle - minRot)%360 / rotRes);
	}
	
	public CoordinateFrame2D getLocalWorldFrame(){
		return this.localWorldFrame;
	}
	
	public Point2D getPoint2D(int idx){
		return points[idx];
	}
	
	public Point2D[] getPoints2D(boolean[] mask){
		ArrayList<Point2D> points= new ArrayList<Point2D>();
		for(int i=0; i<numOfRays; i++){
			if(this.points[i]==null){
				continue;
			}
			if(mask==null || mask[i]){
				points.add(this.points[i]);
			}
		}
		return points.toArray(new Point2D[points.size()]);
	}
	
	
}


