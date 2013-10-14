package detection;

import java.util.ArrayList;

import VelodyneDataIO.VirtualScan2D;
import velodyne2d.CoordinateFrame2D;
import velodyne2d.FrameTransformer2D;
import velodyne2d.Line;
import velodyne2d.Point2D;
import velodyne2d.Polar;
import velodyne2d.Vector;
import velodyne2d.VirtualScan;

public class VehicleModel {
	public static double default_width = 2;//default vehicle height
	public static double default_length = 5;//default vehicle height
	public static double default_height = 2;//default vehicle height
	public static double default_speed = 2;//default vehicle height
	//all in world frame
	public Point2D center; //vehicle position in world frame
	public double yaw;//radius unit, from world_x to body_x clock-wise 
	public Vector direction;//the vector along direction of vehicle, always point to front
	public double speed;
	Shape shape;
	//vehicle frame, w_P_wv, w_R_v w.r.t world frame
	CoordinateFrame2D bodyFrame;
	
	//corners and arrows for visualization, in local framework
	Point2D[] corners;// fl, fr, bl, br;
	Point2D[] arrows;//bl and br -> fm(mid of fl and fr)
	Edge[] edges;//front, right, back, left
	
	//predicted sensor measurement
	static double rotation_thres = 180; 
	boolean[] detectedCorner;//indicate if the corner was detected
	
	/**
	 * center and direction are all w.r.t world frame
	 * @param center
	 * @param direction
	 * @param width
	 * @param length
	 * @param speed
	 */
	public VehicleModel(Point2D center, Vector direction, double width, double length, double speed) {
		this.center = center;
		this.direction = direction;
		this.shape = new Shape(width, length, null);
		this.speed = speed;
		this.makeModel();
	}
	
	private void makeModel(){
		Point2D fl=new Point2D(shape.length/2, -shape.width/2);
		Point2D fr=new Point2D(shape.length/2, shape.width/2);
		Point2D bl=new Point2D(-shape.length/2, -shape.width/2);
		Point2D br=new Point2D(-shape.length/2, shape.width/2);
		Point2D fm = new Point2D(shape.length/2, 0);
		corners = new Point2D[] {fl, fr, br, bl};
		arrows = new Point2D[] {br, bl, fm};
		edges = new Edge[] {new Edge(new Line(fl, fr)), new Edge(new Line(fr, br)), new Edge(new Line(bl, br)), new Edge(new Line(fl, bl))};
		//w.r.t world frame(NED), bodyFrame is front-right-down, direction is front, bodyFrame is w_R_v
		bodyFrame = new CoordinateFrame2D(new double[] {center.x, center.y, direction.x, -direction.y, direction.y, direction.x});
		//shape parameters, estimated by KF
	}
	/**
	 * push the vehicle to next state
	 * given the angle changed in initial state, acceleration added to speed and angle changed on final state 
	 * @param initAngle
	 * @param acc
	 * @param finalAngle
	 */
	public void predict(double initAngle, double acc, double finalAngle){
		double x = direction.x;
		double y = direction.y;
		direction.x = x * Math.cos(initAngle) - y * Math.sin(initAngle);
		direction.y = x * Math.sin(initAngle) + y * Math.cos(initAngle);
		speed += acc;
		center.x = center.x + direction.x * speed;
		center.y = center.y + direction.y * speed;
		x = direction.x;
		y = direction.y;
		direction.x = x * Math.cos(finalAngle) - y * Math.sin(finalAngle);
		direction.y = x * Math.sin(finalAngle) + y * Math.cos(finalAngle);
		makeModel();
	}
	
	public VehicleModel reproduce(){
		Point2D p = new Point2D(center.x + direction.x * speed, center.y + direction.y * speed);
		return new VehicleModel(p, direction, shape.width, shape.length, speed);
	}
	
	public VehicleModel copy(){
		return new VehicleModel(new Point2D(center.x, center.y), new Vector(direction.x, direction.y), shape.width, shape.length, speed);
	}
	
	/**
	 * calculate score for the predicted measuremnets in the given scan 
	 */
	public void calculateScore(VirtualScan scan, FrameTransformer2D trans){
		for(Edge e : edges){
			if(e.isDet()){
				for(int i=0; i<e.meas.size(); i++){
					RayMeas m = e.meas.get(i);
					double real = scan.getDistance(m.idx);
					m.diff = (m.distance - real);
				}
			}
		}
	}
	
	/**
	 * predict the measurements of this VehicleModel in the given scan
	 * @param scan
	 * @param trans
	 */
	public void predictMeasurement(VirtualScan scan, FrameTransformer2D trans){
		this.predictDetectCorner(scan, trans);
		//front edge
		if(this.detectedCorner[0] && this.detectedCorner[1]){
			this.generateMeasurements(edges[0], scan, trans);
			//System.out.printf("generate %d points on edge 0\n", edges[0].meas.size());
		}else{
			edges[0].clear();
		}
		//right edge
		if(this.detectedCorner[1] && this.detectedCorner[2]){
			this.generateMeasurements(edges[1], scan, trans);
			//System.out.printf("generate %d points on edge 1\n", edges[0].meas.size());
		}else{
			edges[1].clear();
		}
		//back edge
		if(this.detectedCorner[2] && this.detectedCorner[3]){
			this.generateMeasurements(edges[2], scan, trans);
			//System.out.printf("generate %d points on edge 2\n", edges[0].meas.size());
		}else{
			edges[2].clear();
		}
		//right edge
		if(this.detectedCorner[0] && this.detectedCorner[3]){
			this.generateMeasurements(edges[3], scan, trans);
			//System.out.printf("generate %d points on edge 3\n", edges[0].meas.size());
		}else{
			edges[3].clear();
		}
		
		//for debug
		//calculateScore(scan, trans);
	}
	
	/**
	 * generate a RayMeas array measurements on given edge(Line)
	 * w.r.t scan's localWorldFrame
	 * the RayMeas list is modified inside the edge object, including idx of ray in scan and distance
	 * @param edge
	 * @param scan
	 * @param trans
	 */
	private void generateMeasurements(Edge edge, VirtualScan scan, FrameTransformer2D trans){
		edge.clear();
		Line line = trans.transform(bodyFrame, scan.getLocalWorldFrame(), edge.line);
		//find start and end rays
		Polar start = scan.XY2polar(line.p1);
		Polar end = scan.XY2polar(line.p2);
		double minAngle = start.angle <= end.angle ? start.angle : end.angle;
		double maxAngle = start.angle > end.angle ? start.angle : end.angle;
		//swap min and max if rotation happens on -180 and +180
		if(maxAngle - minAngle > rotation_thres){
			double tmp = minAngle;
			minAngle = maxAngle;
			maxAngle = tmp+360;
		}
		int minIdx = scan.angle2Index(minAngle);
		int maxIdx = scan.angle2Index(maxAngle);
		
		//System.out.printf("angle (%f, %f), index (%d, %d)\n", minAngle, maxAngle, minIdx, maxIdx);
		
		int mod = scan.getColNum();
		int tmp = minIdx%mod;
		for(int i=minIdx; (i%mod)!=maxIdx; i++){
			//edge.meas.add(new RayMeas(i, scan.getDistance(i%mod)));
			//edge.measPoints.add(scan.getPoint2D(i%mod));
			//find intersection of ray and edge
			Vector v = new Vector(scan.index2Angle(i) * VirtualScan.DEG2RAD);
			Point2D p = this.findIntersection(trans.transform(bodyFrame, scan.getLocalWorldFrame(), edge.line), v);
			if(p==null) continue;
			Polar polar = scan.XY2polar(p);
			edge.meas.add(new RayMeas(scan.angle2Index(polar.angle), polar.dist));
			edge.measPoints.add(p);
		}
	}
	/**
	 * determine which corners will be detected in the given scan
	 * so that the detected edges can be determined 
	 * this.detetctedCorner will be modified
	 * @param scan
	 * @param trans
	 */
	private void predictDetectCorner(VirtualScan scan, FrameTransformer2D trans){
		detectedCorner = new boolean[this.corners.length];
		double[] angles = new double[this.corners.length];
		Point2D[] cornersInNew = trans.transform(bodyFrame, scan.getLocalWorldFrame(), corners);
		int closest=-1; double minDist=Double.MAX_VALUE;
		for(int i=0; i<cornersInNew.length; i++){
			Polar polar = scan.XY2polar(cornersInNew[i]);
			angles[i] = polar.angle;
			//for closest point
			if(polar.dist<minDist){
				minDist = polar.dist;
				closest = i;
			}
		}
		detectedCorner[closest] = true;
		//find 2 bounded angles
		int minIdx=-1, maxIdx=-1;
		double minAngle = Double.MAX_VALUE, maxAngle = -Double.MAX_VALUE;
		for(int i=0; i<angles.length; i++){
			if(angles[i]<minAngle){
				minAngle = angles[i]; minIdx = i;
			}
			if(angles[i]>maxAngle){
				maxAngle = angles[i]; maxIdx = i;
			}
		}
		if(maxAngle - minAngle <= rotation_thres){
			detectedCorner[minIdx] = true;
			detectedCorner[maxIdx] = true;
		}else{//if rotation has been found, like -160, -170, 170, 160
			minAngle = Double.MAX_VALUE; maxAngle = -Double.MAX_VALUE;
			for(int i=0; i<angles.length; i++){
				if(angles[i]<0) angles[i] += 360;
				if(angles[i]<minAngle){
					minAngle = angles[i]; minIdx = i;
				}
				if(angles[i]>maxAngle){
					maxAngle = angles[i]; maxIdx = i;
				}
			}
			detectedCorner[minIdx] = true;
			detectedCorner[maxIdx] = true;
		}
	}
	
	/**
	 * calculate the intersection of line and a ray from origin 0 along vector v(normalized)
	 * @param line
	 * @param w
	 * @return
	 */
	public Point2D findIntersection(Line line, Vector v){
		double v1 = v.y * line.p1.x - v.x * line.p1.y;
		Vector u = new Vector(line.p1, line.p2);
		u.normalize();
		double v2 = v.x * u.y - v.y * u.x;
		if(v2==0) return null;//parellel
		return new Point2D(line.p1.x + u.x * v1/v2, line.p1.y + u.y * v1/v2);
	}
	
	public CoordinateFrame2D getBodyFrame(){
		return bodyFrame;
	}
	
	public Point2D[] getCornerPoints(CoordinateFrame2D newFrame, FrameTransformer2D trans){
		return trans.transform(bodyFrame, newFrame, this.corners);
	}
	
	public Point2D[] getArrowPoints(CoordinateFrame2D newFrame, FrameTransformer2D trans){
		return trans.transform(bodyFrame, newFrame, this.arrows);
	}
	
	public Point2D[] getDetCorners(CoordinateFrame2D newFrame, FrameTransformer2D trans){
		Point2D[] newp = trans.transform(bodyFrame, newFrame, this.corners);
		ArrayList<Point2D> pts = new ArrayList<Point2D>();
		for(int i=0; i<newp.length; i++){
			if(this.detectedCorner[i]){
				pts.add(newp[i]);
			}
		}
		return pts.toArray(new Point2D[pts.size()]);
	}
	
	public Point2D[] getMeasurements(){
		ArrayList<Point2D> pts = new ArrayList<Point2D>();
		for(Edge edge : edges){
			pts.addAll(edge.measPoints);
		}
		return pts.toArray(new Point2D[pts.size()]);
	}
	
	public ArrayList<RayMeas> getRayMeas(){
		ArrayList<RayMeas> rayMeas = new ArrayList<RayMeas>();
		for(Edge edge : edges){
			rayMeas.addAll(edge.meas);
		}
		return rayMeas;
	}
	
	public String toString(){
		return String.format("center %s, direction %s, speed %s\n", center, direction, speed);
	}
}

class Edge{
	Line line;
	ArrayList<RayMeas> meas;
	ArrayList<Point2D> measPoints;
	public Edge(Line l) {
		this.line = l;
		meas = new ArrayList<RayMeas>();
		measPoints = new ArrayList<Point2D>();
	}
	public void clear(){
		meas.clear();
		measPoints.clear();
	}
	public boolean isDet(){
		return meas.size()!=0;
	}
}

class Shape{
	double width, length;
	double w_std, l_std;
	Point2D anchor;
	
	public Shape(double w, double l, Point2D a) {
		this.width=w;
		this.length=l;
		this.w_std=1;
		this.l_std=1;
		this.anchor = a;
	}
}