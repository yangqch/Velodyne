package velodyne2d;

import java.util.ArrayList;

import detection.LineExtractor;
import detection.Motion;

public class Segment {
	static int MaxColIdxDiff=100;
	//start and end ray idx in virtual scan
	private CoordinateFrame2D mFrame;
	private int startIdx;
	private int lastColIdx;
	private ArrayList<Point2D> pointList;//contour points 
	private VirtualScan scan;
	private int id;
	private Point2D[] points;
	private Point2D center;
	
	//functional helper
	private Motion motion;
	private LineExtractor lineExt;
	
	/**
	 * create segment with scan, segmentId and localWorldFrame
	 * startIdx is an indicator for the start point in case the segment cover the boundary of polar coordinate system(-180~180)
	 * @param s
	 * @param segId
	 * @param frame
	 */
	public Segment(VirtualScan s, int segId, CoordinateFrame2D frame) {
		this.scan = s;
		this.id = segId;
		pointList = new ArrayList<Point2D>();
		startIdx = -1;
		lastColIdx = -1;
		
		this.mFrame = frame;
	}
	/**
	 * add xy distance to the segment, segment only pick the closest distance in one colIdx
	 * if the colIdx cross a wide range, it implies the segment cover the boundary angles(-180~180), change startIdx
	 * @param colIdx
	 * @param XYDist
	 */
	public void put(int colIdx, double XYDist){
		Point2D p = scan.polar2XY(new Polar(scan.index2Angle(colIdx), XYDist));
		this.pointList.add(p);
		if(startIdx==-1){
			startIdx = 0;
		}else{
			if(colIdx - lastColIdx > MaxColIdxDiff){//rotation at the end of virtual table
				if(startIdx!=0){
					throw new RuntimeException(String.format("more than one rotation found in a segment %d->%d", lastColIdx, colIdx));
				}
				startIdx = this.pointList.size()-1;
			}
		}
		lastColIdx = colIdx;
	}
	
	public void update(){
		rotate();//change the point order
		center = new Point2D(0, 0);
		for(Point2D p : this.points){
			center.x += p.x;
			center.y += p.y;
		}
		center.x/=this.points.length;
		center.y/=this.points.length;
	}
	
	/**
	 * create new point array and motion array
	 * rotate the point so that startIdx=0 (fix the segment crossing -x degree and x degree)
	 */
	public void rotate(){
		int num = pointList.size();
		this.points = new Point2D[num];
		for(int i=0; i<num; i++){
			this.points[i] = this.pointList.get((startIdx+i)%num);
		}
		startIdx=0;
	}
	
	public Point2D[] getPoints(){
		if(this.points==null){//if not rotated, move points from List to array with adjusted order
			this.rotate();
		}
		return this.points;
	}
	
	public int getNumOfPts(){
		return pointList.size();
	}
	
	public CoordinateFrame2D getFrame(){
		return mFrame;
	}
	
	public void setMotion(Motion m){
		this.motion = m;
	}
	
	public Motion getMotion(){
		return this.motion;
	}
	
	public void setLineExtractor(LineExtractor lineExt){
		this.lineExt = lineExt;
	}
	
	public LineExtractor getLineExtractor(){
		return this.lineExt;
	}
	
	public int getId(){
		return id;
	}
	
	public Point2D getCenter(){
		return center;
	}
}
