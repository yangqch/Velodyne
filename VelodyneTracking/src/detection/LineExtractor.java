package detection;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import javax.management.RuntimeErrorException;

import velodyne2d.Line;
import velodyne2d.Point2D;
import velodyne2d.Segment;
import velodyne2d.Vector;

public class LineExtractor {
	static double MaxDistToBreak=0.3;
	static int maxBreakPoints=5;
	static int minNumOfPoints=5;
	//
	private Line[] lines;
	private Point2D[] breakPoints;
	private ArrayList<Integer> breakPointIdx; 
	//
	private Segment segment;
	private boolean isDebug;
	
	public LineExtractor(Segment seg) {
		this.breakPointIdx = new ArrayList<Integer>();
		this.segment = seg;
		seg.setLineExtractor(this);
	}
	
	public Point2D[] getBreakPoints(){
		return breakPoints;
	}
	
	public Line[] getLines(){
		return lines;
	}
	/**
	 * extract lines from points
	 * if non-convex shape, return false, lines and breakPoints will be empty
	 * @param points
	 * @return
	 */
	public boolean extractLines(){
		Point2D[] points = this.segment.getPoints();
		//find break points
		this.findBreakPoints(points, this.breakPointIdx);
		breakPoints = new Point2D[breakPointIdx.size()];//create it even no breakPointIdx
		lines = breakPointIdx.size()==0 ? new Line[0] : new Line[breakPointIdx.size()-1];
		
		if(breakPointIdx.size() == 0) return false;
		//make lines based on break points
		int idx1 = breakPointIdx.get(0);
		int cnt=0;
		for(int i=1; i<breakPointIdx.size(); i++){
			int idx2=breakPointIdx.get(i);
			lines[cnt]=new Line(points[idx1], points[idx2]);
			breakPoints[cnt] = points[idx1];
			idx1=idx2;
			cnt++;
		}
		breakPoints[cnt] = points[idx1];
		return true;
	}
	
	int getBreakPointIdx(int i){
		return breakPointIdx.get(i);
	}
	
	/**
	 * find break points, if non-convex found, clear breakPoints
	 * @param points
	 * @return
	 */
	private ArrayList<Integer> findBreakPoints(Point2D[] points, ArrayList<Integer> breakPointIdx){
		breakPointIdx.clear();
		breakPointIdx.add(0);
		boolean isValidShape = this.breakPointDFS(points, 0, points.length-1, breakPointIdx);
		if(!isValidShape){
			breakPointIdx.clear();
		}else{
			breakPointIdx.add(points.length-1);
			Collections.sort(breakPointIdx);	
		}
		return breakPointIdx;
	}
	
	private boolean breakPointDFS(Point2D[] points, int start, int end, ArrayList<Integer> breakPointIdx){
		//System.out.printf("break point dfs %d to %d\n", start, end);
		if(end-start<minNumOfPoints) return true;//only process points number larger than 5
		
		Vector norm = Vector.makeNormalVector(points[start], points[end]);
		double max=Double.MIN_VALUE;
		int maxIdx=-1;
		for(int i=start+1; i<end; i++){
			Vector vec = new Vector(points[start], points[i]);
			double proj = norm.dot(vec);
			//if(dist<0) throw new RuntimeErrorException(new Error("breakPointDFS: find distance<0 for point to line of two end point"));
			if(proj<0)
				return false;
			
			double dist = Math.sqrt( vec.x*vec.x + vec.y*vec.y - proj*proj );
			//System.out.printf("point %d, proj %f, distance %f\n", i, proj, dist);
			if(dist>max){
				max=dist; maxIdx=i;
			}
		}
		//System.out.printf("find max idx %d, max distance %f, p1 %s, p2 %s, p %s\norm vector %s", maxIdx, max, points[start], points[end], points[maxIdx], norm);
		if(max>MaxDistToBreak){//break
			//System.out.printf("find break point idx %d, max distance %f", maxIdx, max);
			breakPointIdx.add(maxIdx);
			if(!breakPointDFS(points, maxIdx, end, breakPointIdx)) return false;
			if(!breakPointDFS(points, start, maxIdx, breakPointIdx)) return false;
		}
		
		return true;
	}
}
