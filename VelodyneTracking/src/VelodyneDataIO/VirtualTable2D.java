package VelodyneDataIO;

import calibration.CoordinateFrame;

public class VirtualTable2D {
	static final double RAD2DEG=180/Math.PI;
	static final double DEG2RAD=Math.PI/180;
	
	private float minRot=-180;
	private float maxRot=180;
	private float resolution=0.5f;
	
	private double[] distVector;
	private Point3D[] points;
	private int numOfPoints;
	
	private VirtualTable parentVTable;//parent virtual table
	
	public VirtualTable2D(VirtualTable parent) {
		this.parentVTable = parent;
		this.numOfPoints = this.parentVTable.getColNum();
		this.distVector = new double[this.numOfPoints];
		points = new Point3D[this.numOfPoints];
		
		this.convertFromVTable(this.parentVTable);
	}
	
	public void convertFromVTable(VirtualTable vt){
		for(int j=0; j<this.numOfPoints; j++){
			double minDist = Double.MAX_VALUE;
			Point3D nearest = null;
			for(int i=0; i<vt.getRowNum(); i++){
				Point3D p = vt.getPoint3D(i, j);
				double dist = Math.sqrt(p.x*p.x + p.y*p.y);
				if(dist <minDist){
					nearest = p;
					minDist = dist;
				}
			}
			this.distVector[j]=minDist;
			this.points[j]=nearest;
		}
	}
	
	private int deg2Idx(float deg){
		return (int)Math.floor((deg-minRot)/resolution);
	}
	
	private float idx2Deg(int idx){
		return (float)((idx+0.5)*resolution+minRot);
	}
	
	public Point3D[] getPoints(){
		return points;
	}
}
