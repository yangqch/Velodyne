package velodyne2d;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map.Entry;

import prepocess.ConnCompFilter;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;
import calibration.BodyFrame;
import calibration.CoordinateFrame;

public class VirtualScanFactory {
	
	private HashMap<Integer, Segment> curSegmentMap;
	private HashMap<Integer, Segment> nextSegmentMap;
	private VirtualScan curScan;
	private VirtualScan prevScan;
	private VirtualScan nextScan;
	
	public VirtualScanFactory() {
		this.curSegmentMap = new HashMap<Integer, Segment>();
		this.nextSegmentMap = new HashMap<Integer, Segment>();
	}
	/**
	 * generate new scan, save prevScan, scan and nextScan
	 * only return the curScan
	 * @param vt
	 * @param localWorldFrame3D
	 * @param bodyFrame3D
	 * @param smallCompFilter: filter the noise in scan
	 * @param compFilter: make mask for segmentation
	 * @return
	 */
	public VirtualScan convert3Dto2D(VirtualTable vt, CoordinateFrame localWorldFrame3D, CoordinateFrame bodyFrame3D, ConnCompFilter smallCompFilter, ConnCompFilter compFilter){
		this.prevScan = this.curScan;
		this.curScan = this.nextScan; 
		this.nextScan = new VirtualScan(vt.getMinRot(), vt.getMaxRot(), vt.getColNum(), CoordinateFrame2D.fromCoordinateFrame3D(localWorldFrame3D, false), CoordinateFrame2D.fromCoordinateFrame3D(bodyFrame3D, true));
		
		boolean[][] mask = smallCompFilter.getCompMask();
		//put data in vt to scan with comp mask
		for(int i=0; i<vt.getColNum(); i++){
			this.nextScan.put(i, vt.getMinDistInCol(i, mask, true));
		}
		//make segments from vt and compFilter
		makeSegments(vt, compFilter);
		
		return this.curScan;
	}
	
	/**
	 * make segments table, so that each segment has its contour points
	 * nextSegement will be assigned to curSegment
	 * new segments in nextScan will be generated
	 * @param vt
	 * @param compFilter
	 */
	private void makeSegments(VirtualTable vt, ConnCompFilter compFilter){
		this.curSegmentMap = this.nextSegmentMap;
		this.nextSegmentMap = new HashMap<Integer, Segment>();
		
		int[][] labels = compFilter.getLables();
		int rows = vt.getRowNum();
		int cols = vt.getColNum();
		Point3D origin = new Point3D(0, 0, 0);
		for(int j=0;j<cols;j++){
			HashMap<Integer, Double> compDistMap = new HashMap<Integer, Double>();
			for(int i=0;i<rows;i++){
				Point3D p = vt.getPoint3D(i, j);
				int label = labels[i][j];
				if(compFilter.isValidComp(label)){
					double XYDist = p.calcXYDist(origin);
					if(!compDistMap.containsKey(label) || compDistMap.get(label)>XYDist){
						compDistMap.put(label, XYDist);
					}
				}
			}
			//in compDistMap are the closest points for each component
			for(Entry<Integer, Double> entry: compDistMap.entrySet()){
				int segId = entry.getKey();
				if(!this.nextSegmentMap.containsKey(segId)){
					this.nextSegmentMap.put(segId, new Segment(this.nextScan, segId, this.nextScan.getLocalWorldFrame()));
				}
				this.nextSegmentMap.get(segId).put(j, entry.getValue());
			}
		}
		for(Segment seg : this.nextSegmentMap.values()){
			seg.update();
		}
	}
	
	public VirtualScan getScan(){
		return curScan;
	}
	
	public VirtualScan getPrevScan(){
		return prevScan;
	}
	
	public VirtualScan getNextScan(){
		return nextScan;
	}
	
	public Collection<Segment> getSegments(){
		return curSegmentMap.values();
	}
	
	public boolean hasAllFrames(){
		return prevScan!=null && curScan!=null && nextScan!=null;
	}
}
