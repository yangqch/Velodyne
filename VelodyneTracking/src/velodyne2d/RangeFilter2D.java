package velodyne2d;

import calibration.BodyFrame;

/**
 * make mask for 2D scan based on XY range w.r.t body_frame of ego vehicle 
 * @author qichi
 *
 */
public class RangeFilter2D {
	boolean[] mask;//true-valid, false-invalid
	
	/**
	 * 
	 * @param size : size of scan
	 */
	public RangeFilter2D(int size) {
		mask = new boolean[size];
	}
	
	public void makeRangeMask(VirtualScan scan, FrameTransformer2D trans, double xmin, double xmax, double ymin, double ymax){
		Point2D[] points = scan.getPoints2D(null);
		if(mask.length!=points.length){
			throw new RuntimeException(String.format("RangeFilter2D has mask size %d, but scan size %d", mask.length, points.length));
		}
		for(int i=0; i<mask.length; i++){
			if(points[i]==null){
				mask[i]=false;
				continue;
			}
			Point2D p = trans.transform(scan.localWorldFrame, scan.bodyFrame, points[i]);
			if(p.x<=xmax && p.x>=xmin && p.y<=ymax && p.y>=ymin){
				mask[i]=true;
			}else{
				mask[i]=false;
			}
		}
	}
	
	public boolean[] getMask(){
		return mask;
	}
}
