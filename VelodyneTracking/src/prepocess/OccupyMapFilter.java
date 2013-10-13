package prepocess;


import java.nio.FloatBuffer;
import java.util.ArrayList;

import com.jogamp.common.nio.Buffers;

import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;
import gridmap_generic.HeightCell;
import gridmap_generic.OccupyCell;
import gridmap_generic.QuadGridmap;
import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

/**
 * filter out the virtual table points based on HeightGridMap
 * @author qichi
 *
 */
public class OccupyMapFilter extends BasicFilter{
	//private QuadGridmap<OccupyCell> quadMap;
	private FrameTransformer trans;
	ArrayList<Point3D> occupied; 
	private int validPointNum;

	public OccupyMapFilter(VirtualTable vt, FrameTransformer trans) {
		super(vt);
		this.trans = trans;
		occupied = new ArrayList<Point3D>();
	}
	/**
	 * set this.mask to be true if cell(i,j) is free (motion) 
	 * @param vt
	 * @param body
	 * @param occupyThres
	 * @param mask: if mask[i][j]=false, set this.mask[i][j]=false
	 */
	public void filter(VirtualTable vt, QuadGridmap<OccupyCell> quadMap, CoordinateFrame localWorFrame, double occupyThres, boolean[][] mask) throws Exception{
		occupied.clear();
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				if(mask!=null && !mask[i][j]){
					this.mask[i][j]=false;
					continue;
				}
				Point3D p = vt.getPoint3D(i, j);
				double dist = vt.getDistance(i, j);
				if(p==null || dist>=LidarFrame.MAX_RANGE){//null point and max range point will not be used
					this.mask[i][j]=false;
				}else{
					Point3D w_p = trans.transform4D(localWorFrame, null, p);
					double isOccupy = quadMap.getCell(w_p).getValue();
					//System.out.printf("occupyProb %.2f\n", isOccupy);
					if(isOccupy > occupyThres){
						this.mask[i][j]=false;//occupied
					}else{
						this.mask[i][j]=true;//motion, set to true
						occupied.add(p);
					}
				}
			}
		}
	}
	
	public FloatBuffer getGroundDataBuffer(){
		this.validPointNum=0;
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.occupied.size()*3);
		for(int i=0; i<this.occupied.size(); i++){
			Point3D p = this.occupied.get(i);
			fb.put((float)p.x);fb.put((float)p.y);fb.put((float)p.z);
			this.validPointNum++;
		}

		fb.rewind();
		return fb;
	}
	
	
	public int getDataNum(){
		return this.validPointNum;
	}
}
