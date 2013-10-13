package prepocess;

import java.nio.FloatBuffer;
import java.util.ArrayList;

import com.jogamp.common.nio.Buffers;

import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;
import gridmap_generic.HeightCell;
import gridmap_generic.QuadGridmap;
import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

/**
 * filter out the virtual table points based on HeightGridMap
 * @author qichi
 *
 */
public class HeightMapFilter extends BasicFilter{
	//private QuadGridmap<HeightCell> quadMap;
	private FrameTransformer trans;
	
	Point3D[] groundPoints;
	int validPointNum;
	
	public HeightMapFilter(VirtualTable vt, FrameTransformer trans) {
		super(vt);
		this.trans = trans;
	}
	
	public void filter(VirtualTable vt, QuadGridmap<HeightCell> quadMap, double minHeight, double maxHeight, CoordinateFrame localWorldFrame, boolean saveGound) throws Exception{
		ArrayList<Point3D> ground = new ArrayList<Point3D>();
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				Point3D p = vt.getPoint3D(i, j);
				double dist = vt.getDistance(i, j);
				if(p==null || dist>=LidarFrame.MAX_RANGE){//null point and max range point are not used in this map
					mask[i][j]=false;
				}else{
					Point3D w_p = trans.transform4D(localWorldFrame, null, p);
					double groundHeight = quadMap.getCell(w_p).getValue();
					
					if(saveGound){
						ground.add(new Point3D(w_p.x, w_p.y, groundHeight));
					}
					//don't forget north east DOWN!!! lower height has larger numeric value
					if(groundHeight-w_p.z >= minHeight && groundHeight-w_p.z <= maxHeight){
						mask[i][j]=true;
					}else{
						mask[i][j]=false;
					}
				}
			}
		}
		this.groundPoints = new Point3D[ground.size()];
		for(int i=0; i<ground.size(); i++){
			Point3D p = trans.transform4D(null, localWorldFrame, ground.get(i));
			this.groundPoints[i] = p;
		}
	}
	
	public FloatBuffer getGroundDataBuffer(){
		this.validPointNum=0;
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.groundPoints.length*3);
		
		for(Point3D p: this.groundPoints){
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
