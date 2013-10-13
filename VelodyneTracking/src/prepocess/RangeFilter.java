package prepocess;

import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

public class RangeFilter extends BasicFilter{
	
	public RangeFilter(VirtualTable vt) {
		super(vt);
	}
	
	public void filterDistance(VirtualTable vt, double distMin, double distMax){
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				double dist=vt.getDistance(i, j);
				if(dist==0||dist==-1||dist<distMin || dist>distMax){
					mask[i][j]=false;
				}
				else{
					mask[i][j]=true;
				}
			}
		}
	}
	
	public void filterX(VirtualTable vt, double xMin, double xMax){
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				Point3D p3 = vt.getPoint3D(i, j);
				if(p3==null || p3.x<xMin || p3.x>xMax){
					mask[i][j]=false;
				}else{
					mask[i][j]=true;
				}
			}
		}
	}
	
	public void filterY(VirtualTable vt, double yMin, double yMax){
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				Point3D p3 = vt.getPoint3D(i, j);
				if(p3==null || p3.y<yMin || p3.y>yMax){
					mask[i][j]=false;
				}else{
					mask[i][j]=true;
				}
			}
		}
	}
	
	public void filterZ(VirtualTable vt, double zMin, double zMax){
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				Point3D p3 = vt.getPoint3D(i, j);
				if(p3==null || p3.z<zMin || p3.z>zMax){
					mask[i][j]=false;
				}else{
					mask[i][j]=true;
				}
			}
		}
	}
}
