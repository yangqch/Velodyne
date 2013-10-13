package gridmap_generic;

import javax.management.RuntimeErrorException;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.Point3D;

public class QuadGridmap<T extends Cell> {
	double org_x, org_y;//origin left-upper corner, in meters 
	double size_x, size_y;//size of matrix in meters 
	int rows, cols;//size in cells
	double res_x, res_y;
	
	private Gridmap<T>[] gridmaps = new Gridmap[4];//upLeft, upRight, downLeft, downRight
	
	public QuadGridmap(Gridmap<T> upLeft, Gridmap<T> upRight, Gridmap<T> downLeft, Gridmap<T> downRight) {
		gridmaps[0] = upLeft;
		gridmaps[1] = upRight;
		gridmaps[2] = downLeft;
		gridmaps[3] = downRight;

		if(upLeft!=null){
			org_x = upLeft.org_x;
			org_y = upLeft.org_y;
		}else if(downRight!=null){
			org_x = downRight.org_x - downRight.attr.size_x;
			org_y = downRight.org_y + downRight.attr.size_y;
		}else{
			throw new RuntimeErrorException(new Error("QuadGridmap has neight upLeft nor downRight"));
		}
		
		for(int i=0; i<4; i++){
			if(gridmaps[i]!=null){
				size_x = gridmaps[i].attr.size_x*2;
				size_y = gridmaps[i].attr.size_y*2;
				rows = gridmaps[i].attr.rows*2;
				cols = gridmaps[i].attr.cols*2;
				res_x = gridmaps[i].attr.res_x;
				res_y = gridmaps[i].attr.res_y;
				break;
			}
		}
	}
	/**
	 * update the cell occupancy score using the observations
	 * @param pos: position of lidar
	 * @param observations: point cloud 
	 */
	public void updateCells(Point3D pos, Point3D[] observations){
		
	}
	/**
	 * vehicle position to index
	 * @param x-north, column
	 * @param y-east, row
	 * @return index in QuadGridmap
	 */
	private Index2D posToIndex(double x, double y){
		double dx = x - org_x; double dy = org_y - y;
		int col = (int)Math.floor(dx/res_x);
		int row = (int)Math.floor(dy/res_y);
		
		if(col<0 || row<0 || col>=this.cols || row>=this.rows){
			System.err.println("bad position leads to exceeding QuadGridmap index");
			System.err.printf("org(%.2f,%.2f), (%.2f, %.2f) -> (%d, %d)\n",org_x, org_y, x, y, col, row);
			return null;
		}
		return new Index2D(row, col);
	}
	
	public boolean isComplete(){
		return gridmaps[0]!=null && gridmaps[1]!=null && gridmaps[2]!=null && gridmaps[3]!=null;
	}
	
	/**
	 * given QuadMatrix index return cell
	 * @param index
	 * @param thres
	 * @return
	 */
	private T getCell(Index2D quadIndex) throws Exception{
		if(quadIndex.col<cols/2 && quadIndex.row<rows/2){
			//upLeft
			if(this.gridmaps[0]==null){
				throw new Exception("cell query out of map");
			}
			return this.gridmaps[0].getCell(quadIndex.row, quadIndex.col);
		}else if(quadIndex.col>=cols/2 && quadIndex.row<rows/2){
			//upRight, index.col-cols/2
			if(this.gridmaps[1]==null){
				throw new Exception("cell query out of map");
			}
			return this.gridmaps[1].getCell(quadIndex.row, quadIndex.col-cols/2);
		}else if(quadIndex.col<cols/2 && quadIndex.row>=rows/2){
			//downLeft, index.row-rows/2
			if(this.gridmaps[2]==null){
				throw new Exception("cell query out of map");
			}
			return this.gridmaps[2].getCell(quadIndex.row-rows/2, quadIndex.col);
		}else{
			//downRight, index.col-cols/2, index.row-rows/2
			if(this.gridmaps[3]==null){
				throw new Exception("cell query out of map");
			}
			return this.gridmaps[3].getCell(quadIndex.row-rows/2, quadIndex.col-cols/2);
		}
	}
	/**
	 * given 3D position return occupancy of cell
	 * @param obs
	 * @return
	 */
	public T getCell(Point3D obs) throws Exception{
		Index2D quadIndex = this.posToIndex(obs.y, obs.x);//east, north
		return this.getCell(quadIndex);
	}
}
