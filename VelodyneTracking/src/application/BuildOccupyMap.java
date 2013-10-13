package application;

import gridmap_generic.GridmapAttribute;
import gridmap_generic.GridmapMatrix;
import gridmap_generic.OccupyCell;
import gridmap_generic.QuadGridmap;

import java.io.File;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;
import VelodyneView.LidarFrameProcessor;
import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;

public class BuildOccupyMap {

	static void updateOccupyCell(GridmapMatrix<OccupyCell> gmm, float time, CoordinateFrame localWorldFrame, Point3D[] w_lowPoints, Point3D[] w_midPoints) throws Exception{
		QuadGridmap<OccupyCell> quad = gmm.makeQuadGridmap(localWorldFrame.getPosition().y, localWorldFrame.getPosition().x, LidarFrame.MAX_RANGE);
		if(quad==null){//quad=null when lidar range gets out of map boundary
			throw new Exception("Building occupy map: can't make quad, out of map boundary");
		}
		for(Point3D p : w_lowPoints){
			OccupyCell cell = quad.getCell(p);
			cell.updateTime(time);
			cell.update(-1);//low points means free space
		}
		for(Point3D p : w_midPoints){
			OccupyCell cell = quad.getCell(p);
			cell.updateTime(time);
			cell.update(1);//mid points means free space
		}
	}
	
	
	public static void buildOccupyGridMap(String low_level_data_filename, String mid_level_data_filename, String occupy_map_dir, double conn_comp_neighbor_thres, int conn_comp_size_thres, 
			double east_org, double north_org, double east_range, double north_range){
		File lowDataFile = new File(low_level_data_filename);
		File midDataFile = new File(mid_level_data_filename);
		File mapFile = new File(occupy_map_dir);
		
		LidarFrameFactory lowFac = null;
		LidarFrameFactory midFac = null;
		GridmapMatrix<OccupyCell> matrix = null;
		OccupyCell emptyCell = new OccupyCell();
		
		try{
			matrix = GridmapMatrix.makeGridmapMatrix(mapFile, 
					east_org, north_org, east_range, north_range,
					new GridmapAttribute(100, 100, 0.1, 0.1), emptyCell);//size, resolution in meter unit
			
			System.out.println(matrix);
			
			FrameTransformer trans = new FrameTransformer();
			lowFac = new LidarFrameFactory(lowDataFile);
			midFac = new LidarFrameFactory(midDataFile);
			
			LidarFrameProcessor lowProcessor = new LidarFrameProcessor(lowFac, null, null);
			lowProcessor.getReady(0);
			LidarFrameProcessor midProcessor = new LidarFrameProcessor(midFac, null, null);
			midProcessor.getReady(0);
			//read lidar frame and transform to 2D virtual table
			while(true){
				lowProcessor.findConnComp(0, 0);//why?
				midProcessor.findConnComp(conn_comp_neighbor_thres, conn_comp_size_thres);
				//time
				float time = lowProcessor.getCurFrame().getTime();
				if(time!=midProcessor.getCurFrame().getTime()){
					throw new Exception("time of low and mid data file are out of synchronized");
				}
				//body frame
				CoordinateFrame localWorldFrame = lowProcessor.getCurFrame().getLocalWorldFrame();
				//prepare points in world frame 
				Point3D[] w_lowPoints = trans.transform4D(localWorldFrame, null, lowProcessor.getVirtualTable().getPoint3D(null, false));
				Point3D[] w_midPoints = trans.transform4D(localWorldFrame, null, midProcessor.getVirtualTable().getPoint3D(midProcessor.getCompFilter().getCompMask(), false));
				//update grid occupancy using laser data
				updateOccupyCell(matrix, time, localWorldFrame, w_lowPoints, w_midPoints);
				//read next
				lowProcessor.readNextFrame();
				midProcessor.readNextFrame();
			}
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			lowFac.close();
			midFac.close();
			matrix.finish();
		}		
	}
	
	public static void updateOccupyGridMap(String low_level_data_filename, String mid_level_data_filename, String occupy_map_dir, double conn_comp_neighbor_thres, int conn_comp_size_thres){
		File lowDataFile = new File(low_level_data_filename);
		File midDataFile = new File(mid_level_data_filename);
		File mapFile = new File(occupy_map_dir);
		
		LidarFrameFactory lowFac = null;
		LidarFrameFactory midFac = null;
		GridmapMatrix<OccupyCell> matrix = null;
		OccupyCell emptyCell = new OccupyCell();
		
		try{
			matrix = GridmapMatrix.loadGridmapMatrix(mapFile, emptyCell, false);
			
			System.out.println(matrix);
			
			FrameTransformer trans = new FrameTransformer();
			lowFac = new LidarFrameFactory(lowDataFile);
			midFac = new LidarFrameFactory(midDataFile);
			
			LidarFrameProcessor lowProcessor = new LidarFrameProcessor(lowFac, null, null);
			lowProcessor.getReady(0);
			LidarFrameProcessor midProcessor = new LidarFrameProcessor(midFac, null, null);
			midProcessor.getReady(0);
			//read lidar frame and transform to 2D virtual table
			while(true){
				lowProcessor.findConnComp(0, 0);
				midProcessor.findConnComp(conn_comp_neighbor_thres, conn_comp_size_thres);
				//time
				float time = lowProcessor.getCurFrame().getTime();
				if(time!=midProcessor.getCurFrame().getTime()){
					throw new Exception("time of low and mid data file are out of synchronized");
				}
				//body frame
				CoordinateFrame localWorldFrame = lowProcessor.getCurFrame().getLocalWorldFrame();
				//prepare points in world frame 
				Point3D[] w_lowPoints = trans.transform4D(localWorldFrame, null, lowProcessor.getVirtualTable().getPoint3D(null, false));
				Point3D[] w_midPoints = trans.transform4D(localWorldFrame, null, midProcessor.getVirtualTable().getPoint3D(midProcessor.getCompFilter().getCompMask(), false));
				//update grid occupancy using laser data
				updateOccupyCell(matrix, time, localWorldFrame, w_lowPoints, w_midPoints);
				//read next
				lowProcessor.readNextFrame();
				midProcessor.readNextFrame();
			}
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			lowFac.close();
			midFac.close();
			matrix.finish();
		}		
	}

	public static void main(String[] argv){
		boolean build_or_update = false;
		String low_level_data_filename = "";
		String mid_level_data_filename = "";
		String occupy_map_dir = "";
		double conn_comp_neighbor_thres = 0;//thres for neighbor points
		int conn_comp_size_thres = 0;//num of points
		int east_org=0, north_org=0, east_range=0, north_range=0;
		try{
			if(argv[0].equals("-b")){
				build_or_update = true;
			}else if(argv[0].equals("-u")){
				build_or_update = false;
			}else{
				throw new Exception("1st argument is build_or_update, -b: build, -u: update");
			}
			low_level_data_filename = argv[1];
			mid_level_data_filename = argv[2];
			occupy_map_dir = argv[3];
			conn_comp_neighbor_thres = Double.parseDouble(argv[4]);
			conn_comp_size_thres = Integer.parseInt(argv[5]);
			
			if(build_or_update){
				east_org=Integer.parseInt(argv[6]);
				north_org=Integer.parseInt(argv[7]);
				east_range=Integer.parseInt(argv[8]);
				north_range=Integer.parseInt(argv[9]);
			}
		}catch(Exception e){
			e.printStackTrace();
			System.out.println("USAGE: java -jar *.jar build_or_update(-b: build, -u: update) low_level_data_filename high_level_data_filename occupy_map_filename conn_comp_neighbor_thres(thres for neighbor points distance) conn_comp_size_thres(min num of points)\n" +
					"the last two are used for mid_object only");
			System.exit(-1);
		}
		if(build_or_update){
			buildOccupyGridMap(low_level_data_filename, mid_level_data_filename, occupy_map_dir, conn_comp_neighbor_thres, conn_comp_size_thres, east_org, north_org, east_range, north_range);
		}else{
			updateOccupyGridMap(low_level_data_filename, mid_level_data_filename, occupy_map_dir, conn_comp_neighbor_thres, conn_comp_size_thres);
		}
		
		//build_draw_small_map(low_level_data_filename, mid_level_data_filename, occupy_map_dir, conn_comp_neighbor_thres, conn_comp_size_thres);
	}
}
