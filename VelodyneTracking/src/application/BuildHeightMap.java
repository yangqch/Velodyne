package application;

import gridmap_generic.GridmapAttribute;
import gridmap_generic.GridmapMatrix;
import gridmap_generic.HeightCell;
import gridmap_generic.QuadGridmap;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;

import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

public class BuildHeightMap {
	/**
	 * make height map using StaticCell
	 * @param f
	 */
	static void updateHeightCell(GridmapMatrix<HeightCell> gmm, LidarFrame lf, FrameTransformer trans) throws Exception{
		VirtualTable virtualTable = new VirtualTable();
		VirtualTable.convertLidarFrame(lf, virtualTable);
		
		CoordinateFrame localWorldFrame = lf.getLocalWorldFrame();
		
		//Point3D[] obs = virtualTable.getPoint3D(null);
		Point3D[] obs = lf.getDataPoints(false);//don't need point in max range
		Point3D[] w_obs = trans.transform4D(localWorldFrame, null, obs);
		QuadGridmap<HeightCell> quad = gmm.makeQuadGridmap(localWorldFrame.getPosition().y, localWorldFrame.getPosition().x, lf.MAX_RANGE);
		if(quad==null){//quad=null when lidar range gets out of map boundary
			throw new Exception("Building height map: can't make quad, out of map boundary");
		}
		for(Point3D p3 : w_obs){
			HeightCell cell = quad.getCell(p3);
			cell.update(p3.z);
		}
	}
	
	public static void checkMemory(){
        int mb = 1024*1024;
        
        //Getting the runtime reference from system
        Runtime runtime = Runtime.getRuntime();
         
        System.out.println("##### Heap utilization statistics [MB] #####");
         
        //Print used memory
        System.out.println("Used Memory:"
            + (runtime.totalMemory() - runtime.freeMemory()) / mb);
 
        //Print free memory
        System.out.println("Free Memory:"
            + runtime.freeMemory() / mb);
         
        //Print total available memory
        System.out.println("Total Memory:" + runtime.totalMemory() / mb);
 
        //Print Maximum available memory
        System.out.println("Max Memory:" + runtime.maxMemory() / mb);
	}

	
	public static void buildHeightGridMap(String data_filename, String height_map_dir, double east_org, double north_org, double east_range, double north_range){
//		File dataFile = new File("/home/qichi/Qichi_Velodyne/processed_data/filtered_component_lidar_frame.dat");
		//File dataFile = new File("/home/qichi/Qichi_Velodyne/AidedINS/realtime/data/VELODYNE_agg_raw_road_use_midstate_intensity.dat");
		//File mapFile = new File("/home/qichi/Qichi_Velodyne/map/HeightMap/lidar_frame_Lowest5Height");
//		File mapFile = new File("/home/qichi/Qichi_Velodyne/map/HeightMap/SmallTest");
		File dataFile = new File(data_filename);
		File mapFile = new File(height_map_dir);		
		
		LidarFrameFactory lff=null;
		GridmapMatrix<HeightCell> matrix = null;
		HeightCell emptyCell = new HeightCell(5);
		FrameTransformer trans = new FrameTransformer();
		/////////////////////////make new map or load map////////////////////////////////////////////////////////
		try{
			matrix = GridmapMatrix.makeGridmapMatrix(mapFile, 
					east_org, north_org, east_range, north_range,
					new GridmapAttribute(100, 100, 0.1, 0.1), emptyCell);//size, resolution in meter unit
//			matrix = GridmapMatrix.makeGridmapMatrix(mapFile, 
//					-1600, 3000, 500, 500, 
//					new GridmapAttribute(100, 100, 0.1, 0.1), emptyCell);
			//matrix = GridmapMatrix.loadGridmapMatrix(mapFile);
			System.out.println(matrix);
		//////////////////////update frame by frame///////////////////////////////////////////////////////////////
			lff = new LidarFrameFactory(dataFile);
			//read lidar frame and transform to 2D virtual table
			while(true){
				LidarFrame lf = lff.getLidarFrame();
				//update grid occupancy using laser data
				updateHeightCell(matrix, lf, trans);
			}
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			lff.close();
			matrix.finish();
		}
	}

	public static void updateHeightGridMap(String data_filename, String height_map_dir){
//		File dataFile = new File("/home/qichi/Qichi_Velodyne/processed_data/filtered_component_lidar_frame.dat");
		//File dataFile = new File("/home/qichi/Qichi_Velodyne/AidedINS/realtime/data/VELODYNE_agg_raw_road_use_midstate_intensity.dat");
		//File mapFile = new File("/home/qichi/Qichi_Velodyne/map/HeightMap/lidar_frame_Lowest5Height");
//		File mapFile = new File("/home/qichi/Qichi_Velodyne/map/HeightMap/SmallTest");
		File dataFile = new File(data_filename);
		File mapFile = new File(height_map_dir);		
		
		LidarFrameFactory lff=null;
		GridmapMatrix<HeightCell> matrix = null;
		HeightCell emptyCell = new HeightCell(5);
		FrameTransformer trans = new FrameTransformer();
		/////////////////////////make new map or load map////////////////////////////////////////////////////////
		try{
			matrix = GridmapMatrix.loadGridmapMatrix(mapFile, emptyCell, false);
			System.out.println(matrix);
		//////////////////////update frame by frame///////////////////////////////////////////////////////////////
			lff = new LidarFrameFactory(dataFile);
			//read lidar frame and transform to 2D virtual table
			while(true){
				LidarFrame lf = lff.getLidarFrame();
				//update grid occupancy using laser data
				updateHeightCell(matrix, lf, trans);
			}
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			lff.close();
			matrix.finish();
		}
	}

	
	
	public static void main(String[] argv){
		boolean build_or_update = false;
		String data_filename = "";
		String height_map_dir = "";
		int east_org=0, north_org=0, east_range=0, north_range=0;
		try{
			if(argv[0].equals("-b")){
				build_or_update = true;
			}else if(argv[0].equals("-u")){
				build_or_update = false;
			}else{
				throw new Exception("1st argument is build_or_update, -b: build, -u: update");
			}
			data_filename = argv[1];
			height_map_dir = argv[2];
			if(build_or_update){
				east_org=Integer.parseInt(argv[3]);
				north_org=Integer.parseInt(argv[4]);
				east_range=Integer.parseInt(argv[5]);
				north_range=Integer.parseInt(argv[6]);
			}
		}catch(Exception e){
			e.printStackTrace();
			System.out.println("USAGE: java -jar *.jar build_or_update(-b: build, -u: update) lidar_data_filename height_map_filename [if update: east_org north_org east_range(to west) north_range(to south)]");
			System.exit(-1);
		}
		if(build_or_update){
			buildHeightGridMap(data_filename, height_map_dir, east_org, north_org, east_range, north_range);
		}else{
			updateHeightGridMap(data_filename, height_map_dir);
		}
		
	}
}
