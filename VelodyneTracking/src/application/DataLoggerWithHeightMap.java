package application;

import gridmap_generic.GridmapMatrix;
import gridmap_generic.HeightCell;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;


import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;
import VelodyneView.LidarFrameProcessor;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;

public class DataLoggerWithHeightMap {
	
	public static void main(String argv[]){
		String raw_data_filename = "";
		String dst_data_filename = "";
		String height_map_dir = "";
		double low=-1;
		double high=-1;
		try{
			raw_data_filename = argv[0];
			dst_data_filename = argv[1];
			height_map_dir = argv[2];
			low = Double.parseDouble(argv[3]);
			high = Double.parseDouble(argv[4]);
		}catch(Exception e){
			e.printStackTrace();
			System.out.println("USAGE: java -jar *.jar raw_data_filename dst_data_filename height_map_filename min_height max_height");
			System.exit(-1);
		}
		
		log_data_using_height_map(raw_data_filename, dst_data_filename, height_map_dir, low, high);
	}
	
	static void log_data_using_height_map(String raw_data_filename, String dst_data_filename, String height_map_dir, double low, double high){
		DataLogger vtLogger = new DataLogger();
//		FrameTransformer mTransformer = new FrameTransformer();
//		CoordinateFrame calibFrame = new CoordinateFrame(new double[] {0,0,0,
//				0.9999800441981136, 0.00037833642524528284, 0.0036987820702358474,
//				0.00037833642524528284, 0.9896323094339619, -0.10125933784433956,
//				-0.003718152332877361, 0.10179006768396229, 0.9947925857075474});

		LidarFrameFactory lfFac = null;
		LidarFrameProcessor processor = null;		
		DataOutputStream dos = null;
		
		int pointCnt=0;
		try{
			File inputFile = new File(raw_data_filename);
			GridmapMatrix<HeightCell> matrix = GridmapMatrix.loadGridmapMatrix(new File(height_map_dir), new HeightCell(1), true);
			dos = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(new File(dst_data_filename))));

			lfFac=new LidarFrameFactory(inputFile);
			processor = new LidarFrameProcessor(lfFac, matrix, null);//no occupy map in this application
			processor.getReady(0);//start from time 0
				
			int cnt=0;
			while(true){
				processor.filterWithHeightGridmap(low, high);//low, high
				//log data
				//$(1char)_time(1f)_LidarFrame(12f)_numOfPoint(1long)_Points(3*numOfPoint float)
				pointCnt+=vtLogger.logVirtualTable(dos, processor.getCurFrame(), processor.getVirtualTable(), processor.getHeightMapFilter().getMask());
				//flush every 100 frames
				cnt++;					
				if(cnt%100==0){
					dos.flush();
					System.out.printf("total point logged %d\n", pointCnt);
					pointCnt=0;
				}
				processor.readNextFrame();//read next frame
			}
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			BuildHeightMap.checkMemory();
			
			processor.stop();
			try{
				dos.close();
			}catch(Exception e){
				
			}
			System.out.println("finish");
		}
	}

}
