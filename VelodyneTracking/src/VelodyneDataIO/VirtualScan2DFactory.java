package VelodyneDataIO;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import calibration.CoordinateFrame;

public class VirtualScan2DFactory {
	private DataInputStream dataStream;
	
	public VirtualScan2DFactory(File dataFile) throws IOException{
		dataStream = new DataInputStream(new BufferedInputStream(new FileInputStream(dataFile)));
	}
	
	public VirtualScan2D getVirtualScan2D() throws IOException{
		if(this.dataStream.readByte()!=36) throw new IOException("cannot find start symbol");
		
		float time = this.dataStream.readFloat();
		
		CoordinateFrame groundFrame = CoordinateFrame.readFromStream(this.dataStream);
		CoordinateFrame lidarFrame = CoordinateFrame.readFromStream(this.dataStream);
		
		int numOfPoint = this.dataStream.readInt();
		VirtualScan2D vs2d = new VirtualScan2D(time, groundFrame, numOfPoint);
		for(int i=0;i<numOfPoint;i++){
			vs2d.put(i, this.dataStream.readFloat(), this.dataStream.readFloat());
		}
		
		return vs2d;
	}
	
}
