package VelodyneDataIO;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

public class VirtualTableFactory {
	DataInputStream reader;
	
	public VirtualTableFactory(File data_file) throws IOException{
		this.reader = new DataInputStream(new BufferedInputStream(new FileInputStream(data_file)));
	}
	
//	public VirtualTable getVirtualTable() throws IOException{
//		if(this.reader.readByte()!=36) throw new IOException("cannot find start symbol");
//		//read table min, max, resolution
//		
//		//read elements, rows first
//	}
}
