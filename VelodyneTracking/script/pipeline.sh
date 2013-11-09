data_dir=$1

raw_frame=$data_dir"/VELODYNE_agg_raw_road_use_midstate.dat"
frame_data=$data_dir"/frame.dat"
vt_data=$data_dir"/vt.dat"
low_data=$data_dir"/low_object_0_0.3.dat"
mid_data=$data_dir"/mid_object_0.3_1.5.dat"

height_map=$data_dir"/map/HeightMap"

java -Xms1024M -Xmx2048M -jar ./log_data_transformed.jar -f $raw_frame $frame_data
java -Xms1024M -Xmx2048M -jar ./log_data_transformed.jar -v $raw_frame $vt_data

#build heigh map with bound limit in (east, north) framework. the most west, the most north, range along west->east, range along north->south
java -Xms1024M -Xmx2048M -jar ./build_height_map.jar -b $frame_data $height_map $2 $3 $4 $5
#extract points in a certain height range, low hight in meter
java -Xms1024M -Xmx2048M -jar ./log_data_with_height_map.jar $vt_data $low_data $height_map 0 0.3
java -Xms1024M -Xmx2048M -jar ./log_data_with_height_map.jar $vt_data $mid_data $height_map 0.3 1.5
