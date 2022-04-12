## About this repo
This repository is created by following the README of https://github.com/AutoLidarPerception/tracking_lib.

## Install catkin
sudo apt_get update
sudo apt install python3-catkin-tools python3-osrf-pycommon

## Zsh lines to create the whole project
```
mkdir track1
cd track1
mkdir -p src/common
mkdir -p src/perception/libs     
cd src/common
git clone https://github.com/cyqian97/common_lib.git libs
cd ../perception/libs
git clone https://github.com/cyqian97/roi_filters_lib.git roi_filters
git clone https://github.com/cyqian97/object_builders_lib.git object_builders
git clone https://github.com/cyqian97/segmenters_lib.git segmenters
git clone https://github.com/cyqian97/feature_extractors_lib.git
git clone https://github.com/cyqian97/tracking_lib.git          
cd ../../..
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Install kitti_ros
First clone the repository,
```
cd src
git clone https://github.com/cyqian97/kitti_ros.git
```

Install a few packages and run catkin. Do not use a venv! Otherwise different pythons will be used and that will mess things up.
```
python3 -m pip install -r kitti_ros/requirements.txt
python3 -m pip install empy                   
python3 -m pip install catkin-pkg  
python3 -m pip install pyyaml
python3 -m pip install numpy -U
python3 -m pip install pykitti    
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Download Kitti raw data
Download the synced data and the calibration files from [Kitti website](cvlibs.net/datasets/kitti/raw_data.php). Refer to the README file in kitti_ros repository for the data folder structure and how to specify the data path.

## How to use
In the workspace folder, run ```source devel/setup.zsh``` in all terminals.
In terminal 0, run ```roscore```.
In terminal 1, run ```roslaunch segmenters_lib demo.launch```.
In termical 2, run ```sudo chmod 777 /dev/input/event2```. Note that the device id for the keyboard can vary between different computers, one can used
```
/dev/input/by-id
ls -l
```
to find the correct id, sometimes it does not list the keyboard, at which time one can also try ```cat /proc/bus/input/devices```. The variable 'keyboard_file' in 'kitti_player.lauch' should also be modified correspondingly, or one can specify it in the roslaunch command.
After configuring the keyboard access, we then run ```roslaunch kitti_ros kitti_player.launch```. In this terminal, one can use the keyboard to control the how the data is played. Right arrow key plays the next frame and space key starts continuous playing.

## Fix errors (already updated in the forked repositories)
Because of the current pcl 1.10 requirs c++14 and above, we need to change all
```
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-std=c++14)
```
to

```
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
```

Problem also occurs when compiling /$CATKIN_WS/src/perception/libs/segmenters/src/don_segmenter.cpp, simply change the following code starting at line 105
```
for (; iter != clusters_indices.end(); ++iter) {
    PointICloudPtr cluster(new PointICloud);
    pcl::copyPointCloud<PointN, PointI>(*don_cloud_filtered, *iter,
                                        *cluster);

    cloud_clusters.push_back(cluster);
}
```
to
```
    for (const pcl::PointIndices & indices : clusters_indices) {
        PointICloudPtr cluster(new PointICloud);

        pcl::PointCloud<pcl::PointNormal> cloud_in = *don_cloud_filtered;
        pcl::PointCloud<pcl::PointXYZI> cloud_out = *cluster;

        // Allocate enough space and copy the basics
        cloud_out.points.resize (indices.indices.size ());
        cloud_out.header   = cloud_in.header;
        cloud_out.width    = std::uint32_t (indices.indices.size ());
        cloud_out.height   = 1;
        cloud_out.is_dense = cloud_in.is_dense;
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
        cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

        // Iterate over each point
        for (std::size_t i = 0; i < indices.indices.size (); ++i) {
            pcl::copyPoint (cloud_in.points[indices.indices[i]], cloud_out.points[i]);
        }

        cloud_clusters.push_back(cluster);
    }

```
following the tutorial https://github.com/AutoLidarPerception/segmenters_lib/issues/15.

For the kitti_ros package, more things need to be modified.
In the forked repository, the grammar of print and exception has been adapted to python3.
The package thread needed to be renamed to _thread.
'numpy' needs to be updated.
Some ```sys.path.append``` need to be added for python to find the packages.
The dot ```.``` needs to be added to as ```form .package import classname``` in the user-defined package 'utils' to avoid recursive import.
Function name of ```kitti.get_oxts_packets_and_poses``` needs to be changed to ```kitti.load_oxts_packets_and_poses``` in 'kitti_player.py'.
