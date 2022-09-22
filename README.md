# Voxblox Ground Truth
+ Create ground truth voxblox `.tsdf` or `.pcd` or `.ply`(not mesh, pointcloud) maps from `Gazebo` worlds

## Install
+ Make sure that [Voxblox](https://github.com/ethz-asl/voxblox#table-of-contents) and [Gazebo](http://gazebosim.org/tutorials?tut=ros_installing) are installed, then run
```bash
cd ~/your_workspace/src
git clone https://github.com/engcang/voxblox_ground_truth

cd ~/your_workspace
catkin build voxblox_ground_truth
source devel/setup.bash
```

## Gazebo plugin
#### Demo / Your own world
+ In order to use the plugin, it must be loaded as part of your Gazebo world.
+ To do this, add the following line to your `.world` file right after the `<world name='default'>` tag:
```xml
<plugin name="voxblox_ground_truth_plugin" filename="libvoxblox_ground_truth_plugin.so"/>
```

+ Start the demo by running
```bash
roslaunch voxblox_ground_truth gazebo_plugin_demo.launch
```

+ Set the desired voxel size with
```bash
rosparam set /voxblox_ground_truth/voxel_size 0.05
```

Then wait for Gazebo and Rviz finish loading. Once they're ready, call
```bash
rosservice call /gazebo/save_voxblox_ground_truth_to_tsdf "file_path: '$HOME/filename.tsdf'"

or

rosservice call /gazebo/save_voxblox_ground_truth_to_pcd "file_path: '$HOME/filename.pcd'"

or

rosservice call /gazebo/save_voxblox_ground_truth_to_ply "file_path: '$HOME/filename.ply'"
```