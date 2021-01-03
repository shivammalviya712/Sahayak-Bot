## Setup

> ```ImportError: No module named gazebo_ros_link_attacher.srv```
> - Reinstalled task4_spawn_models.pyc

> **Gazebo not loading**
> - Just wait man it will load
> - One time pain in ass

> ```[Wrn] [msgs.cc:1852] Conversion of sensor type[depth] not supported.```
> - Not solved
> - Instructor told to move on
---

## Explore
> - Make urdf link and joints graph

> - Visulaize urdf file generated in moveit_setup_assistant

> - Two cameras -> Choose 2nd one

```
imageTopicName: /camera/color/image_raw2
depthImageTopicName: /camera/depth/image_raw2
pointCloudTopicName: depth/points2 (camera2/depth/points2)
```

> - Difference between above pieces of shit -> Explore message types
> - See ros messages info
> - [Pointcloud](https://youtu.be/yXCkyuo8bcs)
```
imageTopicName: /camera/color/image_raw2: sensor_msgs/Image
depthImageTopicName: /camera/depth/image_raw2: sensor_msgs/Image
pointCloudTopicName: depth/points2 (camera2/depth/points2): sensor_msgs/PointCloud2
```

> - Visualize this images
```
rosrun image_view image_view image:=/camera/depth/image_raw2
```

> - Visualize pointcloud2 in rviz
```
For frame [camera_depth_frame]: Frame [camera_depth_frame] does not exist
```
>       - Visualize tf_tree
>       - Transfrom is not getting published
>       - Start jointStatePublisher and robotStatePublisher
>       - Make new package using moveit setup assistant
---

## Generate octomap
> - Start move_group node
>   - ros_controller.yaml
>   - trajectory_controller.yaml
>   - joint_state_controller.yaml
>   - rviz
>   - move group node
```
[ERROR] [1608451086.851663015]: Group 'gripper' is not a chain
[ERROR] [1608451086.851793328]: Kinematics solver of type 'kdl_kinematics_plugin/KDLKinematicsPlugin' could not be initialized for group 'gripper'
[ERROR] [1608451086.851897584]: Kinematics solver could not be instantiated for joint group gripper.
```
> - Remove kinematics solver from the package

```
[ERROR] [1608460956.311072813, 2.437000000]: ebot_base link was not found
```
> - Load complete urdf file in moveit_setup_assistant


```
[ERROR] [1608461396.076735933]: No sensor plugin specified for octomap updater 0; ignoring.
```
> - Specify pluggins

> - Configure the 3D sensors on your robot with MoveIt
>   - Point cloud

```
[ERROR] [1608527089.121164405, 4.343000000]: Transform error: Lookup would require extrapolation into the future.  Requested time 4.260000000 but the latest data is at time 4.200000000, when looking up transform from frame [FWL] to frame [camera_depth_frame2]
[ERROR] [1608527089.121428651, 4.343000000]: Transform cache was not updated. Self-filtering may fail.
```
> - [tf/Tutorials/tf and Time (Python)](http://library.isr.ist.utl.pt/docs/roswiki/tf(2f)Tutorials(2f)tf(20)and(20)Time(2028)Python(29).html)
> - tf_monitor
> - rqt_tf_tree
> - Why robot_state_publisher publishes at different frequency?
> - Why some publisher have 10000 publish rate?
> - Increase joint_state_publisher rate
>   - 20 works fine

> - Consuming too much resources (cpu)
>   - Reduce the octomap range (Instructor) -> Doesm't help
>   - Even turning off gazebo's gui doesn't help 
---

## Save octomap
> - [Saving tutorial](https://www.youtube.com/watch?v=Ib1ISnLlD38)
> - Explore octomap_server
>   - octomap_server loads a 3D map (as Octree-based OctoMap) and distributes it to other nodes in a compact binary format. See octomap_saver on how to request or save a map file.
> - Explore octomap_mapping
> - TODO: Check later if required

---

## PCL python interface
> - [python-pcl dependency error](https://github.com/strawlab/python-pcl/issues/317)
> - PCL installed but at what cost (whole ros got deleted (-_-))
> - Install ros
> - PCL and ros melodic can't be together
> - Here comes conda, create new env and change the interpreter path in the script
> - [Of course!! Some errors](https://github.com/strawlab/python-pcl/issues/285)
> - [About methods of PCL classes](https://nlesc.github.io/python-pcl/)
> - [Ros to pcl message conversion](https://www.programcreek.com/python/example/99841/sensor_msgs.msg.PointCloud2)
> - [Filter codes](https://github.com/mithi/point-cloud-filter)

> - _**Dependency error**_
>   - Create softlinks(trial1)
        
        ```
        conda create -n trial1 python=2.7.17
        conda install -c sirokujira pcl --channel conda-forge
        conda install -c sirokujira python-pcl --channel conda-forge
        ln -s libboost_system.so.1.72.0 libboost_system.so.1.66.0
        ln -s libboost_filesystem.so.1.72.0 libboost_filesystem.so.1.66.0
        ln -s libboost_thread.so.1.72.0 libboost_thread.so.1.66.0
        ln -s libboost_date_time.so.1.72.0 libboost_date_time.so.1.66.0
        ln -s libboost_iostreams.so.1.72.0 libboost_iostreams.so.1.66.0
        ln -s libboost_chrono.so.1.72.0 libboost_chrono.so.1.66.0
        ln -s libboost_atomic.so.1.72.0 libboost_atomic.so.1.66.0
        ln -s libboost_regex.so.1.72.0 libboost_regex.so.1.66.0
        ```

        ```
        ImportError: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so.1.8: undefined symbol: _ZN3pcl9PCDWriter10writeASCIIERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_14PCLPointCloud2ERKN5Eigen6MatrixIfLi4ELi1ELi0ELi4ELi1EEERKNSC_10QuaternionIfLi0EEEi

        ```
>       - Couldn't solve this

>   - [Build from source](https://github.com/strawlab/python-pcl/issues/158)
>   - [udacity/RoboND-Perception-Exercises](https://github.com/udacity/RoboND-Perception-Exercises)
    ```
    cl/_pcl_180.cpp:1:2: error: #error Do not use this file, it is the result of a failed Cython compilation.
    #error Do not use this file, it is the result of a failed Cython compilation.
    ^~~~~
    error: command 'x86_64-linux-gnu-gcc' failed with exit status 1
    ```
>       - Some issue with cython version
>           - Cython version
        
        ```
        pcl/pxi/PointCloud_PointXYZRGB_180.pxi:104:29: Cannot take address of Python variable
        ```

>           - Works with Cython=0.25.2 
>   - [link](https://github.com/strawlab/python-pcl/issues/278)

> - Explore sensor_msgs/PointCloud2
> - Convert ROS pc to PCL pc

### Voxel Grid filter 
> - Downsampling using the Voxel Grid Filter
> - The idea behind data downsampling is just to speed things up 
>  less points means less time needed to spend within the segmentation loop.

### Passthrough filter
> - Filter out points which are outside a specific range of a specific axis

### Ransac Plane fitting
> - [Youtube](https://www.youtube.com/watch?v=9D5rrtCC_E0)
> - [Medium](https://medium.com/@ajithraj_gangadharan/3d-ransac-algorithm-for-lidar-pcd-segmentation-315d2a51351)
> - Seperate outliers and inliers


### Statistically outlier filter
> - This filters out the statistical noise of the scene
> - Uses point neighborhood statistics to filter outlier data
> - TODO: [Issue when you use for PointCloud_PointXYZRGB](https://github.com/udacity/RoboND-Perception-Exercises/issues/18)

> - Visualise *.pcd file

```
pcl_viewer -multiview 1 <pcd_filepath>
```

### Cropbox Filter
> - Available in PointCloud, not in PointCloud_PointXYZRGB()
> - I implemented it using 3 passthrough filters in series, for PointCloud_PointXYZRGB()
> - Available in PointCloud

### Euclidean cluster extraction
> - [KDtree](https://youtu.be/ivdmGcZo6U8)
> - In nutshell we create clusters of points, and thus separate objects
> - Available in PointCloud, not in PointCloud_PointXYZRGB()

> - TODO: Order of filter can be optimised later if required, by checking the time taken

---
## Recognition and localisation 
> -  Explore all the algorithms
>       - [Pointnet](https://arxiv.org/pdf/1612.00593.pdf)
>       - [Attentional PointNet](https://hal.inria.fr/hal-02156555/document)


> - Pointcloud map
>      - [Map_merge_3d](http://wiki.ros.org/map_merge_3d)
>      - [Laser_assembler](http://wiki.ros.org/laser_assembler)
>      - [Octomap](https://github.com/OctoMap/octomap_ros/blob/melodic-devel/include/octomap_ros/conversions.h)

> - PCD model of objects
>      - [Help](https://products.aspose.app/3d/conversion/dae-to-pdf)
>      - [Help](https://github.com/PaulBernier/obj2pcd)

> - Train Pointnet
>      - [Architecture and Implementation](https://medium.com/@luis_gonzales/an-in-depth-look-at-pointnet-111d7efdaa1a)
>      - [Tutorial](https://towardsdatascience.com/deep-learning-on-point-clouds-implementing-pointnet-in-google-colab-1fd65cd3a263)
>      - Implement your own motherfucking pointnet
>      - [Pre-trained model in pytorch](https://github.com/meder411/PointNet-PyTorch?files=1)
>      - [Pre-trained model in tensorflow](https://github.com/TianzhongSong/PointNet-Keras)
>      - [Point Cloud Deep Learning Extension Library for PyTorch](https://pypi.org/project/torch-points3d/)

> - I could load Pointnet model on using pytorch
>      - [Pre-trained model in pytorch](https://github.com/meder411/PointNet-PyTorch?files=1)
>      - Change some part of the codes so that it can work on your cpu

---
> **_Thank me later you noob coders_**