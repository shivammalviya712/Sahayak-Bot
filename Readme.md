
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

## Make octomap
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

> - Reduce the octomap range (Instructor)
>   - Doesm't help
> - Consuming too much resources (cpu)
>   - Even turning off gazebo's gui doesn't help 

> - Save octomap