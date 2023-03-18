# openpcdet-ros_try  
-------------------------

0302:  
`catkin_tools`安装方法：[链接](https://catkin-tools.readthedocs.io/en/latest/installing.html)  

0303:  
try to run commend`roslaunch openpcdet 3d_object_detector.launch` in zsh, but get  
```
Traceback (most recent call last):
  File "/home/xilm/fuxian/workspace/OpenPCDet_ws/src/OpenPCDet_ros/src/inference.py", line 30, in <module>
    import torch
ModuleNotFoundError: No module named 'torch'
[object_3d_detector_node-2] process has died [pid 206938, exit code 1, cmd /home/xilm/fuxian/workspace/OpenPCDet_ws/src/OpenPCDet_ros/src/inference.py __name:=object_3d_detector_node __log:=/home/xilm/.ros/log/8df7978a-b96e-11ed-9991-95647510a683/object_3d_detector_node-2.log].
log file: /home/xilm/.ros/log/8df7978a-b96e-11ed-9991-95647510a683/object_3d_detector_node-2*.log
```

Have not found the proper solution...  
To be continued...  
Another solution: `source activate base(this is my env name)`, enter conda environment, then make.  

0312：
Solution find!  
Some path code has been modified.  
BTW, when run rosbag, notice if there is "RUNNING". If not, press space to turn state as running so that the RViz can show the outcome.  

0317： 开始尝试复现其他框架  
1. 将原来的sequence.bag换为kitti里其他的raw data：  
- 在终端运行`rosbag play kitti_sequence11_half.bag -r 1 --pause`后，按下空格，显示为running;接着运行`rostopic list`查看bag发布了什么话题：  
```
(base) xilm@xilm-MS-7D17:~$ rostopic list
/clock
/rosout
/rosout_agg
/velodyne_points
```  
- 使用从kitti官网下载的raw data：2011_09_26_0005，利用kitti2bag转为bag后，将bag进行play查看发布的topic：  
```
(base) xilm@xilm-MS-7D17:~$ rostopic list
/clock
/kitti/camera_color_left/camera_info
/kitti/camera_color_left/image_raw
/kitti/camera_color_right/camera_info
/kitti/camera_color_right/image_raw
/kitti/camera_gray_left/camera_info
/kitti/camera_gray_left/image_raw
/kitti/camera_gray_right/camera_info
/kitti/camera_gray_right/image_raw
/kitti/oxts/gps/fix
/kitti/oxts/gps/vel
/kitti/oxts/imu
/kitti/velo/pointcloud
/rosout
/rosout_agg
/tf
/tf_static
```  
可以看到，这里的点云节点名称从`/velodyne_points`改为了`/kitti/velo/pointcloud`,由此，将`config.yaml`和`.rviz`文件中的名称改过来  
改完之后发现rviz上无法显示点云，但是能进行检测  
- 突然发现，这里运行rosbag时`rosbag play kitti_2011_09_26_drive_0005_synced.bag -r 0.1 --pause`,-r后面的参数为0.1是帧率，只有帧率低了，检测框才能和点云帧对齐，因为检测框发布的很慢。据此修改了pointpillar_ros的代码，把launch文件里直接运行rosbag的命令去掉，在终端自己运行kitti的bag，帧率调整为0.1就可以看到好的检测结果了。  
- 这么说，还需要解决如何让检测框快速发布的问题，不然根本不可能进行实时检测。。。  
0318:  
1. 解决了更换一个kitti bag的问题，就是通过修改之前的两个文件，然后在rviz上修改一下global options的fixed frame,将其从velodyne改为velo_link  
![1679105917(1)](https://user-images.githubusercontent.com/96283702/226078139-bb5ada82-88e9-42c9-a00a-1215c9a58df6.png)  
2. 接下来准备再更换几个kitti的bag，看看有没有共性问题，方法就是下载raw data后使用kitti2bag转换  
- 下载raw data，这里我使用0084；  
- 接着把他们解压，解压后应为2011_09_26-->synced+calib，然后回到2011_09_26前一个目录下进行kitti2bag；  
- 查看它的rostopic  
```
(base) xilm@xilm-MS-7D17:~$ rostopic list
/clicked_point
/clock
/detect_3dbox
/initialpose
/kitti/camera_color_left/camera_info
/kitti/camera_color_left/image_raw
/kitti/camera_color_right/camera_info
/kitti/camera_color_right/image_raw
/kitti/camera_gray_left/camera_info
/kitti/camera_gray_left/image_raw
/kitti/camera_gray_right/camera_info
/kitti/camera_gray_right/image_raw
/kitti/oxts/gps/fix
/kitti/oxts/gps/vel
/kitti/oxts/imu
/kitti/velo/pointcloud
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static
```  
3. 接着更换测试的模型文件，之前使用的是pv_rcnn，现在要对openpcdet里面的其他文件都进行一个测试：  
